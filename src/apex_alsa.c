// SPDX-License-Identifier: GPL-2.0
/*
 * ALSA front-end for the Apex/Gasket driver. See apex_alsa.h for rationale.
 *
 * Design (mirrors the V4L2 layer's "coexistence + copy" approach):
 *  - One PCM device with a playback substream (input audio into the NPU) and
 *    a capture substream (output audio from the NPU).
 *  - Each substream uses its own vmalloc DMA ring buffer allocated by the
 *    ALSA core; the Apex coherent buffer is never used as the ring directly,
 *    only copied to/from, so the existing coherent mmap path is untouched.
 *  - The capture side is advanced by the OUTPUT_ACTV_QUEUE interrupt via the
 *    in-kernel callback (coexisting with eventfd). Each interrupt copies one
 *    period of result audio from the coherent buffer into the ring and calls
 *    snd_pcm_period_elapsed().
 *  - The playback side copies each elapsed period from the ring into the
 *    coherent input region and records that input is ready; the actual
 *    inference is triggered by the userspace runtime through legacy ioctls.
 *    A host timer paces the playback period callbacks.
 */

#include "apex_alsa.h"

#include "gasket_core.h"
#include "gasket_interrupt.h"

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/io.h>
#include <linux/hrtimer.h>
#include <linux/math64.h>
#include <linux/version.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>

/* Must match APEX_INTERRUPT_OUTPUT_ACTV_QUEUE in apex_driver.c. */
#define APEX_ALSA_DONE_INTERRUPT 3

/* Supported PCM format constraints (userspace negotiates within these). */
#define APEX_ALSA_RATE_MIN     8000
#define APEX_ALSA_RATE_MAX     48000
#define APEX_ALSA_CH_MIN       1
#define APEX_ALSA_CH_MAX       2
#define APEX_ALSA_BUFFER_BYTES (256 * 1024)
#define APEX_ALSA_PERIOD_MIN   256
#define APEX_ALSA_PERIOD_MAX   (128 * 1024)

struct apex_alsa_stream {
	struct snd_pcm_substream *substream;
	/* Current DMA ring position in bytes (mod buffer size). */
	unsigned int pos;
	unsigned int period_bytes;
	unsigned int buffer_bytes;
	bool running;
};

struct apex_alsa {
	struct gasket_dev *gasket_dev;
	struct snd_card *card;
	struct snd_pcm *pcm;

	spinlock_t lock;

	struct apex_alsa_stream playback;
	struct apex_alsa_stream capture;

	/* Paces playback period callbacks (input side has no HW irq). */
	struct hrtimer playback_timer;
	ktime_t playback_period_time;

	bool cb_registered;
};

static const struct snd_pcm_hardware apex_alsa_hw = {
	.info = SNDRV_PCM_INFO_INTERLEAVED |
		SNDRV_PCM_INFO_BLOCK_TRANSFER |
		SNDRV_PCM_INFO_MMAP |
		SNDRV_PCM_INFO_MMAP_VALID,
	.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S32_LE |
		   SNDRV_PCM_FMTBIT_FLOAT_LE,
	.rates = SNDRV_PCM_RATE_8000_48000,
	.rate_min = APEX_ALSA_RATE_MIN,
	.rate_max = APEX_ALSA_RATE_MAX,
	.channels_min = APEX_ALSA_CH_MIN,
	.channels_max = APEX_ALSA_CH_MAX,
	.buffer_bytes_max = APEX_ALSA_BUFFER_BYTES,
	.period_bytes_min = APEX_ALSA_PERIOD_MIN,
	.period_bytes_max = APEX_ALSA_PERIOD_MAX,
	.periods_min = 2,
	.periods_max = 1024,
};

/* ---------------------------------------------------------------------------
 * Capture: advanced by the OUTPUT_ACTV_QUEUE interrupt (result audio ready).
 * Runs in hard-IRQ context; bounded copy + period_elapsed only.
 * ------------------------------------------------------------------------- */
static void apex_alsa_done_cb(void *cb_data, int interrupt_index)
{
	struct apex_alsa *aa = cb_data;
	struct apex_alsa_stream *s = &aa->capture;
	struct snd_pcm_substream *ss;
	struct gasket_coherent_buffer *coh;
	unsigned char *ring;
	size_t copy_len, first;
	unsigned long flags;
	bool elapsed = false;

	if (interrupt_index != APEX_ALSA_DONE_INTERRUPT)
		return;

	spin_lock_irqsave(&aa->lock, flags);
	ss = s->substream;
	if (!s->running || !ss || !ss->runtime) {
		spin_unlock_irqrestore(&aa->lock, flags);
		return;
	}

	coh = &aa->gasket_dev->coherent_buffer;
	ring = ss->runtime->dma_area;
	copy_len = s->period_bytes;
	if (coh->length_bytes && copy_len > coh->length_bytes)
		copy_len = coh->length_bytes;

	if (ring && coh->virt_base && copy_len) {
		/* Copy one period, wrapping at the ring boundary. */
		first = min_t(size_t, copy_len, s->buffer_bytes - s->pos);
		memcpy_fromio(ring + s->pos, coh->virt_base, first);
		if (copy_len > first)
			memcpy_fromio(ring, coh->virt_base + first,
				      copy_len - first);
		s->pos += copy_len;
		if (s->pos >= s->buffer_bytes)
			s->pos -= s->buffer_bytes;
		elapsed = true;
	}
	spin_unlock_irqrestore(&aa->lock, flags);

	if (elapsed)
		snd_pcm_period_elapsed(ss);
}

/* ---------------------------------------------------------------------------
 * Playback: host-timer paced. Each period, copy from the ring into the
 * coherent input region. Actual inference is triggered by userspace ioctls.
 * ------------------------------------------------------------------------- */
static enum hrtimer_restart apex_alsa_playback_timer(struct hrtimer *t)
{
	struct apex_alsa *aa =
		container_of(t, struct apex_alsa, playback_timer);
	struct apex_alsa_stream *s = &aa->playback;
	struct snd_pcm_substream *ss;
	struct gasket_coherent_buffer *coh;
	unsigned char *ring;
	size_t copy_len, first;
	unsigned long flags;
	bool elapsed = false;

	spin_lock_irqsave(&aa->lock, flags);
	ss = s->substream;
	if (!s->running || !ss || !ss->runtime) {
		spin_unlock_irqrestore(&aa->lock, flags);
		return HRTIMER_NORESTART;
	}

	coh = &aa->gasket_dev->coherent_buffer;
	ring = ss->runtime->dma_area;
	copy_len = s->period_bytes;
	if (coh->length_bytes && copy_len > coh->length_bytes)
		copy_len = coh->length_bytes;

	if (ring && coh->virt_base && copy_len) {
		first = min_t(size_t, copy_len, s->buffer_bytes - s->pos);
		memcpy_toio(coh->virt_base, ring + s->pos, first);
		if (copy_len > first)
			memcpy_toio(coh->virt_base + first, ring,
				    copy_len - first);
		s->pos += copy_len;
		if (s->pos >= s->buffer_bytes)
			s->pos -= s->buffer_bytes;
		elapsed = true;
	}

	hrtimer_forward_now(t, aa->playback_period_time);
	spin_unlock_irqrestore(&aa->lock, flags);

	if (elapsed)
		snd_pcm_period_elapsed(ss);

	return HRTIMER_RESTART;
}

/* ---------------------------------------------------------------------------
 * PCM ops
 * ------------------------------------------------------------------------- */
static int apex_alsa_open(struct snd_pcm_substream *ss)
{
	ss->runtime->hw = apex_alsa_hw;
	return 0;
}

static int apex_alsa_close(struct snd_pcm_substream *ss)
{
	return 0;
}

static int apex_alsa_hw_params(struct snd_pcm_substream *ss,
			       struct snd_pcm_hw_params *params)
{
	return 0; /* managed buffer allocation handled by the ALSA core */
}

static int apex_alsa_hw_free(struct snd_pcm_substream *ss)
{
	return 0;
}

static int apex_alsa_prepare(struct snd_pcm_substream *ss)
{
	struct apex_alsa *aa = snd_pcm_substream_chip(ss);
	struct snd_pcm_runtime *rt = ss->runtime;
	struct apex_alsa_stream *s;
	unsigned long flags;

	s = (ss->stream == SNDRV_PCM_STREAM_PLAYBACK) ?
		&aa->playback : &aa->capture;

	spin_lock_irqsave(&aa->lock, flags);
	s->pos = 0;
	s->period_bytes = snd_pcm_lib_period_bytes(ss);
	s->buffer_bytes = snd_pcm_lib_buffer_bytes(ss);
	spin_unlock_irqrestore(&aa->lock, flags);

	if (ss->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		/* One period's worth of wall-clock time at the negotiated rate. */
		u64 ns = div_u64((u64)rt->period_size * NSEC_PER_SEC, rt->rate);

		aa->playback_period_time = ns_to_ktime(ns);
	}
	return 0;
}

static int apex_alsa_trigger(struct snd_pcm_substream *ss, int cmd)
{
	struct apex_alsa *aa = snd_pcm_substream_chip(ss);
	struct apex_alsa_stream *s;
	unsigned long flags;
	bool playback = (ss->stream == SNDRV_PCM_STREAM_PLAYBACK);

	s = playback ? &aa->playback : &aa->capture;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		spin_lock_irqsave(&aa->lock, flags);
		s->substream = ss;
		s->running = true;
		spin_unlock_irqrestore(&aa->lock, flags);
		if (playback)
			hrtimer_start(&aa->playback_timer,
				      aa->playback_period_time,
				      HRTIMER_MODE_REL);
		return 0;
	case SNDRV_PCM_TRIGGER_STOP:
		spin_lock_irqsave(&aa->lock, flags);
		s->running = false;
		spin_unlock_irqrestore(&aa->lock, flags);
		if (playback)
			hrtimer_cancel(&aa->playback_timer);
		return 0;
	default:
		return -EINVAL;
	}
}

static snd_pcm_uframes_t apex_alsa_pointer(struct snd_pcm_substream *ss)
{
	struct apex_alsa *aa = snd_pcm_substream_chip(ss);
	struct apex_alsa_stream *s;
	unsigned int pos;
	unsigned long flags;

	s = (ss->stream == SNDRV_PCM_STREAM_PLAYBACK) ?
		&aa->playback : &aa->capture;

	spin_lock_irqsave(&aa->lock, flags);
	pos = s->pos;
	spin_unlock_irqrestore(&aa->lock, flags);

	return bytes_to_frames(ss->runtime, pos);
}

static const struct snd_pcm_ops apex_alsa_ops = {
	.open = apex_alsa_open,
	.close = apex_alsa_close,
	.hw_params = apex_alsa_hw_params,
	.hw_free = apex_alsa_hw_free,
	.prepare = apex_alsa_prepare,
	.trigger = apex_alsa_trigger,
	.pointer = apex_alsa_pointer,
};

/* ---------------------------------------------------------------------------
 * Card creation / teardown
 * ------------------------------------------------------------------------- */
int apex_alsa_init(struct gasket_dev *gasket_dev)
{
	struct apex_alsa *aa;
	struct snd_card *card;
	struct snd_pcm *pcm;
	int ret;

	ret = snd_card_new(gasket_dev->dev, SNDRV_DEFAULT_IDX1, "apex",
			   THIS_MODULE, sizeof(*aa), &card);
	if (ret < 0) {
		dev_err(gasket_dev->dev,
			"apex-alsa: snd_card_new failed: %d\n", ret);
		return ret;
	}

	aa = card->private_data;
	aa->gasket_dev = gasket_dev;
	aa->card = card;
	spin_lock_init(&aa->lock);
	/*
	 * hrtimer_init()+assigning .function was replaced by hrtimer_setup()
	 * in kernel 6.15. Use whichever the running kernel provides.
	 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 15, 0)
	hrtimer_setup(&aa->playback_timer, apex_alsa_playback_timer,
		      CLOCK_MONOTONIC, HRTIMER_MODE_REL);
#else
	hrtimer_init(&aa->playback_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	aa->playback_timer.function = apex_alsa_playback_timer;
#endif

	strscpy(card->driver, "apex", sizeof(card->driver));
	strscpy(card->shortname, "Apex Edge TPU", sizeof(card->shortname));
	snprintf(card->longname, sizeof(card->longname),
		 "Apex Edge TPU audio (%s)", dev_name(gasket_dev->dev));

	/* One PCM, one playback + one capture substream. */
	ret = snd_pcm_new(card, "apex-pcm", 0, 1, 1, &pcm);
	if (ret < 0) {
		dev_err(gasket_dev->dev,
			"apex-alsa: snd_pcm_new failed: %d\n", ret);
		goto err_free;
	}
	pcm->private_data = aa;
	strscpy(pcm->name, "Apex PCM", sizeof(pcm->name));
	aa->pcm = pcm;

	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, &apex_alsa_ops);
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, &apex_alsa_ops);

	/* Vmalloc DMA ring buffers, managed and mmappable by the ALSA core. */
	snd_pcm_set_managed_buffer_all(pcm, SNDRV_DMA_TYPE_VMALLOC, NULL,
				       0, APEX_ALSA_BUFFER_BYTES);

	/* Register the completion callback (coexists with eventfd). */
	ret = gasket_interrupt_register_callback(gasket_dev,
						 APEX_ALSA_DONE_INTERRUPT,
						 apex_alsa_done_cb, aa);
	if (ret) {
		dev_warn(gasket_dev->dev,
			 "apex-alsa: IRQ callback registration failed: %d\n",
			 ret);
		/* Non-fatal: capture side just won't advance. */
	} else {
		aa->cb_registered = true;
	}

	ret = snd_card_register(card);
	if (ret < 0) {
		dev_err(gasket_dev->dev,
			"apex-alsa: snd_card_register failed: %d\n", ret);
		goto err_unreg_cb;
	}

	gasket_dev->alsa_priv = aa;
	dev_info(gasket_dev->dev, "apex-alsa: registered sound card '%s'\n",
		 card->shortname);
	return 0;

err_unreg_cb:
	if (aa->cb_registered)
		gasket_interrupt_register_callback(gasket_dev,
						   APEX_ALSA_DONE_INTERRUPT,
						   NULL, NULL);
err_free:
	snd_card_free(card);
	return ret;
}

void apex_alsa_cleanup(struct gasket_dev *gasket_dev)
{
	struct apex_alsa *aa = gasket_dev->alsa_priv;

	if (!aa)
		return;

	if (aa->cb_registered) {
		gasket_interrupt_register_callback(gasket_dev,
						   APEX_ALSA_DONE_INTERRUPT,
						   NULL, NULL);
		aa->cb_registered = false;
	}
	hrtimer_cancel(&aa->playback_timer);

	/* snd_card_free frees card->private_data (aa) as well. */
	snd_card_free(aa->card);
	gasket_dev->alsa_priv = NULL;
}
