// SPDX-License-Identifier: GPL-2.0
/*
 * V4L2 front-end for the Apex/Gasket driver. See apex_v4l2.h for the design
 * rationale. This file is intentionally self-contained and only touches the
 * rest of the driver through the public gasket_* / interrupt-callback APIs,
 * so it can be dropped in without reworking the core.
 */

#include "apex_v4l2.h"

#include "gasket_core.h"
#include "gasket_interrupt.h"

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/list.h>
#include <linux/version.h>
#include <linux/io.h>

#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-common.h>
#include <media/v4l2-event.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-dma-contig.h>

/*
 * The Apex "inference output ready" interrupt index. Kept as a local constant
 * to avoid a build dependency on apex_driver.c internals; must match
 * APEX_INTERRUPT_OUTPUT_ACTV_QUEUE in apex_driver.c.
 */
#define APEX_V4L2_DONE_INTERRUPT 3

/* Format negotiation bounds. Width carries the byte length; height is 1. */
#define APEX_V4L2_MIN_SIZE 1u
#define APEX_V4L2_MAX_SIZE (16u * 1024u * 1024u) /* 16 MiB ceiling */
#define APEX_V4L2_DEF_SIZE 4096u

struct apex_v4l2_buffer {
	struct vb2_v4l2_buffer vb;
	struct list_head list;
};

struct apex_v4l2 {
	struct gasket_dev *gasket_dev;

	struct v4l2_device v4l2_dev;
	struct video_device vdev;
	struct vb2_queue queue;

	/* Serialises ioctl / queue operations (used as vdev + queue lock). */
	struct mutex lock;

	/* Protects buf_list and streaming against the IRQ callback. */
	spinlock_t irq_lock;
	struct list_head buf_list;

	/* Negotiated frame length in bytes (sizeimage). */
	u32 sizeimage;

	bool streaming;
	bool cb_registered;

	u32 sequence;
};

static inline struct apex_v4l2 *to_apex_v4l2(struct v4l2_device *v4l2_dev)
{
	return container_of(v4l2_dev, struct apex_v4l2, v4l2_dev);
}

static inline struct apex_v4l2_buffer *to_apex_buf(struct vb2_v4l2_buffer *vbuf)
{
	return container_of(vbuf, struct apex_v4l2_buffer, vb);
}

/* ---------------------------------------------------------------------------
 * Interrupt callback: inference output ready.
 *
 * Runs in hard-IRQ context (see gasket_interrupt_register_callback). Copies the
 * coherent buffer into the head vb2 buffer and completes it. Kept minimal.
 * ------------------------------------------------------------------------- */
static void apex_v4l2_done_cb(void *cb_data, int interrupt_index)
{
	struct apex_v4l2 *av = cb_data;
	struct apex_v4l2_buffer *buf;
	struct gasket_coherent_buffer *coh;
	unsigned long flags;
	void *dst;
	size_t copy_len;

	if (interrupt_index != APEX_V4L2_DONE_INTERRUPT)
		return;

	spin_lock_irqsave(&av->irq_lock, flags);
	if (!av->streaming || list_empty(&av->buf_list)) {
		spin_unlock_irqrestore(&av->irq_lock, flags);
		return;
	}
	buf = list_first_entry(&av->buf_list, struct apex_v4l2_buffer, list);
	list_del(&buf->list);
	spin_unlock_irqrestore(&av->irq_lock, flags);

	coh = &av->gasket_dev->coherent_buffer;
	dst = vb2_plane_vaddr(&buf->vb.vb2_buf, 0);
	copy_len = min_t(size_t, av->sizeimage,
			 (size_t)vb2_plane_size(&buf->vb.vb2_buf, 0));
	if (coh->length_bytes && copy_len > coh->length_bytes)
		copy_len = coh->length_bytes;

	if (dst && coh->virt_base && copy_len) {
		/*
		 * coherent_buffer.virt_base is annotated __iomem for the BAR
		 * case, but Apex coherent memory is host RAM from
		 * dma_alloc_coherent; a plain copy is correct here.
		 */
		memcpy_fromio(dst, coh->virt_base, copy_len);
	}

	vb2_set_plane_payload(&buf->vb.vb2_buf, 0, copy_len);
	buf->vb.vb2_buf.timestamp = ktime_get_ns();
	buf->vb.sequence = av->sequence++;
	buf->vb.field = V4L2_FIELD_NONE;
	vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_DONE);
}

/* ---------------------------------------------------------------------------
 * vb2 queue operations
 * ------------------------------------------------------------------------- */
static int apex_queue_setup(struct vb2_queue *vq, unsigned int *nbuffers,
			    unsigned int *nplanes, unsigned int sizes[],
			    struct device *alloc_devs[])
{
	struct apex_v4l2 *av = vb2_get_drv_priv(vq);

	if (*nplanes) {
		if (*nplanes != 1 || sizes[0] < av->sizeimage)
			return -EINVAL;
		return 0;
	}

	if (*nbuffers < 2)
		*nbuffers = 2;

	*nplanes = 1;
	sizes[0] = av->sizeimage;
	return 0;
}

static int apex_buf_prepare(struct vb2_buffer *vb)
{
	struct apex_v4l2 *av = vb2_get_drv_priv(vb->vb2_queue);

	if (vb2_plane_size(vb, 0) < av->sizeimage)
		return -EINVAL;

	vb2_set_plane_payload(vb, 0, av->sizeimage);
	return 0;
}

static void apex_buf_queue(struct vb2_buffer *vb)
{
	struct apex_v4l2 *av = vb2_get_drv_priv(vb->vb2_queue);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct apex_v4l2_buffer *buf = to_apex_buf(vbuf);
	unsigned long flags;

	spin_lock_irqsave(&av->irq_lock, flags);
	list_add_tail(&buf->list, &av->buf_list);
	spin_unlock_irqrestore(&av->irq_lock, flags);
}

static void apex_return_all_buffers(struct apex_v4l2 *av,
				    enum vb2_buffer_state state)
{
	struct apex_v4l2_buffer *buf, *tmp;
	unsigned long flags;
	LIST_HEAD(done);

	spin_lock_irqsave(&av->irq_lock, flags);
	list_splice_init(&av->buf_list, &done);
	spin_unlock_irqrestore(&av->irq_lock, flags);

	list_for_each_entry_safe(buf, tmp, &done, list) {
		list_del(&buf->list);
		vb2_buffer_done(&buf->vb.vb2_buf, state);
	}
}

static int apex_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct apex_v4l2 *av = vb2_get_drv_priv(vq);
	unsigned long flags;
	int ret;

	av->sequence = 0;

	/* Register the in-kernel completion callback (coexists with eventfd). */
	ret = gasket_interrupt_register_callback(av->gasket_dev,
						 APEX_V4L2_DONE_INTERRUPT,
						 apex_v4l2_done_cb, av);
	if (ret) {
		dev_err(av->gasket_dev->dev,
			"apex-v4l2: failed to register IRQ callback: %d\n", ret);
		apex_return_all_buffers(av, VB2_BUF_STATE_QUEUED);
		return ret;
	}
	av->cb_registered = true;

	spin_lock_irqsave(&av->irq_lock, flags);
	av->streaming = true;
	spin_unlock_irqrestore(&av->irq_lock, flags);

	return 0;
}

static void apex_stop_streaming(struct vb2_queue *vq)
{
	struct apex_v4l2 *av = vb2_get_drv_priv(vq);
	unsigned long flags;

	spin_lock_irqsave(&av->irq_lock, flags);
	av->streaming = false;
	spin_unlock_irqrestore(&av->irq_lock, flags);

	if (av->cb_registered) {
		gasket_interrupt_register_callback(av->gasket_dev,
						   APEX_V4L2_DONE_INTERRUPT,
						   NULL, NULL);
		av->cb_registered = false;
	}

	apex_return_all_buffers(av, VB2_BUF_STATE_ERROR);
}

static const struct vb2_ops apex_vb2_ops = {
	.queue_setup = apex_queue_setup,
	.buf_prepare = apex_buf_prepare,
	.buf_queue = apex_buf_queue,
	.start_streaming = apex_start_streaming,
	.stop_streaming = apex_stop_streaming,
	.wait_prepare = vb2_ops_wait_prepare,
	.wait_finish = vb2_ops_wait_finish,
};

/* ---------------------------------------------------------------------------
 * V4L2 ioctl operations
 * ------------------------------------------------------------------------- */
static int apex_querycap(struct file *file, void *priv,
			 struct v4l2_capability *cap)
{
	struct apex_v4l2 *av = video_drvdata(file);

	strscpy(cap->driver, "apex", sizeof(cap->driver));
	strscpy(cap->card, "Apex Edge TPU (V4L2)", sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s",
		 dev_name(av->gasket_dev->dev));
	return 0;
}

static void apex_fill_fmt(struct apex_v4l2 *av, struct v4l2_format *f)
{
	struct v4l2_pix_format *pix = &f->fmt.pix;

	memset(pix, 0, sizeof(*pix));
	pix->pixelformat = V4L2_PIX_FMT_GREY; /* generic 8-bit container */
	pix->width = av->sizeimage;
	pix->height = 1;
	pix->field = V4L2_FIELD_NONE;
	pix->bytesperline = av->sizeimage;
	pix->sizeimage = av->sizeimage;
	pix->colorspace = V4L2_COLORSPACE_RAW;
}

static int apex_enum_fmt(struct file *file, void *priv,
			 struct v4l2_fmtdesc *f)
{
	if (f->index != 0)
		return -EINVAL;
	f->pixelformat = V4L2_PIX_FMT_GREY;
	return 0;
}

static int apex_g_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct apex_v4l2 *av = video_drvdata(file);

	apex_fill_fmt(av, f);
	return 0;
}

static u32 apex_clamp_size(u32 size)
{
	if (size < APEX_V4L2_MIN_SIZE)
		size = APEX_V4L2_MIN_SIZE;
	if (size > APEX_V4L2_MAX_SIZE)
		size = APEX_V4L2_MAX_SIZE;
	return size;
}

static int apex_try_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct v4l2_pix_format *pix = &f->fmt.pix;
	u32 size;

	/*
	 * Userspace picks the frame byte length via width (or sizeimage if it
	 * sets that instead). Height is always 1; the buffer is opaque bytes.
	 */
	size = pix->sizeimage ? pix->sizeimage : pix->width;
	size = apex_clamp_size(size);

	memset(pix, 0, sizeof(*pix));
	pix->pixelformat = V4L2_PIX_FMT_GREY;
	pix->width = size;
	pix->height = 1;
	pix->field = V4L2_FIELD_NONE;
	pix->bytesperline = size;
	pix->sizeimage = size;
	pix->colorspace = V4L2_COLORSPACE_RAW;
	return 0;
}

static int apex_s_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct apex_v4l2 *av = video_drvdata(file);
	int ret;

	if (vb2_is_busy(&av->queue))
		return -EBUSY;

	ret = apex_try_fmt(file, priv, f);
	if (ret)
		return ret;

	av->sizeimage = f->fmt.pix.sizeimage;
	return 0;
}

/*
 * Route any ioctl not understood by the V4L2 core to the original
 * Gasket/Apex ioctl handler, preserving the full legacy interface.
 */
static long apex_default_ioctl(struct file *file, void *fh, bool valid_prio,
			       unsigned int cmd, void *arg)
{
	struct apex_v4l2 *av = video_drvdata(file);
	const struct gasket_driver_desc *desc;

	if (!av || !av->gasket_dev)
		return -ENOTTY;

	desc = av->gasket_dev->internal_desc ?
		gasket_get_driver_desc(av->gasket_dev) : NULL;
	if (!desc || !desc->ioctl_handler_cb)
		return -ENOTTY;

	/*
	 * arg points to a kernel copy for _IOWR commands the core recognises;
	 * for unknown commands the core passes the raw userspace pointer via
	 * the .unlocked variant. The Gasket handler expects a __user pointer,
	 * so forward the original argument as provided by the framework.
	 */
	return desc->ioctl_handler_cb(file, cmd, (void __user *)arg);
}

static const struct v4l2_ioctl_ops apex_ioctl_ops = {
	.vidioc_querycap = apex_querycap,
	.vidioc_enum_fmt_vid_cap = apex_enum_fmt,
	.vidioc_g_fmt_vid_cap = apex_g_fmt,
	.vidioc_s_fmt_vid_cap = apex_s_fmt,
	.vidioc_try_fmt_vid_cap = apex_try_fmt,

	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_prepare_buf = vb2_ioctl_prepare_buf,
	.vidioc_expbuf = vb2_ioctl_expbuf,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,

	.vidioc_subscribe_event = v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,

	.vidioc_default = apex_default_ioctl,
};

static const struct v4l2_file_operations apex_v4l2_fops = {
	.owner = THIS_MODULE,
	.open = v4l2_fh_open,
	.release = vb2_fop_release,
	.unlocked_ioctl = video_ioctl2,
	.mmap = vb2_fop_mmap,
	.poll = vb2_fop_poll,
};

/* ---------------------------------------------------------------------------
 * Registration / teardown
 * ------------------------------------------------------------------------- */
int apex_v4l2_init(struct gasket_dev *gasket_dev)
{
	struct apex_v4l2 *av;
	struct vb2_queue *q;
	int ret;

	av = kzalloc(sizeof(*av), GFP_KERNEL);
	if (!av)
		return -ENOMEM;

	av->gasket_dev = gasket_dev;
	av->sizeimage = APEX_V4L2_DEF_SIZE;
	mutex_init(&av->lock);
	spin_lock_init(&av->irq_lock);
	INIT_LIST_HEAD(&av->buf_list);

	ret = v4l2_device_register(gasket_dev->dev, &av->v4l2_dev);
	if (ret) {
		dev_err(gasket_dev->dev,
			"apex-v4l2: v4l2_device_register failed: %d\n", ret);
		goto err_free;
	}

	q = &av->queue;
	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_MMAP | VB2_DMABUF | VB2_USERPTR;
	q->drv_priv = av;
	q->buf_struct_size = sizeof(struct apex_v4l2_buffer);
	q->ops = &apex_vb2_ops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	/*
	 * The vb2 field that sets the minimum number of queued buffers before
	 * streaming was renamed from min_buffers_needed to min_queued_buffers
	 * in kernel 6.10. Support both.
	 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 10, 0)
	q->min_queued_buffers = 1;
#else
	q->min_buffers_needed = 1;
#endif
	q->lock = &av->lock;
	q->dev = gasket_dev->dev;

	ret = vb2_queue_init(q);
	if (ret) {
		dev_err(gasket_dev->dev,
			"apex-v4l2: vb2_queue_init failed: %d\n", ret);
		goto err_unreg_v4l2;
	}

	snprintf(av->vdev.name, sizeof(av->vdev.name), "apex-%s",
		 dev_name(gasket_dev->dev));
	av->vdev.fops = &apex_v4l2_fops;
	av->vdev.ioctl_ops = &apex_ioctl_ops;
	av->vdev.release = video_device_release_empty;
	av->vdev.v4l2_dev = &av->v4l2_dev;
	av->vdev.queue = q;
	av->vdev.lock = &av->lock;
	av->vdev.device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	video_set_drvdata(&av->vdev, av);

	ret = video_register_device(&av->vdev, VFL_TYPE_VIDEO, -1);
	if (ret) {
		dev_err(gasket_dev->dev,
			"apex-v4l2: video_register_device failed: %d\n", ret);
		goto err_unreg_v4l2;
	}

	/* Stash on the gasket device via drvdata of the v4l2_device. */
	dev_set_drvdata(&av->v4l2_dev.dev, av);
	gasket_dev->v4l2_priv = av;

	dev_info(gasket_dev->dev, "apex-v4l2: registered /dev/video%d\n",
		 av->vdev.num);
	return 0;

err_unreg_v4l2:
	v4l2_device_unregister(&av->v4l2_dev);
err_free:
	mutex_destroy(&av->lock);
	kfree(av);
	return ret;
}

void apex_v4l2_cleanup(struct gasket_dev *gasket_dev)
{
	struct apex_v4l2 *av = gasket_dev->v4l2_priv;

	if (!av)
		return;

	video_unregister_device(&av->vdev);
	v4l2_device_unregister(&av->v4l2_dev);
	mutex_destroy(&av->lock);
	gasket_dev->v4l2_priv = NULL;
	kfree(av);
}
