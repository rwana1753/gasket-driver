/* SPDX-License-Identifier: GPL-2.0 */
/*
 * ALSA front-end for the Apex/Gasket driver.
 *
 * Exposes an ALSA sound card with one PCM device offering both directions,
 * used to move audio between userspace and the Apex accelerator for audio
 * neural networks (TTS / STT / audio classification):
 *
 *   - Playback (userspace writes, user -> kernel): input audio for STT and
 *     classification models. Incoming PCM is copied into the Apex coherent
 *     buffer and marked ready; the actual inference trigger and model binding
 *     remain the userspace runtime's responsibility via the legacy ioctls.
 *
 *   - Capture (userspace reads, kernel -> user): output audio produced by the
 *     accelerator (e.g. TTS synthesis, denoise/AEC output). Driven by the
 *     Apex OUTPUT_ACTV_QUEUE interrupt via the in-kernel callback registered
 *     with gasket_interrupt_register_callback() (coexists with eventfd).
 *
 * Format (rate / channels / sample format) is negotiated by userspace through
 * the normal ALSA hw_params path rather than hardcoded.
 */

#ifndef __APEX_ALSA_H__
#define __APEX_ALSA_H__

struct gasket_dev;

/*
 * Create and register the ALSA card for @gasket_dev. Non-fatal on failure;
 * the rest of the driver continues to work without a sound card.
 */
int apex_alsa_init(struct gasket_dev *gasket_dev);

/* Tear down the ALSA card. Safe if init failed or was never called. */
void apex_alsa_cleanup(struct gasket_dev *gasket_dev);

#endif /* __APEX_ALSA_H__ */
