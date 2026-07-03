/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Framebuffer (fbdev) front-end for the Apex/Gasket driver.
 *
 * This does NOT drive a display. Apex has no scanout hardware; these nodes
 * exist only to satisfy userspace that requires /dev/fbN, used as mmap'able
 * pixel buffers for image neural networks (e.g. text-to-image).
 *
 * Two separate nodes are registered, one per direction:
 *
 *   - INPUT  fb : userspace WRITES the input image here, then issues
 *                 APEX_FBIO_SYNC to copy it into the coherent input window.
 *                 The actual inference is triggered by the userspace runtime
 *                 via the legacy Gasket ioctls.
 *
 *   - OUTPUT fb : on the OUTPUT_ACTV_QUEUE interrupt the coherent output
 *                 window is copied here (in-kernel callback, coexists with
 *                 eventfd); userspace READS the result via mmap, and may block
 *                 on APEX_FBIO_WAIT until the next result arrives.
 *
 * Coherent input/output offsets are a per-model userspace contract, set via
 * APEX_FBIO_SET_LAYOUT (the kernel does not interpret the buffer contents).
 */

#ifndef __APEX_FBDEV_H__
#define __APEX_FBDEV_H__

struct gasket_dev;

/* Create and register both /dev/fbN nodes (input + output). Non-fatal. */
int apex_fbdev_init(struct gasket_dev *gasket_dev);

/* Tear down both framebuffers. Safe if init failed or was never called. */
void apex_fbdev_cleanup(struct gasket_dev *gasket_dev);

#endif /* __APEX_FBDEV_H__ */
