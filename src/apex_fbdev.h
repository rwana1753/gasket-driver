/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Framebuffer (fbdev) front-end for the Apex/Gasket driver.
 *
 * This does NOT drive a display. Apex has no scanout hardware; this node
 * exists only to satisfy userspace that requires a /dev/fbN interface, and it
 * is used as a bidirectional mmap'able pixel buffer for image neural networks
 * (e.g. text-to-image):
 *
 *   - Userspace WRITES an input image into the framebuffer, then triggers
 *     inference via the legacy Gasket ioctls; the fb contents are copied into
 *     the Apex coherent input region at trigger time (or on FBIO sync).
 *   - Userspace READS the result image back from the framebuffer; on the
 *     OUTPUT_ACTV_QUEUE interrupt the coherent result is copied into the fb
 *     memory (in-kernel callback, coexists with eventfd).
 *
 * Geometry/format is reported and settable through the normal fb_var/fix
 * screeninfo path. There is no true vsync; FBIO_WAITFORVSYNC is repurposed to
 * block until the next inference-completion interrupt.
 */

#ifndef __APEX_FBDEV_H__
#define __APEX_FBDEV_H__

struct gasket_dev;

/* Create and register /dev/fbN. Non-fatal on failure. */
int apex_fbdev_init(struct gasket_dev *gasket_dev);

/* Tear down the framebuffer. Safe if init failed or was never called. */
void apex_fbdev_cleanup(struct gasket_dev *gasket_dev);

#endif /* __APEX_FBDEV_H__ */
