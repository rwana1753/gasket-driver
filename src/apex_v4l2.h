/* SPDX-License-Identifier: GPL-2.0 */
/*
 * V4L2 front-end for the Apex/Gasket driver.
 *
 * This layer exposes a /dev/videoN node so that userspace which requires a
 * V4L2 device (VIDIOC_QUERYCAP / S_FMT / REQBUFS / QBUF / DQBUF / STREAMON)
 * can drive the Apex accelerator's output path.
 *
 * Design notes:
 *  - The frame geometry (size/format) is negotiated by userspace through
 *    S_FMT rather than hardcoded. The single supported "pixel format" is an
 *    opaque byte buffer (V4L2_PIX_FMT_GREY is used as a generic 8-bit
 *    container) whose sizeimage is chosen by userspace within limits.
 *  - Streaming buffers are vb2 buffers backed by dma-contig memory, entirely
 *    separate from the Gasket coherent buffer.
 *  - On the Apex OUTPUT_ACTV_QUEUE interrupt (inference output ready), the
 *    contents of the Gasket coherent buffer are *copied* into the head vb2
 *    buffer and it is completed. The pre-existing eventfd delivery for the
 *    same interrupt is left untouched (coexistence).
 *  - All original Gasket/Apex ioctls remain reachable through .vidioc_default.
 */

#ifndef __APEX_V4L2_H__
#define __APEX_V4L2_H__

struct gasket_dev;

/*
 * Initialise and register the V4L2 device for @gasket_dev.
 * Safe to call once per device after gasket_enable_device().
 * Returns 0 on success or a negative errno. On failure the caller may
 * proceed without a video node (non-fatal to the rest of the driver).
 */
int apex_v4l2_init(struct gasket_dev *gasket_dev);

/*
 * Tear down the V4L2 device previously created by apex_v4l2_init().
 * Safe to call even if init failed or was never called.
 */
void apex_v4l2_cleanup(struct gasket_dev *gasket_dev);

#endif /* __APEX_V4L2_H__ */
