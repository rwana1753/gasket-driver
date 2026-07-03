// SPDX-License-Identifier: GPL-2.0
/*
 * Framebuffer (fbdev) front-end for the Apex/Gasket driver.
 * See apex_fbdev.h for the rationale and the "no scanout" disclaimer.
 *
 * The framebuffer memory is a vmalloc'd linear buffer exposed to userspace via
 * fb_mmap (vm_map). It is synchronised with the Apex coherent buffer by copy:
 *  - result (coherent -> fb) on the OUTPUT_ACTV_QUEUE interrupt callback;
 *  - input  (fb -> coherent) on an explicit FBIO sync / pan ioctl.
 * The coherent mmap path used by the native char device is never touched.
 */

#include "apex_fbdev.h"

#include "gasket_core.h"
#include "gasket_interrupt.h"

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/io.h>
#include <linux/fb.h>
#include <linux/vmalloc.h>
#include <linux/mm.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/uaccess.h>
#include <linux/version.h>

/* Must match APEX_INTERRUPT_OUTPUT_ACTV_QUEUE in apex_driver.c. */
#define APEX_FBDEV_DONE_INTERRUPT 3

/* Default geometry; userspace may change via FBIOPUT_VSCREENINFO. */
#define APEX_FB_DEF_W    512u
#define APEX_FB_DEF_H    512u
#define APEX_FB_MAX_W    8192u
#define APEX_FB_MAX_H    8192u
#define APEX_FB_MAX_BYTES (64u * 1024u * 1024u)

/* Repurpose the standard vsync-wait ioctl as "wait for next result". */
#ifndef FBIO_WAITFORVSYNC
#define FBIO_WAITFORVSYNC _IOW('F', 0x20, __u32)
#endif

struct apex_fbdev {
	struct gasket_dev *gasket_dev;
	struct fb_info *info;

	spinlock_t lock;

	/* Linear framebuffer memory (vmalloc). */
	void *mem;
	size_t mem_size;

	/* Completion sequencing for the repurposed vsync-wait. */
	wait_queue_head_t wait;
	u32 result_seq;

	bool cb_registered;
};

/* ---------------------------------------------------------------------------
 * Result path: coherent -> fb, on the completion interrupt (hard IRQ).
 * ------------------------------------------------------------------------- */
static void apex_fbdev_done_cb(void *cb_data, int interrupt_index)
{
	struct apex_fbdev *af = cb_data;
	struct gasket_coherent_buffer *coh;
	unsigned long flags;
	size_t copy_len;

	if (interrupt_index != APEX_FBDEV_DONE_INTERRUPT)
		return;

	spin_lock_irqsave(&af->lock, flags);
	coh = &af->gasket_dev->coherent_buffer;
	copy_len = af->mem_size;
	if (coh->length_bytes && copy_len > coh->length_bytes)
		copy_len = coh->length_bytes;
	if (af->mem && coh->virt_base && copy_len)
		memcpy_fromio(af->mem, coh->virt_base, copy_len);
	af->result_seq++;
	spin_unlock_irqrestore(&af->lock, flags);

	wake_up_interruptible(&af->wait);
}

/* ---------------------------------------------------------------------------
 * Input path: fb -> coherent. Triggered explicitly by userspace.
 * ------------------------------------------------------------------------- */
static void apex_fbdev_push_input(struct apex_fbdev *af)
{
	struct gasket_coherent_buffer *coh;
	unsigned long flags;
	size_t copy_len;

	spin_lock_irqsave(&af->lock, flags);
	coh = &af->gasket_dev->coherent_buffer;
	copy_len = af->mem_size;
	if (coh->length_bytes && copy_len > coh->length_bytes)
		copy_len = coh->length_bytes;
	if (af->mem && coh->virt_base && copy_len)
		memcpy_toio(coh->virt_base, af->mem, copy_len);
	spin_unlock_irqrestore(&af->lock, flags);
}

/* ---------------------------------------------------------------------------
 * fb_ops
 * ------------------------------------------------------------------------- */
static int apex_fb_check_var(struct fb_var_screeninfo *var,
			     struct fb_info *info)
{
	u32 w = var->xres, h = var->yres, bpp = var->bits_per_pixel;

	if (bpp != 24 && bpp != 32)
		bpp = 32;
	if (w == 0 || w > APEX_FB_MAX_W)
		w = APEX_FB_DEF_W;
	if (h == 0 || h > APEX_FB_MAX_H)
		h = APEX_FB_DEF_H;
	if ((u64)w * h * (bpp / 8) > APEX_FB_MAX_BYTES)
		return -EINVAL;

	var->xres = w;
	var->yres = h;
	var->xres_virtual = w;
	var->yres_virtual = h;
	var->bits_per_pixel = bpp;
	var->grayscale = 0;

	/* Little-endian RGB(A) channel layout. */
	var->red.offset = 0;
	var->green.offset = 8;
	var->blue.offset = 16;
	var->red.length = var->green.length = var->blue.length = 8;
	if (bpp == 32) {
		var->transp.offset = 24;
		var->transp.length = 8;
	} else {
		var->transp.offset = 0;
		var->transp.length = 0;
	}
	return 0;
}

static int apex_fb_set_par(struct fb_info *info)
{
	struct fb_var_screeninfo *var = &info->var;

	info->fix.line_length = var->xres * (var->bits_per_pixel / 8);
	info->fix.visual = FB_VISUAL_TRUECOLOR;
	return 0;
}

static int apex_fb_setcolreg(unsigned int regno, unsigned int red,
			     unsigned int green, unsigned int blue,
			     unsigned int transp, struct fb_info *info)
{
	if (regno >= 16)
		return -EINVAL;
	if (info->var.bits_per_pixel == 32 || info->var.bits_per_pixel == 24) {
		if (regno < 16 && info->pseudo_palette) {
			u32 *pal = info->pseudo_palette;

			pal[regno] = (red & 0xff00) >> 8 |
				     (green & 0xff00) |
				     (blue & 0xff00) << 8;
		}
	}
	return 0;
}

/* No real display: blanking is a no-op that reports success. */
static int apex_fb_blank(int blank, struct fb_info *info)
{
	return 0;
}

static int apex_fb_mmap(struct fb_info *info, struct vm_area_struct *vma)
{
	struct apex_fbdev *af = info->par;
	unsigned long start = vma->vm_start;
	unsigned long size = vma->vm_end - vma->vm_start;
	unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;
	char *pos = af->mem;

	if (offset + size > af->mem_size)
		return -EINVAL;
	pos += offset;

	while (size > 0) {
		unsigned long pfn = vmalloc_to_pfn(pos);

		if (remap_pfn_range(vma, start, pfn, PAGE_SIZE, vma->vm_page_prot))
			return -EAGAIN;
		start += PAGE_SIZE;
		pos += PAGE_SIZE;
		size -= min(size, (unsigned long)PAGE_SIZE);
	}
	return 0;
}

static int apex_fb_ioctl(struct fb_info *info, unsigned int cmd,
			 unsigned long arg)
{
	struct apex_fbdev *af = info->par;
	u32 cur;

	switch (cmd) {
	case FBIO_WAITFORVSYNC:
		/*
		 * No real vsync. Repurposed: also push the current fb contents
		 * to the coherent input region (userspace has finished writing
		 * its input image), then block until the next result arrives.
		 */
		apex_fbdev_push_input(af);
		cur = READ_ONCE(af->result_seq);
		return wait_event_interruptible(af->wait,
					READ_ONCE(af->result_seq) != cur);
	default:
		return -ENOTTY;
	}
}

static const struct fb_ops apex_fb_ops = {
	.owner = THIS_MODULE,
	.fb_check_var = apex_fb_check_var,
	.fb_set_par = apex_fb_set_par,
	.fb_setcolreg = apex_fb_setcolreg,
	.fb_blank = apex_fb_blank,
	.fb_mmap = apex_fb_mmap,
	.fb_ioctl = apex_fb_ioctl,
	/* Provide the CPU-draw helpers so tools like fbi/cat still work. */
	.fb_fillrect = cfb_fillrect,
	.fb_copyarea = cfb_copyarea,
	.fb_imageblit = cfb_imageblit,
};

/* ---------------------------------------------------------------------------
 * Registration / teardown
 * ------------------------------------------------------------------------- */
int apex_fbdev_init(struct gasket_dev *gasket_dev)
{
	struct apex_fbdev *af;
	struct fb_info *info;
	size_t size;
	int ret;

	info = framebuffer_alloc(sizeof(struct apex_fbdev), gasket_dev->dev);
	if (!info)
		return -ENOMEM;

	af = info->par;
	af->gasket_dev = gasket_dev;
	af->info = info;
	spin_lock_init(&af->lock);
	init_waitqueue_head(&af->wait);

	/* Default geometry -> buffer size. */
	size = (size_t)APEX_FB_DEF_W * APEX_FB_DEF_H * 4;
	size = PAGE_ALIGN(size);
	af->mem = vzalloc(size);
	if (!af->mem) {
		ret = -ENOMEM;
		goto err_release;
	}
	af->mem_size = size;

	info->fbops = &apex_fb_ops;
	/*
	 * Mark as a virtual (non-IO) framebuffer. FBINFO_VIRTFB tells the fb
	 * core the memory is regular kernel memory, not device IO.
	 */
#ifdef FBINFO_VIRTFB
	info->flags = FBINFO_VIRTFB;
#endif
	/*
	 * screen_buffer (void *) was introduced in 5.2; older kernels only
	 * have screen_base (char __iomem *). Set whichever exists.
	 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 2, 0)
	info->screen_buffer = af->mem;
#else
	info->screen_base = (char __iomem __force *)af->mem;
#endif
	info->screen_size = size;
	info->pseudo_palette = kzalloc(sizeof(u32) * 16, GFP_KERNEL);
	if (!info->pseudo_palette) {
		ret = -ENOMEM;
		goto err_vfree;
	}

	strscpy(info->fix.id, "apex-fb", sizeof(info->fix.id));
	info->fix.type = FB_TYPE_PACKED_PIXELS;
	info->fix.visual = FB_VISUAL_TRUECOLOR;
	info->fix.accel = FB_ACCEL_NONE;
	info->fix.smem_len = size;
	/* No physical scanout address; smem_start left 0 (virtual fb). */

	info->var.xres = APEX_FB_DEF_W;
	info->var.yres = APEX_FB_DEF_H;
	info->var.bits_per_pixel = 32;
	apex_fb_check_var(&info->var, info);
	apex_fb_set_par(info);

	ret = register_framebuffer(info);
	if (ret < 0) {
		dev_err(gasket_dev->dev,
			"apex-fbdev: register_framebuffer failed: %d\n", ret);
		goto err_palette;
	}

	ret = gasket_interrupt_register_callback(gasket_dev,
						 APEX_FBDEV_DONE_INTERRUPT,
						 apex_fbdev_done_cb, af);
	if (ret) {
		dev_warn(gasket_dev->dev,
			 "apex-fbdev: IRQ callback registration failed: %d\n",
			 ret);
		/* Non-fatal: result path just won't auto-update. */
	} else {
		af->cb_registered = true;
	}

	gasket_dev->fbdev_priv = af;
	dev_info(gasket_dev->dev, "apex-fbdev: registered /dev/fb%d (no scanout)\n",
		 info->node);
	return 0;

err_palette:
	kfree(info->pseudo_palette);
err_vfree:
	vfree(af->mem);
err_release:
	framebuffer_release(info);
	return ret;
}

void apex_fbdev_cleanup(struct gasket_dev *gasket_dev)
{
	struct apex_fbdev *af = gasket_dev->fbdev_priv;
	struct fb_info *info;

	if (!af)
		return;
	info = af->info;

	if (af->cb_registered) {
		gasket_interrupt_register_callback(gasket_dev,
						   APEX_FBDEV_DONE_INTERRUPT,
						   NULL, NULL);
		af->cb_registered = false;
	}

	unregister_framebuffer(info);
	kfree(info->pseudo_palette);
	vfree(af->mem);
	framebuffer_release(info);
	gasket_dev->fbdev_priv = NULL;
}
