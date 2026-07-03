// SPDX-License-Identifier: GPL-2.0
/*
 * Framebuffer (fbdev) front-end for the Apex/Gasket driver.
 * See apex_fbdev.h for the rationale and the "no scanout" disclaimer.
 *
 * Two independent nodes are registered, one per direction, so each node has a
 * single clear role instead of overloading one fb for both read and write:
 *
 *   INPUT  fb  -> APEX_FBIO_SYNC pushes fb -> coherent[in_offset]
 *   OUTPUT fb  <- OUTPUT_ACTV_QUEUE irq copies coherent[out_offset] -> fb,
 *                 APEX_FBIO_WAIT blocks until the next result
 *
 * Both nodes share the same coherent layout (in/out offsets+lengths), settable
 * from either node via APEX_FBIO_SET_LAYOUT. The coherent mmap path used by
 * the native char device is never touched.
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

/*
 * Userspace-settable coherent-buffer layout, shared by both nodes. The kernel
 * does not know where a model places its input/output activations; libedgetpu
 * supplies these at model-load time. Offsets/lengths are relative to the
 * coherent buffer.
 */
struct apex_fb_layout {
	__u32 in_offset;
	__u32 in_len;
	__u32 out_offset;
	__u32 out_len;
};

#define APEX_FBIO_SET_LAYOUT _IOW('F', 0x40, struct apex_fb_layout)
#define APEX_FBIO_GET_LAYOUT _IOR('F', 0x41, struct apex_fb_layout)
/* Input node: push current fb contents into the coherent input window. */
#define APEX_FBIO_SYNC       _IO('F', 0x42)
/* Output node: block until the next inference result has been copied in. */
#define APEX_FBIO_WAIT       _IO('F', 0x43)

enum apex_fb_dir {
	APEX_FB_INPUT = 0,
	APEX_FB_OUTPUT = 1,
};

/* Shared per-device state (layout + the two nodes). */
struct apex_fbdev {
	struct gasket_dev *gasket_dev;

	/* Guards layout and result_seq against the IRQ callback. */
	spinlock_t lock;

	/* Coherent windows (shared by both nodes). */
	u32 in_offset;
	u32 in_len;
	u32 out_offset;
	u32 out_len;

	/* Output completion sequencing for APEX_FBIO_WAIT. */
	wait_queue_head_t wait;
	u32 result_seq;

	bool cb_registered;

	struct apex_fb_node {
		struct apex_fbdev *parent;
		enum apex_fb_dir dir;
		struct fb_info *info;
		void *mem;
		size_t mem_size;
	} node[2];
};

/* ---------------------------------------------------------------------------
 * Result path: coherent[out_offset] -> OUTPUT fb, on completion interrupt.
 * Runs in hard-IRQ context; bounded copy only.
 * ------------------------------------------------------------------------- */
static void apex_fbdev_done_cb(void *cb_data, int interrupt_index)
{
	struct apex_fbdev *af = cb_data;
	struct apex_fb_node *out = &af->node[APEX_FB_OUTPUT];
	struct gasket_coherent_buffer *coh;
	unsigned long flags;
	size_t copy_len;

	if (interrupt_index != APEX_FBDEV_DONE_INTERRUPT)
		return;

	spin_lock_irqsave(&af->lock, flags);
	coh = &af->gasket_dev->coherent_buffer;

	copy_len = min_t(size_t, af->out_len, out->mem_size);
	if (coh->length_bytes && af->out_offset < coh->length_bytes) {
		copy_len = min_t(size_t, copy_len,
				 coh->length_bytes - af->out_offset);
		if (out->mem && coh->virt_base && copy_len)
			memcpy_fromio(out->mem,
				      coh->virt_base + af->out_offset, copy_len);
	}
	af->result_seq++;
	spin_unlock_irqrestore(&af->lock, flags);

	wake_up_interruptible(&af->wait);
}

/* ---------------------------------------------------------------------------
 * Input path: INPUT fb -> coherent[in_offset]. Explicit, via APEX_FBIO_SYNC.
 * ------------------------------------------------------------------------- */
static void apex_fbdev_push_input(struct apex_fbdev *af)
{
	struct apex_fb_node *in = &af->node[APEX_FB_INPUT];
	struct gasket_coherent_buffer *coh;
	unsigned long flags;
	size_t copy_len;

	spin_lock_irqsave(&af->lock, flags);
	coh = &af->gasket_dev->coherent_buffer;

	copy_len = min_t(size_t, af->in_len, in->mem_size);
	if (coh->length_bytes && af->in_offset < coh->length_bytes) {
		copy_len = min_t(size_t, copy_len,
				 coh->length_bytes - af->in_offset);
		if (in->mem && coh->virt_base && copy_len)
			memcpy_toio(coh->virt_base + af->in_offset,
				    in->mem, copy_len);
	}
	spin_unlock_irqrestore(&af->lock, flags);
}

/* ---------------------------------------------------------------------------
 * fb_ops (shared by both nodes; behaviour differs by node->dir)
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
	if (info->pseudo_palette) {
		u32 *pal = info->pseudo_palette;

		pal[regno] = (red & 0xff00) >> 8 |
			     (green & 0xff00) |
			     (blue & 0xff00) << 8;
	}
	return 0;
}

static int apex_fb_blank(int blank, struct fb_info *info)
{
	return 0; /* no real display */
}

static int apex_fb_mmap(struct fb_info *info, struct vm_area_struct *vma)
{
	struct apex_fb_node *node = info->par;
	unsigned long start = vma->vm_start;
	unsigned long size = vma->vm_end - vma->vm_start;
	unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;
	char *pos = node->mem;

	if (offset + size > node->mem_size)
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

static int apex_fb_set_layout(struct apex_fbdev *af, void __user *arg)
{
	struct gasket_coherent_buffer *coh = &af->gasket_dev->coherent_buffer;
	struct apex_fb_layout lay;
	unsigned long flags;

	if (copy_from_user(&lay, arg, sizeof(lay)))
		return -EFAULT;
	if (coh->length_bytes) {
		if ((u64)lay.in_offset + lay.in_len > coh->length_bytes)
			return -EINVAL;
		if ((u64)lay.out_offset + lay.out_len > coh->length_bytes)
			return -EINVAL;
	}
	spin_lock_irqsave(&af->lock, flags);
	af->in_offset = lay.in_offset;
	af->in_len = lay.in_len;
	af->out_offset = lay.out_offset;
	af->out_len = lay.out_len;
	spin_unlock_irqrestore(&af->lock, flags);
	return 0;
}

static int apex_fb_get_layout(struct apex_fbdev *af, void __user *arg)
{
	struct apex_fb_layout lay;
	unsigned long flags;

	spin_lock_irqsave(&af->lock, flags);
	lay.in_offset = af->in_offset;
	lay.in_len = af->in_len;
	lay.out_offset = af->out_offset;
	lay.out_len = af->out_len;
	spin_unlock_irqrestore(&af->lock, flags);
	if (copy_to_user(arg, &lay, sizeof(lay)))
		return -EFAULT;
	return 0;
}

static int apex_fb_ioctl(struct fb_info *info, unsigned int cmd,
			 unsigned long arg)
{
	struct apex_fb_node *node = info->par;
	struct apex_fbdev *af = node->parent;
	u32 cur;

	switch (cmd) {
	/* Layout can be queried/set from either node. */
	case APEX_FBIO_SET_LAYOUT:
		return apex_fb_set_layout(af, (void __user *)arg);
	case APEX_FBIO_GET_LAYOUT:
		return apex_fb_get_layout(af, (void __user *)arg);

	/* Input-only: push the written image into the coherent input window. */
	case APEX_FBIO_SYNC:
		if (node->dir != APEX_FB_INPUT)
			return -ENOTTY;
		apex_fbdev_push_input(af);
		return 0;

	/* Output-only: block until the next result has been copied in. */
	case APEX_FBIO_WAIT:
		if (node->dir != APEX_FB_OUTPUT)
			return -ENOTTY;
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
	.fb_fillrect = cfb_fillrect,
	.fb_copyarea = cfb_copyarea,
	.fb_imageblit = cfb_imageblit,
};

/* ---------------------------------------------------------------------------
 * Per-node registration helpers
 * ------------------------------------------------------------------------- */
static int apex_fb_register_node(struct apex_fbdev *af, enum apex_fb_dir dir)
{
	struct gasket_dev *gasket_dev = af->gasket_dev;
	struct apex_fb_node *node = &af->node[dir];
	struct fb_info *info;
	size_t size;
	int ret;

	/*
	 * fb_info carries a back-pointer (par) to the node, not the whole
	 * device, so use a plain framebuffer_alloc and wire par manually.
	 */
	info = framebuffer_alloc(0, gasket_dev->dev);
	if (!info)
		return -ENOMEM;

	node->parent = af;
	node->dir = dir;
	node->info = info;
	info->par = node;

	size = PAGE_ALIGN((size_t)APEX_FB_DEF_W * APEX_FB_DEF_H * 4);
	node->mem = vzalloc(size);
	if (!node->mem) {
		ret = -ENOMEM;
		goto err_release;
	}
	node->mem_size = size;

	info->fbops = &apex_fb_ops;
#ifdef FBINFO_VIRTFB
	info->flags = FBINFO_VIRTFB;
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 2, 0)
	info->screen_buffer = node->mem;
#else
	info->screen_base = (char __iomem __force *)node->mem;
#endif
	info->screen_size = size;
	info->pseudo_palette = kzalloc(sizeof(u32) * 16, GFP_KERNEL);
	if (!info->pseudo_palette) {
		ret = -ENOMEM;
		goto err_vfree;
	}

	snprintf(info->fix.id, sizeof(info->fix.id), "apex-fb-%s",
		 dir == APEX_FB_INPUT ? "in" : "out");
	info->fix.type = FB_TYPE_PACKED_PIXELS;
	info->fix.visual = FB_VISUAL_TRUECOLOR;
	info->fix.accel = FB_ACCEL_NONE;
	info->fix.smem_len = size;

	info->var.xres = APEX_FB_DEF_W;
	info->var.yres = APEX_FB_DEF_H;
	info->var.bits_per_pixel = 32;
	apex_fb_check_var(&info->var, info);
	apex_fb_set_par(info);

	ret = register_framebuffer(info);
	if (ret < 0) {
		dev_err(gasket_dev->dev,
			"apex-fbdev: register_framebuffer(%s) failed: %d\n",
			info->fix.id, ret);
		goto err_palette;
	}

	dev_info(gasket_dev->dev, "apex-fbdev: registered /dev/fb%d (%s, no scanout)\n",
		 info->node, info->fix.id);
	return 0;

err_palette:
	kfree(info->pseudo_palette);
err_vfree:
	vfree(node->mem);
	node->mem = NULL;
err_release:
	framebuffer_release(info);
	node->info = NULL;
	return ret;
}

static void apex_fb_unregister_node(struct apex_fb_node *node)
{
	struct fb_info *info = node->info;

	if (!info)
		return;
	unregister_framebuffer(info);
	kfree(info->pseudo_palette);
	vfree(node->mem);
	node->mem = NULL;
	framebuffer_release(info);
	node->info = NULL;
}

/* ---------------------------------------------------------------------------
 * Public init / cleanup
 * ------------------------------------------------------------------------- */
int apex_fbdev_init(struct gasket_dev *gasket_dev)
{
	struct apex_fbdev *af;
	u32 max_coh, half;
	int ret;

	af = kzalloc(sizeof(*af), GFP_KERNEL);
	if (!af)
		return -ENOMEM;

	af->gasket_dev = gasket_dev;
	spin_lock_init(&af->lock);
	init_waitqueue_head(&af->wait);

	/*
	 * Default coherent layout: non-overlapping lower(input)/upper(output)
	 * split. The coherent buffer is allocated lazily, so base the default
	 * on the maximum coherent size; every copy is independently bounds-
	 * checked against the live length_bytes. Userspace should set the real
	 * per-model layout via APEX_FBIO_SET_LAYOUT.
	 */
	max_coh = (u32)(MAX_NUM_COHERENT_PAGES * PAGE_SIZE);
	half = max_coh / 2;
	af->in_offset = 0;
	af->in_len = half;
	af->out_offset = half;
	af->out_len = max_coh - half;

	ret = apex_fb_register_node(af, APEX_FB_INPUT);
	if (ret)
		goto err_free;

	ret = apex_fb_register_node(af, APEX_FB_OUTPUT);
	if (ret)
		goto err_unreg_in;

	ret = gasket_interrupt_register_callback(gasket_dev,
						 APEX_FBDEV_DONE_INTERRUPT,
						 apex_fbdev_done_cb, af);
	if (ret) {
		dev_warn(gasket_dev->dev,
			 "apex-fbdev: IRQ callback registration failed: %d\n",
			 ret);
		/* Non-fatal: output node just won't auto-update. */
	} else {
		af->cb_registered = true;
	}

	gasket_dev->fbdev_priv = af;
	return 0;

err_unreg_in:
	apex_fb_unregister_node(&af->node[APEX_FB_INPUT]);
err_free:
	kfree(af);
	return ret;
}

void apex_fbdev_cleanup(struct gasket_dev *gasket_dev)
{
	struct apex_fbdev *af = gasket_dev->fbdev_priv;

	if (!af)
		return;

	if (af->cb_registered) {
		gasket_interrupt_register_callback(gasket_dev,
						   APEX_FBDEV_DONE_INTERRUPT,
						   NULL, NULL);
		af->cb_registered = false;
	}

	apex_fb_unregister_node(&af->node[APEX_FB_OUTPUT]);
	apex_fb_unregister_node(&af->node[APEX_FB_INPUT]);
	kfree(af);
	gasket_dev->fbdev_priv = NULL;
}
