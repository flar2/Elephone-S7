/*
 * Copyright (c) 2015 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <drm/drmP.h>
#include <drm/drm_gem.h>
#include <drm/mediatek_drm.h>

#include "mtk_drm_gem.h"

static struct mtk_drm_gem_obj *mtk_drm_gem_init(struct drm_device *dev,
						unsigned long size)
{
	struct mtk_drm_gem_obj *mtk_gem_obj;
	int ret;

	size = round_up(size, PAGE_SIZE);

	mtk_gem_obj = kzalloc(sizeof(*mtk_gem_obj), GFP_KERNEL);
	if (!mtk_gem_obj)
		return ERR_PTR(-ENOMEM);

	ret = drm_gem_object_init(dev, &mtk_gem_obj->base, size);
	if (ret < 0) {
		DRM_ERROR("failed to initialize gem object\n");
		kfree(mtk_gem_obj);
		return ERR_PTR(ret);
	}

	return mtk_gem_obj;
}

struct mtk_drm_gem_obj *mtk_drm_gem_create(struct drm_device *dev,
					   unsigned long size, bool alloc_kmap)
{
	struct mtk_drm_gem_obj *mtk_gem;
	struct drm_gem_object *obj;
	int ret;

	mtk_gem = mtk_drm_gem_init(dev, size);
	if (IS_ERR(mtk_gem))
		return ERR_CAST(mtk_gem);

	obj = &mtk_gem->base;

	init_dma_attrs(&mtk_gem->dma_attrs);
	dma_set_attr(DMA_ATTR_WRITE_COMBINE, &mtk_gem->dma_attrs);

	if (!alloc_kmap)
		dma_set_attr(DMA_ATTR_NO_KERNEL_MAPPING, &mtk_gem->dma_attrs);

	mtk_gem->cookie = dma_alloc_attrs(dev->dev, obj->size,
				(dma_addr_t *)&mtk_gem->dma_addr, GFP_KERNEL,
				&mtk_gem->dma_attrs);
	if (!mtk_gem->cookie) {
		DRM_ERROR("failed to allocate %zx byte dma buffer", obj->size);
		ret = -ENOMEM;
		goto err_gem_free;
	}

	if (alloc_kmap)
		mtk_gem->kvaddr = mtk_gem->cookie;

	DRM_INFO("cookie = %p dma_addr = %pad\n",
			mtk_gem->cookie, &mtk_gem->dma_addr);

	return mtk_gem;

err_gem_free:
	drm_gem_object_release(obj);
	kfree(mtk_gem);
	return ERR_PTR(ret);
}

void mtk_drm_gem_free_object(struct drm_gem_object *obj)
{
	struct mtk_drm_gem_obj *mtk_gem = to_mtk_gem_obj(obj);

	dma_free_attrs(obj->dev->dev, obj->size, mtk_gem->cookie,
		       mtk_gem->dma_addr, &mtk_gem->dma_attrs);

	/* release file pointer to gem object. */
	drm_gem_object_release(obj);

	kfree(mtk_gem);
}

int mtk_drm_gem_dumb_create(struct drm_file *file_priv, struct drm_device *dev,
			    struct drm_mode_create_dumb *args)
{
	struct mtk_drm_gem_obj *mtk_gem;
	int ret;

	args->pitch = args->width * DIV_ROUND_UP(args->bpp, 8);
	#ifdef CONFIG_MALI_DMA_BUF_MAP_ON_ATTACH
	/*
	* align to 8 bytes since Mali requires it.
	*/
	args->pitch = ALIGN(args->pitch, 8);
	#endif
	args->size = args->pitch * args->height;

	mtk_gem = mtk_drm_gem_create(dev, args->size, false);
	if (IS_ERR(mtk_gem))
		return PTR_ERR(mtk_gem);

	/*
	 * allocate a id of idr table where the obj is registered
	 * and handle has the id what user can see.
	 */
	ret = drm_gem_handle_create(file_priv, &mtk_gem->base, &args->handle);
	if (ret)
		goto err_handle_create;

	/* drop reference from allocate - handle holds it now. */
	drm_gem_object_unreference_unlocked(&mtk_gem->base);

	return 0;

err_handle_create:
	mtk_drm_gem_free_object(&mtk_gem->base);
	return ret;
}

int mtk_drm_gem_dumb_map_offset(struct drm_file *file_priv,
				struct drm_device *dev, uint32_t handle,
				uint64_t *offset)
{
	struct drm_gem_object *obj;
	int ret;

	obj = drm_gem_object_lookup(dev, file_priv, handle);
	if (!obj) {
		DRM_ERROR("failed to lookup gem object.\n");
		return -EINVAL;
	}

	ret = drm_gem_create_mmap_offset(obj);
	if (ret)
		goto out;

	*offset = drm_vma_node_offset_addr(&obj->vma_node);
	DRM_DEBUG_KMS("offset = 0x%llx\n", *offset);

out:
	drm_gem_object_unreference_unlocked(obj);
	return ret;
}

static int mtk_drm_gem_object_mmap(struct drm_gem_object *obj,
				   struct vm_area_struct *vma)

{
	int ret;
	struct mtk_drm_gem_obj *mtk_gem = to_mtk_gem_obj(obj);
	struct drm_device *drm = obj->dev;

	/*
	 * dma_alloc_attrs() allocated a struct page table for rk_obj, so clear
	 * VM_PFNMAP flag that was set by drm_gem_mmap_obj()/drm_gem_mmap().
	 */
	vma->vm_flags &= ~VM_PFNMAP;
	vma->vm_pgoff = 0;

	ret = dma_mmap_attrs(drm->dev, vma, mtk_gem->cookie, mtk_gem->dma_addr,
			     obj->size, &mtk_gem->dma_attrs);
	if (ret)
		drm_gem_vm_close(vma);

	return ret;
}

int mtk_drm_gem_mmap_buf(struct drm_gem_object *obj, struct vm_area_struct *vma)
{
	int ret;

	ret = drm_gem_mmap_obj(obj, obj->size, vma);
	if (ret)
		return ret;

	return mtk_drm_gem_object_mmap(obj, vma);
}

int mtk_drm_gem_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct drm_gem_object *obj;
	int ret;

	ret = drm_gem_mmap(filp, vma);
	if (ret)
		return ret;

	obj = vma->vm_private_data;

	return mtk_drm_gem_object_mmap(obj, vma);
}

/*
 * Allocate a sg_table for this GEM object.
 * Note: Both the table's contents, and the sg_table itself must be freed by
 *       the caller.
 * Returns a pointer to the newly allocated sg_table, or an ERR_PTR() error.
 */
struct sg_table *mtk_gem_prime_get_sg_table(struct drm_gem_object *obj)
{
	struct mtk_drm_gem_obj *mtk_gem = to_mtk_gem_obj(obj);
	struct drm_device *drm = obj->dev;
	struct sg_table *sgt;
	int ret;

	sgt = kzalloc(sizeof(*sgt), GFP_KERNEL);
	if (!sgt)
		return ERR_PTR(-ENOMEM);

	ret = dma_get_sgtable_attrs(drm->dev, sgt, mtk_gem->cookie,
				    mtk_gem->dma_addr, obj->size,
				    &mtk_gem->dma_attrs);
	if (ret) {
		DRM_ERROR("failed to allocate sgt, %d\n", ret);
		kfree(sgt);
		return ERR_PTR(ret);
	}

	return sgt;
}

int mtk_gem_map_offset_ioctl(struct drm_device *drm, void *data,
			     struct drm_file *file_priv)
{
	struct drm_mtk_gem_map_off *args = data;

	return mtk_drm_gem_dumb_map_offset(file_priv, drm, args->handle,
					   &args->offset);
}

int mtk_gem_create_ioctl(struct drm_device *dev, void *data,
			 struct drm_file *file_priv)
{
	struct mtk_drm_gem_obj *mtk_gem;
	struct drm_mtk_gem_create *args = data;
	int ret;

	mtk_gem = mtk_drm_gem_create(dev, args->size, false);
	if (IS_ERR(mtk_gem))
		return PTR_ERR(mtk_gem);

	/*
	 * allocate a id of idr table where the obj is registered
	 * and handle has the id what user can see.
	 */
	ret = drm_gem_handle_create(file_priv, &mtk_gem->base, &args->handle);
	if (ret)
		goto err_handle_create;

	/* drop reference from allocate - handle holds it now. */
	drm_gem_object_unreference_unlocked(&mtk_gem->base);

	return 0;

err_handle_create:
	mtk_drm_gem_free_object(&mtk_gem->base);
	return ret;
}
