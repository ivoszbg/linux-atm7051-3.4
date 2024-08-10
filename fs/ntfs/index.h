/*
 * index.h - Defines for NTFS kernel index handling.  Part of the Linux-NTFS
 *	     project.
 *
 * Copyright (c) 2004 Anton Altaparmakov
 *
 * This program/include file is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as published
 * by the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program/include file is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program (in the main directory of the Linux-NTFS
 * distribution in the file COPYING); if not, write to the Free Software
 * Foundation,Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef _LINUX_NTFS_INDEX_H
#define _LINUX_NTFS_INDEX_H

#include <linux/fs.h>

#include "types.h"
#include "layout.h"
#include "inode.h"
#include "attrib.h"
#include "mft.h"
#include "aops.h"

#if 0
/**
 * @idx_ni:	index inode containing the @entry described by this context
 * @entry:	index entry (points into @ir or @ia)
 * @data:	index entry data (points into @entry)
 * @data_len:	length in bytes of @data
 * @is_in_root:	'true' if @entry is in @ir and 'false' if it is in @ia
 * @ir:		index root if @is_in_root and NULL otherwise
 * @actx:	attribute search context if @is_in_root and NULL otherwise
 * @base_ni:	base inode if @is_in_root and NULL otherwise
 * @ia:		index block if @is_in_root is 'false' and NULL otherwise
 * @page:	page if @is_in_root is 'false' and NULL otherwise
 *
 * @idx_ni is the index inode this context belongs to.
 *
 * @entry is the index entry described by this context.  @data and @data_len
 * are the index entry data and its length in bytes, respectively.  @data
 * simply points into @entry.  This is probably what the user is interested in.
 *
 * If @is_in_root is 'true', @entry is in the index root attribute @ir described
 * by the attribute search context @actx and the base inode @base_ni.  @ia and
 * @page are NULL in this case.
 *
 * If @is_in_root is 'false', @entry is in the index allocation attributeand
 *@ia and @page point to the index allocation block and the mapped, in this
 *case. locked page it is in, respectively.  @ir, @actx and @base_ni are NULL
 *
 * To obtain a context call ntfs_index_ctx_get().
 *
 * We use this context to allow ntfs_index_lookup() to return the found index
 * @entry and its @data without having to allocate a buffer and copy the @entry
 * and/or its @data into it.
 *
 * When finished with the @entry and its @data, call ntfs_index_ctx_put() to
 * free the context and other associated resources.
 *
 * If the index entry was modified, call flush_dcache_index_entry_page()
 * immediately after the modification and either ntfs_index_entry_mark_dirty()
 * or ntfs_index_entry_write() before the call to ntfs_index_ctx_put() to
 * ensure that the changes are written to disk.
 */

typedef struct {
	ntfs_inode *idx_ni;
	INDEX_ENTRY *entry;
	void *data;
	u16 data_len;
	bool is_in_root;
	INDEX_ROOT *ir;
	ntfs_attr_search_ctx *actx;
	ntfs_inode *base_ni;
	INDEX_ALLOCATION *ia;
	struct page *page;
} ntfs_index_context;
#endif

#define  VCN_INDEX_ROOT_PARENT  ((VCN)-2)

#define  MAX_PARENT_VCN		32

/**
 * struct ntfs_index_context -
 * @ni:			inode containing the @entry described by this context
 * @name:		name of the index described by this context
 * @name_len:		length of the index name
 * @entry:		index entry (points into @ir or @ia)
 * @data:		index entry data (points into @entry)
 * @data_len:		length in bytes of @data
 * @is_in_root:		TRUE if @entry is in @ir or FALSE if it is in @ia
 * @ir:			index root if @is_in_root or NULL otherwise
 * @actx:		attribute search context if in root or NULL otherwise
 * @ia:			index block if @is_in_root is FALSE or NULL otherwise
 * @ia_na:		opened INDEX_ALLOCATION attribute
 * @parent_pos:		parent entries' positions in the index block
 * @parent_vcn:		entry's parent node or VCN_INDEX_ROOT_PARENT
 * @new_vcn:            new VCN if we need to create a new index block
 * @median:		move to the parent if splitting index blocks
 * @ib_dirty:		TRUE if index block was changed
 * @block_size:		index block size
 * @vcn_size_bits:	VCN size bits for this index block
 *
 * @ni is the inode this context belongs to.
 *
 * @entry is the index entry described by this context.  @data and @data_len
 * are the index entry data and its length in bytes, respectively.  @data
 * simply points into @entry.  This is probably what the user is interested in.
 *
 * If @is_in_root is TRUE, @entry is in the index root attribute @ir described
 * by the attribute search context @actx and inode @ni.  @ia and
 * @ib_dirty are undefined in this case.
 *
 * If @is_in_root is FALSE, @entry is in the index allocation attribute and @ia
 * point to the index allocation block and VCN where it's placed,
 * respectively. @ir and @actx are NULL in this case. @ia_na is opened
 * INDEX_ALLOCATION attribute. @ib_dirty is TRUE if index block was changed and
 * FALSE otherwise.
 *
 * To obtain a context call ntfs_index_ctx_get().
 *
 * When finished with the @entry and its @data, call ntfs_index_ctx_put() to
 * free the context and other associated resources.
 *
 * If the index entry was modified, call ntfs_index_entry_mark_dirty() before
 * the call to ntfs_index_ctx_put() to ensure that the changes are written
 * to disk.
 */

typedef struct {
	ntfs_inode *ni;
	int mftmapped;
	ntfschar *name;
	u32 name_len;
	INDEX_ENTRY *entry;
	void *data;
	u16 data_len;
	COLLATION_RULE cr;
	int is_in_root;
	INDEX_ROOT *ir;
	ntfs_attr_search_ctx *actx;
	INDEX_BLOCK *ib;
	int parent_pos[MAX_PARENT_VCN];  /* parent entries' positions */
	VCN parent_vcn[MAX_PARENT_VCN]; /* entry's parent nodes */
	int pindex;	     /* maximum it's the number of the parent nodes  */
	u32 block_size;
	u8 block_size_bit;
	u8 vcn_size_bits;
	struct page *page;

} ntfs_index_context;

extern ntfs_index_context *ntfs_index_ctx_get(ntfs_inode *ni,
				       ntfschar *name, u32 name_len);
extern void ntfs_index_ctx_put(ntfs_index_context *ictx);

extern int ntfs_index_lookup(const void *key, const int key_len,
		ntfs_index_context *ictx);

extern int ntfs_ie_add(ntfs_index_context *icx, INDEX_ENTRY *ie);


extern int ntfs_index_remove(ntfs_inode *dir_ni, ntfs_inode *ni,
		const void *key, const int keylen);

extern int ntfs_index_add_filename(ntfs_inode *ni, FILE_NAME_ATTR *fn,
			MFT_REF mref);
#ifdef NTFS_RW

/**
 * ntfs_index_entry_flush_dcache_page - flush_dcache_page() for index entries
 * @ictx:	ntfs index context describing the index entry
 *
 * Call flush_dcache_page() for the page in which an index entry resides.
 *
 * This must be called every time an index entry is modified, just after the
 * modification.
 *
 * If the index entry is in the index root attribute, simply flush the page
 * containing the mft record containing the index root attribute.
 *
 * If the index entry is in an index block belonging to the index allocation
 * attribute, simply flush the page cache page containing the index block.
 */
static inline void ntfs_index_entry_flush_dcache_page(ntfs_index_context *ictx)
{
	if (ictx->is_in_root)
		flush_dcache_mft_record_page(ictx->actx->ntfs_ino);
	else
		flush_dcache_page(ictx->page);
}

/**
 * ntfs_index_entry_mark_dirty - mark an index entry dirty
 * @ictx:	ntfs index context describing the index entry
 *
 * Mark the index entry described by the index entry context @ictx dirty.
 *
 * If the index entry is in the index root attribute, simply mark the mft
 * record containing the index root attribute dirty.  This ensures the mft
 * record, and hence the index root attribute, will be written out to disk
 * later.
 *
 * If the index entry is in an index block belonging to the index allocation
 * attribute, mark the buffers belonging to the index record as well as the
 * page cache page the index block is in dirty.  This automatically marks the
 * VFS inode of the ntfs index inode to which the index entry belongs dirty,
 * too (I_DIRTY_PAGES) and this in turn ensures the page buffers, and hence the
 * dirty index block, will be written out to disk later.
 */
static inline void ntfs_index_entry_mark_dirty(ntfs_index_context *ictx)
{
	if (ictx->is_in_root)
		mark_mft_record_dirty(ictx->actx->ntfs_ino);
	else
		mark_ntfs_record_dirty(ictx->page,
			(u8 *)ictx->ib - (u8 *)page_address(ictx->page));
}

#endif /* NTFS_RW */

#endif /* _LINUX_NTFS_INDEX_H */
