/*
 * index.c - NTFS kernel index handling.  Part of the Linux-NTFS project.
 *
 * Copyright (c) 2004-2005 Anton Altaparmakov
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

#include "aops.h"
#include "collate.h"
#include "debug.h"
#include "index.h"
#include "ntfs.h"
#include "dir.h"
#include "reparse.h"
#include "malloc.h"
#include "bitmap.h"


#define STATUS_KEEP_SEARCHING			(3)
#ifdef NTFS_RW

static int ntfs_ir_lookup2(ntfs_index_context *ictx);

static void ntfs_ih_insert(INDEX_HEADER *ih, INDEX_ENTRY *orig_ie, VCN new_vcn,
			  int pos);

static int ntfs_ib_split(ntfs_index_context *icx, INDEX_BLOCK *ib);

static int ntfs_ir_make_space(ntfs_index_context *icx, int data_size);

#endif
static VCN *ntfs_ie_get_vcn_addr(INDEX_ENTRY *ie)
{
	return (VCN *)((u8 *)ie + le16_to_cpu(ie->length) - sizeof(VCN));
}

/**
 *  Get the subnode vcn to which the index entry refers.
 */
VCN ntfs_ie_get_vcn(INDEX_ENTRY *ie)
{
	return sle64_to_cpup(ntfs_ie_get_vcn_addr(ie));
}

static INDEX_ENTRY *ntfs_ie_get_first(INDEX_HEADER *ih)
{
	return (INDEX_ENTRY *)((u8 *)ih + le32_to_cpu(ih->entries_offset));
}

static INDEX_ENTRY *ntfs_ie_get_next(INDEX_ENTRY *ie)
{
	return (INDEX_ENTRY *)((char *)ie + le16_to_cpu(ie->length));
}

static u8 *ntfs_ie_get_end(INDEX_HEADER *ih)
{
	/* FIXME: check if it isn't overflowing the index block size */
	return (u8 *)ih + le32_to_cpu(ih->index_length);
}

static int ntfs_ie_end(INDEX_ENTRY *ie)
{
	return ie->flags & INDEX_ENTRY_END || !ie->length;
}

#ifdef NTFS_RW
/**
 *  Find the last entry in the index block
 */
static INDEX_ENTRY *ntfs_ie_get_last(INDEX_ENTRY *ie, char *ies_end)
{
	ntfs_debug("Entering\n");

	while ((char *)ie < ies_end && !ntfs_ie_end(ie))
		ie = ntfs_ie_get_next(ie);

	return ie;
}

static INDEX_ENTRY *ntfs_ie_get_by_pos(INDEX_HEADER *ih, int pos)
{
	INDEX_ENTRY *ie;

	ntfs_debug("pos: %d\n", pos);

	ie = ntfs_ie_get_first(ih);

	while (pos-- > 0)
		ie = ntfs_ie_get_next(ie);

	return ie;
}

static INDEX_ENTRY *ntfs_ie_prev(INDEX_HEADER *ih, INDEX_ENTRY *ie)
{
	INDEX_ENTRY *ie_prev = NULL;
	INDEX_ENTRY *tmp;

	ntfs_debug("Entering\n");

	tmp = ntfs_ie_get_first(ih);

	while (tmp != ie) {
		ie_prev = tmp;
		tmp = ntfs_ie_get_next(tmp);
	}

	return ie_prev;
}

static int ntfs_ih_numof_entries(INDEX_HEADER *ih)
{
	int n;
	INDEX_ENTRY *ie;
	u8 *end;

	ntfs_debug("Entering\n");

	end = ntfs_ie_get_end(ih);
	ie = ntfs_ie_get_first(ih);
	for (n = 0; !ntfs_ie_end(ie) && (u8 *)ie < end; n++)
		ie = ntfs_ie_get_next(ie);
	return n;
}

static int ntfs_ih_one_entry(INDEX_HEADER *ih)
{
	return ntfs_ih_numof_entries(ih) == 1;
}

static int ntfs_ih_zero_entry(INDEX_HEADER *ih)
{
	return ntfs_ih_numof_entries(ih) == 0;
}


static void ntfs_ie_delete(INDEX_HEADER *ih, INDEX_ENTRY *ie)
{
	u32 new_size;

	ntfs_debug("Entering\n");

	new_size = le32_to_cpu(ih->index_length) - le16_to_cpu(ie->length);
	ih->index_length = cpu_to_le32(new_size);
	memmove(ie, (u8 *)ie + le16_to_cpu(ie->length),
		new_size - ((u8 *)ie - (u8 *)ih));
}


static void ntfs_ie_set_vcn(INDEX_ENTRY *ie, VCN vcn)
{
	*ntfs_ie_get_vcn_addr(ie) = cpu_to_le64(vcn);
}

static VCN ntfs_icx_parent_vcn(ntfs_index_context *icx)
{
	return icx->parent_vcn[icx->pindex];
}

static VCN ntfs_icx_parent_pos(ntfs_index_context *icx)
{
	return icx->parent_pos[icx->pindex];
}
#endif
static int ntfs_icx_parent_inc(ntfs_index_context *icx)
{
	icx->pindex++;
	if (icx->pindex >= MAX_PARENT_VCN) {
		ntfs_error(icx->ni->vol->sb, "Index is over %d"
				" level deep", MAX_PARENT_VCN);
		return -EOPNOTSUPP;
	}
	return 0;
}

#ifdef NTFS_RW
static int ntfs_icx_parent_dec(ntfs_index_context *icx)
{
	icx->pindex--;
	if (icx->pindex < 0) {
		ntfs_error(icx->ni->vol->sb, "Corrupt index pointer (%d)",
				icx->pindex);
		return -EINVAL;
	}
	return 0;
}

static s64 ntfs_ib_vcn_to_pos(ntfs_index_context *icx, VCN vcn)
{
	return vcn << icx->vcn_size_bits;
}

static VCN ntfs_ib_pos_to_vcn(ntfs_index_context *icx, s64 pos)
{
	return pos >> icx->vcn_size_bits;
}

void ntfs_ib_init(INDEX_BLOCK *ib, u32 ib_size, VCN ib_vcn,
				  INDEX_HEADER_FLAGS node_type)
{
	int ih_size = sizeof(INDEX_HEADER);

	ntfs_debug("ib_vcn: %lld ib_size: %u\n", (long long)ib_vcn, ib_size);


	ib->magic = magic_INDX;
	ib->usa_ofs = cpu_to_le16(sizeof(INDEX_BLOCK));
	ib->usa_count = cpu_to_le16(ib_size / NTFS_BLOCK_SIZE + 1);
	/* Set USN to 1 */
	*(u16 *)((char *)ib + le16_to_cpu(ib->usa_ofs)) = cpu_to_le16(1);
	ib->lsn = cpu_to_le64(0);

	ib->index_block_vcn = cpu_to_sle64(ib_vcn);

	ib->index.entries_offset = cpu_to_le32((ih_size +
			le16_to_cpu(ib->usa_count) * 2 + 7) & ~7);
	ib->index.index_length = 0;
	ib->index.allocated_size = cpu_to_le32(ib_size -
					       (sizeof(INDEX_BLOCK) - ih_size));
	ib->index.flags = node_type;
	return;
}

/**
 *  Find the median by going through all the entries
 */
static INDEX_ENTRY *ntfs_ie_get_median(INDEX_HEADER *ih)
{
	INDEX_ENTRY *ie, *ie_start;
	u8 *ie_end;
	int i = 0, median;

	ntfs_debug("Entering\n");

	ie = ie_start = ntfs_ie_get_first(ih);
	ie_end   = (u8 *)ntfs_ie_get_end(ih);

	while ((u8 *)ie < ie_end && !ntfs_ie_end(ie)) {
		ie = ntfs_ie_get_next(ie);
		i++;
	}
	/*
	 * NOTE: this could be also the entry at the half of the index block.
	 */
	median = i / 2 - 1;

	ntfs_debug("Entries: %d  median: %d\n", i, median);

	for (i = 0, ie = ie_start; i <= median; i++)
		ie = ntfs_ie_get_next(ie);

	return ie;
}

static s64 ntfs_ibm_vcn_to_pos(ntfs_index_context *icx, VCN vcn)
{
	return ntfs_ib_vcn_to_pos(icx, vcn) >> icx->block_size_bit;
}

static s64 ntfs_ibm_pos_to_vcn(ntfs_index_context *icx, s64 pos)
{
	return ntfs_ib_pos_to_vcn(icx, pos << icx->block_size_bit);
}

static int ntfs_ibm_add(ntfs_index_context *icx)
{
	u8 bmp[8];
	int ret;

	ntfs_debug("Entering\n");

	ret = ntfs_attr_exist(icx->ni, AT_BITMAP, icx->name, icx->name_len);
	if (ret < 0)
		return ret;
	else if (ret > 0)
		return 0;
	/*
	 * AT_BITMAP must be at least 8 bytes.
	 */
	memset(bmp, 0, sizeof(bmp));
	ret = ntfs_attr_add(icx->ni, AT_BITMAP, icx->name, icx->name_len,
			  bmp, sizeof(bmp));
	if (ret) {
		ntfs_error(icx->ni->vol->sb, "Failed to add AT_BITMAP");
		return ret;
	}

	return 0;
}

static int ntfs_ibm_modify(ntfs_index_context *icx, VCN vcn, int set)
{
	s64 pos = ntfs_ibm_vcn_to_pos(icx, vcn);
	u32 bpos = pos >> 3;
	u32 bit = 1 << (pos & 7);
	ntfs_inode *ni = icx->ni;
	struct inode *vi = VFS_I(ni);
	struct inode *bmp_vi = NULL;
	struct super_block *sb = ni->vol->sb;
	int ret = 0;
	s64 datasize;
	u8 *bm;

	ntfs_debug("%s vcn: %lld\n", set ? "set" : "clear", (long long)vcn);

	bmp_vi = ntfs_attr_iget(vi, AT_BITMAP, icx->name, icx->name_len);
	if (IS_ERR(bmp_vi)) {
		ntfs_error(vi->i_sb, "Failed to get bitmap attribute.");
		ret = PTR_ERR(bmp_vi);
		return ret;
	}

	datasize = i_size_read(bmp_vi);
	if (set) {
		if (datasize < bpos + 1) {
			bmp_vi->i_size = (datasize + 8) & ~7;
			ret = ntfs_truncate(bmp_vi);
			if (ret) {
				ntfs_error(sb, "Failed to truncate AT_BITMAP");
				goto err_out;
			}
		}
	}

	if (NInoNonResident(NTFS_I(bmp_vi)))
		ret = ntfs_bitmap_set_bits_in_run(bmp_vi, pos, 1, set);
	else {
			ntfs_attr_reinit_search_ctx(icx->actx);
			ret = ntfs_attr_lookup(AT_BITMAP, icx->name,
				icx->name_len, CASE_SENSITIVE, 0,
				NULL, 0, icx->actx);
			if (unlikely(ret))
				goto err_out;
			bm = (u8 *)icx->actx->attr + le16_to_cpu(
				icx->actx->attr->data.resident.value_offset);
			datasize = i_size_read(bmp_vi);
			if (datasize < bpos + 1)  {
				ret = -EIO;
				goto err_out;
			}

			if (set)
				bm[bpos] |= bit;
			else
				bm[bpos] &= ~bit;

			flush_dcache_mft_record_page(icx->ni);
			mark_mft_record_dirty(icx->ni);
		}

err_out:
	iput(bmp_vi);
	return ret;
}


static int ntfs_ibm_set(ntfs_index_context *icx, VCN vcn)
{
	return ntfs_ibm_modify(icx, vcn, 1);
}

static int ntfs_ibm_clear(ntfs_index_context *icx, VCN vcn)
{
	return ntfs_ibm_modify(icx, vcn, 0);
}

static VCN ntfs_ibm_get_free(ntfs_index_context *icx)
{
	u8 *bm;
	int bit;
	s64 vcn, byte, size, offset, npages;
	struct page *page;
	pgoff_t index;
	ntfs_inode *ni = icx->ni;
	struct inode *vi = VFS_I(ni);
	struct inode *bmp_vi;
	struct address_space *mapping;
	u32 size_temp;
	int ret;

	ntfs_debug("Entering\n");

	bmp_vi = ntfs_attr_iget(vi, AT_BITMAP, icx->name, icx->name_len);
	if (IS_ERR(bmp_vi)) {
		ntfs_error(vi->i_sb, "Failed to get bitmap attribute.");
		ret = PTR_ERR(bmp_vi);
		return ret;
	}
	size = i_size_read(bmp_vi);

	if (NInoNonResident(NTFS_I(bmp_vi))) {
		npages = (size + PAGE_CACHE_SIZE - 1) >> PAGE_CACHE_SHIFT;
		offset = 0;
		index = 0;

		for (index = 0; index < npages; index++) {
			mapping = bmp_vi->i_mapping;
			page = ntfs_map_page(mapping, index);
			if (IS_ERR(page))
				return (VCN)PTR_ERR(page);

			bm = page_address(page);

			size_temp = min_t(u32, size - offset, PAGE_CACHE_SIZE);

			for (byte = 0; byte < size_temp; byte++) {
				if (bm[byte] == 255)
					continue;

				for (bit = 0; bit < 8; bit++) {
					if (!(bm[byte] & (1 << bit))) {
						vcn = ntfs_ibm_pos_to_vcn(icx,
							(byte+offset) *
							8 + bit);
						bm[byte] |= 1 << bit;
						flush_dcache_page(icx->page);
						set_page_dirty(icx->page);
						ntfs_unmap_page(page);
						goto out;
					}
				}
			}

			offset += PAGE_CACHE_SIZE;
			ntfs_unmap_page(page);
		}

	} else {
		ntfs_attr_reinit_search_ctx(icx->actx);
		ret = ntfs_attr_lookup(AT_BITMAP, icx->name, icx->name_len,
				CASE_SENSITIVE, 0, NULL, 0, icx->actx);
		if (unlikely(ret)) {
			vcn = (VCN)ret;
			goto out;
		}

		bm = (u8 *)icx->actx->attr + le16_to_cpu(
				icx->actx->attr->data.resident.value_offset);

		for (byte = 0; byte < size; byte++) {
			if (bm[byte] == 255)
				continue;

			for (bit = 0; bit < 8; bit++) {
				if (!(bm[byte] & (1 << bit))) {
					vcn = ntfs_ibm_pos_to_vcn(icx, (byte)
							* 8 + bit);
					bm[byte] |= 1 << bit;
					flush_dcache_mft_record_page(icx->ni);
					mark_mft_record_dirty(icx->ni);
					goto out;
				}
			}
		}

	}
	vcn = ntfs_ibm_pos_to_vcn(icx, size * 8);
	ret = ntfs_ibm_set(icx, vcn);
	if (ret)
		vcn = (VCN)ret;
out:
	ntfs_debug("allocated vcn: %lld\n", (long long)vcn);
	iput(bmp_vi);
	return vcn;
}


void ntfs_ie_add_vcn(INDEX_ENTRY **ie)
{
	INDEX_ENTRY *p = *ie;

	p->length = cpu_to_le16(le16_to_cpu(p->length) + sizeof(VCN));

	p->flags |= INDEX_ENTRY_NODE;
	return;
}

void ntfs_ie_del_vcn(INDEX_ENTRY **ie)
{
	INDEX_ENTRY *p = *ie;

	p->length = cpu_to_le16(le16_to_cpu(p->length) - sizeof(VCN));

	p->flags &= ~INDEX_ENTRY_NODE;
	return;
}

static int ntfs_ib_copy_tail(ntfs_index_context *icx, INDEX_BLOCK *src,
			     INDEX_ENTRY *median, VCN new_vcn)
{
	u8 *ies_end;
	INDEX_ENTRY *ie_head;		/* first entry after the median */
	int tail_size, ret;
	INDEX_BLOCK *dst;
	struct page *page;
	ntfs_inode *ni = icx->ni;
	struct inode *vi = VFS_I(ni);
	struct address_space *mapping;
	pgoff_t index;
	u32 ib_size = le32_to_cpu(icx->ir->index_block_size);
	loff_t new_size;

	ntfs_debug("Entering\n");

	ntfs_debug("ib_vcn: %lld ib_size: %u\n", (long long)new_vcn, ib_size);

	new_size = (new_vcn << icx->vcn_size_bits) + ib_size;
	if (new_size > i_size_read(vi)) {
		i_size_write(vi, new_size);
		ret = ntfs_truncate(vi);
		if (ret)
			return ret;
	}

	mapping = vi->i_mapping;
	index = (new_vcn << icx->vcn_size_bits) >> PAGE_CACHE_SHIFT;

	if (icx->page && icx->page->index == index) {
		page_cache_get(icx->page);
		page = icx->page;
		kmap(page);
	} else {
		page = ntfs_map_page(mapping, index);
		if (IS_ERR(page))
			return PTR_ERR(page);
		lock_page(page);
	}

	dst = page_address(page) + ((new_vcn <<
			icx->vcn_size_bits) & ~PAGE_CACHE_MASK);

	ntfs_ib_init(dst, icx->block_size, new_vcn,
			    src->index.flags & NODE_MASK);

	ie_head = ntfs_ie_get_next(median);

	ies_end = (u8 *)ntfs_ie_get_end(&src->index);
	tail_size = ies_end - (u8 *)ie_head;
	memcpy(ntfs_ie_get_first(&dst->index), ie_head, tail_size);

	dst->index.index_length = cpu_to_le32(tail_size +
			 le32_to_cpu(dst->index.entries_offset));

	flush_dcache_page(page);
	set_page_dirty(page);

	if (!(icx->page) || icx->page->index != page->index)
		unlock_page(page);
	ntfs_unmap_page(page);
	return 0;
}

static void ntfs_ib_cut_tail(ntfs_index_context *icx, INDEX_BLOCK *ib,
			    INDEX_ENTRY *ie)
{
	char *ies_start, *ies_end;
	INDEX_ENTRY *ie_last;

	ntfs_debug("Entering\n");

	ies_start = (char *)ntfs_ie_get_first(&ib->index);
	ies_end   = (char *)ntfs_ie_get_end(&ib->index);

	ie_last   = ntfs_ie_get_last((INDEX_ENTRY *)ies_start, ies_end);
	if (ie_last->flags & INDEX_ENTRY_NODE)
		ntfs_ie_set_vcn(ie_last, ntfs_ie_get_vcn(ie));

	memcpy(ie, ie_last, le16_to_cpu(ie_last->length));

	ib->index.index_length = cpu_to_le32(((char *)ie - ies_start) +
		le16_to_cpu(ie->length) + le32_to_cpu(
		ib->index.entries_offset));

	flush_dcache_page(icx->page);
	set_page_dirty(icx->page);

	return;
}

static int ntfs_ir_insert_median(ntfs_index_context *icx, INDEX_ENTRY *median,
				 VCN new_vcn)
{
	u32 new_size;
	int ret;

	ntfs_debug("Entering\n");

	new_size = le32_to_cpu(icx->ir->index.index_length) +
			le16_to_cpu(median->length);
	if (!(median->flags & INDEX_ENTRY_NODE))
		new_size += sizeof(VCN);

	ret = ntfs_ir_make_space(icx, new_size);
	if (ret)
		return ret;

	ret = ntfs_ir_lookup2(icx);
	if (ret)
		return ret;

	ntfs_ih_insert(&icx->ir->index, median, new_vcn,
			      ntfs_icx_parent_pos(icx));
	return 0;
}

/**
 * On success return STATUS_OK or STATUS_KEEP_SEARCHING.
 * On error return STATUS_ERROR.
 */
static int ntfs_ib_insert(ntfs_index_context *icx, INDEX_ENTRY *ie, VCN new_vcn)
{
	INDEX_BLOCK *ib;
	u32 idx_size, allocated_size;
	int err = 0;
	VCN old_vcn;
	struct page *page;
	ntfs_inode *ni = icx->ni;
	struct inode *vi = VFS_I(ni);
	struct address_space *mapping;
	pgoff_t index;

	ntfs_debug("Entering\n");

	old_vcn = ntfs_icx_parent_vcn(icx);

	mapping = vi->i_mapping;
	index = (old_vcn << icx->vcn_size_bits) >> PAGE_CACHE_SHIFT;

	if (icx->page && icx->page->index == index) {
		page_cache_get(icx->page);
		page = icx->page;
		kmap(page);
	} else {
		page = ntfs_map_page(mapping, index);
		if (IS_ERR(page))
			return PTR_ERR(page);
		lock_page(page);
	}

	ib = page_address(page) + ((old_vcn <<
			icx->vcn_size_bits) & ~PAGE_CACHE_MASK);

	idx_size = le32_to_cpu(ib->index.index_length);
	allocated_size = le32_to_cpu(ib->index.allocated_size);
	/* FIXME: sizeof(VCN) should be included only if ie has no VCN */
	if (idx_size + le16_to_cpu(ie->length) + sizeof(VCN) > allocated_size) {
		err = ntfs_ib_split(icx, ib);
		if (!err)
			err = STATUS_KEEP_SEARCHING;
		goto err_out;
	}

	ntfs_ih_insert(&ib->index, ie, new_vcn, ntfs_icx_parent_pos(icx));

	flush_dcache_page(page);
	set_page_dirty(page);

err_out:
	if (!(icx->page) || icx->page->index != page->index)
		unlock_page(page);
	ntfs_unmap_page(page);
	return err;
}

/**
 * ntfs_ib_split - Split an index block
 *
 * On success return STATUS_OK or STATUS_KEEP_SEARCHING.
 * On error return is STATUS_ERROR.
 */
static int ntfs_ib_split(ntfs_index_context *icx, INDEX_BLOCK *ib)
{
	INDEX_ENTRY *median;
	VCN new_vcn;
	int ret;

	ntfs_debug("Entering\n");
	ret = ntfs_icx_parent_dec(icx);
	if (ret)
		return ret;

	median  = ntfs_ie_get_median(&ib->index);
	new_vcn = ntfs_ibm_get_free(icx);
	if (IS_ERR_VALUE(new_vcn))
		return (int)new_vcn;

	ret = ntfs_ib_copy_tail(icx, ib, median, new_vcn);
	if (ret)
		goto err_out;

	if (ntfs_icx_parent_vcn(icx) == VCN_INDEX_ROOT_PARENT)
		ret = ntfs_ir_insert_median(icx, median, new_vcn);
	else
		ret = ntfs_ib_insert(icx, median, new_vcn);

	if (ret)
		goto err_out;

	ntfs_ib_cut_tail(icx, ib, median);

	return 0;

err_out:
		ntfs_ibm_clear(icx, new_vcn);
	return ret;
}

static int ntfs_ib_alloc(ntfs_index_context *icx, VCN ib_vcn,
				  INDEX_HEADER_FLAGS node_type)
{
	INDEX_BLOCK *ib;
	int ih_size = sizeof(INDEX_HEADER);
	u32 ib_size = le32_to_cpu(icx->ir->index_block_size);
	loff_t new_size;
	struct inode *vi = VFS_I(icx->ni);
	int ret;
	u8 *kaddr;
	struct address_space *ia_mapping;
	pgoff_t index;
	struct page *page;

	ntfs_debug("ib_vcn: %lld ib_size: %u\n", (long long)ib_vcn, ib_size);

	new_size = (ib_vcn << icx->vcn_size_bits) + ib_size;
	if (new_size > i_size_read(vi)) {
		i_size_write(vi, new_size);
		ret = ntfs_truncate(vi);
		if (ret)
			return ret;
	}

	ia_mapping = vi->i_mapping;
	index = ib_vcn << icx->vcn_size_bits >> PAGE_CACHE_SHIFT;

	if (icx->page) {
		unlock_page(icx->page);
		ntfs_unmap_page(icx->page);
		icx->page = NULL;
	}

	page = ntfs_map_page(ia_mapping, index);
	if (IS_ERR(page)) {
		ntfs_error(icx->ni->vol->sb, "Failed to map"
				" index page, error %ld.",
				-PTR_ERR(page));
		ret = PTR_ERR(page);
		return ret;
	}
	lock_page(page);
	icx->page = page;

	kaddr = (u8 *)page_address(page);

	ib = (INDEX_ALLOCATION *)(kaddr + ((ib_vcn <<
			icx->vcn_size_bits) & ~PAGE_CACHE_MASK));

	ib->magic = magic_INDX;
	ib->usa_ofs = cpu_to_le16(sizeof(INDEX_BLOCK));
	ib->usa_count = cpu_to_le16(ib_size / NTFS_BLOCK_SIZE + 1);
	/* Set USN to 1 */
	*(u16 *)((char *)ib + le16_to_cpu(ib->usa_ofs)) = cpu_to_le16(1);
	ib->lsn = cpu_to_le64(0);

	ib->index_block_vcn = cpu_to_sle64(ib_vcn);

	ib->index.entries_offset = cpu_to_le32((ih_size +
			le16_to_cpu(ib->usa_count) * 2 + 7) & ~7);
	ib->index.index_length = 0;
	ib->index.allocated_size = cpu_to_le32(ib_size -
					       (sizeof(INDEX_BLOCK) - ih_size));
	ib->index.flags = node_type;

	icx->ib = ib;

	return 0;
}

static int ntfs_ir_to_ib(ntfs_index_context *icx, VCN ib_vcn)
{
	INDEX_ENTRY *ie_last;
	char *ies_start, *ies_end;
	int i, ret;

	ntfs_debug("Entering\n");

	ret = ntfs_ib_alloc(icx, ib_vcn, LEAF_NODE);
	if (ret)
		return ret;

	ret = ntfs_ir_lookup2(icx);
	if (ret)
		return ret;

	ies_start = (char *)ntfs_ie_get_first(&icx->ir->index);
	ies_end   = (char *)ntfs_ie_get_end(&icx->ir->index);
	ie_last   = ntfs_ie_get_last((INDEX_ENTRY *)ies_start, ies_end);
	/*
	 * Copy all entries, including the termination entry
	 * as well, which can never have any data.
	 */
	i = (char *)ie_last - ies_start + le16_to_cpu(ie_last->length);
	memcpy(ntfs_ie_get_first(&icx->ib->index), ies_start, i);

	icx->ib->index.flags = icx->ir->index.flags;
	icx->ib->index.index_length = cpu_to_le32(i +
			le32_to_cpu(icx->ib->index.entries_offset));
	return 0;
}

static void ntfs_ir_nill(INDEX_ROOT *ir)
{
	INDEX_ENTRY *ie_last;
	char *ies_start, *ies_end;

	ntfs_debug("Entering\n");
	/*
	 * TODO: This function could be much simpler.
	 */
	ies_start = (char *)ntfs_ie_get_first(&ir->index);
	ies_end   = (char *)ntfs_ie_get_end(&ir->index);
	ie_last   = ntfs_ie_get_last((INDEX_ENTRY *)ies_start, ies_end);
	/*
	 * Move the index root termination entry forward
	 */
	if ((char *)ie_last > ies_start) {
		memmove(ies_start, (char *)ie_last,
				le16_to_cpu(ie_last->length));
		ie_last = (INDEX_ENTRY *)ies_start;
	}
}



static int ntfs_ia_add(ntfs_index_context *icx)
{
	int ret;
	ntfs_debug("Entering\n");

	ret = ntfs_ibm_add(icx);
	if (ret)
		return ret;

	ret = ntfs_attr_exist(icx->ni, AT_INDEX_ALLOCATION,
			icx->name, icx->name_len);
	if (ret < 0)
		return ret;
	else if (ret > 0)
		return 0;

	ret = ntfs_attr_add(icx->ni, AT_INDEX_ALLOCATION, icx->name,
			  icx->name_len, NULL, 0);
	if (ret) {
		ntfs_error(icx->ni->vol->sb,
				"Failed to add AT_INDEX_ALLOCATION");
		return ret;
	}

	NInoSetNonResident(icx->ni);
	return 0;
}

static int ntfs_ir_reparent(ntfs_index_context *icx)
{
	INDEX_ENTRY *ie;
	VCN new_ib_vcn;
	struct inode *indexroot_vi;
	ntfs_inode *ni = icx->ni;
	int ret = 0;

	ntfs_debug("Entering\n");

	ret = ntfs_ir_lookup2(icx);
	if (ret)
		goto out;

	if ((icx->ir->index.flags & NODE_MASK) == SMALL_INDEX) {
		ret = ntfs_ia_add(icx);
		if (ret)
			goto out;
	}

	new_ib_vcn = ntfs_ibm_get_free(icx);
	if (new_ib_vcn == -1)
		goto out;

	ret = ntfs_ir_lookup2(icx);
	if (ret)
		goto clear_bmp;

	ret = ntfs_ir_to_ib(icx, new_ib_vcn);
	if (ret) {
		ntfs_error(icx->ni->vol->sb, "Failed to move "
				"index root to index block");
		goto clear_bmp;
	}

	flush_dcache_page(icx->page);
	set_page_dirty(icx->page);

	ntfs_ir_nill(icx->ir);

	ie = ntfs_ie_get_first(&icx->ir->index);
	ie->flags |= INDEX_ENTRY_NODE;
	ie->length = cpu_to_le16(sizeof(INDEX_ENTRY_HEADER) + sizeof(VCN));

	icx->ir->index.flags = LARGE_INDEX;
	icx->ir->index.index_length = cpu_to_le32(
			le32_to_cpu(icx->ir->index.entries_offset)
			+ le16_to_cpu(ie->length));
	icx->ir->index.allocated_size = icx->ir->index.index_length;

	indexroot_vi = ntfs_attr_iget(VFS_I(ni), AT_INDEX_ROOT, I30, 4);
	if (IS_ERR(indexroot_vi)) {
		ntfs_error(ni->vol->sb, "Failed to get index root attribute.");
		ret = PTR_ERR(indexroot_vi);
		goto clear_bmp;
	}

	i_size_write(indexroot_vi, sizeof(INDEX_ROOT) - sizeof(INDEX_HEADER) +
			le32_to_cpu(icx->ir->index.allocated_size));
	ret = ntfs_truncate(indexroot_vi);
	iput(indexroot_vi);
	if (ret)
		goto clear_bmp;

	/*
	 *  FIXME: do it earlier if we have enough space in IR (should always),
	 *  so in error case we wouldn't lose the IB.
	 */
	ntfs_ie_set_vcn(ie, new_ib_vcn);
	ret = 0;
out:
	return ret;
clear_bmp:
	ntfs_ibm_clear(icx, new_ib_vcn);
	return ret;;
}



/**
 *  Insert @ie index entry at @pos entry. Used @ih values should be ok already.
 */
static void ntfs_ie_insert(INDEX_HEADER *ih, INDEX_ENTRY *ie, INDEX_ENTRY *pos)
{
	int ie_size = le16_to_cpu(ie->length);

	ntfs_debug("Entering\n");

	ih->index_length = cpu_to_le32(le32_to_cpu(ih->index_length) + ie_size);
	memmove((u8 *)pos + ie_size, pos,
			le32_to_cpu(ih->index_length) - ((u8 *)pos -
			(u8 *)ih) - ie_size);
	memcpy(pos, ie, ie_size);
}
#if 0
static INDEX_ENTRY *ntfs_ie_dup(INDEX_ENTRY *ie)
{
	INDEX_ENTRY *dup;

	ntfs_debug("Entering\n");

	dup = kmem_cache_alloc(ntfs_ie_cache, GFP_NOFS);
	if (dup)
		memcpy(dup, ie, le16_to_cpu(ie->length));

	return dup;
}
#endif
static INDEX_ENTRY *ntfs_ie_dup_novcn(INDEX_ENTRY *ie)
{
	INDEX_ENTRY *dup;
	int size = le16_to_cpu(ie->length);

	ntfs_debug("Entering\n");

	if (ie->flags & INDEX_ENTRY_NODE)
		size -= sizeof(VCN);

	dup = kmem_cache_alloc(ntfs_ie_cache, GFP_NOFS);
	if (dup) {
		memcpy(dup, ie, size);
		dup->flags &= ~INDEX_ENTRY_NODE;
		dup->length = cpu_to_le16(size);
	}
	return dup;
}
#endif
#if 0
/**
 * ntfs_index_ctx_get - allocate and initialize a new index context
 * @idx_ni:	ntfs index inode with which to initialize the context
 *
 * Allocate a new index context, initialize it with @idx_ni and return it.
 * Return NULL if allocation failed.
 *
 * Locking:  Caller must hold i_mutex on the index inode.
 */
ntfs_index_context *ntfs_index_ctx_get(ntfs_inode *idx_ni)
{
	ntfs_index_context *ictx;

	ictx = kmem_cache_alloc(ntfs_index_ctx_cache, GFP_NOFS);
	if (ictx)
		*ictx = (ntfs_index_context){ .idx_ni = idx_ni };
	return ictx;
}
#endif
/**
 * ntfs_index_ctx_get - allocate and initialize a new index context
 * @ni:		ntfs inode with which to initialize the context
 * @name:	name of the which context describes
 * @name_len:	length of the index name
 *
 * Allocate a new index context, initialize it with @ni and return it.
 * Return NULL if allocation failed.
 */
ntfs_index_context *ntfs_index_ctx_get(ntfs_inode *ni,
				       ntfschar *name, u32 name_len)
{
	ntfs_index_context *icx;

	ntfs_debug("Entering\n");

	if (ni->nr_extents == -1)
		ni = ni->ext.base_ntfs_ino;
	icx = kmem_cache_alloc(ntfs_index_ctx_cache, GFP_NOFS);
	if (icx)
		*icx = (ntfs_index_context) {
			.ni = ni,
			.name = name,
			.name_len = name_len,
		};
	return icx;
}
#if 0
/**
 * ntfs_index_ctx_put - release an index context
 * @ictx:	index context to free
 *
 * Release the index context @ictx, releasing all associated resources.
 *
 * Locking:  Caller must hold i_mutex on the index inode.
 */
void ntfs_index_ctx_put(ntfs_index_context *ictx)
{
	if (ictx->entry) {
		if (ictx->is_in_root) {
			if (ictx->actx)
				ntfs_attr_put_search_ctx(ictx->actx);
			if (ictx->base_ni)
				unmap_mft_record(ictx->base_ni);
		} else {
			struct page *page = ictx->page;
			if (page) {
				DEBUG_ON(!PageLocked(page));
				unlock_page(page);
				ntfs_unmap_page(page);
			}
		}
	}
	kmem_cache_free(ntfs_index_ctx_cache, ictx);
	return;
}
#endif

static void ntfs_index_ctx_free(ntfs_index_context *icx)
{
	struct page *page;
	ntfs_debug("Entering\n");

	if (!icx->entry)
		return;

	if (icx->actx)
		ntfs_attr_put_search_ctx(icx->actx);
	if (icx->mftmapped)
		unmap_mft_record(icx->ni);
		page = icx->page;
		if (page) {
			DEBUG_ON(!PageLocked(page));
			unlock_page(page);
			ntfs_unmap_page(page);
		}
}

/**
 * ntfs_index_ctx_put - release an index context
 * @icx:	index context to free
 *
 * Release the index context @icx, releasing all associated resources.
 */
void ntfs_index_ctx_put(ntfs_index_context *icx)
{
	ntfs_index_ctx_free(icx);
	kmem_cache_free(ntfs_index_ctx_cache, icx);
}

/**
 * ntfs_index_ctx_reinit - reinitialize an index context
 * @icx:	index context to reinitialize
 *
 * Reinitialize the index context @icx so it can be used for ntfs_index_lookup.
 */
void ntfs_index_ctx_reinit(ntfs_index_context *icx)
{
	ntfs_debug("Entering\n");

	ntfs_index_ctx_free(icx);

	*icx = (ntfs_index_context) {
		.ni = icx->ni,
		.name = icx->name,
		.name_len = icx->name_len,
	};
}

#ifdef NTFS_RW
/**
 * ntfs_ir_truncate - Truncate index root attribute
 *
 * Returns STATUS_OK, STATUS_RESIDENT_ATTRIBUTE_FILLED_MFT or STATUS_ERROR.
 */
static int ntfs_ir_truncate(ntfs_index_context *icx, int data_size)
{
	ntfs_inode *ni = icx->ni;
	int ret = 0;
	struct inode *indexroot_vi;

	ntfs_debug("Entering\n");

	/*
	 *  INDEX_ROOT must be resident and its entries can be moved to
	 *  INDEX_BLOCK, so ENOSPC isn't a real error.
	 */
	if (!ni)
		return -EINVAL;

	indexroot_vi = ntfs_attr_iget(VFS_I(ni), AT_INDEX_ROOT, I30, 4);
	if (IS_ERR(indexroot_vi)) {
		ntfs_error(ni->vol->sb, "Failed to get index root attribute.");
		ret = PTR_ERR(indexroot_vi);
		return ret;
	}


	indexroot_vi->i_size = data_size + offsetof(INDEX_ROOT, index);
	ret = ntfs_truncate(indexroot_vi);
	iput(indexroot_vi);
	if (!ret) {

		ret = ntfs_ir_lookup2(icx);
		if (ret)
			return ret;

		icx->ir->index.allocated_size = cpu_to_le32(data_size);

	} else if (ret < 0)
		ntfs_error(icx->ni->vol->sb, "Failed to truncate INDEX_ROOT");

	return ret;
}

static int ntfs_ih_takeout(ntfs_index_context *icx, INDEX_HEADER *ih,
			   INDEX_ENTRY *ie, struct page *page)
{
	INDEX_ENTRY *ie_roam;
	int ret = 0;

	ntfs_debug("Entering\n");
	ie_roam = ntfs_ie_dup_novcn(ie);
	if (!ie_roam)
		return -ENOMEM;

	ntfs_ie_delete(ih, ie);
	if (ntfs_icx_parent_vcn(icx) == VCN_INDEX_ROOT_PARENT) {
		flush_dcache_mft_record_page(icx->actx->ntfs_ino);
		mark_mft_record_dirty(icx->actx->ntfs_ino);
	} else {
		flush_dcache_page(page);
		set_page_dirty(page);
	}
	if (page && (!(icx->page) || icx->page->index != page->index))
		unlock_page(page);
	ntfs_index_ctx_reinit(icx);

	ret = ntfs_ie_add(icx, ie_roam);

	if (page && (!(icx->page) || icx->page->index != page->index))
		lock_page(page);
	kmem_cache_free(ntfs_ie_cache, ie_roam);
	return ret;
}

/**
 *  Used if an empty index block to be deleted has END entry as the parent
 *  in the INDEX_ROOT which is the only one there.
 */
static void ntfs_ir_leafify(ntfs_index_context *icx, INDEX_HEADER *ih)
{
	INDEX_ENTRY *ie;

	ntfs_debug("Entering\n");

	ie = ntfs_ie_get_first(ih);
	ie->flags &= ~INDEX_ENTRY_NODE;
	ie->length = cpu_to_le16(le16_to_cpu(ie->length) - sizeof(VCN));

	ih->index_length = cpu_to_le32(le32_to_cpu(ih->index_length)
			- sizeof(VCN));
	ih->flags &= ~LARGE_INDEX;

	/* Not fatal error */
	ntfs_ir_truncate(icx, le32_to_cpu(ih->index_length));
}

/**
 *  Used if an empty index block to be deleted has END entry as the parent
 *  in the INDEX_ROOT which is not the only one there.
 */
static int ntfs_ih_reparent_end(ntfs_index_context *icx, INDEX_HEADER *ih,
				struct page *page)
{
	INDEX_ENTRY *ie, *ie_prev;

	ntfs_debug("Entering\n");

	ie = ntfs_ie_get_by_pos(ih, ntfs_icx_parent_pos(icx));
	ie_prev = ntfs_ie_prev(ih, ie);

	ntfs_ie_set_vcn(ie, ntfs_ie_get_vcn(ie_prev));

	return ntfs_ih_takeout(icx, ih, ie_prev, page);
}

/**
 * ntfs_ir_make_space - Make more space for the index root attribute
 *
 * On success return STATUS_OK or STATUS_KEEP_SEARCHING.
 * On error return STATUS_ERROR.
 */
static int ntfs_ir_make_space(ntfs_index_context *icx, int data_size)
{
	int ret;
	ntfs_debug("Entering\n");

	ret = ntfs_ir_truncate(icx, data_size);

	if (ret == STATUS_RESIDENT_ATTRIBUTE_FILLED_MFT) {
		ret = ntfs_ir_reparent(icx);
		if (!ret)
			ret = STATUS_KEEP_SEARCHING;
		else
			ntfs_error(icx->ni->vol->sb,
					"Failed to nodify INDEX_ROOT");
	}
	return ret;
}


static void ntfs_ih_insert(INDEX_HEADER *ih, INDEX_ENTRY *orig_ie, VCN new_vcn,
			  int pos)
{
	INDEX_ENTRY *ie_node, *ie;
	VCN old_vcn;
	bool add_vcn = false;

	ntfs_debug("Entering\n");

	ie = orig_ie;

	if (!(ie->flags & INDEX_ENTRY_NODE)) {
		add_vcn = true;
		ntfs_ie_add_vcn(&ie);
	}

	ie_node = ntfs_ie_get_by_pos(ih, pos);
	old_vcn = ntfs_ie_get_vcn(ie_node);
	ntfs_ie_set_vcn(ie_node, new_vcn);

	ntfs_ie_insert(ih, ie, ie_node);
	ntfs_ie_set_vcn(ie_node, old_vcn);

	if (add_vcn)
		ntfs_ie_del_vcn(&ie);

	return;
}

#endif

/**
 * Find a key in the index block.
 *
 * Return values:
 *
 */
static int ntfs_ie_lookup(const void *key, const int key_len,
			  ntfs_index_context *icx, INDEX_HEADER *ih,
			  VCN *vcn, INDEX_ENTRY **ie_out)
{
	INDEX_ENTRY *ie;
	u8 *index_end;
	int rc, item = 0;
	struct super_block *sb = icx->ni->vol->sb;

	ntfs_debug("Entering\n");

	index_end = ntfs_ie_get_end(ih);

	/*
	 * Loop until we exceed valid memory (corruption case) or until we
	 * reach the last entry.
	 */
	for (ie = ntfs_ie_get_first(ih); ; ie = ntfs_ie_get_next(ie)) {
		/* Bounds checks. */
		if ((u8 *)ie + sizeof(INDEX_ENTRY_HEADER) > index_end ||
		    (u8 *)ie + le16_to_cpu(ie->length) > index_end) {
			ntfs_error(sb, "Index entry out of bounds in inode "
					"%llu.\n",
					(unsigned long long)icx->ni->mft_no);
			return -ERANGE;
		}
		/*
		 * The last entry cannot contain a key.  It can however contain
		 * a pointer to a child node in the B+tree so we just break out.
		 */
		if (ntfs_ie_end(ie))
			break;
		/*
		 * Not a perfect match, need to do full blown collation so we
		 * know which way in the B+tree we have to go.
		 */
		rc = ntfs_collate(icx->ni->vol, icx->cr, key, key_len, &ie->key,
				  le16_to_cpu(ie->key_length));
		/*
		 * If @key collates before the key of the current entry, there
		 * is definitely no such key in this index but we might need to
		 * descend into the B+tree so we just break out of the loop.
		 */
		if (rc == -1)
			break;

		if (!rc) {
			*ie_out = ie;
			icx->parent_pos[icx->pindex] = item;
			return 0;
		}

		item++;
	}
	/*
	 * We have finished with this index block without success. Check for the
	 * presence of a child node and if not present return with errno ENOENT,
	 * otherwise we will keep searching in another index block.
	 */
	if (!(ie->flags & INDEX_ENTRY_NODE)) {
		ntfs_debug("Index entry wasn't found.\n");
		*ie_out = ie;
		return -ENOENT;
	}

	/* Get the starting vcn of the index_block holding the child node. */
	*vcn = ntfs_ie_get_vcn(ie);
	if (*vcn < 0) {
		ntfs_error(sb, "Negative vcn in inode %llu",
				(unsigned long long)icx->ni->mft_no);
		return -EINVAL;
	}

	ntfs_debug("Parent entry number %d\n", item);
	icx->parent_pos[icx->pindex] = item;

	return STATUS_KEEP_SEARCHING;
}

static int ntfs_ia_check(ntfs_index_context *icx, INDEX_BLOCK *ib, VCN vcn)
{
	struct super_block *sb = icx->ni->vol->sb;
	u32 ib_size = (unsigned)le32_to_cpu(ib->index.allocated_size) + 0x18;

	ntfs_debug("Entering\n");

	if (!ntfs_is_indx_record(ib->magic)) {

		ntfs_error(sb, "Corrupt index block signature: vcn %lld inode "
			       "%llu\n", (long long)vcn,
			       (unsigned long long)icx->ni->mft_no);
		return -1;
	}

	if (sle64_to_cpu(ib->index_block_vcn) != vcn) {

		ntfs_error(sb, "Corrupt index block: VCN (%lld) is different "
			       "from expected VCN (%lld) in inode %llu\n",
			       (long long)sle64_to_cpu(ib->index_block_vcn),
			       (long long)vcn,
			       (unsigned long long)icx->ni->mft_no);
		return -1;
	}

	if (ib_size != icx->block_size) {

		ntfs_error(sb, "Corrupt index block : VCN (%lld) of inode %llu "
			       "has a size (%u) differing from the index "
			       "specified size (%u)\n", (long long)vcn,
			       (unsigned long long)icx->ni->mft_no, ib_size,
			       icx->block_size);
		return -1;
	}
	return 0;
}



static INDEX_ROOT *ntfs_ir_lookup(ntfs_inode *ni, ntfschar *name,
		u32 name_len, MFT_RECORD *m, ntfs_attr_search_ctx **ctx)
{
	ATTR_RECORD *a;
	INDEX_ROOT *ir = NULL;
	struct super_block *sb = ni->vol->sb;
	int err;

	if (*ctx)
		ntfs_attr_reinit_search_ctx(*ctx);
	else {
		*ctx = ntfs_attr_get_search_ctx(ni, m);
		if (unlikely(!(*ctx)))
			return NULL;
	}
	/* Find the index root attribute in the mft record. */
	err = ntfs_attr_lookup(AT_INDEX_ROOT, name, name_len,
			CASE_SENSITIVE, 0, NULL, 0, *ctx);
	if (unlikely(err)) {
		if (err == -ENOENT) {
			ntfs_error(sb, "Index root attribute missing in inode "
					"0x%lx.", ni->mft_no);
		}
		goto  err_out;
	}

	a = (*ctx)->attr;

	if (a->non_resident) {
		ntfs_error(sb, "Non-resident $INDEX_ROOT detected");
		goto err_out;
	}

	/* Get to the index root value (it has been verified in read_inode). */
	ir = (INDEX_ROOT *)((char *)a +
			le16_to_cpu(a->data.resident.value_offset));

err_out:
	if (!ir) {
		ntfs_attr_put_search_ctx(*ctx);
		*ctx = NULL;
	}
	return ir;
}

#ifdef NTFS_RW
static int ntfs_ir_lookup2(ntfs_index_context *ictx)
{
	ictx->ir = ntfs_ir_lookup(ictx->ni, ictx->name,
			ictx->name_len, NULL, &(ictx->actx));
	if (!ictx->ir)
		return -EIO;

	return 0;
}
#endif

/**
 * ntfs_index_lookup - find a key in an index and return its index entry
 * @key:	[IN] key for which to search in the index
 * @key_len:	[IN] length of @key in bytes
 * @ictx:	[IN/OUT] context describing the index and the returned entry
 *
 * Before calling ntfs_index_lookup(), @ictx must have been obtained from a
 * call to ntfs_index_ctx_get().
 *
 * Look for the @key in the index specified by the index lookup context @ictx.
 * ntfs_index_lookup() walks the contents of the index looking for the @key.
 *
 * If the @key is found in the index, 0 is returned and @ictx is setup to
 * describe the index entry containing the matching @key.  @ictx->entry is the
 * index entry and @ictx->data and @ictx->data_len are the index entry data and
 * its length in bytes, respectively.
 *
 * If the @key is not found in the index, -ENOENT is returned and @ictx is
 * setup to describe the index entry whose key collates immediately after the
 * search @key, i.e. this is the position in the index at which an index entry
 * with a key of @key would need to be inserted.
 *
 * If an error occurs return the negative error code and @ictx is left
 * untouched.
 *
 * When finished with the entry and its data, call ntfs_index_ctx_put() to free
 * the context and other associated resources.
 *
 * If the index entry was modified, call flush_dcache_index_entry_page()
 * immediately after the modification and either ntfs_index_entry_mark_dirty()
 * or ntfs_index_entry_write() before the call to ntfs_index_ctx_put() to
 * ensure that the changes are written to disk.
 *
 * Locking:  - Caller must hold i_mutex on the index inode.
 *	     - Each page cache page in the index allocation mapping must be
 *	       locked whilst being accessed otherwise we may find a corrupt
 *	       page due to it being under ->writepage at the moment which
 *	       applies the mst protection fixups before writing out and then
 *	       removes them again after the write is complete after which it
 *	       unlocks the page.
 */
int ntfs_index_lookup(const void *key, const int key_len,
		ntfs_index_context *ictx)
{
	VCN vcn, old_vcn;
	ntfs_inode *ni = ictx->ni;
	ntfs_volume *vol = ni->vol;
	struct super_block *sb = vol->sb;
	MFT_RECORD *m;
	INDEX_ROOT *ir;
	INDEX_ENTRY *ie;
	INDEX_BLOCK *ib;
	u8 *kaddr;
	struct address_space *ia_mapping;
	struct page *page;
	int err = 0;

	ntfs_debug("Entering.");

	if (!key || key_len <= 0) {
		ntfs_error(sb, "key: %p  key_len: %d", key, key_len);
		return -EINVAL;
	}

	/* Get hold of the mft record for the index inode. */
	m = map_mft_record(ni);
	if (IS_ERR(m)) {
		ntfs_error(sb, "map_mft_record() failed with error code %ld.",
				-PTR_ERR(m));
		return PTR_ERR(m);
	}
	ictx->mftmapped = 1;

	ir = ntfs_ir_lookup(ni, ictx->name, ictx->name_len, m, &ictx->actx);
	if (!ir)
		return -EIO;

	ictx->block_size = le32_to_cpu(ir->index_block_size);
	if (ictx->block_size < NTFS_BLOCK_SIZE) {
		err = -EINVAL;
		ntfs_error(sb, "Index block size (%d) is smaller than the "
				"sector size (%d)", ictx->block_size,
				NTFS_BLOCK_SIZE);
		goto err_out;
	}

	ictx->block_size_bit = ffs(ictx->block_size) - 1;

	if (ni->vol->cluster_size <= ictx->block_size)
		ictx->vcn_size_bits = ni->vol->cluster_size_bits;
	else
		ictx->vcn_size_bits = ni->vol->sector_size_bits;

	ictx->cr = ir->collation_rule;
	if (!ntfs_is_collation_rule_supported(ictx->cr)) {
		err = -EOPNOTSUPP;
		ntfs_error(sb, "Unknown collation rule 0x%x",
				(unsigned)le32_to_cpu(ictx->cr));
		goto err_out;
	}

	old_vcn = VCN_INDEX_ROOT_PARENT;

	err = ntfs_ie_lookup(key, key_len, ictx, &ir->index, &vcn, &ie);
	if ((err < 0) && (err != -ENOENT))
		goto err_out;
	ictx->ir = ir;

	if (err != STATUS_KEEP_SEARCHING) {
		/* STATUS_OK or STATUS_NOT_FOUND */
		ictx->is_in_root = true;
		ictx->parent_vcn[ictx->pindex] = old_vcn;
		goto done;
	}

	ia_mapping = VFS_I(ni)->i_mapping;

descend_into_child_node:

	page = ntfs_map_page(ia_mapping, vcn <<
			ictx->vcn_size_bits >> PAGE_CACHE_SHIFT);
	if (IS_ERR(page)) {
		ntfs_error(sb, "Failed to map index page, error %ld.",
				-PTR_ERR(page));
		err = PTR_ERR(page);
		goto err_out;
	}
	lock_page(page);
	kaddr = (u8 *)page_address(page);

fast_descend_into_child_node:
	ictx->parent_vcn[ictx->pindex] = old_vcn;
	err = ntfs_icx_parent_inc(ictx);
	if (err)
		goto err_out;
	old_vcn = vcn;

	ib = (INDEX_ALLOCATION *)(kaddr + ((vcn <<
			ictx->vcn_size_bits) & ~PAGE_CACHE_MASK));
	if (ntfs_ia_check(ictx, ib, vcn))
		goto err_out;

	err = ntfs_ie_lookup(key, key_len, ictx, &ib->index, &vcn, &ie);
	if (err != STATUS_KEEP_SEARCHING) {
		if ((err < 0) && (err != -ENOENT))
			goto err_out;

		/* STATUS_OK or STATUS_NOT_FOUND */
		ictx->is_in_root = false;
		ictx->page = page;
		ictx->ib = ib;
		ictx->parent_vcn[ictx->pindex] = vcn;
		goto done;
	}

	if ((ib->index.flags & NODE_MASK) == LEAF_NODE) {
		ntfs_error(sb, "Index entry with child node found in a leaf "
			       "node in inode 0x%llx.\n",
			       (unsigned long long)ni->mft_no);
		goto err_out;
	}

	if (old_vcn << ictx->vcn_size_bits>>
			PAGE_CACHE_SHIFT == vcn <<
			ictx->vcn_size_bits >>
			PAGE_CACHE_SHIFT)
		goto fast_descend_into_child_node;
	unlock_page(page);
	ntfs_unmap_page(page);
	goto descend_into_child_node;

err_out:
	return -EIO;
done:
	ictx->entry = ie;
	ictx->data = (u8 *)ie + offsetof(INDEX_ENTRY, key);
	ictx->data_len = le16_to_cpu(ie->key_length);
	ntfs_debug("Done.\n");

	return err;
}

#ifdef NTFS_RW
int ntfs_ie_add(ntfs_index_context *icx, INDEX_ENTRY *ie)
{
	INDEX_HEADER *ih;
	int allocated_size, new_size;
	struct super_block *sb = icx->ni->vol->sb;
	int ret = 0;

#ifdef DEBUG
/* removed by JPA to make function usable for security indexes
	char *fn;
	fn = ntfs_ie_filename_get(ie);
	ntfs_log_trace("file: '%s'\n", fn);
	ntfs_attr_name_free(&fn);
*/
#endif

	while (1) {
		ret = ntfs_index_lookup(&ie->key,
				le16_to_cpu(ie->key_length), icx);
		if (!ret) {
			ret = -EEXIST;
			ntfs_error(sb, "Index already have such entry");
			goto err_out;
		}
		if (ret != -ENOENT) {
			ntfs_error(sb, "Failed to find place for new entry");
			goto err_out;
		}

		if (icx->is_in_root)
			ih = &icx->ir->index;
		else
			ih = &icx->ib->index;

		allocated_size = le32_to_cpu(ih->allocated_size);
		new_size = le32_to_cpu(ih->index_length) +
				le16_to_cpu(ie->length);

		if (new_size <= allocated_size)
			break;

		ntfs_debug("index block sizes: allocated: %d  needed: %d\n",
			       allocated_size, new_size);

		if (icx->is_in_root) {
			ret = ntfs_ir_make_space(icx, new_size);
			if (ret < 0)
				goto err_out;
		} else {
			ret = ntfs_ib_split(icx, icx->ib);
			if (ret < 0)
				goto err_out;
		}

		flush_dcache_mft_record_page(icx->actx->ntfs_ino);
		mark_mft_record_dirty(icx->actx->ntfs_ino);
		ntfs_index_ctx_reinit(icx);
	}
	ret = 0;
	ntfs_ie_insert(ih, ie, icx->entry);
	ntfs_index_entry_mark_dirty(icx);

err_out:
	ntfs_debug("%s\n", ret ? "Failed" : "Done");
	return ret;
}

/**
 * ntfs_index_add_filename - add filename to directory index
 * @ni:		ntfs inode describing directory to which index add filename
 * @fn:		FILE_NAME attribute to add
 * @mref:	reference of the inode which @fn describes
 *
 * Return 0 on success or -1 on error with errno set to the error code.
 */
int ntfs_index_add_filename(ntfs_inode *ni, FILE_NAME_ATTR *fn, MFT_REF mref)
{
	INDEX_ENTRY *ie;
	ntfs_index_context *icx;
	struct super_block *sb = ni->vol->sb;
	int fn_size, ie_size, ret = 0;

	ntfs_debug("Entering\n");

	if (!ni || !fn) {
		ntfs_error(sb, "Invalid arguments.\n");
		return -EINVAL;
	}

	fn_size = (fn->file_name_length * sizeof(ntfschar)) +
			sizeof(FILE_NAME_ATTR);
	ie_size = (sizeof(INDEX_ENTRY_HEADER) + fn_size + 7) & ~7;

	ie = ntfs_malloc_nofs(ie_size);
	if (!ie)
		return -ENOMEM;

	ie->data.dir.indexed_file = cpu_to_le64(mref);
	ie->length 	 = cpu_to_le16(ie_size);
	ie->key_length 	 = cpu_to_le16(fn_size);
	ie->flags = 0;
	ie->reserved = 0;
	memcpy(&ie->key, fn, fn_size);

	icx = ntfs_index_ctx_get(ni, I30, 4);
	if (!icx) {
		ret = -ENOMEM;
		goto out;
	}
	ret = ntfs_ie_add(icx, ie);
	ntfs_index_ctx_put(icx);
out:
	ntfs_free(ie);
	return ret;
}

static int ntfs_index_rm_leaf(ntfs_index_context *icx)
{
	ntfs_inode *ni = icx->ni;
	ntfs_volume *vol = ni->vol;
	struct super_block *sb = vol->sb;
	INDEX_BLOCK *ia = NULL;
	INDEX_HEADER *parent_ih;
	INDEX_ENTRY *ie;
	VCN vcn;
	struct page *page = NULL;
	struct address_space *ia_mapping;
	unsigned long index;
	u8 *kaddr;
	int ret = 0;
	int newpage = 0;

	ntfs_debug("pindex: %d\n", icx->pindex);
	ret = ntfs_icx_parent_dec(icx);
	if (ret)
		return ret;
	ret = ntfs_ibm_clear(icx, icx->parent_vcn[icx->pindex + 1]);
	if (ret)
		return ret;
	vcn = ntfs_icx_parent_vcn(icx);
	if (vcn == VCN_INDEX_ROOT_PARENT)
		parent_ih = &icx->ir->index;
	else {
		ia_mapping = VFS_I(ni)->i_mapping;
		index = vcn << icx->vcn_size_bits >> PAGE_CACHE_SHIFT;

		if (icx->page && icx->page->index == index) {
			page_cache_get(icx->page);
			page = icx->page;
			kmap(page);
		} else {
			page = ntfs_map_page(ia_mapping, index);
			if (IS_ERR(page)) {
				ntfs_error(sb, "Failed to map index page,"
					" error %ld.", -PTR_ERR(page));
				ret = PTR_ERR(page);
				goto out;
			}
			lock_page(page);
		}
		newpage = 1;

		kaddr = (u8 *)page_address(page);
		ia = (INDEX_ALLOCATION *)(kaddr + ((vcn <<
				icx->vcn_size_bits) & ~PAGE_CACHE_MASK));
		/* Catch multi sector transfer fixup errors. */
		if (ntfs_ia_check(icx, ia, vcn)) {
			ret = -EIO;
			goto out;
		}
		parent_ih = &ia->index;
	}

	ie = ntfs_ie_get_by_pos(parent_ih, ntfs_icx_parent_pos(icx));
	if (!ntfs_ie_end(ie)) {
		ret = ntfs_ih_takeout(icx, parent_ih, ie, page);
		goto out;
	}

	if (ntfs_ih_zero_entry(parent_ih)) {

		if (ntfs_icx_parent_vcn(icx) == VCN_INDEX_ROOT_PARENT) {
			ntfs_ir_leafify(icx, parent_ih);
			goto ok;
		}

		if (page && (!(icx->page) || icx->page->index != page->index))
			unlock_page(page);
		ret = ntfs_index_rm_leaf(icx);
		if (page && (!(icx->page) || icx->page->index != page->index))
			lock_page(page);
		goto out;
	}

	ret = ntfs_ih_reparent_end(icx, parent_ih, page);
	if (ret)
		goto out;
ok:
	ret = 0;
out:
	if (newpage) {
		if (!(icx->page) || icx->page->index != page->index)
			unlock_page(page);
		ntfs_unmap_page(page);
	}
	return ret;
}

static int ntfs_index_rm_node(ntfs_index_context *icx)
{
	int entry_pos, pindex;
	VCN vcn, old_vcn;
	INDEX_ALLOCATION *ia = NULL;
	INDEX_ENTRY *ie_succ, *entry = icx->entry;
	INDEX_HEADER *ih;
	ntfs_inode *ni = icx->ni;
	ntfs_volume *vol = ni->vol;
	struct super_block *sb = vol->sb;
	struct address_space *ia_mapping;
	u8 *kaddr;
	struct page *page;
	u32 new_size;
	unsigned long index, next_index;
	int delta, ret = 0;
	ntfs_debug("Entering\n");

	if (!icx->actx)
		return -EIO;

	ie_succ = ntfs_ie_get_next(icx->entry);
	entry_pos = icx->parent_pos[icx->pindex]++;
	pindex = icx->pindex;

	vcn = ntfs_ie_get_vcn(ie_succ);
	ia_mapping = VFS_I(ni)->i_mapping;

	index = vcn << icx->vcn_size_bits >> PAGE_CACHE_SHIFT;

descend:
	if (icx->page && icx->page->index == index) {
		page_cache_get(icx->page);
		page = icx->page;
		kmap(page);
	} else {
		page = ntfs_map_page(ia_mapping, index);
		if (IS_ERR(page)) {
			ntfs_error(sb, "Failed to map index page, error %ld.",
					-PTR_ERR(page));
			ret = PTR_ERR(page);
			goto out;
		}
		lock_page(page);
	}
	kaddr = (u8 *)page_address(page);

descend_fast:
	ia = (INDEX_ALLOCATION *)(kaddr + ((vcn <<
			icx->vcn_size_bits) & ~PAGE_CACHE_MASK));
	/* Catch multi sector transfer fixup errors. */
	if (ntfs_ia_check(icx, ia, vcn))
		goto out;

	ie_succ = ntfs_ie_get_first(&ia->index);

	if (ntfs_icx_parent_inc(icx))
		goto out;

	icx->parent_vcn[icx->pindex] = vcn;
	icx->parent_pos[icx->pindex] = 0;

	if ((ia->index.flags & NODE_MASK) == INDEX_NODE) {
		vcn = ntfs_ie_get_vcn(ie_succ);
		next_index = vcn << icx->vcn_size_bits >> PAGE_CACHE_SHIFT;
		if (next_index == index)
			goto descend_fast;

		if (!(icx->page) || icx->page->index != page->index)
			unlock_page(page);

		ntfs_unmap_page(page);

		index = next_index;
		goto descend;
	}

	if (ntfs_ih_zero_entry(&ia->index)) {
		ret = -EIO;
		ntfs_error(sb, "Empty index block");
		goto out;
	}

	ntfs_ie_add_vcn(&ie_succ);

	old_vcn = ntfs_ie_get_vcn(icx->entry);
	if (icx->is_in_root)
		ih = &icx->ir->index;
	else
		ih = &icx->ib->index;

	delta = le16_to_cpu(ie_succ->length) - le16_to_cpu(icx->entry->length);
	new_size = le32_to_cpu(ih->index_length) + delta;
	if (delta > 0) {
		if (icx->is_in_root) {

			if (!(icx->page) || icx->page->index != page->index)
				unlock_page(page);
			ret = ntfs_ir_make_space(icx, new_size);
			if (!(icx->page) || icx->page->index != page->index)
				lock_page(page);
			if (ret)
				goto out2;

			ih = &icx->ir->index;
			entry = ntfs_ie_get_by_pos(ih, entry_pos);

		} else if (new_size > le32_to_cpu(ih->allocated_size)) {
			icx->pindex = pindex;
			if (!(icx->page) || icx->page->index != page->index)
				unlock_page(page);
			ret = ntfs_ib_split(icx, icx->ib);
			if (!(icx->page) || icx->page->index != page->index)
				lock_page(page);
			if (!ret)
				ret = STATUS_KEEP_SEARCHING;
			goto out2;
		}
	}

	ntfs_ie_delete(ih, entry);
	ntfs_ie_insert(ih, ie_succ, entry);
	ntfs_ie_set_vcn(entry, old_vcn);

	if (icx->is_in_root) {
		ret = ntfs_ir_truncate(icx, new_size);
		if (ret)
			goto out2;
	} else {
		flush_dcache_page(icx->page);
		set_page_dirty(icx->page);
	}
	ntfs_ie_del_vcn(&ie_succ);
	ntfs_ie_delete(&ia->index, ie_succ);
	if (ntfs_ih_zero_entry(&ia->index)) {
		if (!(icx->page) || icx->page->index != page->index)
			unlock_page(page);
		ret = ntfs_index_rm_leaf(icx);
		if (!(icx->page) || icx->page->index != page->index)
			lock_page(page);
		if (ret)
			goto out2;
	} else {
		flush_dcache_page(page);
		set_page_dirty(page);
	}
out2:
out:
	if (!(icx->page) || icx->page->index != page->index)
		unlock_page(page);
	ntfs_unmap_page(page);

	return ret;
}

/**
 * ntfs_index_rm - remove entry from the index
 * @icx:	index context describing entry to delete
 *
 * Delete entry described by @icx from the index. Index context is always
 * reinitialized after use of this function, so it can be used for index
 * lookup once again.
 *
 * Return 0 on success or -1 on error with errno set to the error code.
 */
/*static JPA*/
int ntfs_index_rm(ntfs_index_context *icx)
{
	INDEX_HEADER *ih;
	int ret = 0;

	ntfs_debug("Entering\n");

	if (!icx || (!icx->ib && !icx->ir) || ntfs_ie_end(icx->entry)) {
		printk(KERN_ERR"Invalid arguments.\n");
		ret = -EINVAL;
		goto out;
	}
	if (icx->is_in_root)
		ih = &icx->ir->index;
	else
		ih = &icx->ib->index;

	if (icx->entry->flags & INDEX_ENTRY_NODE) {
		ret = ntfs_index_rm_node(icx);

	} else if (icx->is_in_root || !ntfs_ih_one_entry(ih)) {

		ntfs_ie_delete(ih, icx->entry);

		if (icx->is_in_root) {

			ret = ntfs_ir_truncate(icx,
				le32_to_cpu(ih->index_length));
			if (!ret)
				goto out;
		} else {
			flush_dcache_page(icx->page);
			set_page_dirty(icx->page);
		}
	} else {
		ret = ntfs_index_rm_leaf(icx);
		if (ret)
			goto out;
	}
out:
	return ret;
}

int ntfs_index_remove(ntfs_inode *dir_ni, ntfs_inode *ni,
		const void *key, const int keylen)
{
	int ret = 0;
	ntfs_index_context *icx;

	icx = ntfs_index_ctx_get(dir_ni, I30, 4);
	if (!icx)
		return -ENOMEM;

	while (1) {
		ret = ntfs_index_lookup(key, keylen, icx);
		if (ret)
			goto err_out;
		if ((((FILE_NAME_ATTR *)icx->data)->file_attributes &
				FILE_ATTR_REPARSE_POINT)
		   && !ntfs_possible_symlink(ni)) {
			ret = -EOPNOTSUPP;
			goto err_out;
		}

		ret = ntfs_index_rm(icx);

		if (ret < 0)
			goto err_out;
		else if (ret == 0)
			break;
		flush_dcache_mft_record_page(icx->ni);
		mark_mft_record_dirty(icx->ni);
		ntfs_index_ctx_reinit(icx);

	}

		flush_dcache_mft_record_page(icx->ni);
		mark_mft_record_dirty(icx->ni);
out:
	ntfs_index_ctx_put(icx);
	return ret;
err_out:
	ntfs_error(icx->ni->vol->sb, "Delete failed");
	goto out;
}
#endif

