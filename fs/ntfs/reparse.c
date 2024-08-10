

#include "reparse.h"
#include "layout.h"
#include "volume.h"
#include "types.h"
#include "runlist.h"
#include "debug.h"
#include "inode.h"
#include "attrib.h"
#include "mft.h"
#include "aops.h"


#define IO_REPARSE_TAG_DFS         const_cpu_to_le32(0x8000000A)
#define IO_REPARSE_TAG_DFSR        const_cpu_to_le32(0x80000012)
#define IO_REPARSE_TAG_HSM         const_cpu_to_le32(0xC0000004)
#define IO_REPARSE_TAG_HSM2        const_cpu_to_le32(0x80000006)
#define IO_REPARSE_TAG_MOUNT_POINT const_cpu_to_le32(0xA0000003)
#define IO_REPARSE_TAG_SIS         const_cpu_to_le32(0x80000007)
#define IO_REPARSE_TAG_SYMLINK     const_cpu_to_le32(0xA000000C)

/*
 *		Check whether a reparse point looks like a junction point
 *	or a symbolic link.
 *	Should only be called for files or directories with reparse data
 *
 *	The validity of the target is not checked.
 */

int ntfs_possible_symlink(ntfs_inode *ni)
{
	REPARSE_POINT *reparse_attr;
	struct inode *reparse_vi;
	struct page *page;
	ntfs_attr_search_ctx *ctx = NULL;
	MFT_RECORD *mrec = NULL;
	int ret = 0;

	mrec = map_mft_record(ni);
	if (IS_ERR(mrec)) {
		ntfs_error(ni->vol->sb, "map_mft_record() failed with error"
			" code %ld.", -PTR_ERR(mrec));
		ret = PTR_ERR(mrec);
		goto out;
	}
	ctx = ntfs_attr_get_search_ctx(ni, mrec);
	if (unlikely(!ctx)) {
		ret = -ENOMEM;
		goto out;
	}
	/* Find the index root attribute in the mft record. */
	ret = ntfs_attr_lookup(AT_REPARSE_POINT, NULL, 0, 0, 0, NULL,
			0, ctx);
	if (unlikely(ret)) {
		if (ret == -ENOENT) {
			ntfs_debug("no symlink point "
					"directory inode 0x%lx.",
					ni->mft_no);
			ret = 0;
		}
		goto out;
	}

	ntfs_attr_put_search_ctx(ctx);
	unmap_mft_record(ni);
	ctx = NULL;
	mrec = NULL;

	reparse_vi = ntfs_attr_iget(VFS_I(ni), AT_REPARSE_POINT, NULL, 0);
	if (IS_ERR(reparse_vi)) {
		ntfs_error(ni->vol->sb, "Failed to get attribute inode.");
		ret = PTR_ERR(reparse_vi);
		goto out;
	}

	page = ntfs_map_page(reparse_vi->i_mapping, 0);
	if (IS_ERR(page)) {
		ntfs_error(ni->vol->sb, "Failed to map directory index"
			" page, error %ld.", -PTR_ERR(page));
		ret = PTR_ERR(page);
		goto out;
	}

	reparse_attr = (REPARSE_POINT *)page_address(page);
	switch (reparse_attr->reparse_tag) {
	case IO_REPARSE_TAG_MOUNT_POINT:
	case IO_REPARSE_TAG_SYMLINK:
		ret = 1;
	default:
		;
	}

	ntfs_unmap_page(page);

out:
	if (ctx)
		ntfs_attr_put_search_ctx(ctx);
	if (mrec)
		unmap_mft_record(ni);
	return ret;
}

/*
 *		Delete a reparse index entry
 *
 *	Returns 0 if success
 *		-1 if failure, explained by errno
 */
#if 0
int ntfs_delete_reparse_index(ntfs_inode *ni)
{
	ntfs_index_context *xr;
	ntfs_inode *xrni;
	ntfs_attr *na;
	le32 reparse_tag;
	int res;

	res = 0;
	na = ntfs_attr_open(ni, AT_REPARSE_POINT, AT_UNNAMED, 0);
	if (na) {
			/*
			 * read the existing reparse data (the tag is enough)
			 * and un-index it
			 */
		xr = open_reparse_index(ni->vol);
		if (xr) {
			if (remove_reparse_index(na, xr, &reparse_tag) < 0)
				res = -1;
			xrni = xr->ni;
			ntfs_index_entry_mark_dirty(xr);
			NInoSetDirty(xrni);
			ntfs_index_ctx_put(xr);
			ntfs_inode_close(xrni);
		}
		ntfs_attr_close(na);
	}
	return res;
}
#endif

