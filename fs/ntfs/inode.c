/**
 * inode.c - NTFS kernel inode handling. Part of the Linux-NTFS project.
 *
 * Copyright (c) 2001-2007 Anton Altaparmakov
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

#include <linux/buffer_head.h>
#include <linux/security.h>
#include <linux/fsnotify.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/mount.h>
#include <linux/mutex.h>
#include <linux/pagemap.h>
#include <linux/quotaops.h>
#include <linux/slab.h>

#include "aops.h"
#include "attrib.h"
#include "bitmap.h"
#include "dir.h"
#include "debug.h"
#include "inode.h"
#include "lcnalloc.h"
#include "malloc.h"
#include "mft.h"
#include "time.h"
#include "ntfs.h"
#include "attrlist.h"
#include <linux/msdos_fs.h>

ntfs_inode *ntfs_inode_base(ntfs_inode *ni)
{
	if (ni->nr_extents == -1)
		return ni->ext.base_ntfs_ino;
	return ni;
}

/**
 * ntfs_test_inode - compare two (possibly fake) inodes for equality
 * @vi:		vfs inode which to test
 * @na:		ntfs attribute which is being tested with
 *
 * Compare the ntfs attribute embedded in the ntfs specific part of the vfs
 * inode @vi for equality with the ntfs attribute @na.
 *
 * If searching for the normal file/directory inode, set @na->type to AT_UNUSED.
 * @na->name and @na->name_len are then ignored.
 *
 * Return 1 if the attributes match and 0 if not.
 *
 * NOTE: This function runs with the inode_lock spin lock held so it is not
 * allowed to sleep.
 */
int ntfs_test_inode(struct inode *vi, ntfs_attr *na)
{
	ntfs_inode *ni;

	if (vi->i_ino != na->mft_no)
		return 0;
	ni = NTFS_I(vi);
	/* If !NInoAttr(ni), @vi is a normal file or directory inode. */
	if (likely(!NInoAttr(ni))) {
		/* If not looking for a normal inode this is a mismatch. */
		if (unlikely(na->type != AT_UNUSED))
			return 0;
	} else {
		/* A fake inode describing an attribute. */
		if (ni->type != na->type)
			return 0;
		if (ni->name_len != na->name_len)
			return 0;
		if (na->name_len && memcmp(ni->name, na->name,
				na->name_len * sizeof(ntfschar)))
			return 0;
	}
	/* Match! */
	return 1;
}

int __ntfs_test_real_inode(ntfs_inode *ni, ATTR_TYPE type,
			ntfschar *name, u32 name_len)
{
	if (type != ni->type)
		return 0;
	if (ni->name_len != name_len)
		return 0;
	if (ni->name_len && memcmp(ni->name, name,
				ni->name_len * sizeof(ntfschar)))
		return 0;
	return 1;
}

int ntfs_test_real_inode(ntfs_inode *ni, ATTR_RECORD *attr)
{

	if (attr->type != ni->type)
		return 0;
	if (ni->name_len != attr->name_length)
		return 0;
	if (ni->name_len && memcmp(ni->name, attr + attr->name_offset,
				ni->name_len * sizeof(ntfschar)))
		return 0;
	return 1;
}


/**
 * ntfs_init_locked_inode - initialize an inode
 * @vi:		vfs inode to initialize
 * @na:		ntfs attribute which to initialize @vi to
 *
 * Initialize the vfs inode @vi with the values from the ntfs attribute @na in
 * order to enable ntfs_test_inode() to do its work.
 *
 * If initializing the normal file/directory inode, set @na->type to AT_UNUSED.
 * In that case, @na->name and @na->name_len should be set to NULL and 0,
 * respectively. Although that is not strictly necessary as
 * ntfs_read_locked_inode() will fill them in later.
 *
 * Return 0 on success and -errno on error.
 *
 * NOTE: This function runs with the inode_lock spin lock held so it is not
 * allowed to sleep. (Hence the GFP_ATOMIC allocation.)
 */
static int ntfs_init_locked_inode(struct inode *vi, ntfs_attr *na)
{
	ntfs_inode *ni = NTFS_I(vi);

	vi->i_ino = na->mft_no;

	ni->type = na->type;
	if (na->type == AT_INDEX_ALLOCATION)
		NInoSetMstProtected(ni);

	ni->name = na->name;
	ni->name_len = na->name_len;

	/* If initializing a normal inode, we are done. */
	if (likely(na->type == AT_UNUSED)) {
		DEBUG_ON(na->name);
		DEBUG_ON(na->name_len);
		return 0;
	}

	/* It is a fake inode. */
	NInoSetAttr(ni);

	/*
	 * We have I30 global constant as an optimization as it is the name
	 * in >99.9% of named attributes! The other <0.1% incur a GFP_ATOMIC
	 * allocation but that is ok. And most attributes are unnamed anyway,
	 * thus the fraction of named attributes with name != I30 is actually
	 * absolutely tiny.
	 */
	if (na->name_len && na->name != I30) {
		unsigned int i;

		DEBUG_ON(!na->name);
		i = na->name_len * sizeof(ntfschar);
		ni->name = kmalloc(i + sizeof(ntfschar), GFP_ATOMIC);
		if (!ni->name)
			return -ENOMEM;
		memcpy(ni->name, na->name, i);
		ni->name[na->name_len] = 0;
	}
	return 0;
}

typedef int (*set_t)(struct inode *, void *);
static int ntfs_read_locked_inode(struct inode *vi);
int ntfs_read_locked_attr_inode(struct inode *base_vi, struct inode *vi);
static int ntfs_read_locked_index_inode(struct inode *base_vi,
		struct inode *vi);

/**
 * ntfs_iget - obtain a struct inode corresponding to a specific normal inode
 * @sb:		super block of mounted volume
 * @mft_no:	mft record number / inode number to obtain
 *
 * Obtain the struct inode corresponding to a specific normal inode (i.e. a
 * file or directory).
 *
 * If the inode is in the cache, it is just returned with an increased
 * reference count. Otherwise, a new struct inode is allocated and initialized,
 * and finally ntfs_read_locked_inode() is called to read in the inode and
 * fill in the remainder of the inode structure.
 *
 * Return the struct inode on success. Check the return value with IS_ERR() and
 * if true, the function failed and the error code is obtained from PTR_ERR().
 */
struct inode *ntfs_iget(struct super_block *sb, unsigned long mft_no)
{
	struct inode *vi;
	int err;
	ntfs_attr na;

	na.mft_no = mft_no;
	na.type = AT_UNUSED;
	na.name = NULL;
	na.name_len = 0;

	vi = iget5_locked(sb, mft_no, (test_t)ntfs_test_inode,
			(set_t)ntfs_init_locked_inode, &na);
	if (unlikely(!vi))
		return ERR_PTR(-ENOMEM);

	err = 0;

	/* If this is a freshly allocated inode, need to read it now. */
	if (vi->i_state & I_NEW) {
		err = ntfs_read_locked_inode(vi);
		unlock_new_inode(vi);
	}
	/*
	 * There is no point in keeping bad inodes around if the failure was
	 * due to ENOMEM. We want to be able to retry again later.
	 */
	if (unlikely(err == -ENOMEM)) {
		iput(vi);
		vi = ERR_PTR(err);
	}
	return vi;
}

/**
 * ntfs_attr_iget - obtain a struct inode corresponding to an attribute
 * @base_vi:	vfs base inode containing the attribute
 * @type:	attribute type
 * @name:	Unicode name of the attribute (NULL if unnamed)
 * @name_len:	length of @name in Unicode characters (0 if unnamed)
 *
 * Obtain the (fake) struct inode corresponding to the attribute specified by
 * @type, @name, and @name_len, which is present in the base mft record
 * specified by the vfs inode @base_vi.
 *
 * If the attribute inode is in the cache, it is just returned with an
 * increased reference count. Otherwise, a new struct inode is allocated and
 * initialized, and finally ntfs_read_locked_attr_inode() is called to read the
 * attribute and fill in the inode structure.
 *
 * Note, for index allocation attributes, you need to use ntfs_index_iget()
 * instead of ntfs_attr_iget() as working with indices is a lot more complex.
 *
 * Return the struct inode of the attribute inode on success. Check the return
 * value with IS_ERR() and if true, the function failed and the error code is
 * obtained from PTR_ERR().
 */
struct inode *ntfs_attr_iget(struct inode *base_vi, ATTR_TYPE type,
		ntfschar *name, u32 name_len)
{
	struct inode *vi;
	int err;
	ntfs_attr na;

	/* Make sure no one calls ntfs_attr_iget() for indices. */
	DEBUG_ON(type == AT_INDEX_ALLOCATION);

	na.mft_no = base_vi->i_ino;
	na.type = type;
	na.name = name;
	na.name_len = name_len;

	vi = iget5_locked(base_vi->i_sb, na.mft_no, (test_t)ntfs_test_inode,
			(set_t)ntfs_init_locked_inode, &na);
	if (unlikely(!vi))
		return ERR_PTR(-ENOMEM);

	err = 0;

	/* If this is a freshly allocated inode, need to read it now. */
	if (vi->i_state & I_NEW) {
		err = ntfs_read_locked_attr_inode(base_vi, vi);
		unlock_new_inode(vi);
	}
	/*
	 * There is no point in keeping bad attribute inodes around. This also
	 * simplifies things in that we never need to check for bad attribute
	 * inodes elsewhere.
	 */
	if (unlikely(err)) {
		iput(vi);
		vi = ERR_PTR(err);
	}
	return vi;
}

/**
 * ntfs_index_iget - obtain a struct inode corresponding to an index
 * @base_vi:	vfs base inode containing the index related attributes
 * @name:	Unicode name of the index
 * @name_len:	length of @name in Unicode characters
 *
 * Obtain the (fake) struct inode corresponding to the index specified by @name
 * and @name_len, which is present in the base mft record specified by the vfs
 * inode @base_vi.
 *
 * If the index inode is in the cache, it is just returned with an increased
 * reference count.  Otherwise, a new struct inode is allocated and
 * initialized, and finally ntfs_read_locked_index_inode() is called to read
 * the index related attributes and fill in the inode structure.
 *
 * Return the struct inode of the index inode on success. Check the return
 * value with IS_ERR() and if true, the function failed and the error code is
 * obtained from PTR_ERR().
 */
struct inode *ntfs_index_iget(struct inode *base_vi, ntfschar *name,
		u32 name_len)
{
	struct inode *vi;
	int err;
	ntfs_attr na;

	na.mft_no = base_vi->i_ino;
	na.type = AT_INDEX_ALLOCATION;
	na.name = name;
	na.name_len = name_len;

	vi = iget5_locked(base_vi->i_sb, na.mft_no, (test_t)ntfs_test_inode,
			(set_t)ntfs_init_locked_inode, &na);
	if (unlikely(!vi))
		return ERR_PTR(-ENOMEM);

	err = 0;

	/* If this is a freshly allocated inode, need to read it now. */
	if (vi->i_state & I_NEW) {
		err = ntfs_read_locked_index_inode(base_vi, vi);
		unlock_new_inode(vi);
	}
	/*
	 * There is no point in keeping bad index inodes around.  This also
	 * simplifies things in that we never need to check for bad index
	 * inodes elsewhere.
	 */
	if (unlikely(err)) {
		iput(vi);
		vi = ERR_PTR(err);
	}
	return vi;
}

struct inode *ntfs_alloc_big_inode(struct super_block *sb)
{
	ntfs_inode *ni;

	ntfs_debug("Entering.");
	ni = kmem_cache_alloc(ntfs_big_inode_cache, GFP_NOFS);
	if (likely(ni != NULL)) {
		ni->state = 0;
		return VFS_I(ni);
	}
	ntfs_error(sb, "Allocation of NTFS big inode structure failed.");
	return NULL;
}

void ntfs_destroy_big_inode(struct inode *inode)
{
	ntfs_inode *ni = NTFS_I(inode);

	ntfs_debug("Entering.");
	DEBUG_ON(ni->page);
	if (!atomic_dec_and_test(&ni->count))
		BUG();
	kmem_cache_free(ntfs_big_inode_cache, NTFS_I(inode));
}

static inline ntfs_inode *ntfs_alloc_extent_inode(void)
{
	ntfs_inode *ni;

	ntfs_debug("Entering.");
	ni = kmem_cache_alloc(ntfs_inode_cache, GFP_NOFS);
	if (likely(ni != NULL)) {
		ni->state = 0;
		return ni;
	}
	ntfs_error(NULL, "Allocation of NTFS inode structure failed.");
	return NULL;
}

static void ntfs_destroy_extent_inode(ntfs_inode *ni)
{
	ntfs_debug("Entering.");
	//DEBUG_ON(ni->page);
	if (!atomic_dec_and_test(&ni->count))
	{
		ntfs_error(NULL, "ntfs_destroy_extent_inode erro %d", __LINE__);
		//BUG();
	}
	kmem_cache_free(ntfs_inode_cache, ni);
}

/*
 * The attribute runlist lock has separate locking rules from the
 * normal runlist lock, so split the two lock-classes:
 */
static struct lock_class_key attr_list_rl_lock_class;

/**
 * __ntfs_init_inode - initialize ntfs specific part of an inode
 * @sb:		super block of mounted volume
 * @ni:		freshly allocated ntfs inode which to initialize
 *
 * Initialize an ntfs inode to defaults.
 *
 * NOTE: ni->mft_no, ni->state, ni->type, ni->name, and ni->name_len are left
 * untouched. Make sure to initialize them elsewhere.
 *
 * Return zero on success and -ENOMEM on error.
 */
void __ntfs_init_inode(struct super_block *sb, ntfs_inode *ni)
{
	ntfs_debug("Entering.");
	rwlock_init(&ni->size_lock);
	ni->initialized_size = ni->allocated_size = 0;
	ni->seq_no = 0;
	ni->flag = cpu_to_le32(0) ;
	atomic_set(&ni->count, 1);
	atomic_set(&ni->map_count, 0);
	ni->vol = NTFS_SB(sb);
	ntfs_init_runlist(&ni->runlist);
	mutex_init(&ni->mrec_lock);
	ni->page = NULL;
	ni->page_ofs = 0;
	ni->attr_list_size = 0;
	ni->attr_list = NULL;
	ntfs_init_runlist(&ni->attr_list_rl);
	lockdep_set_class(&ni->attr_list_rl.lock,
				&attr_list_rl_lock_class);
	ni->itype.index.block_size = 0;
	ni->itype.index.vcn_size = 0;
	ni->itype.index.collation_rule = 0;
	ni->itype.index.block_size_bits = 0;
	ni->itype.index.vcn_size_bits = 0;
	mutex_init(&ni->extent_lock);
	ni->nr_extents = 0;
	ni->ext.base_ntfs_ino = NULL;
}

/*
 * Extent inodes get MFT-mapped in a nested way, while the base inode
 * is still mapped. Teach this nesting to the lock validator by creating
 * a separate class for nested inode's mrec_lock's:
 */
static struct lock_class_key extent_inode_mrec_lock_key;

inline ntfs_inode *ntfs_new_extent_inode(struct super_block *sb,
		unsigned long mft_no)
{
	ntfs_inode *ni = ntfs_alloc_extent_inode();

	ntfs_debug("Entering.");
	if (likely(ni != NULL)) {
		__ntfs_init_inode(sb, ni);
		lockdep_set_class(&ni->mrec_lock, &extent_inode_mrec_lock_key);
		ni->mft_no = mft_no;
		ni->type = AT_UNUSED;
		ni->name = NULL;
		ni->name_len = 0;
	}
	return ni;
}

/**
 * ntfs_is_extended_system_file - check if a file is in the $Extend directory
 * @ctx:	initialized attribute search context
 *
 * Search all file name attributes in the inode described by the attribute
 * search context @ctx and check if any of the names are in the $Extend system
 * directory.
 *
 * Return values:
 *	   1: file is in $Extend directory
 *	   0: file is not in $Extend directory
 *    -errno: failed to determine if the file is in the $Extend directory
 */
static int ntfs_is_extended_system_file(ntfs_attr_search_ctx *ctx)
{
	int nr_links, err;

	/* Restart search. */
	ntfs_attr_reinit_search_ctx(ctx);

	/* Get number of hard links. */
	nr_links = VFS_I(ctx->ntfs_ino)->i_nlink;
	/* Loop through all hard links. */
	while (!(err = ntfs_attr_lookup(AT_FILE_NAME, NULL, 0, 0, 0, NULL, 0,
			ctx))) {
		FILE_NAME_ATTR *file_name_attr;
		ATTR_RECORD *attr = ctx->attr;
		u8 *p, *p2;

		nr_links--;
		/*
		 * Maximum sanity checking as we are called on an inode that
		 * we suspect might be corrupt.
		 */
		p = (u8 *)attr + le32_to_cpu(attr->length);
		if (p < (u8 *)ctx->mrec || (u8 *)p > (u8 *)ctx->mrec +
				le32_to_cpu(ctx->mrec->bytes_in_use)) {
err_corrupt_attr:
			ntfs_error(ctx->ntfs_ino->vol->sb, "Corrupt file name "
					"attribute. You should run chkdsk.");
			return -EIO;
		}
		if (attr->non_resident) {
			ntfs_error(ctx->ntfs_ino->vol->sb, "Non-resident file "
					"name. You should run chkdsk.");
			return -EIO;
		}
		if (attr->flags) {
			ntfs_error(ctx->ntfs_ino->vol->sb, "File name with "
					"invalid flags. You should run "
					"chkdsk.");
			return -EIO;
		}
		if (!(attr->data.resident.flags & RESIDENT_ATTR_IS_INDEXED)) {
			ntfs_error(ctx->ntfs_ino->vol->sb, "Unindexed file "
					"name. You should run chkdsk.");
			return -EIO;
		}
		file_name_attr = (FILE_NAME_ATTR *)((u8 *)attr +
				le16_to_cpu(attr->data.resident.value_offset));
		p2 = (u8 *)attr + le32_to_cpu(attr->data.resident.value_length);
		if (p2 < (u8 *)attr || p2 > p)
			goto err_corrupt_attr;
		/* This attribute is ok, but is it in the $Extend directory? */
		if (MREF_LE(file_name_attr->parent_directory) == FILE_Extend)
			return 1;	/* YES, it's an extended system file. */
	}
	if (unlikely(err != -ENOENT))
		return err;
	if (unlikely(nr_links)) {
		ntfs_error(ctx->ntfs_ino->vol->sb, "Inode hard link count "
				"doesn't match number of name attributes. You "
				"should run chkdsk.");
		return -EIO;
	}
	return 0;	/* NO, it is not an extended system file. */
}

/**
 * ntfs_read_locked_inode - read an inode from its device
 * @vi:		inode to read
 *
 * ntfs_read_locked_inode() is called from ntfs_iget() to read the inode
 * described by @vi into memory from the device.
 *
 * The only fields in @vi that we need to/can look at when the function is
 * called are i_sb, pointing to the mounted device's super block, and i_ino,
 * the number of the inode to load.
 *
 * ntfs_read_locked_inode() maps, pins and locks the mft record number i_ino
 * for reading and sets up the necessary @vi fields as well as initializing
 * the ntfs inode.
 *
 * Q: What locks are held when the function is called?
 * A: i_state has I_LOCK set, hence the inode is locked, also
 *    i_count is set to 1, so it is not going to go away
 *    i_flags is set to 0 and we have no business touching it.  Only an ioctl()
 *    is allowed to write to them. We should of course be honouring them but
 *    we need to do that using the IS_* macros defined in include/linux/fs.h.
 *    In any case ntfs_read_locked_inode() has nothing to do with i_flags.
 *
 * Return 0 on success and -errno on error.  In the error case, the inode will
 * have had make_bad_inode() executed on it.
 */
static int ntfs_read_locked_inode(struct inode *vi)
{
	ntfs_volume *vol = NTFS_SB(vi->i_sb);
	ntfs_inode *ni;
	struct inode *bvi;
	MFT_RECORD *m;
	ATTR_RECORD *a;
	STANDARD_INFORMATION *si;
	ntfs_attr_search_ctx *ctx;
	int err = 0;
	bool small_index = false;

	ntfs_debug("Entering for i_ino 0x%lx.", vi->i_ino);

	/* Setup the generic vfs inode parts now. */

	/*
	 * This is for checking whether an inode has changed w.r.t. a file so
	 * that the file can be updated if necessary (compare with f_version).
	 */
	vi->i_version = 1;

	vi->i_uid = vol->uid;
	vi->i_gid = vol->gid;
	vi->i_mode = 0;

	/*
	 * Initialize the ntfs specific part of @vi special casing
	 * FILE_MFT which we need to do at mount time.
	 */
	if (vi->i_ino != FILE_MFT)
		ntfs_init_big_inode(vi);
	ni = NTFS_I(vi);
	m = map_mft_record(ni);
	if (IS_ERR(m)) {
		err = PTR_ERR(m);
		goto err_out;
	}
	ctx = ntfs_attr_get_search_ctx(ni, m);
	if (!ctx) {
		err = -ENOMEM;
		goto unm_err_out;
	}

	if (!(m->flags & MFT_RECORD_IN_USE)) {
		ntfs_error(vi->i_sb, "Inode is not in use!");
		goto unm_err_out;
	}
	if (m->base_mft_record) {
		ntfs_error(vi->i_sb, "Inode is an extent inode!");
		goto unm_err_out;
	}

	/* Transfer information from mft record into vfs and ntfs inodes. */
	vi->i_generation = ni->seq_no = le16_to_cpu(m->sequence_number);

	/*
	 * FIXME: Keep in mind that link_count is two for files which have both
	 * a long file name and a short file name as separate entries, so if
	 * we are hiding short file names this will be too high. Either we need
	 * to account for the short file names by subtracting them or we need
	 * to make sure we delete files even though i_nlink is not zero which
	 * might be tricky due to vfs interactions. Need to think about this
	 * some more when implementing the unlink command.
	 */
	vi->__i_nlink = le16_to_cpu(m->link_count);
	/*
	 * FIXME: Reparse points can have the directory bit set even though
	 * they would be S_IFLNK. Need to deal with this further below when we
	 * implement reparse points / symbolic links but it will do for now.
	 * Also if not a directory, it could be something else, rather than
	 * a regular file. But again, will do for now.
	 */
	/* Everyone gets all permissions. */
/* TODO: get permiss security*/
	vi->i_mode |= S_IRWXUGO;
	/* If read-only, noone gets write permissions. */
	if (IS_RDONLY(vi))
		vi->i_mode &= ~S_IWUGO;
	if (m->flags & MFT_RECORD_IS_DIRECTORY) {
		vi->i_mode |= S_IFDIR;
		/*
		 * Apply the directory permissions mask set in the mount
		 * options.
		 */
		vi->i_mode &= ~vol->dmask;
		/* Things break without this kludge! */
		if (vi->i_nlink > 1)
			vi->__i_nlink = 1;
	} else {
		vi->i_mode |= S_IFREG;
		/* Apply the file permissions mask set in the mount options. */
		vi->i_mode &= ~vol->fmask;
	}
	/*
	 * Find the standard information attribute in the mft record. At this
	 * stage we haven't setup the attribute list stuff yet, so this could
	 * in fact fail if the standard information is in an extent record, but
	 * I don't think this actually ever happens.
	 */
	err = ntfs_attr_lookup(AT_STANDARD_INFORMATION, NULL, 0, 0, 0, NULL, 0,
			ctx);
	if (unlikely(err)) {
		if (err == -ENOENT) {
			/*
			 * TODO: We should be performing a hot fix here (if the
			 * recover mount option is set) by creating a new
			 * attribute.
			 */
			ntfs_error(vi->i_sb, "$STANDARD_INFORMATION attribute "
					"is missing.");
		}
		goto unm_err_out;
	}
	a = ctx->attr;
	/* Get the standard information attribute value. */
	si = (STANDARD_INFORMATION *)((u8 *)a +
			le16_to_cpu(a->data.resident.value_offset));

	/* Transfer information from the standard information into vi. */
	/*
	 * Note: The i_?times do not quite map perfectly onto the NTFS times,
	 * but they are close enough, and in the end it doesn't really matter
	 * that much...
	 */
	/*
	 * mtime is the last change of the data within the file. Not changed
	 * when only metadata is changed, e.g. a rename doesn't affect mtime.
	 */
	vi->i_mtime = ntfs2utc(si->last_data_change_time);
	/*
	 * ctime is the last change of the metadata of the file. This obviously
	 * always changes, when mtime is changed. ctime can be changed on its
	 * own, mtime is then not changed, e.g. when a file is renamed.
	 */
	vi->i_ctime = ntfs2utc(si->last_mft_change_time);
	/*
	 * Last access to the data within the file. Not changed during a rename
	 * for example but changed whenever the file is written to.
	 */
	vi->i_atime = ntfs2utc(si->last_access_time);

	ni->flag = si->file_attributes;

	/* Find the attribute list attribute if present. */
	ntfs_attr_reinit_search_ctx(ctx);
	err = ntfs_attr_lookup(AT_ATTRIBUTE_LIST, NULL, 0, 0, 0, NULL, 0, ctx);
	if (err) {
		if (unlikely(err != -ENOENT)) {
			ntfs_error(vi->i_sb, "Failed to lookup attribute list "
					"attribute.");
			goto unm_err_out;
		}
	} else /* if (!err) */ {
		if (vi->i_ino == FILE_MFT)
			goto skip_attr_list_load;
		ntfs_debug("Attribute list found in inode 0x%lx.", vi->i_ino);
		NInoSetAttrList(ni);
		a = ctx->attr;
		if (a->flags & ATTR_COMPRESSION_MASK) {
			ntfs_error(vi->i_sb, "Attribute list attribute is "
					"compressed.");
			goto unm_err_out;
		}
		if (a->flags & ATTR_IS_ENCRYPTED ||
				a->flags & ATTR_IS_SPARSE) {
			if (a->non_resident) {
				ntfs_error(vi->i_sb, "Non-resident attribute "
						"list attribute is encrypted/"
						"sparse.");
				goto unm_err_out;
			}
			ntfs_warning(vi->i_sb, "Resident attribute list "
					"attribute in inode 0x%lx is marked "
					"encrypted/sparse which is not true.  "
					"However, Windows allows this and "
					"chkdsk does not detect or correct it "
					"so we will just ignore the invalid "
					"flags and pretend they are not set.",
					vi->i_ino);
		}
		/* Now allocate memory for the attribute list. */
		ni->attr_list_size = (u32)ntfs_attr_size(a);
		ni->attr_list = ntfs_malloc_nofs(ni->attr_list_size);
		if (!ni->attr_list) {
			ntfs_error(vi->i_sb, "Not enough memory to allocate "
					"buffer for attribute list.");
			err = -ENOMEM;
			goto unm_err_out;
		}
		if (a->non_resident) {
			NInoSetAttrListNonResident(ni);
			if (a->data.non_resident.lowest_vcn) {
				ntfs_error(vi->i_sb, "Attribute list has non "
						"zero lowest_vcn.");
				goto unm_err_out;
			}
			/*
			 * Setup the runlist. No need for locking as we have
			 * exclusive access to the inode at this time.
			 */
			ni->attr_list_rl.rl = ntfs_mapping_pairs_decompress(vol,
					a, NULL);
			if (IS_ERR(ni->attr_list_rl.rl)) {
				err = PTR_ERR(ni->attr_list_rl.rl);
				ni->attr_list_rl.rl = NULL;
				ntfs_error(vi->i_sb, "Mapping pairs "
						"decompression failed.");
				goto unm_err_out;
			}
			/* Now load the attribute list. */
			err = load_attribute_list(vol, &ni->attr_list_rl,
					ni->attr_list, ni->attr_list_size,
					sle64_to_cpu(a->data.non_resident.
					initialized_size));
			if (err) {
				ntfs_error(vi->i_sb, "Failed to load "
						"attribute list attribute.");
				goto unm_err_out;
			}
		} else /* if (!a->non_resident) */ {
			if ((u8 *)a + le16_to_cpu(a->data.resident.value_offset)
					+ le32_to_cpu(
					a->data.resident.value_length) >
					(u8 *)ctx->mrec +
					vol->mft_record_size) {
				ntfs_error(vi->i_sb, "Corrupt attribute list "
						"in inode.");
				goto unm_err_out;
			}
			/* Now copy the attribute list. */
			memcpy(ni->attr_list, (u8 *)a + le16_to_cpu(
					a->data.resident.value_offset),
					le32_to_cpu(
					a->data.resident.value_length));
		}
	}
skip_attr_list_load:
	/*
	 * If an attribute list is present we now have the attribute list value
	 * in ntfs_ino->attr_list and it is ntfs_ino->attr_list_size bytes.
	 */
	if (S_ISDIR(vi->i_mode)) {
		loff_t bvi_size;
		ntfs_inode *bni;
		INDEX_ROOT *ir;
		u8 *ir_end, *index_end;

		/* It is a directory, find index root attribute. */
		ntfs_attr_reinit_search_ctx(ctx);
		err = ntfs_attr_lookup(AT_INDEX_ROOT, I30, 4, CASE_SENSITIVE,
				0, NULL, 0, ctx);
		if (unlikely(err)) {
			if (err == -ENOENT) {
				/* FIXME: File is corrupt! Hot-fix with empty
				     index root attribute if recovery option is
				     set.*/
				ntfs_error(vi->i_sb, "$INDEX_ROOT attribute "
						"is missing.");
			}
			goto unm_err_out;
		}
		a = ctx->attr;
		/* Set up the state. */
		if (unlikely(a->non_resident)) {
			ntfs_error(vol->sb, "$INDEX_ROOT attribute is not "
					"resident.");
			goto unm_err_out;
		}
		/* Ensure the attribute name is placed before the value. */
		if (unlikely(a->name_length && (le16_to_cpu(a->name_offset) >=
				le16_to_cpu(a->data.resident.value_offset)))) {
			ntfs_error(vol->sb, "$INDEX_ROOT attribute name is "
					"placed after the attribute value.");
			goto unm_err_out;
		}
		/*
		 * Compressed/encrypted index root just means that the newly
		 * created files in that directory should be created compressed/
		 * encrypted. However index root cannot be both compressed and
		 * encrypted.
		 */
		if (a->flags & ATTR_COMPRESSION_MASK) {
			printk("Ignore the compression flag in index root\n");
//			NInoSetCompressed(ni);
		}
		if (a->flags & ATTR_IS_ENCRYPTED) {
			if (a->flags & ATTR_COMPRESSION_MASK) {
				ntfs_error(vi->i_sb, "Found encrypted and "
						"compressed attribute.");
				goto unm_err_out;
			}
			NInoSetEncrypted(ni);
		}
		if (a->flags & ATTR_IS_SPARSE)
			NInoSetSparse(ni);
		ir = (INDEX_ROOT *)((u8 *)a +
				le16_to_cpu(a->data.resident.value_offset));
		ir_end = (u8 *)ir + le32_to_cpu(a->data.resident.value_length);
		if (ir_end > (u8 *)ctx->mrec + vol->mft_record_size) {
			ntfs_error(vi->i_sb, "$INDEX_ROOT attribute is "
					"corrupt.");
			goto unm_err_out;
		}
		index_end = (u8 *)&ir->index +
				le32_to_cpu(ir->index.index_length);
		if (index_end > ir_end) {
			ntfs_error(vi->i_sb, "Directory index is corrupt.");
			goto unm_err_out;
		}
		if (ir->type != AT_FILE_NAME) {
			ntfs_error(vi->i_sb, "Indexed attribute is not "
					"$FILE_NAME.");
			goto unm_err_out;
		}
		if (ir->collation_rule != COLLATION_FILE_NAME) {
			ntfs_error(vi->i_sb, "Index collation rule is not "
					"COLLATION_FILE_NAME.");
			goto unm_err_out;
		}
		ni->itype.index.collation_rule = ir->collation_rule;
		ni->itype.index.block_size = le32_to_cpu(ir->index_block_size);
		if (ni->itype.index.block_size &
				(ni->itype.index.block_size - 1)) {
			ntfs_error(vi->i_sb, "Index block size (%u) is not a "
					"power of two.",
					ni->itype.index.block_size);
			goto unm_err_out;
		}
		if (ni->itype.index.block_size > PAGE_CACHE_SIZE) {
			ntfs_error(vi->i_sb, "Index block size (%u) > "
					"PAGE_CACHE_SIZE (%ld) is not "
					"supported.  Sorry.",
					ni->itype.index.block_size,
					PAGE_CACHE_SIZE);
			err = -EOPNOTSUPP;
			goto unm_err_out;
		}
		if (ni->itype.index.block_size < NTFS_BLOCK_SIZE) {
			ntfs_error(vi->i_sb, "Index block size (%u) < "
					"NTFS_BLOCK_SIZE (%i) is not "
					"supported.  Sorry.",
					ni->itype.index.block_size,
					NTFS_BLOCK_SIZE);
			err = -EOPNOTSUPP;
			goto unm_err_out;
		}
		ni->itype.index.block_size_bits =
				ffs(ni->itype.index.block_size) - 1;
		/* Determine the size of a vcn in the directory index. */
		if (vol->cluster_size <= ni->itype.index.block_size) {
			ni->itype.index.vcn_size = vol->cluster_size;
			ni->itype.index.vcn_size_bits = vol->cluster_size_bits;
		} else {
			ni->itype.index.vcn_size = vol->sector_size;
			ni->itype.index.vcn_size_bits = vol->sector_size_bits;
		}

		/* Setup the index allocation attribute, even if not present. */
		NInoSetMstProtected(ni);
		ni->type = AT_INDEX_ALLOCATION;
		ni->name = I30;
		ni->name_len = 4;

		if (!(ir->index.flags & LARGE_INDEX)) {
			/* No index allocation. */
			vi->i_size = ni->initialized_size =
					ni->allocated_size = 0;
			small_index = true;
			/* We are done with the mft record, so we release it. */
		} /* LARGE_INDEX: Index allocation present. Setup state. */

		/* Find index allocation attribute. */
		ntfs_attr_reinit_search_ctx(ctx);
		err = ntfs_attr_lookup(AT_INDEX_ALLOCATION, I30, 4,
				CASE_SENSITIVE, 0, NULL, 0, ctx);
		if (unlikely(err)) {
			if (err == -ENOENT) {
				if (small_index) {
					ntfs_attr_put_search_ctx(ctx);
					unmap_mft_record(ni);
					m = NULL;
					ctx = NULL;
					goto skip_large_dir_stuff;
				} else
					ntfs_error(vi->i_sb,
						"$INDEX_ALLOCATION "
						"attribute is not present but "
						"$INDEX_ROOT indicated it is.");
			} else
				ntfs_error(vi->i_sb, "Failed to lookup "
						"$INDEX_ALLOCATION "
						"attribute.");
			goto unm_err_out;
		}
		NInoSetIndexAllocPresent(ni);
		a = ctx->attr;
		if (!a->non_resident) {
			ntfs_error(vi->i_sb, "$INDEX_ALLOCATION attribute "
					"is resident.");
			goto unm_err_out;
		}
		/*
		 * Ensure the attribute name is placed before the mapping pairs
		 * array.
		 */
		if (unlikely(a->name_length && (le16_to_cpu(a->name_offset) >=
				le16_to_cpu(
				a->data.non_resident.mapping_pairs_offset)))) {
			ntfs_error(vol->sb, "$INDEX_ALLOCATION attribute name "
					"is placed after the mapping pairs "
					"array.");
			goto unm_err_out;
		}
		if (a->flags & ATTR_IS_ENCRYPTED) {
			ntfs_error(vi->i_sb, "$INDEX_ALLOCATION attribute "
					"is encrypted.");
			goto unm_err_out;
		}
		if (a->flags & ATTR_IS_SPARSE) {
			ntfs_error(vi->i_sb, "$INDEX_ALLOCATION attribute "
					"is sparse.");
			goto unm_err_out;
		}
		if (a->flags & ATTR_COMPRESSION_MASK) {
			ntfs_error(vi->i_sb, "$INDEX_ALLOCATION attribute "
					"is compressed.");
			goto unm_err_out;
		}
		if (a->data.non_resident.lowest_vcn) {
			ntfs_error(vi->i_sb, "First extent of "
					"$INDEX_ALLOCATION attribute has non "
					"zero lowest_vcn.");
			goto unm_err_out;
		}
		vi->i_size = sle64_to_cpu(a->data.non_resident.data_size);
		ni->initialized_size = sle64_to_cpu(
				a->data.non_resident.initialized_size);
		ni->allocated_size = sle64_to_cpu(
				a->data.non_resident.allocated_size);
		/*
		 * We are done with the mft record, so we release it. Otherwise
		 * we would deadlock in ntfs_attr_iget().
		 */
		ntfs_attr_put_search_ctx(ctx);
		unmap_mft_record(ni);
		m = NULL;
		ctx = NULL;


		/* Get the index bitmap attribute inode. */
		bvi = ntfs_attr_iget(vi, AT_BITMAP, I30, 4);
		if (IS_ERR(bvi)) {
			ntfs_error(vi->i_sb, "Failed to get bitmap attribute.");
			err = PTR_ERR(bvi);
			goto unm_err_out;
		}
		bni = NTFS_I(bvi);
		if (NInoCompressed(bni) || NInoEncrypted(bni) ||
				NInoSparse(bni)) {
			ntfs_error(vi->i_sb, "$BITMAP attribute is compressed "
					"and/or encrypted and/or sparse.");
			goto iput_unm_err_out;
		}
		/* Consistency check bitmap size vs. index allocation size. */
		bvi_size = i_size_read(bvi);
		if ((bvi_size << 3) < (vi->i_size >>
				ni->itype.index.block_size_bits)) {
			ntfs_error(vi->i_sb, "Index bitmap too small (0x%llx) "
					"for index allocation (0x%llx).",
					bvi_size << 3, vi->i_size);
			goto iput_unm_err_out;
		}
		/* No longer need the bitmap attribute inode. */
		iput(bvi);
skip_large_dir_stuff:
		/* Setup the operations for this inode. */
		vi->i_op = &ntfs_dir_inode_ops;
		vi->i_fop = &ntfs_dir_ops;
	} else {
		/* It is a file. */
		ntfs_attr_reinit_search_ctx(ctx);

		/* Setup the data attribute, even if not present. */
		ni->type = AT_DATA;
		ni->name = NULL;
		ni->name_len = 0;

		/* Find first extent of the unnamed data attribute. */
		err = ntfs_attr_lookup(AT_DATA, NULL, 0, 0, 0, NULL, 0, ctx);
		if (unlikely(err)) {
			vi->i_size = ni->initialized_size =
					ni->allocated_size = 0;
			if (err != -ENOENT) {
				ntfs_error(vi->i_sb, "Failed to lookup $DATA "
						"attribute.");
				goto unm_err_out;
			}
			/*
			 * FILE_Secure does not have an unnamed $DATA
			 * attribute, so we special case it here.
			 */
			if (vi->i_ino == FILE_Secure)
				goto no_data_attr_special_case;
			/*
			 * Most if not all the system files in the $Extend
			 * system directory do not have unnamed data
			 * attributes so we need to check if the parent
			 * directory of the file is FILE_Extend and if it is
			 * ignore this error. To do this we need to get the
			 * name of this inode from the mft record as the name
			 * contains the back reference to the parent directory.
			 */
			if (ntfs_is_extended_system_file(ctx) > 0)
				goto no_data_attr_special_case;
			/* FIXME: File is corrupt! Hot-fix with empty data
			    attribute if recovery option is set.*/
			ntfs_error(vi->i_sb, "$DATA attribute is missing.");
			goto unm_err_out;
		}
		a = ctx->attr;
		/* Setup the state. */
		if (a->flags & (ATTR_COMPRESSION_MASK | ATTR_IS_SPARSE)) {
			if (a->flags & ATTR_COMPRESSION_MASK) {
				NInoSetCompressed(ni);
				if (vol->cluster_size > 4096) {
					ntfs_error(vi->i_sb, "Found "
							"compressed data but "
							"compression is "
							"disabled due to "
							"cluster size (%i) > "
							"4kiB.",
							vol->cluster_size);
					goto unm_err_out;
				}
				if ((a->flags & ATTR_COMPRESSION_MASK)
						!= ATTR_IS_COMPRESSED) {
					ntfs_error(vi->i_sb, "Found unknown "
							"compression method "
							"or corrupt file.");
					goto unm_err_out;
				}
			}
			if (a->flags & ATTR_IS_SPARSE)
				NInoSetSparse(ni);
		}
		if (a->flags & ATTR_IS_ENCRYPTED) {
			if (NInoCompressed(ni)) {
				ntfs_error(vi->i_sb, "Found encrypted and "
						"compressed data.");
				goto unm_err_out;
			}
			NInoSetEncrypted(ni);
		}
		if (a->non_resident) {
			NInoSetNonResident(ni);
			if (NInoCompressed(ni) || NInoSparse(ni)) {
				if (NInoCompressed(ni) && a->data.non_resident.
						compression_unit != 4) {
					ntfs_error(vi->i_sb, "Found "
							"non-standard "
							"compression unit (%u "
							"instead of 4).  "
							"Cannot handle this.",
							a->data.non_resident.
							compression_unit);
					err = -EOPNOTSUPP;
					goto unm_err_out;
				}
				if (a->data.non_resident.compression_unit) {
					ni->itype.compressed.block_size = 1U <<
							(a->data.non_resident.
							compression_unit +
							vol->cluster_size_bits);
					ni->itype.compressed.block_size_bits =
							ffs(ni->itype.
							compressed.
							block_size) - 1;
					ni->itype.compressed.block_clusters =
							1U << a->data.
							non_resident.
							compression_unit;
				} else {
					ni->itype.compressed.block_size = 0;
					ni->itype.compressed.block_size_bits =
							0;
					ni->itype.compressed.block_clusters =
							0;
				}
				ni->itype.compressed.size = sle64_to_cpu(
						a->data.non_resident.
						compressed_size);
			}
			if (a->data.non_resident.lowest_vcn) {
				ntfs_error(vi->i_sb, "First extent of $DATA "
						"attribute has non zero "
						"lowest_vcn.");
				goto unm_err_out;
			}
			vi->i_size = sle64_to_cpu(
					a->data.non_resident.data_size);
			ni->initialized_size = sle64_to_cpu(
					a->data.non_resident.initialized_size);
			ni->allocated_size = sle64_to_cpu(
					a->data.non_resident.allocated_size);
		} else { /* Resident attribute. */
			vi->i_size = ni->initialized_size = le32_to_cpu(
					a->data.resident.value_length);
			ni->allocated_size = le32_to_cpu(a->length) -
					le16_to_cpu(
					a->data.resident.value_offset);
			if (vi->i_size > ni->allocated_size) {
				ntfs_error(vi->i_sb, "Resident data attribute "
						"is corrupt (size exceeds "
						"allocation).");
				goto unm_err_out;
			}
		}
no_data_attr_special_case:
		/* We are done with the mft record, so we release it. */
		ntfs_attr_put_search_ctx(ctx);
		unmap_mft_record(ni);
		m = NULL;
		ctx = NULL;
		/* Setup the operations for this inode. */
		vi->i_op = &ntfs_file_inode_ops;
		vi->i_fop = &ntfs_file_ops;
	}
	if (NInoMstProtected(ni))
		vi->i_mapping->a_ops = &ntfs_mst_aops;
	else if(NInoNonResident(ni) && !NInoCompressed(ni) && !NInoEncrypted(ni))
		vi->i_mapping->a_ops = &ntfs_nonresident_aops;
	else
		vi->i_mapping->a_ops = &ntfs_aops;

	/*
	 * The number of 512-byte blocks used on disk (for stat). This is in so
	 * far inaccurate as it doesn't account for any named streams or other
	 * special non-resident attributes, but that is how Windows works, too,
	 * so we are at least consistent with Windows, if not entirely
	 * consistent with the Linux Way. Doing it the Linux Way would cause a
	 * significant slowdown as it would involve iterating over all
	 * attributes in the mft record and adding the allocated/compressed
	 * sizes of all non-resident attributes present to give us the Linux
	 * correct size that should go into i_blocks (after division by 512).
	 */
	if (S_ISREG(vi->i_mode) && (NInoCompressed(ni) || NInoSparse(ni)))
		vi->i_blocks = ni->itype.compressed.size >> 9;
	else
		vi->i_blocks = ni->allocated_size >> 9;
	ntfs_debug("Done.");
	return 0;
iput_unm_err_out:
	iput(bvi);
unm_err_out:
	if (!err)
		err = -EIO;
	if (ctx)
		ntfs_attr_put_search_ctx(ctx);
	if (m)
		unmap_mft_record(ni);
err_out:
	ntfs_error(vol->sb, "Failed with error code %i.  Marking corrupt "
			"inode 0x%lx as bad.  Run chkdsk.", err, vi->i_ino);
	make_bad_inode(vi);
	if (err != -EOPNOTSUPP && err != -ENOMEM)
		NVolSetErrors(vol);
	return err;
}

/**
 * ntfs_read_locked_attr_inode - read an attribute inode from its base inode
 * @base_vi:	base inode
 * @vi:		attribute inode to read
 *
 * ntfs_read_locked_attr_inode() is called from ntfs_attr_iget() to read the
 * attribute inode described by @vi into memory from the base mft record
 * described by @base_ni.
 *
 * ntfs_read_locked_attr_inode() maps, pins and locks the base inode for
 * reading and looks up the attribute described by @vi before setting up the
 * necessary fields in @vi as well as initializing the ntfs inode.
 *
 * Q: What locks are held when the function is called?
 * A: i_state has I_LOCK set, hence the inode is locked, also
 *    i_count is set to 1, so it is not going to go away
 *
 * Return 0 on success and -errno on error.  In the error case, the inode will
 * have had make_bad_inode() executed on it.
 *
 * Note this cannot be called for AT_INDEX_ALLOCATION.
 */
int ntfs_read_locked_attr_inode(struct inode *base_vi, struct inode *vi)
{
	ntfs_volume *vol = NTFS_SB(vi->i_sb);
	ntfs_inode *ni, *base_ni;
	MFT_RECORD *m;
	ATTR_RECORD *a;
	ntfs_attr_search_ctx *ctx;
	int err = 0;

	ntfs_debug("Entering for i_ino 0x%lx.", vi->i_ino);

	ntfs_init_big_inode(vi);

	ni	= NTFS_I(vi);
	base_ni = NTFS_I(base_vi);

	/* Just mirror the values from the base inode. */
	vi->i_version	= base_vi->i_version;
	vi->i_uid	= base_vi->i_uid;
	vi->i_gid	= base_vi->i_gid;
	vi->__i_nlink	= base_vi->i_nlink;
	vi->i_mtime	= base_vi->i_mtime;
	vi->i_ctime	= base_vi->i_ctime;
	vi->i_atime	= base_vi->i_atime;
	vi->i_generation = ni->seq_no = base_ni->seq_no;

	/* Set inode type to zero but preserve permissions. */
	vi->i_mode	= base_vi->i_mode & ~S_IFMT;

	m = map_mft_record(base_ni);
	if (IS_ERR(m)) {
		err = PTR_ERR(m);
		goto err_out;
	}
	ctx = ntfs_attr_get_search_ctx(base_ni, m);
	if (!ctx) {
		err = -ENOMEM;
		goto unm_err_out;
	}
	/* Find the attribute. */
	err = ntfs_attr_lookup(ni->type, ni->name, ni->name_len,
			CASE_SENSITIVE, 0, NULL, 0, ctx);
	if (unlikely(err))
		goto unm_err_out;
	a = ctx->attr;
	if (a->flags & (ATTR_COMPRESSION_MASK | ATTR_IS_SPARSE)) {
		if (a->flags & ATTR_COMPRESSION_MASK) {
			/*Ignore the compression flag in index root */
			if (ni->type != AT_INDEX_ROOT) {
				NInoSetCompressed(ni);
				if ((ni->type != AT_DATA) || (ni->type == AT_DATA &&
						ni->name_len)) {
					ntfs_error(vi->i_sb, "Found compressed "
							"non-data or named data "
							"attribute.  Please report "
							"you saw this message to "
							"linux-ntfs-dev@lists."
							"sourceforge.net");
					goto unm_err_out;
				}
			}
			if (vol->cluster_size > 4096) {
				ntfs_error(vi->i_sb, "Found compressed "
						"attribute but compression is "
						"disabled due to cluster size "
						"(%i) > 4kiB.",
						vol->cluster_size);
				goto unm_err_out;
			}
			if ((a->flags & ATTR_COMPRESSION_MASK) !=
					ATTR_IS_COMPRESSED) {
				ntfs_error(vi->i_sb, "Found unknown "
						"compression method.");
				goto unm_err_out;
			}
		}
		/*
		 * The compressed/sparse flag set in an index root just means
		 * to compress all files.
		 */
		if (NInoMstProtected(ni) && ni->type != AT_INDEX_ROOT) {
			ntfs_error(vi->i_sb, "Found mst protected attribute "
					"but the attribute is %s.  Please "
					"report you saw this message to "
					"linux-ntfs-dev@lists.sourceforge.net",
					NInoCompressed(ni) ? "compressed" :
					"sparse");
			goto unm_err_out;
		}
		if (a->flags & ATTR_IS_SPARSE)
			NInoSetSparse(ni);
	}
	if (a->flags & ATTR_IS_ENCRYPTED) {
		if (NInoCompressed(ni)) {
			ntfs_error(vi->i_sb, "Found encrypted and compressed "
					"data.");
			goto unm_err_out;
		}
		/*
		 * The encryption flag set in an index root just means to
		 * encrypt all files.
		 */
		if (NInoMstProtected(ni) && ni->type != AT_INDEX_ROOT) {
			ntfs_error(vi->i_sb, "Found mst protected attribute "
					"but the attribute is encrypted.  "
					"Please report you saw this message "
					"to linux-ntfs-dev@lists.sourceforge."
					"net");
			goto unm_err_out;
		}
		if (ni->type != AT_DATA) {
			ntfs_error(vi->i_sb, "Found encrypted non-data "
					"attribute.");
			goto unm_err_out;
		}
		NInoSetEncrypted(ni);
	}
	if (!a->non_resident) {
		/* Ensure the attribute name is placed before the value. */
		if (unlikely(a->name_length && (le16_to_cpu(a->name_offset) >=
				le16_to_cpu(a->data.resident.value_offset)))) {
			ntfs_error(vol->sb, "Attribute name is placed after "
					"the attribute value.");
			goto unm_err_out;
		}
		if (NInoMstProtected(ni)) {
			ntfs_error(vi->i_sb, "Found mst protected attribute "
					"but the attribute is resident.  "
					"Please report you saw this message to "
					"linux-ntfs-dev@lists.sourceforge.net");
			goto unm_err_out;
		}
		vi->i_size = ni->initialized_size = le32_to_cpu(
				a->data.resident.value_length);
		ni->allocated_size = le32_to_cpu(a->length) -
				le16_to_cpu(a->data.resident.value_offset);
		if (vi->i_size > ni->allocated_size) {
			ntfs_error(vi->i_sb, "Resident attribute is corrupt "
					"(size exceeds allocation).");
			goto unm_err_out;
		}
	} else {
		NInoSetNonResident(ni);
		/*
		 * Ensure the attribute name is placed before the mapping pairs
		 * array.
		 */
		if (unlikely(a->name_length && (le16_to_cpu(a->name_offset) >=
				le16_to_cpu(
				a->data.non_resident.mapping_pairs_offset)))) {
			ntfs_error(vol->sb, "Attribute name is placed after "
					"the mapping pairs array.");
			goto unm_err_out;
		}
		if (NInoCompressed(ni) || NInoSparse(ni)) {
			if (NInoCompressed(ni) && a->data.non_resident.
					compression_unit != 4) {
				ntfs_error(vi->i_sb, "Found non-standard "
						"compression unit (%u instead "
						"of 4).  Cannot handle this.",
						a->data.non_resident.
						compression_unit);
				err = -EOPNOTSUPP;
				goto unm_err_out;
			}
			if (a->data.non_resident.compression_unit) {
				ni->itype.compressed.block_size = 1U <<
						(a->data.non_resident.
						compression_unit +
						vol->cluster_size_bits);
				ni->itype.compressed.block_size_bits =
						ffs(ni->itype.compressed.
						block_size) - 1;
				ni->itype.compressed.block_clusters = 1U <<
						a->data.non_resident.
						compression_unit;
			} else {
				ni->itype.compressed.block_size = 0;
				ni->itype.compressed.block_size_bits = 0;
				ni->itype.compressed.block_clusters = 0;
			}
			ni->itype.compressed.size = sle64_to_cpu(
					a->data.non_resident.compressed_size);
		}
		if (a->data.non_resident.lowest_vcn) {
			ntfs_error(vi->i_sb, "First extent of attribute has "
					"non-zero lowest_vcn.");
			goto unm_err_out;
		}
		vi->i_size = sle64_to_cpu(a->data.non_resident.data_size);
		ni->initialized_size = sle64_to_cpu(
				a->data.non_resident.initialized_size);
		ni->allocated_size = sle64_to_cpu(
				a->data.non_resident.allocated_size);
	}
	/* Setup the operations for this attribute inode. */
	vi->i_op = NULL;
	vi->i_fop = NULL;
	if (NInoMstProtected(ni))
		vi->i_mapping->a_ops = &ntfs_mst_aops;
	else if(NInoNonResident(ni) && !NInoCompressed(ni) && !NInoEncrypted(ni))
		vi->i_mapping->a_ops = &ntfs_nonresident_aops;
	else
		vi->i_mapping->a_ops = &ntfs_aops;

	if ((NInoCompressed(ni) || NInoSparse(ni)) && ni->type != AT_INDEX_ROOT)
		vi->i_blocks = ni->itype.compressed.size >> 9;
	else
		vi->i_blocks = ni->allocated_size >> 9;
	/*
	 * Make sure the base inode does not go away and attach it to the
	 * attribute inode.
	 */
	igrab(base_vi);
	ni->ext.base_ntfs_ino = base_ni;
	ni->nr_extents = -1;

	ntfs_attr_put_search_ctx(ctx);
	unmap_mft_record(base_ni);

	ntfs_debug("Done.");
	return 0;

unm_err_out:
	if (!err)
		err = -EIO;
	if (ctx)
		ntfs_attr_put_search_ctx(ctx);
	unmap_mft_record(base_ni);
err_out:
	ntfs_error(vol->sb, "Failed with error code %i while reading attribute "
			"inode (mft_no 0x%lx, type 0x%x, name_len %i).  "
			"Marking corrupt inode and base inode 0x%lx as bad.  "
			"Run chkdsk.", err, vi->i_ino, ni->type, ni->name_len,
			base_vi->i_ino);
	make_bad_inode(vi);
	if (err != -ENOMEM)
		NVolSetErrors(vol);
	return err;
}

/**
 * ntfs_read_locked_index_inode - read an index inode from its base inode
 * @base_vi:	base inode
 * @vi:		index inode to read
 *
 * ntfs_read_locked_index_inode() is called from ntfs_index_iget() to read the
 * index inode described by @vi into memory from the base mft record described
 * by @base_ni.
 *
 * ntfs_read_locked_index_inode() maps, pins and locks the base inode for
 * reading and looks up the attributes relating to the index described by @vi
 * before setting up the necessary fields in @vi as well as initializing the
 * ntfs inode.
 *
 * Note, index inodes are essentially attribute inodes (NInoAttr() is true)
 * with the attribute type set to AT_INDEX_ALLOCATION.  Apart from that, they
 * are setup like directory inodes since directories are a special case of
 * indices ao they need to be treated in much the same way.  Most importantly,
 * for small indices the index allocation attribute might not actually exist.
 * However, the index root attribute always exists but this does not need to
 * have an inode associated with it and this is why we define a new inode type
 * index.  Also, like for directories, we need to have an attribute inode for
 * the bitmap attribute corresponding to the index allocation attribute and we
 * can store this in the appropriate field of the inode, just like we do for
 * normal directory inodes.
 *
 * Q: What locks are held when the function is called?
 * A: i_state has I_LOCK set, hence the inode is locked, also
 *    i_count is set to 1, so it is not going to go away
 *
 * Return 0 on success and -errno on error.  In the error case, the inode will
 * have had make_bad_inode() executed on it.
 */
static int ntfs_read_locked_index_inode(struct inode *base_vi, struct inode *vi)
{
	loff_t bvi_size;
	ntfs_volume *vol = NTFS_SB(vi->i_sb);
	ntfs_inode *ni, *base_ni, *bni;
	struct inode *bvi;
	MFT_RECORD *m;
	ATTR_RECORD *a;
	ntfs_attr_search_ctx *ctx;
	INDEX_ROOT *ir;
	u8 *ir_end, *index_end;
	int err = 0;

	ntfs_debug("Entering for i_ino 0x%lx.", vi->i_ino);
	ntfs_init_big_inode(vi);
	ni	= NTFS_I(vi);
	base_ni = NTFS_I(base_vi);
	/* Just mirror the values from the base inode. */
	vi->i_version	= base_vi->i_version;
	vi->i_uid	= base_vi->i_uid;
	vi->i_gid	= base_vi->i_gid;
	vi->__i_nlink	= base_vi->i_nlink;
	vi->i_mtime	= base_vi->i_mtime;
	vi->i_ctime	= base_vi->i_ctime;
	vi->i_atime	= base_vi->i_atime;
	vi->i_generation = ni->seq_no = base_ni->seq_no;
	/* Set inode type to zero but preserve permissions. */
	vi->i_mode	= base_vi->i_mode & ~S_IFMT;
	/* Map the mft record for the base inode. */
	m = map_mft_record(base_ni);
	if (IS_ERR(m)) {
		err = PTR_ERR(m);
		goto err_out;
	}
	ctx = ntfs_attr_get_search_ctx(base_ni, m);
	if (!ctx) {
		err = -ENOMEM;
		goto unm_err_out;
	}
	/* Find the index root attribute. */
	err = ntfs_attr_lookup(AT_INDEX_ROOT, ni->name, ni->name_len,
			CASE_SENSITIVE, 0, NULL, 0, ctx);
	if (unlikely(err)) {
		if (err == -ENOENT)
			ntfs_error(vi->i_sb, "$INDEX_ROOT attribute is "
					"missing.");
		goto unm_err_out;
	}
	a = ctx->attr;
	/* Set up the state. */
	if (unlikely(a->non_resident)) {
		ntfs_error(vol->sb, "$INDEX_ROOT attribute is not resident.");
		goto unm_err_out;
	}
	/* Ensure the attribute name is placed before the value. */
	if (unlikely(a->name_length && (le16_to_cpu(a->name_offset) >=
			le16_to_cpu(a->data.resident.value_offset)))) {
		ntfs_error(vol->sb, "$INDEX_ROOT attribute name is placed "
				"after the attribute value.");
		goto unm_err_out;
	}
	/*
	 * Compressed/encrypted/sparse index root is not allowed, except for
	 * directories of course but those are not dealt with here.
	 */
	if (a->flags & (ATTR_COMPRESSION_MASK | ATTR_IS_ENCRYPTED |
			ATTR_IS_SPARSE)) {
		ntfs_error(vi->i_sb, "Found compressed/encrypted/sparse index "
				"root attribute.");
		goto unm_err_out;
	}
	ir = (INDEX_ROOT *)((u8 *)a + le16_to_cpu(
			a->data.resident.value_offset));
	ir_end = (u8 *)ir + le32_to_cpu(a->data.resident.value_length);
	if (ir_end > (u8 *)ctx->mrec + vol->mft_record_size) {
		ntfs_error(vi->i_sb, "$INDEX_ROOT attribute is corrupt.");
		goto unm_err_out;
	}
	index_end = (u8 *)&ir->index + le32_to_cpu(ir->index.index_length);
	if (index_end > ir_end) {
		ntfs_error(vi->i_sb, "Index is corrupt.");
		goto unm_err_out;
	}
	if (ir->type) {
		ntfs_error(vi->i_sb, "Index type is not 0 (type is 0x%x).",
				le32_to_cpu(ir->type));
		goto unm_err_out;
	}
	ni->itype.index.collation_rule = ir->collation_rule;
	ntfs_debug("Index collation rule is 0x%x.",
			le32_to_cpu(ir->collation_rule));
	ni->itype.index.block_size = le32_to_cpu(ir->index_block_size);
	if (ni->itype.index.block_size & (ni->itype.index.block_size - 1)) {
		ntfs_error(vi->i_sb, "Index block size (%u) is not a power of "
				"two.", ni->itype.index.block_size);
		goto unm_err_out;
	}
	if (ni->itype.index.block_size > PAGE_CACHE_SIZE) {
		ntfs_error(vi->i_sb, "Index block size (%u) > PAGE_CACHE_SIZE "
				"(%ld) is not supported.  Sorry.",
				ni->itype.index.block_size, PAGE_CACHE_SIZE);
		err = -EOPNOTSUPP;
		goto unm_err_out;
	}
	if (ni->itype.index.block_size < NTFS_BLOCK_SIZE) {
		ntfs_error(vi->i_sb, "Index block size (%u) < NTFS_BLOCK_SIZE "
				"(%i) is not supported.  Sorry.",
				ni->itype.index.block_size, NTFS_BLOCK_SIZE);
		err = -EOPNOTSUPP;
		goto unm_err_out;
	}
	ni->itype.index.block_size_bits = ffs(ni->itype.index.block_size) - 1;
	/* Determine the size of a vcn in the index. */
	if (vol->cluster_size <= ni->itype.index.block_size) {
		ni->itype.index.vcn_size = vol->cluster_size;
		ni->itype.index.vcn_size_bits = vol->cluster_size_bits;
	} else {
		ni->itype.index.vcn_size = vol->sector_size;
		ni->itype.index.vcn_size_bits = vol->sector_size_bits;
	}
	/* Check for presence of index allocation attribute. */
	if (!(ir->index.flags & LARGE_INDEX)) {
		/* No index allocation. */
		vi->i_size = ni->initialized_size = ni->allocated_size = 0;
		/* We are done with the mft record, so we release it. */
		ntfs_attr_put_search_ctx(ctx);
		unmap_mft_record(base_ni);
		m = NULL;
		ctx = NULL;
		goto skip_large_index_stuff;
	} /* LARGE_INDEX:  Index allocation present.  Setup state. */
	NInoSetIndexAllocPresent(ni);
	/* Find index allocation attribute. */
	ntfs_attr_reinit_search_ctx(ctx);
	err = ntfs_attr_lookup(AT_INDEX_ALLOCATION, ni->name, ni->name_len,
			CASE_SENSITIVE, 0, NULL, 0, ctx);
	if (unlikely(err)) {
		if (err == -ENOENT)
			ntfs_error(vi->i_sb, "$INDEX_ALLOCATION attribute is "
					"not present but $INDEX_ROOT "
					"indicated it is.");
		else
			ntfs_error(vi->i_sb, "Failed to lookup "
					"$INDEX_ALLOCATION attribute.");
		goto unm_err_out;
	}
	a = ctx->attr;
	if (!a->non_resident) {
		ntfs_error(vi->i_sb, "$INDEX_ALLOCATION attribute is "
				"resident.");
		goto unm_err_out;
	}
	/*
	 * Ensure the attribute name is placed before the mapping pairs array.
	 */
	if (unlikely(a->name_length && (le16_to_cpu(a->name_offset) >=
			le16_to_cpu(
			a->data.non_resident.mapping_pairs_offset)))) {
		ntfs_error(vol->sb, "$INDEX_ALLOCATION attribute name is "
				"placed after the mapping pairs array.");
		goto unm_err_out;
	}
	if (a->flags & ATTR_IS_ENCRYPTED) {
		ntfs_error(vi->i_sb, "$INDEX_ALLOCATION attribute is "
				"encrypted.");
		goto unm_err_out;
	}
	if (a->flags & ATTR_IS_SPARSE) {
		ntfs_error(vi->i_sb, "$INDEX_ALLOCATION attribute is sparse.");
		goto unm_err_out;
	}
	if (a->flags & ATTR_COMPRESSION_MASK) {
		ntfs_error(vi->i_sb, "$INDEX_ALLOCATION attribute is "
				"compressed.");
		goto unm_err_out;
	}
	if (a->data.non_resident.lowest_vcn) {
		ntfs_error(vi->i_sb, "First extent of $INDEX_ALLOCATION "
				"attribute has non zero lowest_vcn.");
		goto unm_err_out;
	}
	vi->i_size = sle64_to_cpu(a->data.non_resident.data_size);
	ni->initialized_size = sle64_to_cpu(
			a->data.non_resident.initialized_size);
	ni->allocated_size = sle64_to_cpu(a->data.non_resident.allocated_size);
	/*
	 * We are done with the mft record, so we release it.  Otherwise
	 * we would deadlock in ntfs_attr_iget().
	 */
	ntfs_attr_put_search_ctx(ctx);
	unmap_mft_record(base_ni);
	m = NULL;
	ctx = NULL;
	/* Get the index bitmap attribute inode. */
	bvi = ntfs_attr_iget(base_vi, AT_BITMAP, ni->name, ni->name_len);
	if (IS_ERR(bvi)) {
		ntfs_error(vi->i_sb, "Failed to get bitmap attribute.");
		err = PTR_ERR(bvi);
		goto unm_err_out;
	}
	bni = NTFS_I(bvi);
	if (NInoCompressed(bni) || NInoEncrypted(bni) ||
			NInoSparse(bni)) {
		ntfs_error(vi->i_sb, "$BITMAP attribute is compressed and/or "
				"encrypted and/or sparse.");
		goto iput_unm_err_out;
	}
	/* Consistency check bitmap size vs. index allocation size. */
	bvi_size = i_size_read(bvi);
	if ((bvi_size << 3) < (vi->i_size >> ni->itype.index.block_size_bits)) {
		ntfs_error(vi->i_sb, "Index bitmap too small (0x%llx) for "
				"index allocation (0x%llx).", bvi_size << 3,
				vi->i_size);
		goto iput_unm_err_out;
	}
	iput(bvi);
skip_large_index_stuff:
	/* Setup the operations for this index inode. */
	vi->i_op = NULL;
	vi->i_fop = NULL;
	vi->i_mapping->a_ops = &ntfs_mst_aops;
	vi->i_blocks = ni->allocated_size >> 9;
	/*
	 * Make sure the base inode doesn't go away and attach it to the
	 * index inode.
	 */
	igrab(base_vi);
	ni->ext.base_ntfs_ino = base_ni;
	ni->nr_extents = -1;

	ntfs_debug("Done.");
	return 0;
iput_unm_err_out:
	iput(bvi);
unm_err_out:
	if (!err)
		err = -EIO;
	if (ctx)
		ntfs_attr_put_search_ctx(ctx);
	if (m)
		unmap_mft_record(base_ni);
err_out:
	ntfs_error(vi->i_sb, "Failed with error code %i while reading index "
			"inode (mft_no 0x%lx, name_len %i.", err, vi->i_ino,
			ni->name_len);
	make_bad_inode(vi);
	if (err != -EOPNOTSUPP && err != -ENOMEM)
		NVolSetErrors(vol);
	return err;
}

/*
 * The MFT inode has special locking, so teach the lock validator
 * about this by splitting off the locking rules of the MFT from
 * the locking rules of other inodes. The MFT inode can never be
 * accessed from the VFS side (or even internally), only by the
 * map_mft functions.
 */
static struct lock_class_key mft_ni_runlist_lock_key, mft_ni_mrec_lock_key;

/**
 * ntfs_read_inode_mount - special read_inode for mount time use only
 * @vi:		inode to read
 *
 * Read inode FILE_MFT at mount time, only called with super_block lock
 * held from within the read_super() code path.
 *
 * This function exists because when it is called the page cache for $MFT/$DATA
 * is not initialized and hence we cannot get at the contents of mft records
 * by calling map_mft_record*().
 *
 * Further it needs to cope with the circular references problem, i.e. cannot
 * load any attributes other than $ATTRIBUTE_LIST until $DATA is loaded, because
 * we do not know where the other extent mft records are yet and again, because
 * we cannot call map_mft_record*() yet.  Obviously this applies only when an
 * attribute list is actually present in $MFT inode.
 *
 * We solve these problems by starting with the $DATA attribute before anything
 * else and iterating using ntfs_attr_lookup($DATA) over all extents.  As each
 * extent is found, we ntfs_mapping_pairs_decompress() including the implied
 * ntfs_runlists_merge().  Each step of the iteration necessarily provides
 * sufficient information for the next step to complete.
 *
 * This should work but there are two possible pit falls (see inline comments
 * below), but only time will tell if they are real pits or just smoke...
 */
int ntfs_read_inode_mount(struct inode *vi)
{
	VCN next_vcn, last_vcn, highest_vcn;
	s64 block;
	struct super_block *sb = vi->i_sb;
	ntfs_volume *vol = NTFS_SB(sb);
	struct buffer_head *bh;
	ntfs_inode *ni;
	MFT_RECORD *m = NULL;
	ATTR_RECORD *a;
	ntfs_attr_search_ctx *ctx;
	unsigned int i, nr_blocks;
	int err;

	ntfs_debug("Entering.");

	/* Initialize the ntfs specific part of @vi. */
	ntfs_init_big_inode(vi);

	ni = NTFS_I(vi);

	/* Setup the data attribute. It is special as it is mst protected. */
	NInoSetNonResident(ni);
	NInoSetMstProtected(ni);
	NInoSetSparseDisabled(ni);
	ni->type = AT_DATA;
	ni->name = NULL;
	ni->name_len = 0;
	/*
	 * This sets up our little cheat allowing us to reuse the async read io
	 * completion handler for directories.
	 */
	ni->itype.index.block_size = vol->mft_record_size;
	ni->itype.index.block_size_bits = vol->mft_record_size_bits;

	/* Very important! Needed to be able to call map_mft_record*(). */
	vol->mft_ino = vi;

	/* Allocate enough memory to read the first mft record. */
	if (vol->mft_record_size > 64 * 1024) {
		ntfs_error(sb, "Unsupported mft record size %i (max 64kiB).",
				vol->mft_record_size);
		goto err_out;
	}
	i = vol->mft_record_size;
	if (i < sb->s_blocksize)
		i = sb->s_blocksize;
	m = (MFT_RECORD *)ntfs_malloc_nofs(i);
	if (!m) {
		ntfs_error(sb, "Failed to allocate buffer for $MFT record 0.");
		goto err_out;
	}

	/* Determine the first block of the $MFT/$DATA attribute. */
	block = vol->mft_lcn << vol->cluster_size_bits >>
			sb->s_blocksize_bits;
	nr_blocks = vol->mft_record_size >> sb->s_blocksize_bits;
	if (!nr_blocks)
		nr_blocks = 1;

	/* Load $MFT/$DATA's first mft record. */
	for (i = 0; i < nr_blocks; i++) {
		bh = sb_bread(sb, block++);
		if (!bh) {
			ntfs_error(sb, "Device read failed.");
			goto err_out;
		}
		memcpy((char *)m + (i << sb->s_blocksize_bits), bh->b_data,
				sb->s_blocksize);
		brelse(bh);
	}

	/* Apply the mst fixups. */
	if (post_read_mst_fixup((NTFS_RECORD *)m,
		vol->mft_record_size)) {
		/* FIXME: Try to use the $MFTMirr now. */
		ntfs_error(sb, "MST fixup failed. $MFT is corrupt.");
		goto err_out;
	}

	/* Need this to sanity check attribute list references to $MFT. */
	vi->i_generation = ni->seq_no = le16_to_cpu(m->sequence_number);

	/* Provides readpage() and sync_page() for map_mft_record(). */
	vi->i_mapping->a_ops = &ntfs_mst_aops;

	ctx = ntfs_attr_get_search_ctx(ni, m);
	if (!ctx) {
		err = -ENOMEM;
		goto err_out;
	}

	/* Find the attribute list attribute if present. */
	err = ntfs_attr_lookup(AT_ATTRIBUTE_LIST, NULL, 0, 0, 0, NULL, 0, ctx);
	if (err) {
		if (unlikely(err != -ENOENT)) {
			ntfs_error(sb, "Failed to lookup attribute list "
					"attribute. You should run chkdsk.");
			goto put_err_out;
		}
	} else /* if (!err) */ {
		ATTR_LIST_ENTRY *al_entry, *next_al_entry;
		u8 *al_end;
		static const char *es = "  Not allowed.  $MFT is corrupt.  "
				"You should run chkdsk.";

		ntfs_debug("Attribute list attribute found in $MFT.");
		NInoSetAttrList(ni);
		a = ctx->attr;
		if (a->flags & ATTR_COMPRESSION_MASK) {
			ntfs_error(sb, "Attribute list attribute is "
					"compressed.%s", es);
			goto put_err_out;
		}
		if (a->flags & ATTR_IS_ENCRYPTED ||
				a->flags & ATTR_IS_SPARSE) {
			if (a->non_resident) {
				ntfs_error(sb, "Non-resident attribute list "
						"attribute is encrypted/"
						"sparse.%s", es);
				goto put_err_out;
			}
			ntfs_warning(sb, "Resident attribute list attribute "
					"in $MFT system file is marked "
					"encrypted/sparse which is not true.  "
					"However, Windows allows this and "
					"chkdsk does not detect or correct it "
					"so we will just ignore the invalid "
					"flags and pretend they are not set.");
		}
		/* Now allocate memory for the attribute list. */
		ni->attr_list_size = (u32)ntfs_attr_size(a);
		ni->attr_list = ntfs_malloc_nofs(ni->attr_list_size);
		if (!ni->attr_list) {
			ntfs_error(sb, "Not enough memory to allocate buffer "
					"for attribute list.");
			goto put_err_out;
		}
		if (a->non_resident) {
			NInoSetAttrListNonResident(ni);
			if (a->data.non_resident.lowest_vcn) {
				ntfs_error(sb, "Attribute list has non zero "
						"lowest_vcn. $MFT is corrupt. "
						"You should run chkdsk.");
				goto put_err_out;
			}
			/* Setup the runlist. */
			ni->attr_list_rl.rl = ntfs_mapping_pairs_decompress(vol,
					a, NULL);
			if (IS_ERR(ni->attr_list_rl.rl)) {
				err = PTR_ERR(ni->attr_list_rl.rl);
				ni->attr_list_rl.rl = NULL;
				ntfs_error(sb, "Mapping pairs decompression "
						"failed with error code %i.",
						-err);
				goto put_err_out;
			}
			/* Now load the attribute list. */
			err = load_attribute_list(vol, &ni->attr_list_rl,
					ni->attr_list, ni->attr_list_size,
					sle64_to_cpu(a->data.
					non_resident.initialized_size));
			if (err) {
				ntfs_error(sb, "Failed to load attribute list "
						"attribute with error code %i.",
						-err);
				goto put_err_out;
			}
		} else /* if (!ctx.attr->non_resident) */ {
			if ((u8 *)a + le16_to_cpu(
					a->data.resident.value_offset) +
					le32_to_cpu(
					a->data.resident.value_length) >
					(u8 *)ctx->mrec +
					vol->mft_record_size) {
				ntfs_error(sb, "Corrupt attribute list "
						"attribute.");
				goto put_err_out;
			}
			/* Now copy the attribute list. */
			memcpy(ni->attr_list, (u8 *)a + le16_to_cpu(
					a->data.resident.value_offset),
					le32_to_cpu(
					a->data.resident.value_length));
		}
		/* The attribute list is now setup in memory. */
		/*
		 * FIXME: I don't know if this case is actually possible.
		 * According to logic it is not possible but I have seen too
		 * many weird things in MS software to rely on logic... Thus we
		 * perform a manual search and make sure the first $MFT/$DATA
		 * extent is in the base inode. If it is not we abort with an
		 * error and if we ever see a report of this error we will need
		 * to do some magic in order to have the necessary mft record
		 * loaded and in the right place in the page cache. But
		 * hopefully logic will prevail and this never happens...
		 */
		al_entry = (ATTR_LIST_ENTRY *)ni->attr_list;
		al_end = (u8 *)al_entry + ni->attr_list_size;
		for (;; al_entry = next_al_entry) {
			/* Out of bounds check. */
			if ((u8 *)al_entry < ni->attr_list ||
					(u8 *)al_entry > al_end)
				goto em_put_err_out;
			/* Catch the end of the attribute list. */
			if ((u8 *)al_entry == al_end)
				goto em_put_err_out;
			if (!al_entry->length)
				goto em_put_err_out;
			if ((u8 *)al_entry + 6 > al_end || (u8 *)al_entry +
					le16_to_cpu(al_entry->length) > al_end)
				goto em_put_err_out;
			next_al_entry = (ATTR_LIST_ENTRY *)((u8 *)al_entry +
					le16_to_cpu(al_entry->length));
			if (le32_to_cpu(al_entry->type) >
					const_le32_to_cpu(AT_DATA))
				goto em_put_err_out;
			if (AT_DATA != al_entry->type)
				continue;
			/* We want an unnamed attribute. */
			if (al_entry->name_length)
				goto em_put_err_out;
			/* Want the first entry, i.e. lowest_vcn == 0. */
			if (al_entry->lowest_vcn)
				goto em_put_err_out;
			/* First entry has to be in the base mft record. */
			if (MREF_LE(al_entry->mft_reference) != vi->i_ino) {
				/* MFT references do not match, logic fails. */
				ntfs_error(sb, "BUG: The first $DATA extent "
						"of $MFT is not in the base "
						"mft record. Please report "
						"you saw this message to "
						"linux-ntfs-dev@lists."
						"sourceforge.net");
				goto put_err_out;
			} else {
				/* Sequence numbers must match. */
				if (MSEQNO_LE(al_entry->mft_reference) !=
						ni->seq_no)
					goto em_put_err_out;
				/* Got it. All is ok. We can stop now. */
				break;
			}
		}
	}

	ntfs_attr_reinit_search_ctx(ctx);

	/* Now load all attribute extents. */
	a = NULL;
	next_vcn = last_vcn = highest_vcn = 0;
	while (!(err = ntfs_attr_lookup(AT_DATA, NULL, 0, 0, next_vcn, NULL, 0,
			ctx))) {
		runlist_element *nrl;

		/* Cache the current attribute. */
		a = ctx->attr;
		/* $MFT must be non-resident. */
		if (!a->non_resident) {
			ntfs_error(sb, "$MFT must be non-resident but a "
					"resident extent was found. $MFT is "
					"corrupt. Run chkdsk.");
			goto put_err_out;
		}
		/* $MFT must be uncompressed and unencrypted. */
		if (a->flags & ATTR_COMPRESSION_MASK ||
				a->flags & ATTR_IS_ENCRYPTED ||
				a->flags & ATTR_IS_SPARSE) {
			ntfs_error(sb, "$MFT must be uncompressed, "
					"non-sparse, and unencrypted but a "
					"compressed/sparse/encrypted extent "
					"was found. $MFT is corrupt. Run "
					"chkdsk.");
			goto put_err_out;
		}
		/*
		 * Decompress the mapping pairs array of this extent and merge
		 * the result into the existing runlist. No need for locking
		 * as we have exclusive access to the inode at this time and we
		 * are a mount in progress task, too.
		 */
		nrl = ntfs_mapping_pairs_decompress(vol, a, ni->runlist.rl);
		if (IS_ERR(nrl)) {
			ntfs_error(sb, "ntfs_mapping_pairs_decompress() "
					"failed with error code %ld.  $MFT is "
					"corrupt.", PTR_ERR(nrl));
			goto put_err_out;
		}
		ni->runlist.rl = nrl;

		/* Are we in the first extent? */
		if (!next_vcn) {
			if (a->data.non_resident.lowest_vcn) {
				ntfs_error(sb, "First extent of $DATA "
						"attribute has non zero "
						"lowest_vcn. $MFT is corrupt. "
						"You should run chkdsk.");
				goto put_err_out;
			}
			/* Get the last vcn in the $DATA attribute. */
			last_vcn = sle64_to_cpu(
					a->data.non_resident.allocated_size)
					>> vol->cluster_size_bits;
			/* Fill in the inode size. */
			vi->i_size = sle64_to_cpu(
					a->data.non_resident.data_size);
			ni->initialized_size = sle64_to_cpu(
					a->data.non_resident.initialized_size);
			ni->allocated_size = sle64_to_cpu(
					a->data.non_resident.allocated_size);
			/*
			 * Verify the number of mft records does not exceed
			 * 2^32 - 1.
			 */
			if ((vi->i_size >> vol->mft_record_size_bits) >=
					(1ULL << 32)) {
				ntfs_error(sb, "$MFT is too big! Aborting.");
				goto put_err_out;
			}
			/*
			 * We have got the first extent of the runlist for
			 * $MFT which means it is now relatively safe to call
			 * the normal ntfs_read_inode() function.
			 * Complete reading the inode, this will actually
			 * re-read the mft record for $MFT, this time entering
			 * it into the page cache with which we complete the
			 * kick start of the volume. It should be safe to do
			 * this now as the first extent of $MFT/$DATA is
			 * already known and we would hope that we don't need
			 * further extents in order to find the other
			 * attributes belonging to $MFT. Only time will tell if
			 * this is really the case. If not we will have to play
			 * magic at this point, possibly duplicating a lot of
			 * ntfs_read_inode() at this point. We will need to
			 * ensure we do enough of its work to be able to call
			 * ntfs_read_inode() on extents of $MFT/$DATA. But lets
			 * hope this never happens...
			 */
			ntfs_read_locked_inode(vi);
			if (is_bad_inode(vi)) {
				ntfs_error(sb, "ntfs_read_inode() of $MFT "
						"failed. BUG or corrupt $MFT. "
						"Run chkdsk and if no errors "
						"are found, please report you "
						"saw this message to "
						"linux-ntfs-dev@lists."
						"sourceforge.net");
				ntfs_attr_put_search_ctx(ctx);
				/* Revert to the safe super operations. */
				ntfs_free(m);
				return -1;
			}
			/*
			 * Re-initialize some specifics about $MFT's inode as
			 * ntfs_read_inode() will have set up the default ones.
			 */
			/* Set uid and gid to root. */
			vi->i_uid = vi->i_gid = 0;
			/* Regular file. No access for anyone. */
			vi->i_mode = S_IFREG;
			/* No VFS initiated operations allowed for $MFT. */
			vi->i_op = &ntfs_empty_inode_ops;
			vi->i_fop = &ntfs_empty_file_ops;
		}

		/* Get the lowest vcn for the next extent. */
		highest_vcn = sle64_to_cpu(a->data.non_resident.highest_vcn);
		next_vcn = highest_vcn + 1;

		/* Only one extent or error, which we catch below. */
		if (next_vcn <= 0)
			break;

		/* Avoid endless loops due to corruption. */
		if (next_vcn < sle64_to_cpu(
				a->data.non_resident.lowest_vcn)) {
			ntfs_error(sb, "$MFT has corrupt attribute list "
					"attribute. Run chkdsk.");
			goto put_err_out;
		}
	}
	if (err != -ENOENT) {
		ntfs_error(sb, "Failed to lookup $MFT/$DATA attribute extent. "
				"$MFT is corrupt. Run chkdsk.");
		goto put_err_out;
	}
	if (!a) {
		ntfs_error(sb, "$MFT/$DATA attribute not found. $MFT is "
				"corrupt. Run chkdsk.");
		goto put_err_out;
	}
	if (highest_vcn && highest_vcn != last_vcn - 1) {
		ntfs_error(sb, "Failed to load the complete runlist for "
				"$MFT/$DATA. Driver bug or corrupt $MFT. "
				"Run chkdsk.");
		ntfs_debug("highest_vcn = 0x%llx, last_vcn - 1 = 0x%llx",
				(unsigned long long)highest_vcn,
				(unsigned long long)last_vcn - 1);
		goto put_err_out;
	}
	ntfs_attr_put_search_ctx(ctx);
	ntfs_debug("Done.");
	ntfs_free(m);

	/*
	 * Split the locking rules of the MFT inode from the
	 * locking rules of other inodes:
	 */
	lockdep_set_class(&ni->runlist.lock, &mft_ni_runlist_lock_key);
	lockdep_set_class(&ni->mrec_lock, &mft_ni_mrec_lock_key);

	return 0;

em_put_err_out:
	ntfs_error(sb, "Couldn't find first extent of $DATA attribute in "
			"attribute list. $MFT is corrupt. Run chkdsk.");
put_err_out:
	ntfs_attr_put_search_ctx(ctx);
err_out:
	ntfs_error(sb, "Failed. Marking inode as bad.");
	make_bad_inode(vi);
	ntfs_free(m);
	return -1;
}

static void __ntfs_clear_inode(ntfs_inode *ni)
{
	/* Free all alocated memory. */
	down_write(&ni->runlist.lock);
	if (ni->runlist.rl) {
		ntfs_free(ni->runlist.rl);
		ni->runlist.rl = NULL;
	}
	up_write(&ni->runlist.lock);

	if (ni->attr_list) {
		ntfs_free(ni->attr_list);
		ni->attr_list = NULL;
	}

	down_write(&ni->attr_list_rl.lock);
	if (ni->attr_list_rl.rl) {
		ntfs_free(ni->attr_list_rl.rl);
		ni->attr_list_rl.rl = NULL;
	}
	up_write(&ni->attr_list_rl.lock);

	if (ni->name_len && ni->name != I30) {
		/* Catch bugs... */
		DEBUG_ON(!ni->name);
		kfree(ni->name);
	}
}

void ntfs_clear_extent_inode(ntfs_inode *ni)
{
	ntfs_debug("Entering for inode 0x%lx.", ni->mft_no);

	DEBUG_ON(NInoAttr(ni));
	DEBUG_ON(ni->nr_extents != -1);

#ifdef NTFS_RW
	if (NInoDirty(ni)) {
		if (!is_bad_inode(VFS_I(ni->ext.base_ntfs_ino)))
			ntfs_error(ni->vol->sb, "Clearing dirty extent inode!  "
					"Losing data!  This is a BUG!!!");
		/* FIXME:  Do something!!!*/
	}
#endif /* NTFS_RW */

	__ntfs_clear_inode(ni);

	/* Bye, bye... */
	ntfs_destroy_extent_inode(ni);
}

/**
 * ntfs_clear_big_inode - clean up the ntfs specific part of an inode
 * @vi:		vfs inode pending annihilation
 *
 * When the VFS is going to remove an inode from memory, ntfs_clear_big_inode()
 * is called, which deallocates all memory belonging to the NTFS specific part
 * of the inode and returns.
 *
 * If the MFT record is dirty, we commit it before doing anything else.
 */
void ntfs_clear_big_inode(struct inode *vi)
{
	ntfs_inode *ni = NTFS_I(vi);

	truncate_inode_pages(&vi->i_data, 0);
	end_writeback(vi);

#ifdef NTFS_RW
	if (NInoDirty(ni)) {
		bool was_bad = (is_bad_inode(vi));

		/* Committing the inode also commits all extent inodes. */
		ntfs_commit_inode(vi);

		if (!was_bad && (is_bad_inode(vi) || NInoDirty(ni))) {
			ntfs_error(vi->i_sb, "Failed to commit dirty inode "
					"0x%lx.  Losing data!", vi->i_ino);
			/* FIXME:  Do something!!!*/
		}
	}
#endif /* NTFS_RW */

	/* No need to lock at this stage as no one else has a reference. */
	if (ni->nr_extents > 0) {
		int i;

		for (i = 0; i < ni->nr_extents; i++)
			ntfs_clear_extent_inode(ni->ext.extent_ntfs_inos[i]);
		kfree(ni->ext.extent_ntfs_inos);
	}

	__ntfs_clear_inode(ni);

	if (NInoAttr(ni)) {
		/* Release the base inode if we are holding it. */
		if (ni->nr_extents == -1) {
			iput(VFS_I(ni->ext.base_ntfs_ino));
			ni->nr_extents = 0;
			ni->ext.base_ntfs_ino = NULL;
		}
	}

	return;
}

/**
 * ntfs_extent_inode_open - load an extent inode and attach it to its base
 * @base_ni:	base ntfs inode
 * @mref:	mft reference of the extent inode to load (in little endian)
 *
 * First check if the extent inode @mref is already attached to the base ntfs
 * inode @base_ni, and if so, return a pointer to the attached extent inode.
 *
 * If the extent inode is not already attached to the base inode, allocate an
 * ntfs_inode structure and initialize it for the given inode @mref. @mref
 * specifies the inode number / mft record to read, including the sequence
 * number, which can be 0 if no sequence number checking is to be performed.
 *
 * Then, allocate a buffer for the mft record, read the mft record from the
 * volume @base_ni->vol, and attach it to the ntfs_inode structure (->mrec).
 * The mft record is mst deprotected and sanity checked for validity and we
 * abort if deprotection or checks fail.
 *
 * Finally attach the ntfs inode to its base inode @base_ni and return a
 * pointer to the ntfs_inode structure on success or NULL on error, with errno
 * set to the error code.
 *
 * Note, extent inodes are never closed directly. They are automatically
 * disposed off by the closing of the base inode.
 */
ntfs_inode *ntfs_extent_inode_open(ntfs_inode *base_ni, const MFT_REF mref)
{
	MFT_RECORD *m;
	ntfs_inode *ni = NULL;
	ntfs_inode **extent_nis = NULL;
	int i;
	unsigned long mft_no = MREF(mref);
	u16 seq_no = MSEQNO(mref);
	bool destroy_ni = false;

	ntfs_debug("Mapping extent mft record 0x%lx (base mft record 0x%lx).",
			mft_no, base_ni->mft_no);
	/* Make sure the base ntfs inode doesn't go away. */
	atomic_inc(&base_ni->count);
	/*
	 * Check if this extent inode has already been added to the base inode,
	 * in which case just return it. If not found, add it to the base
	 * inode before returning it.
	 */
	mutex_lock(&base_ni->extent_lock);
	if (base_ni->nr_extents > 0) {
		extent_nis = base_ni->ext.extent_ntfs_inos;
		for (i = 0; i < base_ni->nr_extents; i++) {
			if (mft_no != extent_nis[i]->mft_no)
				continue;
			ni = extent_nis[i];
			/* Make sure the ntfs inode doesn't go away. */
			atomic_inc(&ni->count);
			break;
		}
	}
	if (likely(ni != NULL)) {
		mutex_unlock(&base_ni->extent_lock);
		atomic_dec(&base_ni->count);
		/* We found the record; just have to map and return it. */
		m = map_mft_record(ni);
		/* map_mft_record() has incremented this on success. */
		if (likely(!IS_ERR(m))) {
			/* Verify the sequence number. */
			if (likely(le16_to_cpu(m->sequence_number) == seq_no)) {
				ntfs_debug("Done 1.");
				unmap_mft_record(ni);
				return ni;
			}
			unmap_mft_record(ni);
			ntfs_error(base_ni->vol->sb, "Found stale extent mft "
					"reference! Corrupt filesystem. "
					"Run chkdsk.");
			return ERR_PTR(-EIO);
		}
map_err_out:
		ntfs_error(base_ni->vol->sb, "Failed to map extent "
				"mft record, error code %ld.", -PTR_ERR(m));
		return (void *)m;
	}
	/* Record wasn't there. Get a new ntfs inode and initialize it. */
	ni = ntfs_new_extent_inode(base_ni->vol->sb, mft_no);
	if (unlikely(!ni)) {
		mutex_unlock(&base_ni->extent_lock);
		atomic_dec(&base_ni->count);
		return ERR_PTR(-ENOMEM);
	}
	ni->vol = base_ni->vol;
	ni->seq_no = seq_no;
	ni->nr_extents = -1;
	ni->ext.base_ntfs_ino = base_ni;
	/* Now map the record. */
	m = map_mft_record(ni);
	if (IS_ERR(m)) {
		mutex_unlock(&base_ni->extent_lock);
		atomic_dec(&base_ni->count);
		ntfs_clear_extent_inode(ni);
		goto map_err_out;
	}
	/* Verify the sequence number if it is present. */
	if (seq_no && (le16_to_cpu(m->sequence_number) != seq_no)) {
		ntfs_error(base_ni->vol->sb, "Found stale extent mft "
				"reference! Corrupt filesystem. Run chkdsk.");
		destroy_ni = true;
		m = ERR_PTR(-EIO);
		goto unm_err_out;
	}
	/* Attach extent inode to base inode, reallocating memory if needed. */
	if (!(base_ni->nr_extents & 3)) {
		ntfs_inode **tmp;
		int new_size = (base_ni->nr_extents + 4) * sizeof(ntfs_inode *);

		tmp = kmalloc(new_size, GFP_NOFS);
		if (unlikely(!tmp)) {
			ntfs_error(base_ni->vol->sb, "Failed to allocate "
					"internal buffer.");
			destroy_ni = true;
			m = ERR_PTR(-ENOMEM);
			goto unm_err_out;
		}
		if (base_ni->nr_extents) {
			DEBUG_ON(!base_ni->ext.extent_ntfs_inos);
			memcpy(tmp, base_ni->ext.extent_ntfs_inos, new_size -
					4 * sizeof(ntfs_inode *));
			kfree(base_ni->ext.extent_ntfs_inos);
		}
		base_ni->ext.extent_ntfs_inos = tmp;
	}
	base_ni->ext.extent_ntfs_inos[base_ni->nr_extents++] = ni;
	mutex_unlock(&base_ni->extent_lock);
	atomic_dec(&base_ni->count);
	ntfs_debug("Done 2.");
	atomic_inc(&ni->count);
	unmap_mft_record(ni);
	return ni;
unm_err_out:
	unmap_mft_record(ni);
	mutex_unlock(&base_ni->extent_lock);
	atomic_dec(&base_ni->count);
	/*
	 * If the extent inode was not attached to the base inode we need to
	 * release it or we will leak memory.
	 */
	if (destroy_ni)
		ntfs_clear_extent_inode(ni);
	return (void *)m;

}

/**
 * ntfs_inode_attach_all_extents - attach all extents for target inode
 * @ni:		opened ntfs inode for which perform attach
 *
 * Return 0 on success and -1 on error with errno set to the error code.
 */
int ntfs_inode_attach_all_extents(ntfs_inode *ni)
{
	ATTR_LIST_ENTRY *ale;
	u64 prev_attached = 0;
	ntfs_inode *temp_ni;

	if (!ni) {
		ntfs_debug("Invalid arguments.\n");
		return -EINVAL;
	}

	if (ni->nr_extents == -1)
		ni = ni->ext.base_ntfs_ino;

	ntfs_debug("Entering for inode 0x%llx.\n", (long long) ni->mft_no);

	/* Inode haven't got attribute list, thus nothing to attach. */
	if (!NInoAttrList(ni))
		return 0;

	if (!ni->attr_list) {
		ntfs_debug("Corrupt in-memory struct.\n");
		return -EINVAL;
	}

	/* Walk through attribute list and attach all extents. */
	ale = (ATTR_LIST_ENTRY *)ni->attr_list;
	while ((u8 *)ale < ni->attr_list + ni->attr_list_size) {
		if (ni->mft_no != MREF_LE(ale->mft_reference) &&
				prev_attached != MREF_LE(ale->mft_reference)) {
			temp_ni = ntfs_extent_inode_open(
					ni, ale->mft_reference);
			if (IS_ERR(temp_ni)) {
				ntfs_debug("Couldn't attach extent inode.\n");
				return PTR_ERR(temp_ni);
			}
			prev_attached = MREF_LE(ale->mft_reference);
		}
		ale = (ATTR_LIST_ENTRY *)((u8 *)ale + le16_to_cpu(ale->length));
	}
	return 0;
}

int ntfs_inode_detach_all_extents(ntfs_inode *ni)
{
	int i;
	ntfs_inode *temp_ni;

	if (!ni) {
		ntfs_debug("Invalid arguments.\n");
		return -EINVAL;
	}

	if (ni->nr_extents == -1)
		ni = ni->ext.base_ntfs_ino;

	ntfs_debug("Entering for inode 0x%llx.\n", (long long) ni->mft_no);

	/* Inode haven't got attribute list, thus nothing to attach. */
	if (ni->nr_extents == 0)
		return 0;

	for (i = 0; i < ni->nr_extents; i++) {
		temp_ni = ni->ext.extent_ntfs_inos[i];
		atomic_dec(&temp_ni->count);
	}
	return 0;
}
/**
 * ntfs_show_options - show mount options in /proc/mounts
 * @sf:		seq_file in which to write our mount options
 * @mnt:	vfs mount whose mount options to display
 *
 * Called by the VFS once for each mounted ntfs volume when someone reads
 * /proc/mounts in order to display the NTFS specific mount options of each
 * mount. The mount options of the vfs mount @mnt are written to the seq file
 * @sf and success is returned.
 */
int ntfs_show_options(struct seq_file *sf, struct dentry *root)
{
	ntfs_volume *vol = NTFS_SB(root->d_sb);
	int i;

	seq_printf(sf, ",uid=%i", vol->uid);
	seq_printf(sf, ",gid=%i", vol->gid);
	if (vol->fmask == vol->dmask)
		seq_printf(sf, ",umask=0%o", vol->fmask);
	else {
		seq_printf(sf, ",fmask=0%o", vol->fmask);
		seq_printf(sf, ",dmask=0%o", vol->dmask);
	}
	seq_printf(sf, ",nls=%s", vol->nls_map->charset);
	if (NVolCaseSensitive(vol))
		seq_printf(sf, ",case_sensitive");
	if (NVolShowSystemFiles(vol))
		seq_printf(sf, ",show_sys_files");
	if (!NVolSparseEnabled(vol))
		seq_printf(sf, ",disable_sparse");
	for (i = 0; on_errors_arr[i].val; i++) {
		if (on_errors_arr[i].val & vol->on_errors)
			seq_printf(sf, ",errors=%s", on_errors_arr[i].str);
	}
	seq_printf(sf, ",mft_zone_multiplier=%i", vol->mft_zone_multiplier);
	return 0;
}

int ntfs_getattr(struct vfsmount *mnt, struct dentry *dentry,
		  struct kstat *stat)
{
	struct inode *inode = dentry->d_inode;
	generic_fillattr(inode, stat);
	stat->blksize = NTFS_I(inode)->vol->cluster_size;
	return 0;
}

#ifdef NTFS_RW

static const char *es = "  Leaving inconsistent metadata.  Unmount and run "
		"chkdsk.";

/**
 * ntfs_truncate - called when the i_size of an ntfs inode is changed
 * @vi:		inode for which the i_size was changed
 *
 * We only support i_size changes for normal files at present, i.e. not
 * compressed and not encrypted.  This is enforced in ntfs_setattr(), see
 * below.
 *
 * The kernel guarantees that @vi is a regular file (S_ISREG() is true) and
 * that the change is allowed.
 *
 * This implies for us that @vi is a file inode rather than a directory, index,
 * or attribute inode as well as that @vi is a base inode.
 *
 * Returns 0 on success or -errno on error.
 *
 * Called with ->i_mutex held.  In all but one case ->i_alloc_sem is held for
 * writing.  The only case in the kernel where ->i_alloc_sem is not held is
 * mm/filemap.c::generic_file_buffered_write() where vmtruncate() is called
 * with the current i_size as the offset.  The analogous place in NTFS is in
 * fs/ntfs/file.c::ntfs_file_buffered_write() where we call vmtruncate() again
 * without holding ->i_alloc_sem.
 */
int ntfs_truncate(struct inode *vi)
{
	s64 new_size, old_size, nr_freed, new_alloc_size, old_alloc_size, ll;
	VCN highest_vcn, lowest_vcn, last_vcn;
	unsigned long flags;
	ntfs_inode *base_ni, *ni = NTFS_I(vi), *bak_ni = NULL, *ext_ni;
	struct inode *attr_vi;
	ntfs_volume *vol = ni->vol;
	ntfs_attr_search_ctx *ctx;
	MFT_RECORD *m, *ext_m;
	ATTR_RECORD *a;
	const char *te = "  Leaving file length out of sync with i_size.";
	int err, ret, mp_size, size_change, alloc_change;
	u32 attr_len;
	bool real, mp_rebuilt;
	ntfs_debug("Entering for inode 0x%lx.", vi->i_ino);
retry_truncate:
	/*
	 * Lock the runlist for writing and map the mft record to ensure it is
	 * safe to mess with the attribute runlist and sizes.
	 */
	down_write(&ni->runlist.lock);
	if (!NInoAttr(ni))
		base_ni = ni;
	else
		base_ni = ni->ext.base_ntfs_ino;
	m = map_mft_record(base_ni);
	if (IS_ERR(m)) {
		err = PTR_ERR(m);
		ntfs_error(vi->i_sb, "Failed to map mft record for inode 0x%lx "
				"(error code %d).%s", vi->i_ino, err, te);
		ctx = NULL;
		m = NULL;
		goto old_bad_out;
	}

	ctx = ntfs_attr_get_search_ctx(base_ni, m);
	if (unlikely(!ctx)) {
		ntfs_error(vi->i_sb, "Failed to allocate a search context for "
				"inode 0x%lx (not enough memory).%s",
				vi->i_ino, te);
		err = -ENOMEM;
		goto old_bad_out;
	}
	err = ntfs_attr_lookup(ni->type, ni->name, ni->name_len,
			CASE_SENSITIVE, 0, NULL, 0, ctx);
	if (unlikely(err)) {
		if (err == -ENOENT) {
			ntfs_error(vi->i_sb, "Open attribute is missing from "
					"mft record.  Inode 0x%lx is corrupt.  "
					"Run chkdsk.%s", vi->i_ino, te);
			err = -EIO;
		} else
			ntfs_error(vi->i_sb, "Failed to lookup attribute in "
					"inode 0x%lx (error code %d).%s",
					vi->i_ino, err, te);
		goto old_bad_out;
	}
	m = ctx->mrec;
	a = ctx->attr;
	/*
	 * The i_size of the vfs inode is the new size for the attribute value.
	 */
	new_size = i_size_read(vi);
	/* The current size of the attribute value is the old size. */
	old_size = ntfs_attr_size(a);

	/* Calculate the new allocated size. */
	if (NInoNonResident(ni))
		new_alloc_size = (new_size + vol->cluster_size - 1) &
				~(s64)vol->cluster_size_mask;
	else
		new_alloc_size = (new_size + 7) & ~7;
	/* The current allocated size is the old allocated size. */
	read_lock_irqsave(&ni->size_lock, flags);
	old_alloc_size = ni->allocated_size;
	read_unlock_irqrestore(&ni->size_lock, flags);

	/*
	 * The change in the file size.  This will be 0 if no change, >0 if the
	 * size is growing, and <0 if the size is shrinking.
	 */
	size_change = -1;
	if (new_size - old_size >= 0) {
		size_change = 1;
		if (new_size == old_size)
			size_change = 0;
	}
	/* As above for the allocated size. */
	alloc_change = -1;
	if (new_alloc_size - old_alloc_size >= 0) {
		alloc_change = 1;
		if (new_alloc_size == old_alloc_size)
			alloc_change = 0;
	}

	/*
	 * If neither the size nor the allocation are being changed there is
	 * nothing to do.
	 */
	if (!size_change && !alloc_change)
		goto unm_done;
	/* If the size is changing, check if new size is allowed in $AttrDef. */
	if (size_change) {
		err = ntfs_attr_size_bounds_check(vol, ni->type, new_size);
		if (unlikely(err)) {
			if (err == -ERANGE) {
				ntfs_error(vol->sb, "Truncate would cause the "
						"inode 0x%lx to %simum size "
						"for its attribute type "
						"(0x%x).  Aborting truncate.",
						vi->i_ino,
						new_size > old_size ? "exceed "
						"the max" : "go under the min",
						le32_to_cpu(ni->type));
				err = -EFBIG;
			} else {
				ntfs_error(vol->sb, "Inode 0x%lx has unknown "
						"attribute type 0x%x.  "
						"Aborting truncate.",
						vi->i_ino,
						le32_to_cpu(ni->type));
				err = -EIO;
			}
			/* Reset the vfs inode size to the old size. */
			i_size_write(vi, old_size);
			goto err_out;
		}
	}
	if (NInoCompressed(ni) || NInoEncrypted(ni)) {
		ntfs_warning(vi->i_sb, "Changes in inode size are not "
				"supported yet for %s files, ignoring.",
				NInoCompressed(ni) ? "compressed" :
				"encrypted");
		err = -EOPNOTSUPP;
		goto bad_out;
	}
	if (a->non_resident)
		goto do_non_resident_truncate;
	DEBUG_ON(NInoNonResident(ni));
	/* Resize the attribute record to best fit the new attribute size. */
	if (new_size < vol->mft_record_size) {

		err = ntfs_resident_attr_value_resize(m, a, new_size);
		if (!err) {
			/* The resize succeeded! */
			flush_dcache_mft_record_page(ctx->ntfs_ino);
			mark_mft_record_dirty(ctx->ntfs_ino);
			write_lock_irqsave(&ni->size_lock, flags);
			/* Update the sizes in the ntfs inode
			    and all is done. */
			ni->allocated_size = le32_to_cpu(a->length) -
					le16_to_cpu(a->data.
					resident.value_offset);
		/*
		 * Note ntfs_resident_attr_value_resize() has already done any
		 * necessary data clearing in the attribute record.  When the
		 * file is being shrunk vmtruncate() will already have cleared
		 * the top part of the last partial page, i.e. since this is
		 * the resident case this is the page with index 0.  However,
		 * when the file is being expanded, the page cache page data
		 * between the old data_size, i.e. old_size, and the new_size
		 * has not been zeroed.  Fortunately, we do not need to zero it
		 * either since on one hand it will either already be zero due
		 * to both readpage and writepage clearing partial page data
		 * beyond i_size in which case there is nothing to do or in the
		 * case of the file being mmap()ped at the same time, POSIX
		 * specifies that the behaviour is unspecified thus we do not
		 * have to do anything.  This means that in our implementation
		 * in the rare case that the file is mmap()ped and a write
		 * occured into the mmap()ped region just beyond the file size
		 * and writepage has not yet been called to write out the page
		 * (which would clear the area beyond the file size) and we now
		 * extend the file size to incorporate this dirty region
		 * outside the file size, a write of the page would result in
		 * this data being written to disk instead of being cleared.
		 * Given both POSIX and the Linux mmap(2) man page specify that
		 * this corner case is undefined, we choose to leave it like
		 * that as this is much simpler for us as we cannot lock the
		 * relevant page now since we are holding too many ntfs locks
		 * which would result in a lock reversal deadlock.
		 */
			ni->initialized_size = new_size;
			write_unlock_irqrestore(&ni->size_lock, flags);
			goto unm_done;
		} else if (err > 0)
			goto bad_out;
	}
	/* If the above resize failed, this must be an attribute extension. */
	DEBUG_ON(size_change < 0);
	/*
	 * We have to drop all the locks so we can call
	 * ntfs_attr_make_non_resident().  This could be optimised by try-
	 * locking the first page cache page and only if that fails dropping
	 * the locks, locking the page, and redoing all the locking and
	 * lookups.  While this would be a huge optimisation, it is not worth
	 * it as this is definitely a slow code path as it only ever can happen
	 * once for any given file.
	 */
	up_write(&ni->runlist.lock);
	/*
	 * Not enough space in the mft record, try to make the attribute
	 * non-resident and if successful restart the truncation process.
	 */

	err = ntfs_attr_make_non_resident(ni, old_size);

	if (likely(!err)) {
		ntfs_attr_put_search_ctx(ctx);
		unmap_mft_record(base_ni);
		goto retry_truncate;
		}
	/*
	 * Could not make non-resident.  If this is due to this not being
	 * permitted for this attribute type or there not being enough space,
	 * try to make other attributes non-resident.  Otherwise fail.
	 */
	if (unlikely(err != -EPERM && err != -ENOSPC)) {
		ntfs_error(vol->sb, "Cannot truncate inode 0x%lx, attribute "
				"type 0x%x, because the conversion from "
				"resident to non-resident attribute failed "
				"with error code %i.", vi->i_ino,
				(unsigned)le32_to_cpu(ni->type), err);
		if (err != -ENOMEM)
			err = -EIO;
		ntfs_attr_put_search_ctx(ctx);
		unmap_mft_record(base_ni);
		goto conv_err_out;
	}
	/* TODO: Not implemented from here, abort. */
#if 0
	if (err == -ENOSPC)
		ntfs_error(vol->sb, "Not enough space in the mft record/on "
				"disk for the non-resident attribute value.  "
				"This case is not implemented yet.");
	else /* if (err == -EPERM) */
		ntfs_error(vol->sb, "This attribute type may not be "
				"non-resident.  This case is not implemented "
				"yet.");
	err = -EOPNOTSUPP;
	goto conv_err_out;
#endif
	/*  Attempt to make other attributes non-resident.*/
	bak_ni = ctx->ntfs_ino;
	ntfs_attr_init_search_ctx(ctx, NULL, ctx->mrec);

	while (!(ret = ntfs_attr_lookup(AT_UNUSED, NULL, 0, 0, 0,
			NULL, 0, ctx))) {
		ATTR_RECORD *a;

		a = ctx->attr;
		if (a->non_resident)
			continue;

		/*
		 * Check out whether convert is reasonable. Assume that mapping
		 * pairs will take 8 bytes.
		 */
		if (le32_to_cpu(a->length) <= offsetof(ATTR_RECORD,
				data.non_resident.compressed_size) +
				((a->name_length *
				sizeof(ntfschar) + 7) & ~7) + 8)
			continue;


		if (ntfs_test_real_inode(base_ni, a)) {
			attr_vi = VFS_I(base_ni);
			real = true;
		} else {
			attr_vi = ntfs_attr_iget(VFS_I(base_ni), a->type,
				(ntfschar *)((u8 *)a + le16_to_cpu(a->name_offset)),
				a->name_length);
			if (IS_ERR(attr_vi)) {
				err = PTR_ERR(attr_vi);
				ntfs_error(vol->sb, "Couldn't open attribute");
				goto bad_nolock_out;
			}
			real = false;
		}

		if (ntfs_attr_make_non_resident(NTFS_I(attr_vi),
				(u32)i_size_read(attr_vi))) {
			if (!real)
				iput(attr_vi);
			continue;
		}

		if (!real)
			iput(attr_vi);

		ntfs_attr_put_search_ctx(ctx);

		if (bak_ni != base_ni)
			unmap_mft_record(bak_ni);
		bak_ni = NULL;
		unmap_mft_record(base_ni);

		goto retry_truncate;
	}
	/* Check whether error occurred. */
	if (ret != -ENOENT) {
		err = ret;
		ntfs_error(vol->sb, "Attribute lookup failed 1");
		goto bad_nolock_out;
	}

	/* 
	 * The standard information and attribute list attributes can't be
	 * moved out from the base MFT record, so try to move out others. 
	 */
	if (ni->type==AT_STANDARD_INFORMATION || ni->type==AT_ATTRIBUTE_LIST) {
		if ((err = ntfs_inode_free_space(bak_ni, ctx->mrec, offsetof(ATTR_RECORD,
				data.non_resident.non_resident_end) + 8))) {
			ntfs_error(vol->sb, "Could not free space in MFT record");
			goto bad_nolock_out;
		}
		ntfs_attr_put_search_ctx(ctx);
		if (bak_ni != base_ni)
			unmap_mft_record(bak_ni);
		bak_ni = NULL;
		unmap_mft_record(base_ni);
		goto retry_truncate;
	}

	/*
	 * Move the attribute to a new mft record, creating an attribute list
	 * attribute or modifying it if it is already present.
	 */

	/* Point search context back to attribute which we need resize. */
	if (bak_ni != base_ni)
		unmap_mft_record(bak_ni);
	bak_ni = NULL;
	ntfs_attr_init_search_ctx(ctx, base_ni, m);
	if ((err = ntfs_attr_lookup(ni->type, ni->name, ni->name_len, CASE_SENSITIVE,
			0, NULL, 0, ctx))) {
		ntfs_error(vol->sb, "Attribute lookup failed 2");
		goto bad_nolock_out;
	}

	/*
	 * Check whether attribute is already single in this MFT record.
	 * 8 added for the attribute terminator.
	 */
	if (le32_to_cpu(ctx->mrec->bytes_in_use) ==
			le16_to_cpu(ctx->mrec->attrs_offset) +
			le32_to_cpu(ctx->attr->length) + 8) {
		err = -ENOSPC;
		ntfs_debug("MFT record is filled with one attribute\n");
		ret = STATUS_RESIDENT_ATTRIBUTE_FILLED_MFT;
		goto bad_nolock_out;
	}

	if (!NInoAttrList(base_ni)) {
		if ((err = ntfs_inode_add_attrlist(base_ni)))
			goto bad_nolock_out;

		ntfs_attr_put_search_ctx(ctx);
		unmap_mft_record(base_ni);
		goto retry_truncate;
	}

	/* Allocate new mft record. */
	ext_ni = ntfs_mft_record_alloc(vol, 0, base_ni, &ext_m);
	if (IS_ERR(ext_ni)) {
		err = PTR_ERR(ext_ni);
		ntfs_error(vol->sb, "Couldn't allocate new MFT record");
		goto bad_nolock_out;
	}
	/* Move attribute to it. */

	if ((err = ntfs_attr_record_move_to(ctx, ext_ni, ext_m))) {
		ntfs_error(vol->sb, "Couldn't move attribute to new MFT record");
		unmap_mft_record(ext_ni);
		goto bad_nolock_out;
	}

	ntfs_attr_put_search_ctx(ctx);
	unmap_mft_record(ext_ni);
	unmap_mft_record(base_ni);
	/* Try to perform resize once again. */

	goto retry_truncate;

do_non_resident_truncate:
	DEBUG_ON(!NInoNonResident(ni));
#if 0
	if (alloc_change < 0) {
		highest_vcn = sle64_to_cpu(a->data.non_resident.highest_vcn);
		if (highest_vcn > 0 &&
				old_alloc_size >> vol->cluster_size_bits >
				highest_vcn + 1) {
			/*
			 * This attribute has multiple extents.  Not yet
			 * supported.
			 */
			ntfs_error(vol->sb, "Cannot truncate inode 0x%lx, "
					"attribute type 0x%x, because the "
					"attribute is highly fragmented (it "
					"consists of multiple extents) and "
					"this case is not implemented yet.",
					vi->i_ino,
					(unsigned)le32_to_cpu(ni->type));
			err = -EOPNOTSUPP;
			goto bad_out;
		}
	}
#endif
	/*
	 * If the size is shrinking, need to reduce the initialized_size and
	 * the data_size before reducing the allocation.
	 */
	if (size_change < 0) {
		/*
		 * Make the valid size smaller (i_size is already up-to-date).
		 */
		write_lock_irqsave(&ni->size_lock, flags);
		if (new_size < ni->initialized_size) {
			ni->initialized_size = new_size;
			a->data.non_resident.initialized_size =
					cpu_to_sle64(new_size);
		}
		a->data.non_resident.data_size = cpu_to_sle64(new_size);
		write_unlock_irqrestore(&ni->size_lock, flags);
		flush_dcache_mft_record_page(ctx->ntfs_ino);
		mark_mft_record_dirty(ctx->ntfs_ino);
		/* If the allocated size is not changing, we are done. */
		if (!alloc_change)
			goto unm_done;
		/*
		 * If the size is shrinking it makes no sense for the
		 * allocation to be growing.
		 */
		DEBUG_ON(alloc_change > 0);
	} else /* if (size_change >= 0) */ {
		/*
		 * The file size is growing or staying the same but the
		 * allocation can be shrinking, growing or staying the same.
		 */
		if (alloc_change > 0) {
			/*
			 * We need to extend the allocation and possibly update
			 * the data size.  If we are updating the data size,
			 * since we are not touching the initialized_size we do
			 * not need to worry about the actual data on disk.
			 * And as far as the page cache is concerned, there
			 * will be no pages beyond the old data size and any
			 * partial region in the last page between the old and
			 * new data size (or the end of the page if the new
			 * data size is outside the page) does not need to be
			 * modified as explained above for the resident
			 * attribute truncate case.  To do this, we simply drop
			 * the locks we hold and leave all the work to our
			 * friendly helper ntfs_attr_extend_allocation().
			 */


			ntfs_attr_put_search_ctx(ctx);
			unmap_mft_record(base_ni);
			up_write(&ni->runlist.lock);

			ll = ntfs_attr_extend_allocation(ni, new_size,
					size_change > 0 ? new_size : -1, -1);
			err = (ll < 0 ? (int)ll:0);
			/*
			 * ntfs_attr_extend_allocation() will have done error
			 * output already.
			 */
			goto done;
		}
		if (!alloc_change)
			goto alloc_done;
	}
	/* alloc_change < 0 */
	/* Free the clusters. */

	nr_freed = ntfs_cluster_free(ni, new_alloc_size >>
			vol->cluster_size_bits, -1, ctx);
//	m = ctx->mrec;
//	a = ctx->attr;
	if (unlikely(nr_freed < 0)) {
		ntfs_error(vol->sb, "Failed to release cluster(s) (error code "
				"%lli).  Unmount and run chkdsk to recover "
				"the lost cluster(s).", (long long)nr_freed);
		NVolSetErrors(vol);
		nr_freed = 0;
	}
	/* Truncate the runlist. */
	err = ntfs_rl_truncate_nolock(vol, &ni->runlist,
			new_alloc_size >> vol->cluster_size_bits);
	/*
	 * If the runlist truncation failed and/or the search context is no
	 * longer valid, we cannot resize the attribute record or build the
	 * mapping pairs array thus we mark the inode bad so that no access to
	 * the freed clusters can happen.
	 */
	if (unlikely(err || IS_ERR(m))) {
		ntfs_error(vol->sb, "Failed to %s (error code %li).%s",
				IS_ERR(m) ?
				"restore attribute search context" :
				"truncate attribute runlist",
				IS_ERR(m) ? PTR_ERR(m) : err, es);
		err = -EIO;
		goto bad_out;
	}

	mp_rebuilt = false;
	ntfs_attr_reinit_search_ctx(ctx);
	last_vcn = (new_alloc_size >> vol->cluster_size_bits) -1;
	if(new_alloc_size == 0)
		last_vcn = 0;
	while(1) {
		err = ntfs_attr_lookup(ni->type, ni->name, ni->name_len,
				CASE_SENSITIVE, 0, NULL, 0, ctx);
		if (unlikely(err)) {
			if (err == -ENOENT) {
				if(mp_rebuilt)
					break;

				ntfs_error(vi->i_sb, "Open attribute is missing from "
						"mft record.  Inode 0x%lx is corrupt.  "
						"Run chkdsk.%s", vi->i_ino, te);
				err = -EIO;
			} else
				ntfs_error(vi->i_sb, "Failed to lookup attribute in "
						"inode 0x%lx (error code %d).%s",
						vi->i_ino, err, te);
			goto bad_out;
		}
		m = ctx->mrec;
		a = ctx->attr;
		highest_vcn = sle64_to_cpu(a->data.non_resident.highest_vcn);
		lowest_vcn = sle64_to_cpu(a->data.non_resident.lowest_vcn);
		if(last_vcn >= highest_vcn) {
			continue;
		}
		else if (last_vcn >= lowest_vcn) {
			mp_size = ntfs_get_size_for_mapping_pairs(vol, ni->runlist.rl, lowest_vcn, -1);
			if (unlikely(mp_size <= 0)) {
				ntfs_error(vol->sb, "Cannot shrink allocation of inode 0x%lx, "
						"attribute type 0x%x, because determining the "
						"size for the mapping pairs failed with error "
						"code %i.%s", vi->i_ino,
						(unsigned)le32_to_cpu(ni->type), mp_size, es);
				err = -EIO;
				goto bad_out;
			}
			/*
			 * Shrink the attribute record for the new mapping pairs array.  Note,
			 * this cannot fail since we are making the attribute smaller thus by
			 * definition there is enough space to do so.
			 */
			attr_len = le32_to_cpu(a->length);
			err = ntfs_attr_record_resize(m, a, mp_size +
					le16_to_cpu(a->data.non_resident.mapping_pairs_offset));
			DEBUG_ON(err);
			/*
			 * Generate the mapping pairs array directly into the attribute record.
			 */
			err = ntfs_mapping_pairs_build(vol, (u8 *)a +
					le16_to_cpu(a->data.non_resident.mapping_pairs_offset),
					mp_size, ni->runlist.rl, lowest_vcn, -1, NULL);
			if (unlikely(err)) {
				ntfs_error(vol->sb, "Cannot shrink allocation of inode 0x%lx, "
						"attribute type 0x%x, because building the "
						"mapping pairs failed with error code %i.%s",
						vi->i_ino, (unsigned)le32_to_cpu(ni->type),
						err, es);
				err = -EIO;
				goto bad_out;
			}
			/* Update the allocated/compressed size as well as the highest vcn. */
			a->data.non_resident.highest_vcn = cpu_to_sle64((new_alloc_size >>
					vol->cluster_size_bits) - 1);
			mp_rebuilt = true;
		} else {
			err = ntfs_attr_record_rm(ctx);
			if(err)
				goto bad_out;
			mp_rebuilt = true;
		}
	}

	ntfs_attr_reinit_search_ctx(ctx);
	err = ntfs_attr_lookup(ni->type, ni->name, ni->name_len,
			CASE_SENSITIVE, 0, NULL, 0, ctx);
	if (unlikely(err)) {
		if (err == -ENOENT) {
			ntfs_error(vi->i_sb, "Open attribute is missing from "
					"mft record.  Inode 0x%lx is corrupt.  "
					"Run chkdsk.%s", vi->i_ino, te);
			err = -EIO;
		} else
			ntfs_error(vi->i_sb, "Failed to lookup attribute in "
					"inode 0x%lx (error code %d).%s",
					vi->i_ino, err, te);
		goto bad_out;
	}

	/* Get the size for the shrunk mapping pairs array for the runlist. */
	write_lock_irqsave(&ni->size_lock, flags);
	ni->allocated_size = new_alloc_size;
	a->data.non_resident.allocated_size = cpu_to_sle64(new_alloc_size);
	if (NInoSparse(ni) || NInoCompressed(ni)) {
		if (nr_freed) {
			ni->itype.compressed.size -= nr_freed <<
					vol->cluster_size_bits;
			DEBUG_ON(ni->itype.compressed.size < 0);
			a->data.non_resident.compressed_size = cpu_to_sle64(
					ni->itype.compressed.size);
			vi->i_blocks = ni->itype.compressed.size >> 9;
		}
	} else
		vi->i_blocks = new_alloc_size >> 9;
	write_unlock_irqrestore(&ni->size_lock, flags);
	/*
	 * We have shrunk the allocation.  If this is a shrinking truncate we
	 * have already dealt with the initialized_size and the data_size above
	 * and we are done.  If the truncate is only changing the allocation
	 * and not the data_size, we are also done.  If this is an extending
	 * truncate, need to extend the data_size now which is ensured by the
	 * fact that @size_change is positive.
	 */
alloc_done:
	/*
	 * If the size is growing, need to update it now.  If it is shrinking,
	 * we have already updated it above (before the allocation change).
	 */
	if (size_change > 0) {
		a->data.non_resident.data_size = cpu_to_sle64(new_size);
		a->data.non_resident.initialized_size = cpu_to_sle64(new_size);
		ni->initialized_size = cpu_to_sle64(new_size);
	}
	/* Ensure the modified mft record is written out. */
	flush_dcache_mft_record_page(ctx->ntfs_ino);
	mark_mft_record_dirty(ctx->ntfs_ino);
unm_done:
	ntfs_attr_put_search_ctx(ctx);
	unmap_mft_record(base_ni);
	up_write(&ni->runlist.lock);
done:


	/* Update the mtime and ctime on the base inode. */
	/* normally ->truncate shouldn't update ctime or mtime,
	 * but ntfs did before so it got a copy & paste version
	 * of file_update_time.  one day someone should fix this
	 * for real.
	 */
	if (!IS_NOCMTIME(VFS_I(base_ni)) && !IS_RDONLY(VFS_I(base_ni))) {
		struct timespec now = current_fs_time(VFS_I(base_ni)->i_sb);
		int sync_it = 0;

		if (!timespec_equal(&VFS_I(base_ni)->i_mtime, &now) ||
		    !timespec_equal(&VFS_I(base_ni)->i_ctime, &now))
			sync_it = 1;
		VFS_I(base_ni)->i_mtime = now;
		VFS_I(base_ni)->i_ctime = now;

		if (sync_it)
			mark_inode_dirty_sync(VFS_I(base_ni));
	}

	if (likely(!err)) {
		NInoClearTruncateFailed(ni);
		ntfs_debug("Done.");
	}
	return err;
old_bad_out:
	old_size = -1;
bad_out:
	if (err < 0 && err != -ENOMEM && err != -EOPNOTSUPP)
		NVolSetErrors(vol);
	if (err < 0 && err != -EOPNOTSUPP)
		NInoSetTruncateFailed(ni);
	else if (old_size >= 0)
		i_size_write(vi, old_size);
err_out:
	if (ctx)
		ntfs_attr_put_search_ctx(ctx);
	if (m)
		unmap_mft_record(base_ni);
	up_write(&ni->runlist.lock);
out:
	ntfs_debug("Failed.  Returning error code %i.", err);
	return err;
conv_err_out:
	if (err < 0 && err != -ENOMEM && err != -EOPNOTSUPP)
		NVolSetErrors(vol);
	if (err < 0 && err != -EOPNOTSUPP)
		NInoSetTruncateFailed(ni);
	else
		i_size_write(vi, old_size);
	goto out;
bad_nolock_out:
	ntfs_attr_put_search_ctx(ctx);
	if ((bak_ni) && (bak_ni != base_ni))
		unmap_mft_record(bak_ni);
	unmap_mft_record(base_ni);
	goto conv_err_out;
}

/**
 * ntfs_truncate_vfs - wrapper for ntfs_truncate() that has no return value
 * @vi:		inode for which the i_size was changed
 *
 * Wrapper for ntfs_truncate() that has no return value.
 *
 * See ntfs_truncate() description above for details.
 */
void ntfs_truncate_vfs(struct inode *vi)
{
	ntfs_truncate(vi);
}

/**
 * ntfs_setattr - called from notify_change() when an attribute is being changed
 * @dentry:	dentry whose attributes to change
 * @attr:	structure describing the attributes and the changes
 *
 * We have to trap VFS attempts to truncate the file described by @dentry as
 * soon as possible, because we do not implement changes in i_size yet.  So we
 * abort all i_size changes here.
 *
 * We also abort all changes of user, group, and mode as we do not implement
 * the NTFS ACLs yet.
 *
 * Called with ->i_mutex held.  For the ATTR_SIZE (i.e. ->truncate) case, also
 * called with ->i_alloc_sem held for writing.
 *
 * Basically this is a copy of generic notify_change() and inode_setattr()
 * functionality, except we intercept and abort changes in i_size.
 */
int ntfs_setattr(struct dentry *dentry, struct iattr *attr)
{
	struct inode *vi = dentry->d_inode;
	int err;
	unsigned int ia_valid = attr->ia_valid;

	err = inode_change_ok(vi, attr);
	if (err)
		goto out;
	/* We do not support NTFS ACLs yet. */
	if (ia_valid & (ATTR_UID | ATTR_GID | ATTR_MODE)) {
		ntfs_warning(vi->i_sb, "Changes in user/group/mode are not "
				"supported yet, ignoring.");
		err = -EOPNOTSUPP;
		goto out;
	}
	if (ia_valid & ATTR_SIZE) {
		if (attr->ia_size != i_size_read(vi)) {
			ntfs_inode *ni = NTFS_I(vi);
			/*
			 * FIXME: For now we do not support resizing of
			 * compressed or encrypted files yet.
			 */
			if (NInoCompressed(ni) || NInoEncrypted(ni)) {
				ntfs_warning(vi->i_sb, "Changes in inode size "
						"are not supported yet for "
						"%s files, ignoring.",
						NInoCompressed(ni) ?
						"compressed" : "encrypted");
				err = -EOPNOTSUPP;
			} else
				err = vmtruncate(vi, attr->ia_size);
			if (err || ia_valid == ATTR_SIZE)
				goto out;
		} else {
			/*
			 * We skipped the truncate but must still update
			 * timestamps.
			 */
			ia_valid |= ATTR_MTIME | ATTR_CTIME;
		}
	}
	if (ia_valid & ATTR_ATIME)
		vi->i_atime = timespec_trunc(attr->ia_atime,
				vi->i_sb->s_time_gran);
	if (ia_valid & ATTR_MTIME)
		vi->i_mtime = timespec_trunc(attr->ia_mtime,
				vi->i_sb->s_time_gran);
	if (ia_valid & ATTR_CTIME)
		vi->i_ctime = timespec_trunc(attr->ia_ctime,
				vi->i_sb->s_time_gran);
	mark_inode_dirty(vi);
out:
	return err;
}

/**
 * ntfs_write_inode - write out a dirty inode
 * @vi:		inode to write out
 * @sync:	if true, write out synchronously
 *
 * Write out a dirty inode to disk including any extent inodes if present.
 *
 * If @sync is true, commit the inode to disk and wait for io completion.  This
 * is done using write_mft_record().
 *
 * If @sync is false, just schedule the write to happen but do not wait for i/o
 * completion.  In 2.6 kernels, scheduling usually happens just by virtue of
 * marking the page (and in this case mft record) dirty but we do not implement
 * this yet as write_mft_record() largely ignores the @sync parameter and
 * always performs synchronous writes.
 *
 * Return 0 on success and -errno on error.
 */
int ntfs_write_inode(struct inode *vi, int sync)
{
	sle64 nt;
	ntfs_inode *ni = NTFS_I(vi);
	ntfs_attr_search_ctx *ctx;
	MFT_RECORD *m;
	STANDARD_INFORMATION *si;
	int err = 0;
	bool modified = false;
	ATTR_RECORD *a;
	ntfs_debug("Entering for %sinode 0x%lx.", NInoAttr(ni) ? "attr " : "",
			vi->i_ino);
	/*
	 * Dirty attribute inodes are written via their real inodes so just
	 * clean them here.  Access time updates are taken care off when the
	 * real inode is written.
	 */
	if (NInoAttr(ni)) {
		NInoClearDirty(ni);
		ntfs_debug("Done.");
		return 0;
	}
	/* Map, pin, and lock the mft record belonging to the inode. */
	m = map_mft_record_lock(ni);
	if (IS_ERR(m)) {
		err = PTR_ERR(m);
		goto err_out;
	}

	m->link_count = cpu_to_le16(vi->i_nlink);
	/* Update the access times in the standard information attribute. */
	ctx = ntfs_attr_get_search_ctx(ni, m);
	if (unlikely(!ctx)) {
		err = -ENOMEM;
		goto unm_err_out;
	}
	err = ntfs_attr_lookup(AT_STANDARD_INFORMATION, NULL, 0,
			CASE_SENSITIVE, 0, NULL, 0, ctx);
	if (unlikely(err)) {
		ntfs_attr_put_search_ctx(ctx);
		goto unm_err_out;
	}
	si = (STANDARD_INFORMATION *)((u8 *)ctx->attr +
			le16_to_cpu(ctx->attr->data.resident.value_offset));
	/* Update the access times if they have changed. */
	nt = utc2ntfs(vi->i_mtime);
	if (si->last_data_change_time != nt) {
		ntfs_debug("Updating mtime for inode 0x%lx: old = 0x%llx, "
				"new = 0x%llx", vi->i_ino, (long long)
				sle64_to_cpu(si->last_data_change_time),
				(long long)sle64_to_cpu(nt));
		si->last_data_change_time = nt;
		modified = true;
	}
	nt = utc2ntfs(vi->i_ctime);
	if (si->last_mft_change_time != nt) {
		ntfs_debug("Updating ctime for inode 0x%lx: old = 0x%llx, "
				"new = 0x%llx", vi->i_ino, (long long)
				sle64_to_cpu(si->last_mft_change_time),
				(long long)sle64_to_cpu(nt));
		si->last_mft_change_time = nt;
		modified = true;
	}
	nt = utc2ntfs(vi->i_atime);
	if (si->last_access_time != nt) {
		ntfs_debug("Updating atime for inode 0x%lx: old = 0x%llx, "
				"new = 0x%llx", vi->i_ino,
				(long long)sle64_to_cpu(si->last_access_time),
				(long long)sle64_to_cpu(nt));
		si->last_access_time = nt;
		modified = true;
	}

	if (ni->flag != si->file_attributes) {
		si->file_attributes = ni->flag;
		modified = true;
	}

	if (NInoAttrList(ni) && NInoAttrListDirty(ni)) {
		ntfs_attr_reinit_search_ctx(ctx);
		err = ntfs_attr_lookup(AT_ATTRIBUTE_LIST, NULL, 0, 0,
				0, NULL, 0, ctx);
		if (err) {
			ntfs_error(vi->i_sb, "Failed to lookup attribute"
					" list attribute,err= %d.", err);
			goto err_put_out;
		} else /* if (!err) */ {
			ntfs_debug("Attribute list found in inode 0x%lx.",
					vi->i_ino);
			a = ctx->attr;
			if (a->flags & ATTR_COMPRESSION_MASK) {
				ntfs_error(vi->i_sb, "Attribute list "
						"attribute is compressed.");
				goto err_put_out;
			}
			if (a->flags & ATTR_IS_ENCRYPTED ||
					a->flags & ATTR_IS_SPARSE) {
				if (a->non_resident) {
					ntfs_error(vi->i_sb, "Non-resident "
						"attribute list attribute is"
							" encrypted/sparse.");
					goto err_put_out;
				}
				ntfs_warning(vi->i_sb, "Resident attribute"
					" list attribute in inode 0x%lx is"
					" marked encrypted/sparse which is not"
					" true.  However, Windows allows this"
					" and chkdsk does not detect or correct"
					" it so we will just ignore the "
					"invalid flags and pretend they are "
					"not set.", vi->i_ino);
			}
			/* Now allocate memory for the attribute list. */
			if (a->non_resident) {
				NInoSetAttrListNonResident(ni);
				if (a->data.non_resident.lowest_vcn) {
					ntfs_error(vi->i_sb, "Attribute list "
						"has non zero lowest_vcn.");
					goto err_put_out;
				}
				/*
				 * Setup the runlist. No need for locking as
				 * we have exclusive access to the inode at
				 * this time.
				 */
				ni->attr_list_rl.rl =
						ntfs_mapping_pairs_decompress(
						ni->vol, a, NULL);
				if (IS_ERR(ni->attr_list_rl.rl)) {
					err = PTR_ERR(ni->attr_list_rl.rl);
					ni->attr_list_rl.rl = NULL;
					ntfs_error(vi->i_sb, "Mapping pairs "
						"decompression failed.");
					goto err_put_out;
				}

				/* Now load the attribute list. */
				err = write_attribute_list(ni->vol,
					&ni->attr_list_rl,
					ni->attr_list, ni->attr_list_size,
					sle64_to_cpu(a->data.non_resident.
					initialized_size));
				if (err) {
					ntfs_error(vi->i_sb, "Failed to load "
						"attribute list attribute.");
					goto err_put_out;
				}
			} else /* if (!a->non_resident) */ {
				if ((u8 *)a + le16_to_cpu(a->data.resident.
						value_offset) + le32_to_cpu(
						a->data.resident.value_length) >
						(u8 *)ctx->mrec +
						ni->vol->mft_record_size) {
					ntfs_error(vi->i_sb, "Corrupt attribute"
						" list in inode.");
					goto err_put_out;
				}
				/* Now copy the attribute list. */
				memcpy((u8 *)a + le16_to_cpu(
					a->data.resident.value_offset),
					ni->attr_list, le32_to_cpu(
					a->data.resident.value_length));
			modified = true;
			}

		}

		NInoClearAttrListDirty(ni);
	}

	/*
	 * If we just modified the standard information attribute we need to
	 * mark the mft record it is in dirty.  We do this manually so that
	 * mark_inode_dirty() is not called which would redirty the inode and
	 * hence result in an infinite loop of trying to write the inode.
	 * There is no need to mark the base inode nor the base mft record
	 * dirty, since we are going to write this mft record below in any case
	 * and the base mft record may actually not have been modified so it
	 * might not need to be written out.
	 * NOTE: It is not a problem when the inode for $MFT itself is being
	 * written out as mark_ntfs_record_dirty() will only set I_DIRTY_PAGES
	 * on the $MFT inode and hence ntfs_write_inode() will not be
	 * re-invoked because of it which in turn is ok since the dirtied mft
	 * record will be cleaned and written out to disk below, i.e. before
	 * this function returns.
	 */
	if (modified) {
		flush_dcache_mft_record_page(ctx->ntfs_ino);
		if (!NInoTestSetDirty(ctx->ntfs_ino))
			mark_ntfs_record_dirty(ctx->ntfs_ino->page,
					ctx->ntfs_ino->page_ofs);
	}

	ntfs_attr_put_search_ctx(ctx);
	/* Now the access times are updated, write the base mft record. */
	if (NInoDirty(ni))
		err = write_mft_record(ni, m, sync);
	/* Write all attached extent mft records. */
	mutex_lock(&ni->extent_lock);
	if (ni->nr_extents > 0) {
		ntfs_inode **extent_nis = ni->ext.extent_ntfs_inos;
		int i;

		ntfs_debug("Writing %i extent inodes.", ni->nr_extents);
		for (i = 0; i < ni->nr_extents; i++) {
			ntfs_inode *tni = extent_nis[i];

			if (NInoDirty(tni)) {
				MFT_RECORD *tm = map_mft_record(tni);
				int ret;

				if (IS_ERR(tm)) {
					if (!err || err == -ENOMEM)
						err = PTR_ERR(tm);
					continue;
				}
				ret = write_mft_record(tni, tm, sync);
				unmap_mft_record(tni);
				if (unlikely(ret)) {
					if (!err || err == -ENOMEM)
						err = ret;
				}
			}
		}
	}
	mutex_unlock(&ni->extent_lock);

	unmap_mft_record_unlock(ni);

	if (unlikely(err))
		goto err_out;
	ntfs_debug("Done.");
	return 0;
err_put_out:
	ntfs_attr_put_search_ctx(ctx);
unm_err_out:
	unmap_mft_record_unlock(ni);
err_out:
	if (err == -ENOMEM) {
		ntfs_warning(vi->i_sb, "Not enough memory to write inode.  "
				"Marking the inode dirty again, so the VFS "
				"retries later.");
		mark_inode_dirty(vi);
	} else {
		ntfs_error(vi->i_sb, "Failed (error %d):  Run chkdsk.", err);
		NVolSetErrors(ni->vol);
	}
	return err;
}

/**
 * ntfs_inode_free_space - free space in the MFT record of an inode
 * @ni:		ntfs inode in which MFT record needs more free space
 * @size:	amount of space needed to free
 *
 * Return 0 on success or -1 on error with errno set to the error code.
 */
int ntfs_inode_free_space(ntfs_inode *ni, MFT_RECORD *m, int size)
{
	ntfs_attr_search_ctx *ctx;
	struct super_block *sb = ni->vol->sb;
	int freed;
	int err;

	if (!ni || !m || size < 0) {
		ntfs_error(sb, "%s: ni=%p size=%d", __func__, ni, size);
		return -EINVAL;
	}

	ntfs_debug("Entering for inode %lld, size %d\n",
		       (unsigned long long)ni->mft_no, size);

	freed = (le32_to_cpu(m->bytes_allocated) -
				le32_to_cpu(m->bytes_in_use));
	if (size <= freed)
		return 0;

	ctx = ntfs_attr_get_search_ctx(ni, m);
	if (unlikely(!ctx))
		return -ENOMEM;
	/*
	 * $STANDARD_INFORMATION and $ATTRIBUTE_LIST must stay in the base MFT
	 * record, so position search context on the first attribute after them.
	 */
	 err = ntfs_attr_position(AT_FILE_NAME, ctx);
	if (err)
		goto put_err_out;
	while (1) {
		int record_size;
		/*
		 * Check whether attribute is from different MFT record. If so,
		 * find next, because we don't need such.
		 */
		while (ctx->ntfs_ino->mft_no != ni->mft_no) {
retry:
			err = ntfs_attr_position(AT_UNUSED, ctx);
			if (err)
				goto put_err_out;
		}

		if (ntfs_inode_base(ctx->ntfs_ino)->mft_no == FILE_MFT &&
		    ctx->attr->type == AT_DATA)
			goto retry;

		if (ctx->attr->type == AT_INDEX_ROOT)
			goto retry;

		record_size = le32_to_cpu(ctx->attr->length);
		err = ntfs_attr_record_move_away(ctx, 0);
		if (err) {
			ntfs_error(sb, "Failed to move out attribute #2");
			break;
		}
		freed += record_size;

		/* Check whether we are done. */
		if (size <= freed) {
			ntfs_attr_put_search_ctx(ctx);
			return 0;
		}
		/*
		 * Reposition to first attribute after $STANDARD_INFORMATION
		 * and $ATTRIBUTE_LIST instead of simply skipping this attribute
		 * because in the case when we have got only in-memory attribute
		 * list then ntfs_attr_lookup will fail when it tries to find
		 * $ATTRIBUTE_LIST.
		 */
		ntfs_attr_reinit_search_ctx(ctx);
		if (ntfs_attr_position(AT_FILE_NAME, ctx))
			break;
	}
put_err_out:
	ntfs_attr_put_search_ctx(ctx);
	if (err == -ENOSPC)
		ntfs_debug("No attributes left that could be moved out.\n");
	return err;
}


/**
 * ntfs_inode_add_attrlist - add attribute list to inode and fill it
 * @ni: opened ntfs inode to which add attribute list
 *
 * Return 0 on success or -1 on error with errno set to the error code.
 * The following error codes are defined:
 *	EINVAL	- Invalid arguments were passed to the function.
 *	EEXIST	- Attribute list already exist.
 *	EIO	- Input/Ouput error occurred.
 *	ENOMEM	- Not enough memory to perform add.
 */
int ntfs_inode_add_attrlist(ntfs_inode *ni)
{
	int err, ret;
	ntfs_attr_search_ctx *ctx;
	MFT_RECORD *m;
	u8 *al = NULL, *aln;
	int al_len = 0;
	ATTR_LIST_ENTRY *ale = NULL;
	struct super_block *sb = ni->vol->sb;
	struct inode *alvi;

	if (!ni) {
		ntfs_error(sb, "%s", __func__);
		return -EINVAL;
	}

	ntfs_debug("inode %llu\n", (unsigned long long) ni->mft_no);

	if (NInoAttrList(ni) || ni->nr_extents) {
		ntfs_error(sb, "Inode already has attribute list");
		return -EEXIST;
	}

	m = map_mft_record(ni);
	if (IS_ERR(m)) {
		err = PTR_ERR(m);
		goto err_out;
	}

	/* Form attribute list. */
	ctx = ntfs_attr_get_search_ctx(ni, m);
	if (unlikely(!ctx)) {
		err = -ENOMEM;
		goto unmapout;
	}
	/* Walk through all attributes. */
	while (!(ret = ntfs_attr_lookup(AT_UNUSED, NULL,
			0, 0, 0, NULL, 0, ctx))) {
		int ale_size;

		if (ctx->attr->type == AT_ATTRIBUTE_LIST) {
			err = -EIO;
			ntfs_error(sb, "Attribute list already present");
			goto put_err_out;
		}

		ale_size = (sizeof(ATTR_LIST_ENTRY) + sizeof(ntfschar) *
					ctx->attr->name_length + 7) & ~7;
		al_len += ale_size;
		aln = krealloc(al, al_len, GFP_NOFS);
		if (!aln) {
			err = -ENOMEM;
			ntfs_error(sb, "Failed to realloc %d bytes", al_len);
			goto put_err_out;
		}
		ale = (ATTR_LIST_ENTRY *)(aln + ((u8 *)ale - al));
		al = aln;

		memset(ale, 0, ale_size);

		/* Add attribute to attribute list. */
		ale->type = ctx->attr->type;
		ale->length = cpu_to_le16((sizeof(ATTR_LIST_ENTRY) +
			sizeof(ntfschar) * ctx->attr->name_length + 7) & ~7);
		ale->name_length = ctx->attr->name_length;
		ale->name_offset = (u8 *)ale->name - (u8 *)ale;
		if (ctx->attr->non_resident)
			ale->lowest_vcn = ctx->attr->
					data.non_resident.lowest_vcn;
		else
			ale->lowest_vcn = 0;
		ale->mft_reference = MK_LE_MREF(ni->mft_no,
			le16_to_cpu(m->sequence_number));
		ale->instance = ctx->attr->instance;
		memcpy(ale->name, (u8 *)ctx->attr +
				le16_to_cpu(ctx->attr->name_offset),
				ctx->attr->name_length * sizeof(ntfschar));
		ale = (ATTR_LIST_ENTRY *)(al + al_len);
	}
	/* Check for real error occurred. */
	if (ret != -ENOENT) {
		err = ret;
		ntfs_error(sb, "%s: Attribute lookup failed, inode %lld",
				__func__, (long long)ni->mft_no);
		goto put_err_out;
	}

	/* Set in-memory attribute list. */
	ni->attr_list = al;
	ni->attr_list_size = al_len;
	NInoSetAttrList(ni);
	ntfs_attrlist_mark_dirty(ni);

	/* Free space if there is not enough it for $ATTRIBUTE_LIST. */
	if (le32_to_cpu(m->bytes_allocated) -
			le32_to_cpu(m->bytes_in_use) <
			offsetof(ATTR_RECORD, data.resident.resident_end)) {
		err = ntfs_inode_free_space(ni, m,
				offsetof(ATTR_RECORD,
				data.resident.resident_end));
		if (err) {
			/* Failed to free space. */
			ntfs_error(sb, "Failed to free space for attrlist");
			goto rollback;
		}
	}
	/* Add $ATTRIBUTE_LIST to mft record. */
	err = ntfs_resident_attr_record_add(ni, m,
				AT_ATTRIBUTE_LIST, NULL, 0, NULL, 0, 0);
	if (err < 0) {
		ntfs_error(NULL, "Couldn't add $ATTRIBUTE_LIST to MFT");
		goto rollback;
	}

	/* Resize it. */
	alvi = ntfs_attr_iget(VFS_I(ni), AT_ATTRIBUTE_LIST, NULL, 0);
	if (IS_ERR(alvi)) {
		err = PTR_ERR(alvi);
		ntfs_error(sb, "Failed to open just added $ATTRIBUTE_LIST");
		goto remove_attrlist_record;
	}

	i_size_write(alvi, al_len);
	err = ntfs_truncate(alvi);
	if (err) {
		ntfs_error(sb, "Failed to resize just added $ATTRIBUTE_LIST");
		iput(alvi);
		goto remove_attrlist_record;;
	}

	ntfs_attr_put_search_ctx(ctx);
	iput(alvi);
	unmap_mft_record(ni);
	return 0;

remove_attrlist_record:
	/* Prevent ntfs_attr_recorm_rm from freeing attribute list. */
	ni->attr_list = NULL;
	NInoClearAttrList(ni);
	/* Remove $ATTRIBUTE_LIST record. */
	ntfs_attr_reinit_search_ctx(ctx);
	if (!ntfs_attr_lookup(AT_ATTRIBUTE_LIST, NULL, 0,
				CASE_SENSITIVE, 0, NULL, 0, ctx)) {
		if (ntfs_attr_record_rm(ctx))
			ntfs_error(sb, "Rollback failed to remove attrlist");
	} else
		ntfs_error(sb, "Rollback failed to find attrlist");
	/* Setup back in-memory runlist. */
	ni->attr_list = al;
	ni->attr_list_size = al_len;
	NInoSetAttrList(ni);
rollback:
	/*
	 * Scan attribute list for attributes that placed not in the base MFT
	 * record and move them to it.
	 */
	ntfs_attr_reinit_search_ctx(ctx);
	ale = (ATTR_LIST_ENTRY *)al;
	while ((u8 *)ale < al + al_len) {
		if (MREF_LE(ale->mft_reference) != ni->mft_no) {
			if (!ntfs_attr_lookup(ale->type, ale->name,
						ale->name_length,
						CASE_SENSITIVE,
						sle64_to_cpu(ale->lowest_vcn),
						NULL, 0, ctx)) {
				if (ntfs_attr_record_move_to(ctx, ni , m))
					ntfs_error(sb, "Rollback failed to "
							"move attribute");
			} else
				ntfs_error(sb, "Rollback failed to find attr");
			ntfs_attr_reinit_search_ctx(ctx);
		}
		ale = (ATTR_LIST_ENTRY *)((u8 *)ale + le16_to_cpu(ale->length));
	}
	/* Remove in-memory attribute list. */
	ni->attr_list = NULL;
	ni->attr_list_size = 0;
	NInoClearAttrList(ni);
	NInoClearAttrListDirty(ni);
put_err_out:
	ntfs_attr_put_search_ctx(ctx);
unmapout:
	unmap_mft_record(ni);
err_out:
	kfree(al);
	return err;
}

int ntfs_generic_ioctl(struct inode *inode, struct file *filp,
		      unsigned int cmd, unsigned long arg)
{
	ntfs_inode *ni = NTFS_I(inode);
	u32 __user *user_attr = (u32 __user *)arg;

	switch (cmd) {
	case FAT_IOCTL_GET_ATTRIBUTES:
	{
		u32 attr;

		attr = le32_to_cpu(ni->flag);

		return put_user(attr, user_attr);
	}
	case FAT_IOCTL_SET_ATTRIBUTES:
	{
		u32 attr, oldattr;
		int err;
		struct iattr ia;

		err = get_user(attr, user_attr);
		if (err)
			return err;

		mutex_lock(&inode->i_mutex);

		err = mnt_want_write(filp->f_path.mnt);
		if (err)
			goto up_no_drop_write;

		/*
		 * ATTR_VOLUME and ATTR_DIR cannot be changed; this also
		 * prevents the user from turning us into a VFAT
		 * longname entry.  Also, we obviously can't set
		 * any of the NTFS attributes in the high 24 bits.
		 */
		oldattr = le32_to_cpu(ni->flag);

		/* Equivalent to a chmod() */
		ia.ia_valid =  ATTR_CTIME;
		ia.ia_ctime = current_fs_time(inode->i_sb);

		/* The system file  has no attributes */
		if (inode->i_ino < 24) {
			err = -EINVAL;
			goto up;
		}

		/*
		 * The security check is questionable...  We single
		 * out the RO attribute for checking by the security
		 * module, just because it maps to a file mode.
		 */
		err = security_inode_setattr(filp->f_path.dentry, &ia);
		if (err)
			goto up;

		/* This MUST be done before doing anything irreversible... */
		err = ntfs_setattr(filp->f_path.dentry, &ia);
		if (err)
			goto up;

		fsnotify_change(filp->f_path.dentry, ia.ia_valid);

		ni->flag = cpu_to_le32(attr);
		mark_inode_dirty(inode);
up:
		mnt_drop_write(filp->f_path.mnt);
up_no_drop_write:
		mutex_unlock(&inode->i_mutex);
		return err;
	}
	default:
		return -ENOTTY;	/* Inappropriate ioctl for device */
	}
}



#endif /* NTFS_RW */
