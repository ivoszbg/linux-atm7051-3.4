/*
 * Referenced functions exfat_nfs_get_inode,exfat_fh_to_dentry,exfat_fh_to_parent
 * from fs/fat/nfs.c and from fs/libfs.c.
 */

/*
 *  Copyright (C) 2012-2013 Samsung Electronics Co., Ltd.
 *
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version 2
 *  of the License, or (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

/*
 * Support for NFS in ExFat filesystem
 */

#include <linux/exportfs.h>
#include "exfat.h"

static struct inode *exfat_nfs_get_inode(struct super_block *sb,
						u64 ino, u32 generation)
{
	struct inode *inode;

	if (ino < EXFAT_ROOT_INO)
		return NULL;

	inode = ilookup(sb, ino);
	if (inode && generation && (inode->i_generation != generation)) {
		iput(inode);
		inode = NULL;
	}

	return inode;
}

/**
 *  Map a NFS file handle to a corresponding dentry.
 *  The dentry may or may not be connected to the filesystem root.
 */
struct dentry *exfat_fh_to_dentry(struct super_block *sb, struct fid *fid,
						int fh_len, int fh_type)
{
	struct inode *inode = NULL;

	if (fh_len < 2)
		return NULL;

	switch (fh_type) {
	case FILEID_INO32_GEN:
	case FILEID_INO32_GEN_PARENT:
		inode = exfat_nfs_get_inode(sb, fid->i32.ino, fid->i32.gen);
		break;
	}

	return d_obtain_alias(inode);
}

/*
 * Find the parent for a file specified by NFS handle.
 * This requires that the handle contain the i_ino of the parent.
 */
struct dentry *exfat_fh_to_parent(struct super_block *sb, struct fid *fid,
						int fh_len, int fh_type)
{
	struct inode *inode = NULL;

	if (fh_len <= 2)
		return NULL;

	switch (fh_type) {
	case FILEID_INO32_GEN_PARENT:
		inode = exfat_nfs_get_inode(sb, fid->i32.parent_ino,
				  (fh_len > 3 ? fid->i32.parent_gen : 0));
		break;
	}

	return d_obtain_alias(inode);
}
