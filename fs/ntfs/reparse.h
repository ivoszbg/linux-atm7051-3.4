/*
 *
 * Copyright (c) 2008 Jean-Pierre Andre
 *
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program (in the main directory of the NTFS-3G
 * distribution in the file COPYING); if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef REPARSE_H
#define REPARSE_H

#include "layout.h"
#include "volume.h"
#include "types.h"
#include "runlist.h"
#include "debug.h"
#include "inode.h"

char *ntfs_make_symlink(const char *org_path,
			ntfs_inode *ni, int *pattr_size);
int ntfs_possible_symlink(ntfs_inode *ni);

int ntfs_get_ntfs_reparse_data(const char *path,
			char *value, size_t size, ntfs_inode *ni);
int ntfs_set_ntfs_reparse_data(const char *path, const char *value,
			size_t size, int flags, ntfs_inode *ni);
int ntfs_remove_ntfs_reparse_data(const char *path, ntfs_inode *ni);

int ntfs_delete_reparse_index(ntfs_inode *ni);

#endif /* REPARSE_H */

