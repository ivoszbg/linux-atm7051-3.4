/*
 * collate.h - Defines for NTFS kernel collation handling.  Part of the
 *	       Linux-NTFS project.
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

#ifndef _LINUX_NTFS_COLLATE_H
#define _LINUX_NTFS_COLLATE_H

#include "types.h"
#include "volume.h"

static inline bool ntfs_is_collation_rule_supported(COLLATION_RULE cr)
{
	/*
	 * FIXME:  At the moment we only support COLLATION_BINARY and
	 * COLLATION_NTOFS_ULONG, so we return false for everything else for
	 * now.
	 */
	if (cr != COLLATION_BINARY && cr != COLLATION_NTOFS_ULONG
			&& cr != COLLATION_FILE_NAME
			&& cr != COLLATION_NTOFS_SECURITY_HASH
			&& cr != COLLATION_NTOFS_ULONGS)
		return false;
	return true;
}

#define NTFS_COLLATION_ERROR -2
extern int ntfs_collate(ntfs_volume *vol, COLLATION_RULE cr,
		const void *data1, const int data1_len,
		const void *data2, const int data2_len);

#endif /* _LINUX_NTFS_COLLATE_H */
