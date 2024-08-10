
#include "sysctl.h"
#include "logfile.h"
#include "quota.h"
#include "usnjrnl.h"
#include "dir.h"
#include "debug.h"
#include "index.h"
#include "aops.h"
#include "layout.h"
#include "malloc.h"
#include "ntfs.h"
#include "security.h"

/*
 *	Create a default security descriptor for files whose descriptor
 *	cannot be inherited
 */

int ntfs_sd_add_everyone(ntfs_inode *ni)
{
	/* JPA SECURITY_DESCRIPTOR_ATTR *sd; */
	SECURITY_DESCRIPTOR_RELATIVE *sd;
	ACL *acl;
	ACCESS_ALLOWED_ACE *ace;
	SID *sid;
	int ret, sd_len;

	/* Create SECURITY_DESCRIPTOR attribute (everyone has full access). */
	/*
	 * Calculate security descriptor length. We have 2 sub-authorities in
	 * owner and group SIDs, but structure SID contain only one, so add
	 * 4 bytes to every SID.
	 */
	sd_len = sizeof(SECURITY_DESCRIPTOR_ATTR) + 2 * (sizeof(SID) + 4) +
		sizeof(ACL) + sizeof(ACCESS_ALLOWED_ACE);
	sd = (SECURITY_DESCRIPTOR_RELATIVE *)ntfs_malloc_nofs(sd_len);
	if (!sd)
		return -ENOMEM;

	sd->revision = SECURITY_DESCRIPTOR_REVISION;
	sd->control = SE_DACL_PRESENT | SE_SELF_RELATIVE;

	sid = (SID *)((u8 *)sd + sizeof(SECURITY_DESCRIPTOR_ATTR));
	sid->revision = SID_REVISION;
	sid->sub_authority_count = 2;
	sid->sub_authority[0] = const_cpu_to_le32(SECURITY_BUILTIN_DOMAIN_RID);
	sid->sub_authority[1] = const_cpu_to_le32(DOMAIN_ALIAS_RID_ADMINS);
	sid->identifier_authority.value[5] = 5;
	sd->owner = cpu_to_le32((u8 *)sid - (u8 *)sd);

	sid = (SID *)((u8 *)sid + sizeof(SID) + 4);
	sid->revision = SID_REVISION;
	sid->sub_authority_count = 2;
	sid->sub_authority[0] = const_cpu_to_le32(SECURITY_BUILTIN_DOMAIN_RID);
	sid->sub_authority[1] = const_cpu_to_le32(DOMAIN_ALIAS_RID_ADMINS);
	sid->identifier_authority.value[5] = 5;
	sd->group = cpu_to_le32((u8 *)sid - (u8 *)sd);

	acl = (ACL *)((u8 *)sid + sizeof(SID) + 4);
	acl->revision = ACL_REVISION;
	acl->size = const_cpu_to_le16(sizeof(ACL) + sizeof(ACCESS_ALLOWED_ACE));
	acl->ace_count = const_cpu_to_le16(1);
	sd->dacl = cpu_to_le32((u8 *)acl - (u8 *)sd);

	ace = (ACCESS_ALLOWED_ACE *)((u8 *)acl + sizeof(ACL));
	ace->type = ACCESS_ALLOWED_ACE_TYPE;
	ace->flags = OBJECT_INHERIT_ACE | CONTAINER_INHERIT_ACE;
	ace->size = const_cpu_to_le16(sizeof(ACCESS_ALLOWED_ACE));
	ace->mask = const_cpu_to_le32(0x1f01ff); /* FIXME */
	ace->sid.revision = SID_REVISION;
	ace->sid.sub_authority_count = 1;
	ace->sid.sub_authority[0] = const_cpu_to_le32(0);
	ace->sid.identifier_authority.value[5] = 1;

	ret = ntfs_attr_add(ni, AT_SECURITY_DESCRIPTOR, NULL, 0, (u8 *)sd,
			    sd_len);
	if (ret)
		ntfs_error(ni->vol->sb, "Failed to add initial "
			"SECURITY_DESCRIPTOR");

	ntfs_free(sd);
	return ret;
}

