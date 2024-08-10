/*
 * arch/arm/mach-leopard/secure_storage.c
 *
 * secure storage interface
 *
 * Copyright 2012 Actions Semi Inc.
 * Author: Actions Semi, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/debugfs.h>
#include <mach/secure_storage.h>
#include <mach/storage_access.h>

static struct secure_storage *cur_secure_storage = NULL;

int asoc_register_secure_storage(struct secure_storage *ss)
{
    if (!ss || !ss->name)
        return -EINVAL;

    pr_info("%s: register %s\n", __FUNCTION__, ss->name);

    if (cur_secure_storage) {
        pr_warning("%s: register %s override %s\n", __FUNCTION__,
            ss->name, cur_secure_storage->name);
    }

    cur_secure_storage = ss;

    return 0;
}
EXPORT_SYMBOL(asoc_register_secure_storage);

int asoc_unregister_secure_storage(struct secure_storage *ss)
{
    if (!ss || !ss->name)
        return -EINVAL;

    pr_info("%s: unregister %s\n", __FUNCTION__, ss->name);

    if (cur_secure_storage != ss) {
        pr_warning("%s: unregister %s override %s\n", __FUNCTION__,
            ss->name, cur_secure_storage->name);
        return -EINVAL;
    }

    cur_secure_storage = NULL;

    return 0;
}
EXPORT_SYMBOL(asoc_unregister_secure_storage);


static int check_secure_storage(int type, char * buf, int size)
{
    if (!cur_secure_storage) {
        pr_err("%s: no valid secure storage\n", __FUNCTION__);
        return -ENODEV;
    }

    if (!buf || !size) {
        pr_err("%s: invalid buf\n", __FUNCTION__);
        return -EINVAL;
    }

    switch (type)
    {
    case SECURE_STORAGE_DATA_TYPE_SN:
    case SECURE_STORAGE_DATA_TYPE_DRM:
    case SECURE_STORAGE_DATA_TYPE_HDCP:
    case SECURE_STORAGE_DATA_TYPE_DEVNUM:
    case SECURE_STORAGE_DATA_TYPE_EXT:
        break;
    default:
        pr_err("%s: invalid type %d\n", __FUNCTION__, type);
        return -EINVAL;
    }

    return 0;
}

//amend:return the size of read data.
int asoc_read_secure_storage_data(int type, char * buf, int size)
{
    int ret;

    ret = check_secure_storage(type, buf, size);
    if (ret) {
        pr_err("%s: invalid param, ret %d\n", __FUNCTION__, ret);
        return ret;
    }

    if (!cur_secure_storage->read_data) {
        pr_err("%s: cannot support read\n", __FUNCTION__);
        return -ENODEV;
    }

    ret = cur_secure_storage->read_data(type, buf, size);
    if (ret <= 0) {
        pr_err("%s: faild to read secure storage, ret %d\n", __FUNCTION__, ret);
        return ret;
    }

	return ret;
}
EXPORT_SYMBOL(asoc_read_secure_storage_data);

//amend:return the size of write data.
int asoc_write_secure_storage_data(int type, char * buf, int size)
{
    int ret;

    ret = check_secure_storage(type, buf, size);
    if (ret) {
        pr_err("%s: invalid param, ret %d\n", __FUNCTION__, ret);
        return ret;
    }

    if (!cur_secure_storage->write_data) {
        pr_err("%s: cannot support write\n", __FUNCTION__);
        return -ENODEV;
    }

    ret = cur_secure_storage->write_data(type, buf, size);
    if (ret <= 0) {
        pr_err("%s: faild to write secure storage, ret %d\n", __FUNCTION__, ret);
        return ret;
    }

    return ret;
}
EXPORT_SYMBOL(asoc_write_secure_storage_data);

static int check_and_remap_storag_type(int type, int *new_type)
{
    switch (type)
    {
    case STORAGE_DATA_TYPE_HDCP:
        *new_type = SECURE_STORAGE_DATA_TYPE_HDCP;
        break;
    default:
        pr_err("%s: invalid type %d\n", __FUNCTION__, type);
        return -EINVAL;
    }

    return 0;
}

int asoc_read_storage_data(int type, char * buf, int size)
{
    int ret;
    int new_type;
    
    ret = check_and_remap_storag_type(type, &new_type);
    if (ret) {
        pr_err("%s: invalid param, ret %d\n", __FUNCTION__, ret);
        return ret;
    }    
    return asoc_read_secure_storage_data(new_type, buf, size);
}
EXPORT_SYMBOL(asoc_read_storage_data);
