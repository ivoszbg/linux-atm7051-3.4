#include <linux/err.h>
#include <linux/module.h>
#include <linux/ion.h>
#include <linux/asoc_ion.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include "../ion_priv.h"

struct ion_mapper *asoc_user_mapper;
int num_heaps;
struct ion_heap **heaps;

struct ion_device *asoc_ion_device;
EXPORT_SYMBOL(asoc_ion_device);

static int asoc_ion_get_phys(struct ion_client *client,
					unsigned int cmd,
					unsigned long arg)
{
#if 0
	bool valid_handle;
#endif
	struct asoc_ion_phys_data data;
	struct asoc_ion_phys_data *user_data =
				(struct asoc_ion_phys_data *)arg;
	struct ion_buffer *buffer;
	int ret;

	if (copy_from_user(&data, (void __user *)arg, sizeof(data)))
		return -EFAULT;

#if 0
	mutex_lock(&client->lock);
	valid_handle = ion_handle_validate(client, data.handle);
	mutex_unlock(&client->lock);

	if (!valid_handle) {
		WARN(1, "%s: invalid handle passed to get id.\n", __func__);
		return -EINVAL;
	}
#endif

	buffer = ion_handle_buffer(data.handle);
	ret = ion_phys(client, data.handle, &data.phys_addr, &data.size);
	if(ret < 0)
		return ret;

	if (copy_to_user(user_data, &data, sizeof(data)))
		return -EFAULT;

	return 0;
}

static long asoc_ion_ioctl(struct ion_client *client,
				   unsigned int cmd,
				   unsigned long arg)
{
	int ret = -EINVAL;

	switch (cmd) {
	case ASOC_ION_GET_PHY:
		ret = asoc_ion_get_phys(client, cmd, arg);
		break;
	default:
		WARN(1, "Unknown custom ioctl\n");
		return -EINVAL;
	}
	return ret;
}


int asoc_ion_probe(struct platform_device *pdev)
{
	struct ion_platform_data *pdata = pdev->dev.platform_data;
	int err;
	int i;

#ifdef CONFIG_CMA
	set_cma_dev(&(pdev->dev));
#endif
 	ION_PRINT("%s()\n", __func__);

	num_heaps = pdata->nr;

	heaps = kzalloc(sizeof(struct ion_heap *) * pdata->nr, GFP_KERNEL);

	asoc_ion_device = ion_device_create(asoc_ion_ioctl);
	if (IS_ERR_OR_NULL(asoc_ion_device)) {
		kfree(heaps);
		return PTR_ERR(asoc_ion_device);
	}

	/* create the heaps as specified in the board file */
	for (i = 0; i < num_heaps; i++) {
		struct ion_platform_heap *heap_data = &pdata->heaps[i];

		heaps[i] = ion_heap_create(heap_data);
		if (IS_ERR_OR_NULL(heaps[i])) {
			err = PTR_ERR(heaps[i]);
			goto err;
		}
		ion_device_add_heap(asoc_ion_device, heaps[i]);
	}
	platform_set_drvdata(pdev, asoc_ion_device);
	return 0;
err:
	for (i = 0; i < num_heaps; i++) {
		if (heaps[i])
			ion_heap_destroy(heaps[i]);
	}
	kfree(heaps);
	return err;
}

int asoc_ion_remove(struct platform_device *pdev)
{
	struct ion_device *asoc_ion_device = platform_get_drvdata(pdev);
	int i;

	ion_device_destroy(asoc_ion_device);
	for (i = 0; i < num_heaps; i++)
		ion_heap_destroy(heaps[i]);
	kfree(heaps);
	return 0;
}

static struct platform_driver ion_driver = {
	.probe = asoc_ion_probe,
	.remove = asoc_ion_remove,
	.driver = { .name = "ion-asoc" }
};




#include  <linux/debugfs.h>

struct array_data
{
	void *array;
	unsigned elements;
};

static int u32_array_open(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return nonseekable_open(inode, file);
}

static size_t format_array(char *buf, size_t bufsize, const char *fmt,
			   u32 *array, unsigned array_size)
{
	size_t ret = 0;
	unsigned i;
	unsigned j;

	for(i = 0; i < array_size; i++) {

		for (j = 0; j < 32; j++) {
			snprintf(buf+j, bufsize, array[i] & (1 << j)?"+":"_");
		}
		ret += 32;

		if (buf) {
			buf += 32;
			bufsize -= 32;
		}
	}

	ret++;		/* \0 */
	if (buf)
		*buf = '\0';

	return ret;
}

static char *format_array_alloc(const char *fmt, u32 *array, unsigned array_size)
{
	size_t len = format_array(NULL, 0, fmt, array, array_size);
	char *ret;

	printk("%s--%d len %d\n", __func__, __LINE__,len);
	ret = kmalloc(len, GFP_KERNEL);
	if (ret == NULL)
		return NULL;

	format_array(ret, len, fmt, array, array_size);
	return ret;
}

static ssize_t u32_array_write(struct file *file, char __user *buf, size_t len,
			      loff_t *ppos)
{
  return 0;
}

static ssize_t u32_array_read(struct file *file, char __user *buf, size_t len,
			      loff_t *ppos)
{
	struct inode *inode = file->f_path.dentry->d_inode;
	struct array_data *data = inode->i_private;
	size_t size;

	if (*ppos == 0) {
		if (file->private_data) {
			kfree(file->private_data);
			file->private_data = NULL;
		}

		file->private_data = format_array_alloc("%u", data->array, data->elements);
	}

	size = 0;
	if (file->private_data)
		size = strlen(file->private_data);

	return simple_read_from_buffer(buf, len, ppos, file->private_data, size);
}

static int array_release(struct inode *inode, struct file *file)
{
  kfree(file->private_data);
  return 0;
}

static  const struct file_operations u32_array_fops = {
	.owner	= THIS_MODULE,
	.open	= u32_array_open,
	.release= array_release,
	.read	= u32_array_read,
        .write  = u32_array_write,
	.llseek = no_llseek,
};

#include <asm/dma-contiguous.h>

struct cma {
	unsigned long	base_pfn;
	unsigned long	count;
	unsigned long	*bitmap;
};
static struct  array_data arr_data;

void create_ion_debugfs(struct device * cma_dev)
{
	struct dentry * dir = NULL;
	struct dentry * cma_file = NULL;
	struct cma *cma = dev_get_cma_area(cma_dev);

	if (!cma) {
		printk("error %s %d\n", __func__, __LINE__);
		return;
	}


	arr_data.array = cma->bitmap;
	arr_data.elements = cma->count / 32;

	dir = debugfs_create_dir("cma", NULL);
	if (!dir)
		pr_info("%s%d\n", __func__, __LINE__);
	cma_file = debugfs_create_file("cma_alloc_map", S_IALLUGO, dir, &arr_data,
				       &u32_array_fops);
}




static int __init ion_init(void)
{
	return platform_driver_register(&ion_driver);
}

static void __exit ion_exit(void)
{
	platform_driver_unregister(&ion_driver);
}

module_init(ion_init);
module_exit(ion_exit);
