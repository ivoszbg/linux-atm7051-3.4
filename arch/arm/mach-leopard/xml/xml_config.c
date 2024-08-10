#include <linux/export.h>
#include <linux/kernel.h>
#include <linux/stddef.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/utsname.h>
#include <linux/initrd.h>
#include <linux/console.h>
#include <linux/bootmem.h>
#include <linux/seq_file.h>
#include <linux/screen_info.h>
#include <linux/init.h>
#include <linux/kexec.h>
#include <linux/of_fdt.h>
#include <linux/root_dev.h>
#include <linux/cpu.h>
#include <linux/interrupt.h>
#include <linux/smp.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/memblock.h>
#include <linux/bug.h>
#include <linux/compiler.h>
#include <linux/sort.h>

#include <asm/unified.h>
#include <asm/cp15.h>
#include <asm/cpu.h>
#include <asm/cputype.h>
#include <asm/elf.h>
#include <asm/procinfo.h>
#include <asm/sections.h>
#include <asm/setup.h>
#include <asm/smp_plat.h>
#include <asm/mach-types.h>
#include <asm/cacheflush.h>
#include <asm/cachetype.h>
#include <asm/tlbflush.h>

#include <asm/prom.h>
#include <asm/mach/arch.h>
#include <asm/mach/irq.h>
#include <asm/mach/time.h>
#include <asm/system_info.h>
#include <asm/system_misc.h>
#include <asm/traps.h>
#include <asm/unwind.h>
#include <asm/memblock.h>
#include <asm/idmap.h>

#include "mxml.h"

//#define	 DBG_REBUILD_XML
//#define	 DBG_PARSE_XML

#ifdef DBG_REBUILD_XML
#define DEBUG_REBUILD_XML(stuff...)		printk("<REBUILD_XML>" stuff)
int cnt=0;
#else
#define DEBUG_REBUILD_XML(stuff...)		do{}while(0)
#endif

#ifdef DBG_PARSE_XML
#define DEBUG_PARSE_XML(stuff...)		printk("<PARSE_XML>" stuff)
#else
#define DEBUG_PARSE_XML(stuff...)		do{}while(0)
#endif

unsigned int xml_buf_start_phys, xml_buf_start_virt;
unsigned int bin_cfg_xml_buf_start_phys, bin_cfg_xml_buf_start_virt;
unsigned int xml_buf_len, bin_cfg_xml_buf_len;
void *bootmem_for_xml=NULL;
void *bootmem_for_bin_cfg_xml=NULL;

#define SINGLE_ITEM_LEN 32

/* 0-mxml lib functions */

/*
 * 'mxmlWalkNext()' - Walk to the next logical node in the tree.
 *
 * The descend argument controls whether the first child is considered
 * to be the next node. The top node argument constrains the walk to
 * the node's children.
 */

mxml_node_t *				/* O - Next node or NULL */
mxmlWalkNext(mxml_node_t *node,		/* I - Current node */
             mxml_node_t *top,		/* I - Top node */
             int         descend)	/* I - Descend into tree - MXML_DESCEND, MXML_NO_DESCEND, or MXML_DESCEND_FIRST */
{
  if (!node)
    return (NULL);
  else if (node->child && descend)
    return (node->child);
  else if (node == top)
    return (NULL);
  else if (node->next)
    return (node->next);
  else if (node->parent && node->parent != top)
  {
    node = node->parent;

    while (!node->next)
      if (node->parent == top || !node->parent)
        return (NULL);
      else
        node = node->parent;

    return (node->next);
  }
  else
    return (NULL);
}

/*
 * 'mxmlFindElement()' - Find the named element.
 *
 * The search is constrained by the name, attribute name, and value; any
 * NULL names or values are treated as wildcards, so different kinds of
 * searches can be implemented by looking for all elements of a given name
 * or all elements with a specific attribute. The descend argument determines
 * whether the search descends into child nodes; normally you will use
 * MXML_DESCEND_FIRST for the initial search and MXML_NO_DESCEND to find
 * additional direct descendents of the node. The top node argument
 * constrains the search to a particular node's children.
 */

mxml_node_t *				/* O - Element node or NULL */
mxmlFindElement(mxml_node_t *node,	/* I - Current node */
                mxml_node_t *top,	/* I - Top node */
                const char  *name,	/* I - Element name or NULL for any */
		const char  *attr,	/* I - Attribute name, or NULL for none */
		const char  *value,	/* I - Attribute value, or NULL for any */
		int         descend)	/* I - Descend into tree - MXML_DESCEND, MXML_NO_DESCEND, or MXML_DESCEND_FIRST */
{
  const char	*temp;			/* Current attribute value */


 /*
  * Range check input...
  */

  if (!node || !top || (!attr && value))
    return (NULL);

 /*
  * Start with the next node...
  */

  node = mxmlWalkNext(node, top, descend);

 /*
  * Loop until we find a matching element...
  */

  while (node != NULL)
  {
   /*
    * See if this node matches...
    */

    if (node->type == MXML_ELEMENT &&
        node->value.element.name &&
	(!name || !strcmp(node->value.element.name, name)))
    {
     /*
      * See if we need to check for an attribute...
      */

      if (!attr)
        return (node);			/* No attribute search, return it... */

     /*
      * Check for the attribute...
      */

      if ((temp = mxmlElementGetAttr(node, attr)) != NULL)
      {
       /*
        * OK, we have the attribute, does it match?
	*/

	if (!value || !strcmp(value, temp))
	  return (node);		/* Yes, return it... */
      }
    }

   /*
    * No match, move on to the next node...
    */

    if (descend == MXML_DESCEND)
      node = mxmlWalkNext(node, top, MXML_DESCEND);
    else
      node = node->next;
  }

  return (NULL);
}

/*
 * 'mxmlElementGetAttr()' - Get an attribute.
 *
 * This function returns NULL if the node is not an element or the
 * named attribute does not exist.
 */

const char *				/* O - Attribute value or NULL */
mxmlElementGetAttr(mxml_node_t *node,	/* I - Element node */
                   const char  *name)	/* I - Name of attribute */
{
  int		i;			/* Looping var */
  mxml_attr_t	*attr;			/* Cirrent attribute */


#ifdef DEBUG
  printk("mxmlElementGetAttr(node=%p, name=\"%s\")\n",
          node, name ? name : "(null)");
#endif /* DEBUG */

 /*
  * Range check input...
  */

  if (!node || node->type != MXML_ELEMENT || !name)
    return (NULL);

 /*
  * Look for the attribute...
  */

  for (i = node->value.element.num_attrs, attr = node->value.element.attrs;
       i > 0;
       i --, attr ++)
  {
#ifdef DEBUG
    printk("    %s=\"%s\"\n", attr->name, attr->value);
#endif /* DEBUG */

    if (!strcmp(attr->name, name))
    {
#ifdef DEBUG
      printk("    Returning \"%s\"!\n", attr->value);
#endif /* DEBUG */
      return (attr->value);
    }
  }

 /*
  * Didn't find attribute, so return NULL...
  */

#ifdef DEBUG
  printk("    Returning NULL!\n");
#endif /* DEBUG */

  return (NULL);
}


/* 1-mxml tree search functions */
mxml_node_t *search_item(mxml_node_t *top, mxml_node_t *node, char *element_name, char *element_attr_name, char *element_attr_value)
{
    int end_flag=1;
    mxml_node_t *first=node, *next;
    char *value;
    do{
        next = mxmlFindElement (
                                first,
                                top,
                                element_name,
                                NULL,
                                NULL,
                                MXML_DESCEND );
        if(next != NULL)
        {
            value = mxmlElementGetAttr(next, element_attr_name);
            DEBUG_PARSE_XML("value:%s, element_attr_value:%s\n", value, element_attr_value);
            if(!strcmp(value, element_attr_value))
            {
                end_flag = 0;
            }
            else
            {
                first = next;
            }
        }
        else
        {
            end_flag = 0;
        }
    }while(end_flag);

    return next;
}

/*
    tranfer str(like:"1;2;3;4;5") to arry(like:arry[5]={1,2,3,4,5})
 */
int str_to_int_arry(char *str, int *arry)
{
    int end_flag=1;
    int index=0, len;
    char *first=str, *next;
    char tmp[12];

    do
    {
        next = strchr(first, ';');
        if(next == NULL)
        {
            arry[index] = simple_strtol(first, NULL, 10);
            DEBUG_PARSE_XML("arry[%d]:%d\n", index, arry[index]);
            end_flag = 0;
        }
        else
        {
            len = next - first;
            if(len >= 12)
            {
                printk(KERN_ERR "len:%d too long", len);
            }
            tmp[len] = 0;
            memcpy(tmp, first, len);
            arry[index] = simple_strtol(tmp, NULL, 10);
            DEBUG_PARSE_XML("arry[%d]:%d\n", index, arry[index]);
            index++;
            next++; //skip ':'
            first = next;
        }
    }while(end_flag);
    return 0;
}

/*
mxml_node_t *node:
char *buff:
int len:
 */
int get_config_value(mxml_node_t *node, void *buff, int len)
{
    int type_size,type_cnt,size;
    char *tmp;
    int arry_flag=0, string_flag=0;
    int ret=0;

    tmp = mxmlElementGetAttr (node, "type");
    DEBUG_PARSE_XML("type:%s\n", tmp);
    if(!strcmp(tmp, "int"))
    {
        arry_flag = 1;
        type_size = 4;
    }
    else if(!strcmp(tmp, "str"))
    {
        string_flag = 1;
        type_size = 1;
    }
    else
    {
        ret = -1;
        printk(KERN_ERR "wrong type:%s\n", tmp);
        goto out;
    }
/*     tmp = mxmlElementGetAttr (node, "size");
    DEBUG_PARSE_XML("size:%s\n", tmp);
    type_cnt = simple_strtol(tmp, NULL, 10);
    DEBUG_PARSE_XML("type_cnt:%d\n",type_cnt);

    size = type_size*type_cnt;
    if(size > len)
    {
        ret = -2;
        printk(KERN_ERR "buf overflow,len:%d < size:%d\n",len,size);
        goto out;
    } */

    tmp = mxmlElementGetAttr (node, "value");

    DEBUG_PARSE_XML("tmp:%s,arry_flag:%d,string_flag:%d\n",tmp,arry_flag,string_flag);

/*     if((arry_flag == 0) && (string_flag == 0))
    {
        *((int *)buff) = simple_strtol(tmp, NULL, 10);
        DEBUG_PARSE_XML("*((int *)buff):%d\n", *((int *)buff));
    }
    else  */
	if(string_flag == 1)
    {
        strcpy(buff,tmp);
    }
    else if(arry_flag == 1)
    {
        //½«¡°1;2;3;4;5;6;7;8¡±²ð³Éint_arry[0]=1;int_arry[1]=2,etc
        str_to_int_arry(tmp, (int *)buff);
    }

out:
    return ret;
}

/*
    get config value according key of config item
    key:support two layer struct,like:lcd.bklight
 */

typedef struct xml_cfg_s
{
    mxml_node_t *tree;
    rwlock_t xml_rwlock;
    struct xml_cfg_s *next;
}xml_cfg_t;

xml_cfg_t xml_cfg_config,xml_cfg_bin_cfg;
xml_cfg_t *xml_cfg_list_head;

int init_xml_cfg_list(void)
{
    xml_cfg_config.tree = bootmem_for_xml;
    xml_cfg_bin_cfg.tree = bootmem_for_bin_cfg_xml;
    
    xml_cfg_list_head = &xml_cfg_config;
    xml_cfg_config.next = &xml_cfg_bin_cfg;
    xml_cfg_bin_cfg.next = 0;
        
    rwlock_init(&xml_cfg_config.xml_rwlock);
	rwlock_init(&xml_cfg_bin_cfg.xml_rwlock);
	
    return 0;
}
 
int get_config(const char *key, char *buff, int len)
{
    int ret=0;
    int end_flag=1;
    char item_name[SINGLE_ITEM_LEN];
    char *first,*next;
    int item_len;

    mxml_node_t *parent, *top;
    xml_cfg_t *p_kcfg=xml_cfg_list_head;

    for(; (p_kcfg != NULL) && (p_kcfg->tree != NULL); p_kcfg = p_kcfg->next)
    {
    	end_flag = 1;
        top = p_kcfg->tree;
    	
        read_lock(&(p_kcfg->xml_rwlock));

        parent = top;
        first = key;
        do
        {
            next = strrchr(first, '.');
            if(next == NULL)
            {
                //search item which name matches fist~next
                DEBUG_PARSE_XML("fist:%s\n",first);
                parent = search_item(top, parent, "item", "name", first);
                if(parent != NULL)
                {
                    get_config_value(parent, buff, len);
                }
                else
                {
                	DEBUG_PARSE_XML("line:%d,parent == NULL\n", __LINE__);
                }
				end_flag = 0;                
            }
            else
            {
                //search top_item which name matches fist~next
                item_len = next-first;
                if(item_len>SINGLE_ITEM_LEN-1)
                {
                    printk(KERN_ERR"[%s]item:%s too long\n", __FUNCTION__, key);
                }
                item_name[item_len]=0;
                memcpy(item_name, first, item_len);
                DEBUG_PARSE_XML("item_name:%s\n",item_name);
                parent = search_item(top, parent, "top_item", "name", item_name);
                top = parent;
                if(parent == NULL)
                {
                	DEBUG_PARSE_XML("line:%d,parent == NULL\n", __LINE__);
                    end_flag = 0;
                }
                first = ++next;//skip '.'
                DEBUG_PARSE_XML("first:%s\n",first);
            }
        }while(end_flag);

        read_unlock(&(p_kcfg->xml_rwlock));

        if(parent != NULL)
        {
            break;
        }
    }
    if(parent == NULL)
    {
        printk(KERN_INFO"[%s]NOT find item:%s\n", __FUNCTION__, key);
        ret = -1;
    }
    return ret;
}

EXPORT_SYMBOL(get_config);

int get_config_example(void)
{
    int lcd_light[6], ddd, micgain;
    char vendor[32];
    int low_power=0;
    int i=0;

    get_config("gadget.vendor", vendor, 32);
    printk("vendor:%s\n", vendor);

    get_config("snd.ddd", &ddd, sizeof(int));
    printk("snd.ddd:%d\n", ddd);
	
	get_config("charge.low_power", &low_power, sizeof(int));
	printk("low_power:%d\n", low_power); 

    get_config("lcd.backlight", &lcd_light, sizeof(int)*6);
    for(i=0; i<6; i++)
   	{
   		printk("backlight[%d]:%d\n", i, lcd_light[i]);
   	}
	
	get_config("snd.micgain", &micgain, sizeof(int));
	printk("micgain:%d\n", micgain); 		
}

/* 2-rebuild mxml tree functions */
int xml_writel(unsigned int val, unsigned int addr)
{
	DEBUG_REBUILD_XML("addr:0x%x, val:0x%x\n", addr, val);
	*(volatile unsigned int *)addr = val;
}

void fix_pointer_val(unsigned int addr, unsigned int val, int diff_val)
{
	xml_writel(val + diff_val, addr);
}

void fix_xml_node_link(mxml_node_t *tree, int diff_val)
{
	if(tree->next != NULL)
	{
		DEBUG_REBUILD_XML("next\n");		
		fix_pointer_val(&(tree->next), (int)tree->next, diff_val);
	}
	if(tree->prev != NULL)
	{
		DEBUG_REBUILD_XML("prev\n");
		fix_pointer_val(&(tree->prev), (int)tree->prev, diff_val);
	}
	if(tree->parent != NULL)
	{
		DEBUG_REBUILD_XML("parent\n");
		fix_pointer_val(&(tree->parent), (int)tree->parent, diff_val);
	}
	if(tree->child != NULL)
	{
		DEBUG_REBUILD_XML("child\n");
		fix_pointer_val(&(tree->child), (int)tree->child, diff_val);
	}
	if(tree->last_child != NULL)
	{
		DEBUG_REBUILD_XML("last_child\n");
		fix_pointer_val(&(tree->last_child), (int)tree->last_child, diff_val);
	}
}
	
void fix_xml_node_tree(mxml_node_t *tree, int diff_val)
{
	mxml_node_t *tmp=tree;
	mxml_node_t *child;
	mxml_attr_t *attr;
	int i;

	do{
		DEBUG_REBUILD_XML("%s:%d\n", __FUNCTION__, cnt++);
		DEBUG_REBUILD_XML("tmp->type:%d\n", tmp->type);
		fix_xml_node_link(tmp, diff_val);
		DEBUG_REBUILD_XML("tmp->child:0x%x\n", tmp->child);
		switch (tmp->type)
		{
			case MXML_ELEMENT:	
				DEBUG_REBUILD_XML("element.name\n");		
				fix_pointer_val(&(tmp->value.element.name), (unsigned int)(tmp->value.element.name), diff_val);
				fix_pointer_val(&(tmp->value.element.attrs), (unsigned int)(tmp->value.element.attrs), diff_val);
				
		        for (i = tmp->value.element.num_attrs, attr = tmp->value.element.attrs;
		           i > 0;
		           i --, attr ++)
		        {
		        	DEBUG_REBUILD_XML("attr->name\n");		
					fix_pointer_val(&(attr->name), (unsigned int)(attr->name), diff_val);
					DEBUG_REBUILD_XML("attr->value\n");		
					fix_pointer_val(&(attr->value), (unsigned int)(attr->value), diff_val);
					DEBUG_REBUILD_XML("%s:%s\n", attr->name, attr->value);
		        }			
				break;
			case MXML_TEXT:
					if(tmp->value.text.string != NULL)
					{
						DEBUG_REBUILD_XML("text.string\n");		
						fix_pointer_val(&(tmp->value.text.string), (unsigned int)(tmp->value.text.string), diff_val);			
					}
				break;
			default:
				break;
		}			
		child = tmp->child;
		if(child != NULL)
		{
			fix_xml_node_tree(child, diff_val);//diff_val);	
		}			
		tmp = tmp->next;	
	}while(tmp != NULL);
}

void __init rebuild_xml_tree(void)
{
	if (!xml_buf_start_phys)
		return;
		
	bootmem_for_xml = alloc_bootmem_align(xml_buf_len, PAGE_SIZE);
	printk("bootmem_for_xml:0x%08x\n", bootmem_for_xml);
	
	identity_mapping_add(swapper_pg_dir, xml_buf_start_phys, xml_buf_start_phys+PFN_ALIGN(xml_buf_len));
	xml_buf_start_virt = xml_buf_start_phys;
	printk("%s,xml_buf_start_virt:0x%x, xml_buf_start_phys:0x%x\n", __FUNCTION__, xml_buf_start_virt, xml_buf_start_phys);

	memcpy(bootmem_for_xml, xml_buf_start_virt, xml_buf_len);
	
	identity_mapping_del(swapper_pg_dir, xml_buf_start_phys, xml_buf_start_phys+PFN_ALIGN(xml_buf_len));
	if(xml_buf_start_phys >= PHYS_OFFSET)
	    memblock_free(xml_buf_start_phys, xml_buf_len);
	
	printk("%s,bootmem_for_xml:0x%x, xml_buf_start_phys:0x%x\n", __FUNCTION__, bootmem_for_xml, xml_buf_start_phys);	
	fix_xml_node_tree(bootmem_for_xml, bootmem_for_xml-xml_buf_start_phys);

//bin_cfg.xml
	if(bin_cfg_xml_buf_start_phys != 0)
	{
		bootmem_for_bin_cfg_xml = alloc_bootmem_align(bin_cfg_xml_buf_len, PAGE_SIZE);
		printk("bootmem_for_bin_cfg_xml:0x%08x\n", bootmem_for_bin_cfg_xml);

		identity_mapping_add(swapper_pg_dir, bin_cfg_xml_buf_start_phys, bin_cfg_xml_buf_start_phys+PFN_ALIGN(bin_cfg_xml_buf_len));
		bin_cfg_xml_buf_start_virt = bin_cfg_xml_buf_start_phys;
		printk("%s,bin_cfg_xml_buf_start_virt:0x%x, bin_cfg_xml_buf_start_phys:0x%x\n", __FUNCTION__, bin_cfg_xml_buf_start_virt, bin_cfg_xml_buf_start_phys);
			
		memcpy(bootmem_for_bin_cfg_xml, bin_cfg_xml_buf_start_virt, bin_cfg_xml_buf_len);
		if(bin_cfg_xml_buf_start_virt >= PHYS_OFFSET)
			memblock_free(bin_cfg_xml_buf_start_phys, bin_cfg_xml_buf_len);
		
		printk("%s,bootmem_for_bin_cfg_xml:0x%x, bin_cfg_xml_buf_start_phys:0x%x\n", __FUNCTION__, bootmem_for_bin_cfg_xml, bin_cfg_xml_buf_start_phys);	
		fix_xml_node_tree(bootmem_for_bin_cfg_xml, bootmem_for_bin_cfg_xml-bin_cfg_xml_buf_start_phys);
	}
	
	init_xml_cfg_list();	
		
//	get_config_example();
}


/* 3-reserve xml buf from adfus/mbrc */
void __init arm_xml_memblock_reserve(void)
{
	if (!xml_buf_start_phys)
		return;

	/* Reserve the xml region */
	if(xml_buf_start_phys >= PHYS_OFFSET) {
    	memblock_reserve(xml_buf_start_phys, xml_buf_len);
		printk("memblock_reserve,0x%x:0x%x\n", xml_buf_start_phys, xml_buf_len);
    }
	if((bin_cfg_xml_buf_start_phys != 0) && (bin_cfg_xml_buf_start_phys >= PHYS_OFFSET))
	{
		memblock_reserve(bin_cfg_xml_buf_start_phys, bin_cfg_xml_buf_len);
		printk("memblock_reserve,0x%x:0x%x\n", bin_cfg_xml_buf_len, bin_cfg_xml_buf_len);	
	}
}


/* 4-parse ATAG_XML */
static int __init parse_tag_xml(const struct tag *tag)
{
	xml_buf_start_phys = tag->u.xml.xml_buf_start;
	xml_buf_len = tag->u.xml.xml_buf_len;

	bin_cfg_xml_buf_start_phys = tag->u.xml.bin_cfg_xml_buf_start;
	bin_cfg_xml_buf_len = tag->u.xml.bin_cfg_xml_buf_len;
	
	printk("xml_buf_start_phys:0x%x, xml_buf_len:0x%x\n", xml_buf_start_phys, xml_buf_len);
	printk("bin_cfg_xml_buf_start_phys:0x%x, bin_cfg_xml_buf_len:0x%x\n", bin_cfg_xml_buf_start_phys, bin_cfg_xml_buf_len);
	
	return 0;
}

__tagtable(ATAG_XML, parse_tag_xml);
