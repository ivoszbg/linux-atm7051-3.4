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
#include <linux/pinctrl/machine.h>

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

#include <mach/xml_pinctrl.h>
#include <mach/gpio.h>


unsigned int pinctrl_buf_start_phys, pinctrl_buf_start_virt;
unsigned int pinctrl_buf_len;

struct pinctrl_args *p_pinctrl_args;

extern unsigned int nfunctions,ngroups,nmaps;

extern struct asoc_pinmux_func *leopard_functions;
extern struct asoc_pinmux_group *leopard_pingroups;
extern struct pinctrl_map *leopardfpga_pinmux_map;

struct asoc_pinmux_group_data *asoc_pinmux_group_datas;
struct asoc_pinmux_name *asoc_pinmux_func_names;
char ***asoc_pinmux_func_groups;
struct pinctrl_map_data *pinctrl_map_datas;

unsigned int ngpios;
struct gpio_pre_cfg *gpiocfgs;

/*tmp code for 7021*/
extern int chip_4core_test(void);

int gpio_get_pre_cfg(char *gpio_name, struct gpio_pre_cfg *m_gpio)
{
    int i;

    for (i=0; i<ngpios; i++)
    {
        if (strcmp(gpiocfgs[i].name, gpio_name) == 0)
        {
            *m_gpio = gpiocfgs[i];
			if (chip_4core_test() == 0) { /* For support 7021, GPIO A28 transfer to GPIO C22 */
				if ((m_gpio->iogroup == 0) &&(m_gpio->pin_num == 28)) {
					m_gpio->iogroup = 2;
					m_gpio->pin_num = 22;
					printk("tmp code for 7021, transfer to GPIOC22\n");
				}
			}    
            return 0;
        }
    }
    return -1;
}
EXPORT_SYMBOL_GPL(gpio_get_pre_cfg);

void dump_gpiocfg(void)
{
    int i;

	printk("-------gpiocfgs:%d-------\n", ngpios);
    for(i=0; i<ngpios; i++)
    {
        printk("%d:%d:%d:%d:%s\n", \
                                gpiocfgs[i].iogroup, \
                                gpiocfgs[i].pin_num, \
                                gpiocfgs[i].gpio_dir, \
                                gpiocfgs[i].active_level, \
                                gpiocfgs[i].name \
                                );
    }
}

void dump_pinctrl_groups(void)
{
	int i,j;

	printk("-------leopard_pingroups:%d-------\n", ngroups);
	for(i=0; i<ngroups; i++)
	{
		printk("-------ngroups[%d]-------\n", i);
		printk("name:%s\n", leopard_pingroups[i].name);
		printk("npins:%d\n", leopard_pingroups[i].npins);
		for(j=0; j<leopard_pingroups[i].npins; j++)
		{
			printk("%d ", leopard_pingroups[i].pins[j]);
		}
		printk("\n");
		printk("nregcfgs:%d\n", leopard_pingroups[i].nregcfgs);
		for(j=0; j<leopard_pingroups[i].nregcfgs; j++)
		{
			printk("reg:0x%08x, ", leopard_pingroups[i].regcfgs[j].reg);
			printk("mask:0x%08x, ", leopard_pingroups[i].regcfgs[j].mask);
			printk("val:0x%08x\n", leopard_pingroups[i].regcfgs[j].val);
		}
		printk("\n");
	}
}

void dump_pinctrl_functions(void)
{
	int i,j;

	printk("-------leopard_functions:%d-------\n", nfunctions);
	for(i=0; i<nfunctions; i++)
	{
		printk("leopard_functions[%d].name:%s, ", i, leopard_functions[i].name);
		printk("groups=%d:", leopard_functions[i].ngroups);
		for(j=0; j<leopard_functions[i].ngroups; j++)
		{
			printk("%s ", leopard_functions[i].groups[j]);
		}
		printk("\n");
	}	
}

void dump_pinctrl_mappings(void)
{
	int i;

	printk("-------leopardfpga_pinmux_map:%d-------\n", nmaps);
	for(i=0; i<nmaps; i++)
	{
		printk("----------leopardfpga_pinmux_map[%d]----------\n", i);
		printk("dev_name:%s\n", leopardfpga_pinmux_map[i].dev_name);
		printk("name:%s\n", leopardfpga_pinmux_map[i].name);
		printk("type:%d\n", leopardfpga_pinmux_map[i].type);
		printk("ctrl_dev_name:%s\n", leopardfpga_pinmux_map[i].ctrl_dev_name);
		printk("group:%s\n", leopardfpga_pinmux_map[i].data.mux.group);
		printk("function:%s\n", leopardfpga_pinmux_map[i].data.mux.function);
	}
}

void dump_pinctrl(void)
{
	dump_pinctrl_groups();
	dump_pinctrl_functions();
	dump_pinctrl_mappings();
	dump_gpiocfg();
}

void set_asoc_pinctrl_soc_info(struct asoc_pinctrl_soc_info *p_asoc_pinctrl_soc_info)
{
    p_asoc_pinctrl_soc_info->functions = leopard_functions;
    p_asoc_pinctrl_soc_info->nfunctions = nfunctions;
    p_asoc_pinctrl_soc_info->groups = leopard_pingroups;
    p_asoc_pinctrl_soc_info->ngroups = ngroups;

//    dump_pinctrl();
}

void dump_pinctrl_groups_data(void)
{
	int i,j;

	for(i=0; i<MAX_ASOC_PINMUX_GROUPS; i++)
	{
		printk("-------groups_i[%d]-------\n", i);
		printk("name:%s\n", asoc_pinmux_group_datas[i].name);
		for(j=0; j<MAX_PINMUX_GROUP_PINS; j++)
		{
			printk("%d ", asoc_pinmux_group_datas[i].pins[j]);
		}
		printk("\n");
		for(j=0; j<MAX_PINMUX_GROUP_REGCFGS; j++)
		{
			printk("reg:0x%08x, ", asoc_pinmux_group_datas[i].regcfgs[j].reg);
			printk("mask:0x%08x, ", asoc_pinmux_group_datas[i].regcfgs[j].mask);
			printk("val:0x%08x\n", asoc_pinmux_group_datas[i].regcfgs[j].val);
		}
		printk("\n");
	}
}

void __init fix_pinctrl_buf(unsigned int buf, unsigned int diff)
{
	int i,j;

	p_pinctrl_args = (struct pinctrl_args *)buf;

	printk("-------buf:0x%08x, diff:0x%08x-------\n", buf, diff);

	ngroups = p_pinctrl_args->groups_i;
	nfunctions = p_pinctrl_args->fuctions_i;
	nmaps = p_pinctrl_args->mappings_i;
	ngpios = p_pinctrl_args->gpios_i;

	p_pinctrl_args->addr_asoc_pinmux_group_datas += diff;
	p_pinctrl_args->addr_asoc_pinmux_groups += diff;
	p_pinctrl_args->addr_asoc_pinmux_func_names += diff;
	p_pinctrl_args->addr_asoc_pinmux_func_groups += diff;
	p_pinctrl_args->addr_asoc_pinmux_funcs += diff;
	p_pinctrl_args->addr_pinctrl_map_datas += diff;
	p_pinctrl_args->addr_pinctrl_maps += diff;
	p_pinctrl_args->addr_gpio_cfgs += diff;

//	asoc_pinmux_group_datas = p_pinctrl_args->addr_asoc_pinmux_group_datas;
//	dump_pinctrl_groups_data();

	gpiocfgs = (struct gpio_pre_cfg *)p_pinctrl_args->addr_gpio_cfgs;

	//functions
//	printk("%s:%d,nfunctions\n", __func__, __LINE__);
	asoc_pinmux_func_names = (struct asoc_pinmux_name *)p_pinctrl_args->addr_asoc_pinmux_func_names;
	leopard_functions = (struct asoc_pinmux_func *)p_pinctrl_args->addr_asoc_pinmux_funcs;

	for(i=0; i<nfunctions; i++)
	{
//		printk("i:%d,n:%d\n", i, leopard_functions[i].ngroups);
		leopard_functions[i].name = (char *)((unsigned int)leopard_functions[i].name + diff);
		leopard_functions[i].groups = (const char * const *)((unsigned int)leopard_functions[i].groups + diff);
		for(j=0; j<leopard_functions[i].ngroups; j++)
		{
//			printk("0x%08x ", asoc_pinmux_func_names[i].groups[j]);
			asoc_pinmux_func_names[i].groups[j] = (char *)((unsigned int)asoc_pinmux_func_names[i].groups[j]+ diff);
		}
//		printk("\n");
	}
	//groups
//	printk("%s:%d,groups\n", __func__, __LINE__);
	leopard_pingroups = (struct asoc_pinmux_group *)(p_pinctrl_args->addr_asoc_pinmux_groups);
	for(i=0; i<ngroups; i++)
	{
//		printk("n:%d\n",  i, leopard_pingroups[i].npins);
		leopard_pingroups[i].name = (char *)((unsigned int)leopard_pingroups[i].name + diff);
		leopard_pingroups[i].pins = (unsigned int *)((unsigned int)leopard_pingroups[i].pins + diff);
		leopard_pingroups[i].regcfgs = (struct asoc_regcfg *)((unsigned int)leopard_pingroups[i].regcfgs + diff);
	}
	//maps
//	printk("%s:%d,nmaps\n", __func__, __LINE__);
	leopardfpga_pinmux_map = (struct pinctrl_map *)(p_pinctrl_args->addr_pinctrl_maps);
	for(i=0; i<nmaps; i++)
	{
//		printk("i:%d\n", i);
		leopardfpga_pinmux_map[i].ctrl_dev_name = (char *)((unsigned int)leopardfpga_pinmux_map[i].ctrl_dev_name + diff);
		leopardfpga_pinmux_map[i].name = (char *)((unsigned int)leopardfpga_pinmux_map[i].name + diff);
		leopardfpga_pinmux_map[i].dev_name= (char *)((unsigned int)leopardfpga_pinmux_map[i].dev_name + diff);
		leopardfpga_pinmux_map[i].data.mux.group = (char *)((unsigned int)leopardfpga_pinmux_map[i].data.mux.group + diff);
		if(*(leopardfpga_pinmux_map[i].data.mux.group) == 0x0)
		{
			leopardfpga_pinmux_map[i].data.mux.group = NULL;
		}
		leopardfpga_pinmux_map[i].data.mux.function= (char *)((unsigned int)leopardfpga_pinmux_map[i].data.mux.function + diff);
	}
}

void __init setup_pinctrl(void)
{
	void *bootmem_for_pinctrl=NULL;
	
	if (!pinctrl_buf_start_phys)
		return;

	bootmem_for_pinctrl = alloc_bootmem_align(pinctrl_buf_len, PAGE_SIZE);
	printk("bootmem_for_pinctrl:0x%08x\n", bootmem_for_pinctrl);
	
	identity_mapping_add(swapper_pg_dir, pinctrl_buf_start_phys, pinctrl_buf_start_phys+PFN_ALIGN(pinctrl_buf_len));
	pinctrl_buf_start_virt = pinctrl_buf_start_phys;
	printk("%s,pinctrl_buf_start_virt:0x%x, pinctrl_buf_start_phys:0x%x\n", __FUNCTION__, pinctrl_buf_start_virt, pinctrl_buf_start_phys);

	memcpy(bootmem_for_pinctrl, pinctrl_buf_start_virt, pinctrl_buf_len);
	
	identity_mapping_del(swapper_pg_dir, pinctrl_buf_start_phys, pinctrl_buf_start_phys+PFN_ALIGN(pinctrl_buf_len));
	if(pinctrl_buf_start_phys >= PHYS_OFFSET)
    	memblock_free(pinctrl_buf_start_phys, pinctrl_buf_len);

	printk("%s,bootmem_for_pinctrl:0x%x, pinctrl_buf_start_phys:0x%x\n", __FUNCTION__, bootmem_for_pinctrl, pinctrl_buf_start_phys);
	fix_pinctrl_buf(bootmem_for_pinctrl, bootmem_for_pinctrl-pinctrl_buf_start_phys);
}

void __init arm_xml_pinctrl_memblock_reserve(void)
{
	if (!pinctrl_buf_start_phys)
		return;

	/* Reserve the xml region */
	if(pinctrl_buf_start_phys >= PHYS_OFFSET) {
	    memblock_reserve(pinctrl_buf_start_phys, pinctrl_buf_len);
		printk("memblock_reserve,0x%x:0x%x\n", pinctrl_buf_start_phys, pinctrl_buf_len);
    }
}

static int __init parse_tag_pinctrl(const struct tag *tag)
{
	pinctrl_buf_start_phys = tag->u.pinctrl.pinctrl_buf_start;
	pinctrl_buf_len = tag->u.pinctrl.pinctrl_buf_len;

	printk("pinctrl_buf_start_phys:0x%x, pinctrl_buf_len:0x%x\n", pinctrl_buf_start_phys, pinctrl_buf_len);
	return 0;
}

__tagtable(ATAG_PINCTRL, parse_tag_pinctrl);
