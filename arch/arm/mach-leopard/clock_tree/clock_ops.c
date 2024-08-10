
#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <mach/clock.h>
#include <mach/hardware.h>

#define MODULE_RESET_TIME_MS        1
//#define GPU_NIC_OFFSET  16

/*
  GPU MEM/SYS CLk 相对于CORE CLK在reg中的偏移
 */
#define GPU_MEM_OFFSET  16
#define GPU_SYS_OFFSET  24

DEFINE_SPINLOCK(clock_register_lock);
DEFINE_SPINLOCK(clock_reset_register_lock);
DEFINE_SPINLOCK(clock_cmudevclk_lock);

static unsigned long leopard_bus_clk_recalc(struct clk* c);
static void leopard_periph_clk_reset(struct clk *c);
static unsigned long leopard_clk_recalc(struct clk *c);
static unsigned long leopard_periph_clk_get_rate(struct clk *c);
static u32 _read_divisor(struct clk *clk, int* is_step_added);
static int leopard_periph_clk_is_enabled(struct clk *c);
static int leopard_clk_set_parent(struct clk *c, struct clk *parent);
static int leopard_periph_clk_set_rate(struct clk *c, unsigned long rate);

static int is_enabled_periph_clk(unsigned int clk_num)
{
    unsigned int reg =0 , id = 0;
    
    if(clk_num == 0xffffffff)
    {
        return 1;
    }
    
    if(clk_num >= MODULE_CLK_MAX)
    {
        return -EINVAL;
    }
    reg = ((clk_num/32 == 0)? CMU_DEVCLKEN0 : CMU_DEVCLKEN1);
    id = clk_num % 32;
    
    return ((act_readl(reg) & (0x1<<id))? 1 : 0);
}


static int enable_periph_clk(unsigned int clk_num)
{
    unsigned int reg =0 , id = 0, tmp =0;
    unsigned long flags;
    
    if(clk_num == 0xffffffff)
    {
        return 0;
    }
    
    if(clk_num >= MODULE_CLK_MAX)
    {
        return -EINVAL;
    }
    reg = ((clk_num/32 == 0)? CMU_DEVCLKEN0 : CMU_DEVCLKEN1);
    id = clk_num % 32;
    spin_lock_irqsave(&clock_cmudevclk_lock, flags);
    tmp = act_readl(reg);
    if((tmp & (0x1<<id)) == 0)
    {
        tmp = (act_readl(reg) | (0x1<<id));
//        if(clk_num == MODULE_CLK_GPU2D)
//        {
//            tmp |= (0x1<< MODULE_CLK_GPU2DLP);
//        } else if (clk_num == MODULE_CLK_GPU3D)
//        {
//            tmp |= (0x1<< MODULE_CLK_GPU3DLP);
//        }
        
        act_writel(tmp, reg);
        act_readl(reg);
    }
	
	spin_unlock_irqrestore(&clock_cmudevclk_lock, flags);
    
//    if((clk_num == MODULE_CLK_GPU2D ) || (clk_num == MODULE_CLK_GPU3D))
//    {
//        printk("\n %s clk_num:%d, reg[0x%x]:0x%x", __FUNCTION__, clk_num, reg, act_readl(reg));
//    }
    return 0;
}

static int disable_periph_clk(unsigned int clk_num)
{
    unsigned int reg =0 , id = 0, tmp = 0;
    unsigned long flags;
    
    if(clk_num == 0xffffffff)
    {
        return 0;
    }
    if(clk_num >= MODULE_CLK_MAX)
    {
        return -EINVAL;
    }
    
    if(is_enabled_periph_clk(clk_num))
    {
        reg = ((clk_num/32 == 0)? CMU_DEVCLKEN0 : CMU_DEVCLKEN1);
        id = clk_num % 32;
        spin_lock_irqsave(&clock_cmudevclk_lock, flags);
        tmp = (act_readl(reg) & (~(0x1<<id)));
//        if(clk_num == MODULE_CLK_GPU2D)
//        {
//            tmp &= (~(0x1<<MODULE_CLK_GPU2DLP));
//        }
//        else if(clk_num == MODULE_CLK_GPU3D)
//        {
//            tmp &= (~(0x1<<MODULE_CLK_GPU3DLP));
//        }
//        
        act_writel(tmp, reg);
        act_readl(reg);
	    spin_unlock_irqrestore(&clock_cmudevclk_lock, flags);
    }
//    
//    if((clk_num == MODULE_CLK_GPU2D ) || (clk_num == MODULE_CLK_GPU3D))
//    {
//        printk("\n %s clk_num:%d, reg[0x%x]:0x%x", __FUNCTION__, clk_num, reg, act_readl(reg));
//    }
    return 0;
}

static int reset_periph_clk(unsigned int reset_id)
{
    unsigned int reg =0, id=0;
 	unsigned long flags = 0;
    
    if(reset_id == 0xffffffff)
    {
        return 0;
    }
    
    if(reset_id >= MODULE_RESET_ID_MAX)
    {
        return -EINVAL;
    }

    module_reset((enum module_reset_id)reset_id);
	
	/* move to powergate.c */
#if 0
    reg = ((reset_id/32 == 0)? CMU_DEVRST0 : CMU_DEVRST1);
    id = reset_id % 32; 

    spin_lock_irqsave(&clock_reset_register_lock, flags);
    /*先写0，  后写1*/
    act_writel(act_readl(reg) & ~(0x1<<id), reg);
    act_readl(reg); /*OCP Barrier*/
    mdelay(MODULE_RESET_TIME_MS);
    
    act_writel(act_readl(reg) | (0x1<<id), reg);
    act_readl(reg);
    mdelay(MODULE_RESET_TIME_MS);
    
    spin_unlock_irqrestore(&clock_reset_register_lock, flags);
 #endif

    return 0;
}



static unsigned long leopard_clk_recalc(struct clk *c)
{
    unsigned long rate;
    u32 div = 0;
    /*不使用clk_get_rate(c->paretnt),避免spinlock死锁*/
    unsigned long parent_rate;
    int is_step_added = 0;
    
    if(!c)
    {
//        printk("[%s], line:%d", __FUNCTION__, __LINE__); 
        return 0;
    }

    if(!c->parent)
    {
        return c->rate;
    }

    parent_rate = c->parent->rate; 
        
    if((c->flag & DIV_FIXED) != 0)
    {
        div = c->div;
    }
    else
    {
        div = _read_divisor(c, &is_step_added);
    }
    
	if (div == 0)
	{
		return c->rate;
	}
	
	if(is_step_added == 1)
	{
	    rate = (parent_rate*CLK_DIV_FACTOR)/(div*CLK_DIV_FACTOR+1);
	}
	else
	{
        rate = parent_rate/div;
    }   
    return rate;
}

static struct clk_mux_sel *_get_clksel_by_parent(struct clk *clk,
						  struct clk *src_clk)
{
	struct clk_mux_sel *clks;
    if(!clk || !src_clk || !clk->clksel)
    {
        return NULL;
    }

	for (clks = clk->clksel; clks->input; clks++)
		if (clks->input == src_clk)
			break; /* Found the requested parent */

	if (!clks->input) {
		/* This indicates a data problem */
		WARN(1, "clock: Could not find parent clock %s in clksel array "
		     "of clock %s\n", src_clk->name, clk->name);
		return NULL;
	}

	return clks;
}

//static void _write_clksel_reg(struct clk *clk, u32 field_val)
//{
//	u32 v, tmp;
//	v = act_readl(clk->clksel_reg);
//	tmp = (v & clk->clksel_mask)>>__ffs(clk->clksel_mask);
//
//	if(tmp == field_val)
//	{
//		return;
//	}
//
//	v &= ~clk->clksel_mask;
//	v |= field_val << __ffs(clk->clksel_mask);
//	act_writel(v, clk->clksel_reg);
//
//	v = act_readl(clk->clksel_reg); /* OCP barrier */
//}

static void _write_clksel_reg(struct clk *clk, u32 field_val)
{
	u32 v, tmp, offset;
	offset = __ffs(clk->clksel_mask);
	v = act_readl(clk->clksel_reg);
	tmp = ((v & clk->clksel_mask)>>offset);

	if(tmp == field_val)
	{
		return;
	}
	
	if((clk->flag & TYPE_GPU)!= 0)
	{
	    v &= ~clk->clksel_mask;
    	v &= ~(clk->clksel_mask<<GPU_MEM_OFFSET);
    	v &= ~(clk->clksel_mask<<GPU_SYS_OFFSET);
    	v |= (field_val << offset);
    	v |= (field_val << (offset+GPU_MEM_OFFSET));
    	v |= (field_val << (offset+GPU_SYS_OFFSET));
	}
	else
	{
	    v &= ~clk->clksel_mask;
	    v |= (field_val <<offset);
	}
	
	act_writel(v, clk->clksel_reg);
	v = act_readl(clk->clksel_reg); /* OCP barrier */

    return;
}


static void _write_div_reg(struct clk* clk, u32 field_val)
{
	u32 tmp;
    u32 v = act_readl(clk->clksel_reg);
    u32 offset = __ffs(clk->div_mask);
	tmp = ((v & clk->div_mask) >> offset);
	
	if(tmp == field_val)
	{
		return;
	}
	if((clk->flag & TYPE_GPU) != 0)
	{
	    v &= ~clk->div_mask;
        v &= ~(clk->div_mask<<GPU_MEM_OFFSET);
        v &= ~(clk->div_mask<<GPU_SYS_OFFSET);
        v |= (field_val << offset);
        v |= (field_val<< (offset+GPU_MEM_OFFSET));   
        v |= (field_val<< (offset+GPU_SYS_OFFSET));       
	}
	else
	{
        v &= ~clk->div_mask;
        v |= (field_val << offset);     
    }
    act_writel(v, clk->clksel_reg);
    v = act_readl(clk->clksel_reg);/* OCP barrier */
    
    return;
}


//static void _write_gpu_div_reg(struct clk* clk, u32 field_val)
//{
//	u32 tmp;
//    u32 v = act_readl(clk->clksel_reg);
//    u32 offset = __ffs(clk->div_mask);
//	tmp = (v & clk->div_mask) >> offset;
//	if(tmp == field_val)
//	{
//		return;
//	}
//    v &= ~clk->div_mask;
//    v &= ~(clk->div_mask<<GPU_NIC_OFFSET);
//    v |= (field_val << offset);
//    v |= (field_val<< (offset+GPU_NIC_OFFSET));
//    act_writel(v, clk->clksel_reg);
//    v = act_readl(clk->clksel_reg);/* OCP barrier */
//
//   	printk("\n %s clk:%s, field_val:0x%x, reg:0x%x", __FUNCTION__, clk->name, field_val, v);
//
//}

static u32 _clksel_to_divisor(struct clk *clk, u32 field_val, int* is_step_added)
{
	struct clksel_rate *clkr;
    
    if((clk->flag & DIV_UART) != 0)
    {
        /*uart */
        if(field_val > DIV_UART_MAX)
        {
//            printk("\n error at %s, clk:%s, field_val:%d", __FUNCTION__, clk->name, field_val);
            return 0;
        }
        if(field_val == DIV_UART_VALUE_MAX)
        {
            return DIV_UART_MAX;
        }

        return field_val+1;
    }

	if((clk->flag & DIV_ROUND_1_64) != 0)
	{
		 if(field_val > 63)
		 	{
            return 0;
		 	}
			return field_val+1;
	}
	
	for (clkr = clk->rates; clkr->div; clkr++) {
		if (clkr->val == field_val)
		{
			break;
		}
	}

	if (!clkr->div) {
		/* This indicates a data error */
		WARN(1, "clock: Could not find fieldval %d for clock %s parent "
		     "%s\n", field_val, clk->name, clk->parent->name);
		return 0;
	}
	
	if((clkr->flags & CLK_DIV_FLAG_ADD_STEP) != 0)
	{
	    *is_step_added = 1;    
    }
    else
    {
        *is_step_added = 0;
    } 
	return clkr->div;
}

static int set_clksel_by_parent(struct clk* clk, struct clk* parent)
{
    struct clk_mux_sel * sel = NULL;
    if(!clk || !parent)
    {
        return -EINVAL;
    }
    sel = _get_clksel_by_parent(clk, parent);
    if(!sel)
    {
        return -EINVAL;
    }
    _write_clksel_reg(clk, sel->value);
    return 0;
}

/**
 * _read_divisor() - get current divisor applied to parent clock (from hdwr)
 * @clk: struct clk to use.
 *
 * Read the current divisor register value for @clk that is programmed
 * into the hardware, convert it into the actual divisor value, and
 * return it; or return 0 on error.
 */
static u32 _read_divisor(struct clk *clk, int* is_step_added)
{
	u32 v;
    u32 ret=0;
	if (!clk->clksel_reg || !clk->div_mask)
		return 0;

	v = act_readl(clk->clksel_reg);
	v &= clk->div_mask;
	v >>= __ffs(clk->div_mask);
	ret =  _clksel_to_divisor(clk, v, is_step_added);
//    printk("\n[zjl] %s , clk:%s, v:0x%x, ret:%d", __FUNCTION__, clk->name, v, ret);

	return ret;
}

/*
    根据target_rate，返回能满足需求的div，如果返回负数，则表示无法找到合适的div.
    调频策略：就低不就高，也即寻找<= target_rate的rate
 */
static int leopard_clk_round_rate_div(struct clk *clk, unsigned long target_rate,
				u32 *new_div, u32* value, int* is_step_added)
{
    unsigned long parent_rate = 0;
    struct clksel_rate *clkr;
    unsigned long test_rate;
    u32 last_div = 0, last_value=0;
    u32 max_div=0, max_serial_div=0, max_value=0;
    int has_step = 0; /*新增，for 小数分频比*/
  
    if(!clk || !clk->parent)
    {
        return -EINVAL;
    }
    
    *new_div = 1;
    parent_rate = clk->parent->rate;
 
//    printk("\n[zjl] %s clk:%s, target_rate:%ld, parent_rate:%ld",__FUNCTION__, clk->name, target_rate, parent_rate);
    if((target_rate > parent_rate) || (target_rate == 0))
    {
        return -EINVAL;
    }
    
    /*
       判断是否uart clk，这个clk单独处理
    */
    if((clk->flag & DIV_UART) != 0)
    {
        max_serial_div = DIV_UART_SERIAL_MAX;
        max_div = DIV_UART_MAX;
        max_value = DIV_UART_VALUE_MAX;

        last_div =  parent_rate/target_rate;
        
        if(last_div == 0)
        {
            return -EINVAL;
        }
        
        if(last_div>=max_div)
        {
            *new_div = max_div;
            *value = max_value;
            return 0;
        }
           
        if(last_div>max_serial_div)
        {
            *new_div = max_serial_div;
            *value = max_serial_div-1;
            return 0;
        }
                
        /*div belong to the serial range*/
        *new_div = last_div;
        *value = last_div-1;
        return 0;
    }
    
    /*rates from 1 to 64*/
    if((clk->flag & DIV_ROUND_1_64) != 0)
    {
        last_div = parent_rate/target_rate;
        if(last_div == 0)
        {
            return -EINVAL;
        }
        if(last_div>64)
        {
            *new_div = 64;
            *value = 63;
        }
        else
        {
            *new_div = last_div;
            *value = last_div -1;
        }
        return 0;
    }
    
    /*
        一般的clk，需浏览其clkrates，来决定选择什么div
     */
	for (clkr = clk->rates; clkr->div; clkr++) {
//    	/* Sanity check */
//    	if (clkr->div <= last_div)
//    		pr_err("clock: clksel_rate table not sorted "
//    		       "for clock %s", clk->name);
        
        if((clkr->flags & CLK_DIV_FLAG_ADD_STEP)!= 0)
        {
            /*该分频比有小数bit*/
            has_step = 1;
            test_rate = (parent_rate * CLK_DIV_FACTOR)/(clkr->div * CLK_DIV_FACTOR + 1);    
        }
        else
        {
            has_step = 0;
            test_rate = parent_rate/clkr->div;
        }
        
    	if (test_rate <= target_rate)
    	{
    		break; /* found it */
    	}
//    	else
//    	{
//    	    last_div = clkr->div;  
//    	    last_value = clkr->val;
//    	}
    }

//	if (last_div == 0) {
//		printk("\nclock: Could not find divisor for target "
//		       "rate %ld for clock %s parent %s\n", target_rate,
//		       clk->name, clk->parent->name);
//		return -EINVAL;
//	}

    if(clkr->div == 0)
    {
//		printk("\nclock: Could not find divisor for target "
//		       "rate %ld for clock %s parent %s\n", target_rate,
//		       clk->name, clk->parent->name);
        *is_step_added = 0;
		return -EINVAL;
    }
	*new_div = clkr->div;
    *value = clkr->val;
    *is_step_added = has_step;
    
    return 0;
}


#if 0
unsigned long asoc_losc_get_rate(struct clk *clk)
{
    return CMU_LOSC_FREQ;
}

int asoc_losc_is_enabled(struct clk *clk)
{
    return 1;
}

struct clk_ops losc_ops = {
    .get_rate = asoc_losc_get_rate,
    .is_enabled = asoc_losc_is_enabled,
};

/*
    hosc description
 */
static unsigned long asoc_hosc_get_rate(struct clk *clk)
{
    if (!(act_readl(CMU_COREPLL) & CMU_COREPLL_HOEN))
        return 0;
    return CMU_HOSC_FREQ;
}

static int asoc_hosc_is_enabled(struct clk *clk)
{
    return (act_readl(CMU_COREPLL) & CMU_COREPLL_HOEN) ? 1 : 0;
}

static int asoc_hosc_enable(struct clk *clk)
{
    if(!asoc_hosc_is_enabled(clk))
    {
        act_writel(act_readl(CMU_COREPLL) | CMU_COREPLL_HOEN, CMU_COREPLL);
    }
    return 0;
}

static int asoc_hosc_disable(struct clk *clk)
{
    if(asoc_hosc_is_enabled(clk))
    {
        act_writel(act_readl(CMU_COREPLL) & ~CMU_COREPLL_HOEN, CMU_COREPLL);
    }
    return 0;
}



struct clk_ops hosc_ops = {
    .enable = asoc_hosc_enable,
    .disable = asoc_hosc_disable,
    .get_rate = asoc_hosc_get_rate,
    .is_enabled = asoc_hosc_is_enabled
};

#endif

/*
    pll functions
 */

static unsigned long leopard_pll_recalc(struct clk* clk)
{
     unsigned long value = 0;
     unsigned int shift = 0;
     if(clk==NULL)
     {
         return -EINVAL;
     }
     
    if((clk->flag & PLL_FIXED) != 0)
    {
        return clk->rate;
    }
    
     shift = __ffs(clk->u.pll.value_mask);
     value = act_readl(clk->u.pll.enable_reg);
     value &= clk->u.pll.value_mask;
     value >>= shift;
     value *= (clk->u.pll.rate_step);
     return value;
}

static unsigned long leopard_pll_get_rate(struct clk *clk)
{
    unsigned long rate = 0;
    if((clk->flag & PLL_FIXED) != 0)
    {
        return clk->rate;
    }
    rate = leopard_pll_recalc(clk);
    /*预防初始化是，clk->rate = 0,但实际clk又match table->rate,此时c->rate就不会被设置*/
    if(clk->rate == 0)
    {
        clk->rate = rate;
    }
    return rate;
}

static int leopard_pll_set_rate(struct clk *c, unsigned long rate)
{
    unsigned int value = 0;
    unsigned int tmp_rate = 0;
    unsigned int shift;

    if((c == NULL) 
        || (rate == 0)
        || (rate<c->u.pll.min_rate)
        || (rate>c->u.pll.max_rate))
    {
//        printk("\n[zjl] err at  %s %d", __FUNCTION__, __LINE__);
        return -EINVAL;
    }
    
    if(c->rate == rate)
    {
        return 0;
    }
    
    if((c->flag & PLL_FIXED) != 0)
    {
        return 0;
    }
        
    tmp_rate = rate/(c->u.pll.rate_step);
    shift = __ffs(c->u.pll.value_mask);
    value = act_readl(c->u.pll.enable_reg);
    value &= ~(c->u.pll.value_mask);
    value |= (tmp_rate<<shift);
    act_writel(value, c->u.pll.enable_reg);
    act_readl(c->u.pll.enable_reg);
    if(c->u.pll.lock_delay != 0)
    {
        udelay(c->u.pll.lock_delay);
    }
    c->rate = rate;
    
    /*push children*/
    propagate_rate(c);
    
    return 0;
}

static int leopard_pll_is_enabled(struct clk *clk)
{
    return (act_readl(clk->u.pll.enable_reg) & (clk->u.pll.enable_mask)) ? 1 : 0;
}

static int leopard_pll_enable(struct clk *clk)
{
    if(!leopard_pll_is_enabled(clk))
    {
        act_writel(act_readl(clk->u.pll.enable_reg) | clk->u.pll.enable_mask, clk->u.pll.enable_reg);
    }
    
    if(clk->rate == 0)
    {
        clk->rate = leopard_pll_recalc(clk);
    }
    return 0;
}

static int leopard_pll_disable(struct clk *clk)
{
    if((clk->flag & PLL_ALAWAY_ON) != 0)
    {
        return 0;
    }
    
    if(leopard_pll_is_enabled(clk))
    {
    act_writel(act_readl(clk->u.pll.enable_reg) & ~(clk->u.pll.enable_mask), clk->u.pll.enable_reg);
    }
    return 0;
}



struct clk_ops leopard_pllx_ops = {
    .enable = leopard_pll_enable,
    .disable = leopard_pll_disable,
    .get_rate = leopard_pll_get_rate,
    .set_rate = leopard_pll_set_rate,
    .is_enabled = leopard_pll_is_enabled,
    .recalc = leopard_pll_recalc,
};



static int leopard_usbpll_enable(struct clk *clk)
{
    if(!leopard_pll_is_enabled(clk))
    {
        act_writel(act_readl(clk->u.pll.enable_reg) | clk->u.pll.enable_mask, clk->u.pll.enable_reg);
    }
    
    return 0;
}

static int leopard_usbpll_disable(struct clk *clk)
{ 
    if(leopard_pll_is_enabled(clk))
    {
        act_writel(act_readl(clk->u.pll.enable_reg) & ~(clk->u.pll.enable_mask), clk->u.pll.enable_reg);
    }
    return 0;
}

struct clk_ops leopard_usbpll_ops = {
    .enable = leopard_usbpll_enable,
    .disable = leopard_usbpll_disable,
    .is_enabled = leopard_pll_is_enabled,
};

/* virtual cpu clock functions */
/* some clocks can not be stopped (cpu, memory bus) while the SoC is running.
   To change the frequency of these clocks, the parent pll may need to be
   reprogrammed, so the clock must be moved off the pll, the pll reprogrammed,
   and then the clock moved back to the pll.  To hide this sequence, a virtual
   clock handles it.
 */
 
static void leopard_cpu_clk_init(struct clk *c) 
{
//    struct clk_mux_sel *sel;
//    unsigned int value, shift;
//
//    c->state = ON;
//     
//    printk("\n[zjl] %s %d", __FUNCTION__, __LINE__);    
//    shift = __ffs(c->clksel_mask);
//    value = (act_readl(c->clksel_reg) & (c->clksel_mask))>>shift;
//   
//    printk("\n[zjl] %s %d", __FUNCTION__, __LINE__);
//    for(sel=c->clksel;sel->input!=NULL; sel++)
//    {
//        if(sel->value == value)
//        {
//            break;
//        }
//    }
//    if(sel->input == NULL)
//    {
//        printk("\n[zjl] error at %s, %d", __FILE__, __LINE__);
//    }
//    //c->parent = sel->input;
//    printk("\n[zjl] %s %d", __FUNCTION__, __LINE__);
//    clk_reparent(c, sel->input);
//    printk("\n[zjl] %s %d", __FUNCTION__, __LINE__);
}

static unsigned long leopard_cpu_clk_recalc(struct clk *c)
{
    return (c->parent->rate);
}


static int leopard_cpu_clk_set_rate(struct clk *c, unsigned long rate) 
{
    /*切换源头到vce_clk_before_gating*/
    struct clk* vce_clk = NULL;
    struct clk* dma_clk=NULL;
    struct clk* corepll_clk = NULL;
    unsigned int cur_dma_div = 0;
    unsigned int dma_change_rate = 0;
    unsigned int dma_current_rate = 0;
    
    vce_clk = clk_get_by_name(CLK_NAME_VCE_CLK);
    dma_clk = clk_get_by_name(CLK_NAME_DMA_CLK);
    corepll_clk = clk_get_by_name(CLK_NAME_COREPLL);
    
    if(!vce_clk || !dma_clk ||!corepll_clk)
    {
        return -EINVAL;
    }

    dma_current_rate = leopard_periph_clk_get_rate(dma_clk);
    dma_change_rate = dma_current_rate;
    cur_dma_div = c->rate/dma_current_rate;
    if(cur_dma_div<8)
    {
        dma_change_rate = c->rate/8;
    }

    if(dma_change_rate != dma_current_rate)
    {
        /*设置分频比为4*/
        leopard_periph_clk_set_rate(dma_clk, dma_change_rate);
    }

    /*切换源头为vce_clk*/
    leopard_clk_set_parent(c, vce_clk);

    /*设置corepll的值*/
    leopard_pll_set_rate(corepll_clk, rate);

    /*切换回corepll*/
    leopard_clk_set_parent(c, corepll_clk);
    
    c->rate = leopard_cpu_clk_recalc(c);
    
    /*重置dma 分频比*/
    if(c->rate <= dma_current_rate)
    {
        /*设置dma clk div=1*/
        leopard_periph_clk_set_rate(dma_clk, (c->rate));
    }
    else
    {
        leopard_periph_clk_set_rate(dma_clk, dma_current_rate);
    }   
  
    /*push children*/
    propagate_rate(c);
    return 0;
}

static unsigned long leopard_cpu_clk_get_rate(struct clk *c)
{
    unsigned long rate = c->parent->rate;
    
    /*预防初始化时，clk->rate = 0,但实际clk又match table->rate,此时c->rate就不会被设置*/
    if(c->rate == 0)
    {
        c->rate = rate;
    }
   
    return rate;
}

static int leopard_clk_set_parent(struct clk *c, struct clk *parent)
{
    int ret = -EINVAL;
    
    if(!c || !parent)
    {
        return -EINVAL;
    }
    
    /*虚拟的clk或者是固定的clk*/
    if(c->clksel == NULL)
    {
        c->parent = parent;
        return 0;  
    }
    
    /*根据指定的parent，设置硬件寄存器*/
    ret = set_clksel_by_parent(c, parent);
    if(ret!=0)
    {
//        printk("\n[zjl] error at %s %d", __FUNCTION__, __LINE__);
        return ret;
    }
    c->parent = parent;
    /*设置默认rate*/
    if(c->ops && c->ops->recalc)
    {
        c->rate = c->ops->recalc(c);
    }
    return 0;
    
}

static int leopard_cpu_clk_set_parent(struct clk *c, struct clk *parent)
{
    struct clk_mux_sel	*sel = NULL;

    for(sel=c->clksel; sel->input!=NULL; sel++)
    {
        if(sel->input == parent)
        {
            if (c->refcnt)
				clk_enable(parent);
				    
            /*find the parent*/
            _write_clksel_reg(c, sel->value);
            
            if (c->refcnt && c->parent)
				clk_disable(c->parent);
	        c->rate = parent->rate;
			return 0;
        }
    }
    return -EINVAL;
}


struct clk_ops leopard_cpuclk_ops = {
    .init = leopard_cpu_clk_init,
    .set_parent = leopard_cpu_clk_set_parent,
    .set_rate = leopard_cpu_clk_set_rate,
    .get_rate = leopard_cpu_clk_get_rate,
    .recalc = leopard_cpu_clk_recalc,
};

///*check shared bus clk是否需要调整分频比*/
// int leopard_clk_shared_bus_adjust(struct clk* bus)
//{
//    struct clk *c;
//    unsigned long rate = 0;
//	list_for_each_entry(c, &bus->shared_bus_list, u.shared_bus_user.node) {
//	    if (c->u.shared_bus_user.enabled)
//	    {
//	        rate = c->rate;
//	    }
//	}
//}


/* shared bus ops */
/*
 * Some clocks may have multiple downstream users that need to request a
 * higher clock rate.  Shared bus clocks provide a unique shared_bus_user
 * clock to each user.  The frequency of the bus is set to the highest
 * enabled shared_bus_user clock, with a minimum value set by the
 * shared bus.
 */
static int leopard_clk_shared_bus_update(struct clk *bus)
{
    int ret = 0;
    u32 new_div =0, new_value=0;
	struct clk *c;
	unsigned long rate = bus->rate;
    unsigned int prior = 0xf;
    int is_step_added=0;
   
    /*满足优先级为最高的clk的需求*/
	list_for_each_entry(c, &bus->shared_bus_list, u.shared_bus_user.node)
	{
	    
//		if (c->u.shared_bus_user.enabled)
		if(c != NULL)
		{
		    if((c->u.shared_bus_user.priority < prior) && (c->u.shared_bus_user.rate != 0))
		    {
		        rate = c->u.shared_bus_user.rate;
		        prior = c->u.shared_bus_user.priority;
//        	    printk("\n %s clk:%s, request_rate:%d, prior:%d, cur_rate:%d", __FUNCTION__, c->name, rate, prior, c->rate);
		    }
			//rate = max(c->u.shared_bus_user.rate, rate);
        }
    }
    
//    printk("\n %s bus:%s, rate:%d, prior:%d", __FUNCTION__, bus->name, rate, prior);
    
    if(prior == 0xf)
    {
//        printk("\n ###no shared_bus_user enabled???");
        return 0;
    }
    
    /*如果当前rate满足需求，则不用调整*/
	if ((rate == bus->ops->recalc(bus)) || (rate == 0))
		return 0;
    
    ret = clk_set_rate(bus, rate);
    
    /*对其子节点进行调整*/
    list_for_each_entry(c, &bus->shared_bus_list, u.shared_bus_user.node)
	{
	    //if (c->u.shared_bus_user.enabled)
	    if(c != NULL)
	    {
    	    ret = leopard_clk_round_rate_div(c, c->u.shared_bus_user.rate, &new_div, &new_value,&is_step_added);
            if(ret != 0)
            {
                printk("\n[zjl] can not set rate:%ld for clk:%s,parent:%s, parent_clk:%ld. ", c->rate, c->name, c->parent->name, c->parent->rate);
                ret = -EINVAL;
                return ret;
            }
             _write_div_reg(c, new_value);
	    }
    }
    propagate_rate(bus);  
	return ret;
};

static void leopard_clk_shared_bus_init(struct clk *c)
{
//	unsigned long flags;
//    
//    if(!c || !c->parent)
//    {
////        printk("\n[zjl] error at %s %d", __FUNCTION__, __LINE__);
//        return ;
//    }
//    
//	c->state = OFF;
//	c->set = true;
////    printk("\n[zjl] %s %d clk->name:%s, parent->name:%s", __FUNCTION__, __LINE__, c->name, c->parent->name);
//       
////    /*
////      shared bus init 较为特殊，不能通过init_table 的方式初始化，
////      其parent，rate都在数据结构定义时指定。
////     */
//
//    c->flag |= PARENT_FIXED;
//      
//	spin_lock_irqsave(&c->parent->spinlock, flags);
//	list_add_tail(&c->u.shared_bus_user.node,
//		&c->parent->shared_bus_list);
//    c->u.shared_bus_user.rate = c->rate;
//	spin_unlock_irqrestore(&c->parent->spinlock, flags);

}

static int leopard_clk_shared_bus_set_parent(struct clk *c, struct clk *parent)
{
    /*通过set_parent时才来建立shared bus node关系*/
    unsigned long flags;
    struct clk *dev_clk = clk_get_by_name(CLK_NAME_DEVCLK);  
    struct clk* cur_parent = c->parent;
    
    
    if(parent == dev_clk)
    {
        /*如果挂载在dev_clk下*/
        spin_lock_irqsave(&dev_clk->spinlock, flags);
	    list_add_tail(&c->u.shared_bus_user.node,
		    &dev_clk->shared_bus_list);
//		c->u.shared_bus_user.rate = c->rate;
	    spin_unlock_irqrestore(&dev_clk->spinlock, flags);
    }
    
    if(cur_parent == dev_clk)
    {
        /*del from dev_clk*/
        spin_lock_irqsave(&dev_clk->spinlock, flags);
        list_del_init(&c->u.shared_bus_user.node);
        spin_unlock_irqrestore(&dev_clk->spinlock, flags);
    }
    
    leopard_clk_set_parent(c, parent); 
    
    if(cur_parent == dev_clk)
    {
        /*update share bus rate*/
        leopard_clk_shared_bus_update(dev_clk);
    }
    
    return 0;
}

static int leopard_clk_shared_bus_only_set_rate(struct clk *c, unsigned long rate)
{
	unsigned long flags;
	int ret = -EINVAL;
	long new_rate = rate;
	u32 new_div =0, new_value=0;
	int is_step_added = 0;
	
	spin_lock_irqsave(&c->parent->spinlock, flags);
	c->u.shared_bus_user.rate = new_rate;
	spin_unlock_irqrestore(&c->parent->spinlock, flags);

#if 0	  
//	  
//    /*判断调整分频比能否满足需求，主要考虑升频时，降频时不用考虑,devpll不调整*/
//    ret = leopard_clk_round_rate_div(c, rate, &new_div, &new_value);
//    if(ret == 0)
//    { 
//        /*有满足条件的div*/
//        u32 ori_div = _read_divisor(c);
//        if(ori_div == 0)
//        {
//            goto out;
//        }
//        if(ori_div == new_div)
//        {
//            /*不用修改分频比*/
//            ret = 0;
//            goto out;
//        }
//        else
//        {
//            _write_div_reg(c, new_value);
//            ret = 0;
//            goto out;
//        }
//    }
//    else
//    {
//        /*通过修改分频比也无法满足需求，此时考虑提高devpll的频率*/
//    	leopard_clk_shared_bus_update(c->parent);
//        ret = leopard_clk_round_rate_div(c, rate, &new_div, &new_value);
//        if(ret != 0)
//        {
//            printk("\n[zjl] can not set rate:%ld for clk:%s,parent:%s, parent_clk:%ld. ", rate, c->name, c->parent->name, c->parent->rate);
//            ret = -EINVAL;
//            goto out;
//        }
//         _write_div_reg(c, new_value);
//         ret = 0;
//           
//#if 1  /*该处其实可以去掉，待验证。*/
//         /*recal the childern*/
////         printk("\n[zjl] %s %d the devpll rate is changed,propagate_rate the childernes ", __FUNCTION__, __LINE__);
//         propagate_rate(c->parent);
////         printk("\n[zjl] %s %d over propagate_rate the childernes", __FUNCTION__, __LINE__);
//#endif
//    }

#else
    leopard_clk_shared_bus_update(c->parent);    
    ret = leopard_clk_round_rate_div(c, rate, &new_div, &new_value, &is_step_added);
    if(ret != 0)
    {
        printk("\n[%s] can not set rate:%ld for clk:%s,parent:%s, parent_clk:%ld. ", __FUNCTION__, rate, c->name, c->parent->name, c->parent->rate);
        ret = -EINVAL;
        goto out;
    }
     _write_div_reg(c, new_value);
     ret = 0;   
#endif
    
out: 
    if(ret == 0)
    {
        c->div = new_div;
        c->is_step = is_step_added;
        if(is_step_added == 1)
        {
            c->rate = (c->parent->rate * CLK_DIV_FACTOR)/(c->div * CLK_DIV_FACTOR + 1);
        }
        else
        {
            c->rate = c->parent->rate/c->div;
        }
    }
	return ret;
}

static int leopard_clk_shared_bus_set_rate(struct clk *c, unsigned long rate)
{
    struct clk *dev_clk = clk_get_by_name(CLK_NAME_DEVCLK);  
    struct clk* cur_parent = c->parent;

    if(cur_parent == dev_clk)
    {
        return leopard_clk_shared_bus_only_set_rate(c, rate);
    }
    else
    {
        return leopard_periph_clk_set_rate(c, rate);
    }
}


static long leopard_clk_shared_bus_round_rate(struct clk *c, unsigned long rate)
{
	return clk_round_rate(c->parent, rate);
}

static unsigned long leopard_clk_shared_bus_get_rate(struct clk *clk)
{
    return leopard_clk_recalc(clk);
}

static int leopard_clk_shared_bus_enable(struct clk *c)
{
	unsigned long flags;
	int ret;
    struct clk *dev_clk = clk_get_by_name(CLK_NAME_DEVCLK);  
    struct clk* cur_parent = c->parent;
    
	spin_lock_irqsave(&c->parent->spinlock, flags);
	c->u.shared_bus_user.enabled = true;
	spin_unlock_irqrestore(&c->parent->spinlock, flags);
	
//	ret = leopard_clk_shared_bus_update(c->parent);
//	if(ret == 0)
//	{
	    ret = enable_periph_clk(c->module_id);
//	}
    
//    printk("\n %s clk:%s, enabled:%d", __FUNCTION__, c->name, c->u.shared_bus_user.enabled);
	return ret;
}

static int leopard_clk_shared_bus_disable(struct clk *c)
{
	unsigned long flags;
	int ret=0;
    struct clk *dev_clk = clk_get_by_name(CLK_NAME_DEVCLK);  
    struct clk * parent = c->parent;

	spin_lock_irqsave(&c->parent->spinlock, flags);
	c->u.shared_bus_user.enabled = false;
	spin_unlock_irqrestore(&c->parent->spinlock, flags);
//    printk("\n %s clk:%s, enabled:%d", __FUNCTION__, c->name, c->u.shared_bus_user.enabled);

        disable_periph_clk(c->module_id);
        
    if(parent == dev_clk)
    {
	    ret = leopard_clk_shared_bus_update(c->parent);
    }
    
//	if(ret == 0)
//	{
//	    ret = disable_periph_clk(c->module_id);
 //   }

	return ret;
}

struct clk_ops leopard_clk_shared_bus_ops = {
	.init = leopard_clk_shared_bus_init,
	.enable = leopard_clk_shared_bus_enable,
	.disable = leopard_clk_shared_bus_disable,
	.set_rate = leopard_clk_shared_bus_set_rate,
	.get_rate = leopard_clk_shared_bus_get_rate,
//	.round_rate = leopard_clk_shared_bus_round_rate,
	.reset = leopard_periph_clk_reset,
	.recalc = leopard_clk_recalc,
	.is_enabled = leopard_periph_clk_is_enabled,
	.set_parent = leopard_clk_shared_bus_set_parent,
};

/*AMBA bus functions*/
static void	leopard_bus_clk_init(struct clk *c)
{
//    unsigned int value=0,shift=0, tmp=0;
//    struct clksel_rate * rate = NULL;
//        
//    value = act_readl(c->clksel_reg);
//    shift = __ffs(c->div_mask);
//    tmp = ((value & (c->div_mask)) >> shift);
//    for(rate= c->rates; rate->div!=0 ;rate++)
//    {
//        if(tmp == rate->val)
//        {
//            break;
//        }
//    }
//    
//    if(rate->div == 0)
//    {
//        printk("\n[zjl] error at %s %d",__FILE__, __LINE__);
//    }
//    c->div = rate->div;
    
}
static int	leopard_bus_clk_enable(struct clk *c)
{
    return 0;
}

static int	leopard_bus_clk_disable(struct clk *c)
{
    return 0;
}

static unsigned long leopard_bus_clk_recalc(struct clk* c)
{
    unsigned long rate;
    unsigned long parent_rate = c->parent->rate;
    int is_step_added = 0;
    u32 div = _read_divisor(c, &is_step_added);
	if (div == 0)
		return c->rate;
	
	if(is_step_added == 1)
	{
	    rate = (parent_rate*CLK_DIV_FACTOR)/(div*CLK_DIV_FACTOR + 1);
	}
	else
	{	
        rate = parent_rate/div;
    }
    return rate;
}

static unsigned long leopard_bus_clk_get_rate(struct clk *clk)
{
    unsigned long rate = leopard_bus_clk_recalc(clk);
    if(clk->rate == 0)
    {
        clk->rate = rate;
    }
    return rate;
}

static int leopard_bus_clk_set_rate(struct clk *c, unsigned long rate)
{
    unsigned long flags;
    unsigned long parent_rate = c->parent->rate;
    u32 div=0, val=0;
    int ret = -EINVAL;
    int is_step_added=0;
    
    if((c->max_rate!=0) && (rate > c->max_rate))
    {
//        printk("\n[zjl] can not set rate:%ld for clk:%s, max_rate:%ld", rate, c->name, c->max_rate);
        return -EINVAL;
    }
//    printk("\n[zjl] %s clk:%s, rate:%ld, parent_rate:%ld", __FUNCTION__,c->name, rate, parent_rate);
    spin_lock_irqsave(&clock_register_lock, flags);
//    for(p_rate= c->rates; p_rate->div != 0; p_rate++)
//    {
//        if(rate == (parent_rate/p_rate->div))
//        {
////            printk("\n[zjl] find the div:%d", p_rate->div);
//            c->rate = rate;
//            c->div = p_rate->div;   
//            _write_div_reg(c, p_rate->val);  
//            ret = 0;
//            break;
//        }
//    }

    ret = leopard_clk_round_rate_div(c, rate, &div, &val, &is_step_added);
    spin_unlock_irqrestore(&clock_register_lock, flags);
    if(ret != 0)
    {
//        printk("\n[zjl] %s can not set rate:%ld for clk %s ", __FUNCTION__, rate,c->name);
        return ret;
    }
    
    _write_div_reg(c, val);
    if(is_step_added == 1)
    {
        c->rate = (c->parent->rate * CLK_DIV_FACTOR)/(div * CLK_DIV_FACTOR + 1);
    }
    else
    {
        c->rate = c->parent->rate/div; 
    }
    propagate_rate(c); 
    return ret;  
}

struct clk_ops leopard_bus_clk_ops = {
    .init = leopard_bus_clk_init,
    .enable = leopard_bus_clk_enable,
    .disable = leopard_bus_clk_disable,
    .set_rate = leopard_bus_clk_set_rate,
    .get_rate = leopard_bus_clk_get_rate,
    .set_parent = leopard_clk_set_parent,
    .recalc = leopard_bus_clk_recalc,
};

static int	leopard_devclk_enable(struct clk *c)
{
    return 0;
}
static int	leopard_devclk_disable(struct clk *c)
{
    return 0;
}

static void leopard_devclk_init(struct clk *c) 
{
  
//    INIT_LIST_HEAD(&c->shared_bus_list);
    
// #if 0   
//    shift = __ffs(c->clksel_mask);
//    value = act_readl(c->clksel_reg);
//    tmp = (value & (c->clksel_mask))>>shift;
//    
//    /*
//      则设置硬件寄存器满足parent设置
//     */
//    for(sel=c->clksel; sel->input!=NULL; sel++)
//    {
//        if(tmp == sel->value)
//        {
//            ori_sel = sel;
//        }
//        if(sel->input == c->parent)
//        {
//            p_sel = sel;
//        }
//    }
//    printk("\n[zjl] %s %d, ori_reg:0x%x", __FUNCTION__, __LINE__, act_readl(c->clksel_reg));
//
//    if(ori_sel != p_sel)
//    {
//        _write_clksel_reg(c, p_sel->value);
//        printk("\n[zjl] %s %d, aft_reg:0x%x", __FUNCTION__, __LINE__, act_readl(c->clksel_reg));
//    }
//#endif
    return;
}

static int leopard_devclk_set_rate(struct clk *c, unsigned long rate)
{
    int ret = 0;
    ret = clk_set_rate(c->parent, rate);
    if(ret != 0)
    {
//        printk("\n[zjl] error at %s ,clk:%s, rate:%ld", __FUNCTION__, c->name, rate);
        return -EINVAL;
    }
    
    c->rate = rate;  
    propagate_rate(c);   /*recal the childern*/
    return ret;
}

static unsigned long leopard_devclk_get_rate(struct clk *clk)
{
    return clk->rate;
}

static unsigned long leopard_devclk_recalc(struct clk* c)
{

    if(!c->parent)
    {
        return c->rate;
    }    

    return c->parent->rate;
}

/*
    devclk较为特殊，他作为shareed bus clk的parent存在。
    所以init函数中，必须初始化其shared_bus_list
 */
struct clk_ops leopard_devclk_ops = {
    .init = leopard_devclk_init,
    .enable = leopard_devclk_enable,
    .disable = leopard_devclk_disable,
    .set_rate = leopard_devclk_set_rate, 
    .get_rate = leopard_devclk_get_rate,
    .set_parent = leopard_cpu_clk_set_parent,
    .recalc = leopard_devclk_recalc,
};

static void leopard_periph_clk_init(struct clk *c)
{
}

static int leopard_periph_clk_enable(struct clk *c)
{
    if(c == NULL)
    {
        return -EINVAL;
    }   
    return enable_periph_clk(c->module_id);
}

static int leopard_periph_clk_disable(struct clk *c)
{
    if(c == NULL)
    {
        return -EINVAL;
    }
    
    return disable_periph_clk(c->module_id);
}

static struct clk* leopard_periph_clk_get_parent(struct clk *c)
{
    return c->parent;
}

static int leopard_periph_clk_set_rate(struct clk *c, unsigned long rate)
{
    int ret = 0;
    unsigned int div=0, val=0;
    int is_step_added = 0;
    if(c == NULL)
    {
        return -EINVAL;
    }
    
    if((c->max_rate!=0) && (rate>c->max_rate))
    {
//        printk("\n[zjl] can not set rate:%ld for clk:%s, max_rate:%ld", rate, c->name, c->max_rate);
        return -EINVAL;
    }
    ret = leopard_clk_round_rate_div(c, rate, &div, &val, &is_step_added);
    if(ret != 0)
    {
//        printk("\n[zjl] %s can not set rate:%ld for clk:%s", __FUNCTION__, rate, c->name);
        return -EINVAL;
    }
    _write_div_reg(c, val);
    
    c->is_step = is_step_added;
    if(is_step_added == 1)
    {
        c->rate = (c->parent->rate * CLK_DIV_FACTOR)/(div * CLK_DIV_FACTOR +1);
    }
    else
    {
        c->rate = c->parent->rate/div;
    }
//    printk("\n[zjl] %s, clk:%s, rate:%ld, parent_rate:%ld, div:%d, c->rate:%ld", __FUNCTION__, c->name, rate, c->parent->rate, div,c->rate);
    return 0;
}

static void leopard_periph_clk_reset(struct clk *c)
{
    reset_periph_clk(c->reset_id);   
}


static unsigned long leopard_periph_clk_get_rate(struct clk *c)
{
    unsigned long rate = c->rate;
    if(c && c->ops && c->ops->recalc)
    {
        rate = c->ops->recalc(c);
        c->rate = rate;
    }
    return rate;
}


static unsigned long leopard_ddr_clk_get_rate(struct clk *c)
{
    unsigned long rate = leopard_clk_recalc(c);
    if(c->rate == 0)
    {
        c->rate = rate;
    }
    return rate;
}


static int leopard_periph_clk_is_enabled(struct clk *c)
{
    if(c->state == ON)
    {
        if(is_enabled_periph_clk(c->module_id) != 1)
        {
//            printk("\n[zjl] error at %s %d", __FUNCTION__, __LINE__);
            return 0;
        }
        return 1;
    }
    else
    {
        return 0;
    }
    
}

struct clk_ops leopard_periph_clk_ops = {
    .init = leopard_periph_clk_init,
    .enable = leopard_periph_clk_enable,
    .disable = leopard_periph_clk_disable,
    .get_parent = leopard_periph_clk_get_parent,
    .set_rate = leopard_periph_clk_set_rate,
    .get_rate = leopard_periph_clk_get_rate,
    .is_enabled = leopard_periph_clk_is_enabled,
    .recalc = leopard_clk_recalc,
    .set_parent = leopard_clk_set_parent,
    .reset = leopard_periph_clk_reset,
};

 static void leopard_deepcolorpll_init(struct clk *c)
{
}

 static struct clk* leopard_deepcolorpll_get_parent(struct clk *c)
{
    return c->parent;
}


static  unsigned long leopard_deepcolorpll_recalc(struct clk* c)
{
    unsigned long parent_rate;
    unsigned int offset=0,  tmp=0;
    
    if(!c || !c->parent)
    {
        return -EINVAL;
    }
    parent_rate = c->parent->rate;
    offset = parent_rate/4;
    tmp = act_readl(c->u.pll.enable_reg);
    tmp &= c->u.pll.value_mask;
    tmp >>= (__ffs(c->u.pll.value_mask));
//    printk("\n[zjl] %s clk:%s,parent:%s,parent_rate:%ld, value:%d, rate:%ld", __FUNCTION__, c->name, c->parent->name,parent_rate, tmp, (parent_rate+tmp*offset));
    return (parent_rate+tmp*offset);
}

static unsigned long leopard_deepcolorpll_get_rate(struct clk *c)
{
    unsigned long rate = leopard_deepcolorpll_recalc(c);
    if(c->rate == 0)
    {
        c->rate = rate;
    }
    return rate;
}


static int leopard_deepcolorpll_set_rate(struct clk *c, unsigned long rate)
{
    unsigned long parent_rate;
    unsigned int value = 0, offset=0, i=0, tmp=0;
    if(!c || !c->parent)
    {
        return -EINVAL;
    }
    
    parent_rate = c->parent->rate;
    offset = parent_rate/4;
    for(i=0;i<3;i++)
    {
        value = parent_rate + i*offset;
        if(rate <= value)
        {
            /*found it*/
            break;
        }
    }
    if(i == 3)
    {
//        printk("\n[zjl] %s can not set rate:%ld for %s, parent_rate:%ld", __FUNCTION__, rate, c->name, parent_rate);
        return -EINVAL;
    }
    tmp = act_readl(c->u.pll.enable_reg);
    tmp &= ~c->u.pll.value_mask;
    tmp |= (i<<__ffs(c->u.pll.value_mask));
    act_writel(tmp, c->u.pll.enable_reg);
    act_readl(c->u.pll.enable_reg);
    
    if(c->u.pll.lock_delay != 0)
    {
        udelay(c->u.pll.lock_delay);
    }
    
    c->rate = value;  /*not rate*/
    
    return 0;
}

struct clk_ops leopard_deepcolorpll_ops = {
    .init = leopard_deepcolorpll_init,
    .enable = leopard_pll_enable,
    .disable = leopard_pll_disable,
    .get_parent = leopard_deepcolorpll_get_parent,
    .set_rate = leopard_deepcolorpll_set_rate,
    .get_rate = leopard_deepcolorpll_get_rate,
    .is_enabled = leopard_pll_is_enabled,
    .recalc = leopard_deepcolorpll_recalc,
    .set_parent = leopard_clk_set_parent,
};

/*针对固定频率点的pll操作*/
static int leopard_freq_point_pll_set_rate(struct clk *c, unsigned long rate)
{
    unsigned int i = 0, value=0;
    unsigned long* freqpoints = NULL;
    
    if(!c || !(c->u.pll.freq_point_array))
    {
//        printk("\n[zjl] error at %s , clk:%s", __FUNCTION__, c->name);
        return -EINVAL;
    }
    
    freqpoints = c->u.pll.freq_point_array;
    for(i=0; freqpoints[i]!=0; i++)
    {
        if(rate == freqpoints[i])
        {
            /*found it*/
            break;
        }
        if(rate == ~0x0)
        {
            continue;
        }
    }
    
    if(freqpoints[i]==0)
    {
//        printk("\n[zjl] %s can not set rate:%ld", __FUNCTION__, rate);
        return -EINVAL;
    }
    
    value = act_readl(c->u.pll.enable_reg);
    value &= ~(c->u.pll.value_mask);
    value |= (i<<(__ffs(c->u.pll.value_mask)));
    act_writel(value, c->u.pll.enable_reg);
    act_readl(c->u.pll.enable_reg);
    
    if(c->u.pll.lock_delay != 0)
    {
        udelay(c->u.pll.lock_delay);
    }
    c->rate = freqpoints[i];
//    printk("\n[zjl] %s write 0x%x to reg 0x%x", __FUNCTION__, value, c->u.pll.enable_reg);
    return 0;
}

static unsigned long leopard_freq_point_pll_get_rate(struct clk* c)
{
    unsigned int value = 0;
    unsigned long* freqpoints = NULL;

    if(!c || !(c->u.pll.freq_point_array))
    {
//        printk("\n[zjl] error at %s , clk:%s", __FUNCTION__, c->name);
        return -EINVAL;
    }

    freqpoints = c->u.pll.freq_point_array;
    value = act_readl(c->u.pll.enable_reg);
    value &= (c->u.pll.value_mask);
    value >>= (__ffs(c->u.pll.value_mask));
    
    if(c->rate == 0)
    {
        c->rate = freqpoints[value];
    }
    
    return freqpoints[value];
}

struct clk_ops leopard_freq_point_pll_ops = {
    .enable = leopard_pll_enable,
    .disable = leopard_pll_disable,
    .set_rate = leopard_freq_point_pll_set_rate,
    .get_rate = leopard_freq_point_pll_get_rate,
    .is_enabled = leopard_pll_is_enabled,
    .recalc = leopard_freq_point_pll_get_rate,
};


static int leopard_enable_only_clk_set_parent(struct clk *c, struct clk *parent)
{
    /*对于enable_only来说,无法设置rate，只能在设置parent时，修改rate*/
    int is_step_added = 0;
    int ret = leopard_clk_set_parent(c, parent);
    u32 div = 0;
    if(ret == 0)
    {
        if((c->flag & DIV_FIXED) != 0)
        {
           div = c->div;
        }
        else if(c->rates == NULL)
        {
            div = 1;
        }
        else
        {
            div = _read_divisor(c, &is_step_added);
        }
        if(div == 0)
        {
//            printk("\n[zjl] error at %s %d, clk:%s, parent:%s", __FUNCTION__, __LINE__, c->name, parent->name);
            return -EINVAL; 
        }
        
        if(is_step_added == 1)
        {
            c->rate = (c->parent->rate * CLK_DIV_FACTOR)/(div * CLK_DIV_FACTOR + 1);
        }
        else
        {
            c->rate = c->parent->rate/div;
        }
    }
    return ret;
}


struct clk_ops leopard_enable_only_ops = {
    .enable = leopard_periph_clk_enable,
    .disable = leopard_periph_clk_disable,
    .get_parent = leopard_periph_clk_get_parent,
    .set_rate = NULL,  /*can not set rate*/
    .get_rate = leopard_periph_clk_get_rate,
    .is_enabled = leopard_periph_clk_is_enabled,
    .recalc = leopard_clk_recalc,
    .set_parent = leopard_enable_only_clk_set_parent,
};

static void	leopard_ahb_clk_init(struct clk *clk)
{
    int val=0,div=0;
	struct clksel_rate *clkr;
    
    /*AHB 固定频率设置*/
    for (clkr = clk->rates; clkr->div; clkr++) 
    {
        if(clkr->div == clk->div)
        {
            break;
        }   
    }
    if(clkr->div == 0)
    {
        return;
    }
    _write_div_reg(clk, clkr->val);
    return;
}

struct clk_ops leopard_ahb_clk_ops = {
    .init = leopard_ahb_clk_init,
    .get_rate = leopard_periph_clk_get_rate,
    .recalc = leopard_clk_recalc,
    .set_parent = leopard_enable_only_clk_set_parent,
};

struct clk_ops leopard_ddr_clk_ops = {
        .enable = leopard_periph_clk_enable,
        .disable = leopard_periph_clk_disable,
        .get_parent = leopard_periph_clk_get_parent,
//        .set_rate = leopard_ddr_clk_set_rate,
        .get_rate = leopard_ddr_clk_get_rate,
        .is_enabled = leopard_periph_clk_is_enabled,
        .set_parent = leopard_clk_set_parent,
        .recalc = leopard_clk_recalc,
};

static int leopard_mux_clk_set_parent(struct clk *c, struct clk *parent)
{
    /*对于mux来说,无法设置rate，只能在设置parent时，修改rate*/
    int ret = leopard_clk_set_parent(c, parent);
    if(ret == 0)
    {
        c->rate = parent->rate;
    }
    return ret;
}

static unsigned long leopard_mux_clk_recalc(struct clk *c)
{
    if(!c->parent)
    {
        return c->rate;
    }
    else
    {
        return c->parent->rate;
    }
}

struct clk_ops leopard_mux_clk_ops = {
        .get_parent = leopard_periph_clk_get_parent,
        .set_parent = leopard_mux_clk_set_parent,
        .get_rate = leopard_cpu_clk_get_rate,
        .recalc   = leopard_mux_clk_recalc,
};

struct clk_ops leopard_inner_clk_ops = {
    .set_rate = NULL,  /*can not set rate*/
    .get_rate = leopard_periph_clk_get_rate,
    .get_parent = leopard_periph_clk_get_parent,
    .set_parent = leopard_enable_only_clk_set_parent,
    .recalc = leopard_clk_recalc,
};
