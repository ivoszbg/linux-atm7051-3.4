#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/clk.h>
#include <linux/spinlock.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/list.h>
#include <linux/err.h>
#include <mach/clock.h>
#include <mach/hardware.h>

extern struct clk_ops leopard_pllx_ops;
extern struct clk_ops leopard_usbpll_ops;
extern struct clk_ops leopard_cpuclk_ops;
extern struct clk_ops leopard_clk_shared_bus_ops;
extern struct clk_ops leopard_bus_clk_ops;
extern struct clk_ops leopard_devclk_ops;
extern struct clk_ops leopard_periph_clk_ops;
extern struct clk_ops leopard_freq_point_pll_ops;
extern struct clk_ops leopard_deepcolorpll_ops; 
extern struct clk_ops leopard_ddr_clk_ops ;
extern struct clk_ops leopard_enable_only_ops;
extern struct clk_ops leopard_mux_clk_ops;
extern struct clk_ops leopard_inner_clk_ops;
static struct clk leopard_ddr_clk ;
static struct clk leopard_vce_clk;
extern struct clk_ops leopard_ahb_clk_ops;
extern int leopard_clk_set_parent(struct clk *c, struct clk *parent);
extern int leopard_periph_clk_enable(struct clk *c);
extern int leopard_periph_clk_disable(struct clk *c);


#define MODULE_INVALID_ID 0xFFFFFFFF

/*
    losc description
 */
 
static struct clk leopard_losc =
{
    .name = CLK_NAME_LOSC,
    .module_id = MODULE_INVALID_ID,
    .ops = NULL,
    .rate = CLK_RATE_LOSC,
    .parent = NULL,
};


static struct clk leopard_hosc =
{
    .name = CLK_NAME_HOSC,
    .module_id = MODULE_INVALID_ID,
	.ops = NULL,
	.rate = CLK_RATE_HOSC,
	.parent = NULL,
};

/*
    leopard CMU has 10 plls
 */
 
static struct clk leopard_corepll =
{
    .name = CLK_NAME_COREPLL,
    .module_id = MODULE_INVALID_ID,
	.ops = &leopard_pllx_ops,
	.u.pll = 
	{
	    .enable_reg = CMU_COREPLL,
	    .enable_mask = GPLL_ENABLE_MASK,
	    .value_mask = GPLL_VALUE_MASK,
	    .max_rate = COREPLL_CLK_MAX_RATE,
	    .min_rate = COREPLL_CLK_MIN_RATE,
	    .rate_step = COREPLL_CLK_RATE_STEP,
	    .lock_delay = PLL_LOCK_DELAY,
	},
	.flag = PLL_ALAWAY_ON,
};

static struct clk leopard_devpll = {
    .name = CLK_NAME_DEVPLL,
    .module_id = MODULE_INVALID_ID,
    .ops = &leopard_pllx_ops,
    .u.pll = 
	{
	    .enable_reg = CMU_DEVPLL,
	    .enable_mask = GPLL_ENABLE_MASK,
	    .value_mask = GPLL_VALUE_MASK,
	    .max_rate = DEVPLL_CLK_MAX_RATE,
	    .min_rate = DEVPLL_CLK_MIN_RATE,
	    .rate_step = DEVPLL_CLK_RATE_STEP,
	    .lock_delay = PLL_LOCK_DELAY,
	},
    .flag = PLL_ALAWAY_ON,
};

static struct clk leopard_nandpll = {
    .name = CLK_NAME_NANDPLL,
    .module_id = MODULE_INVALID_ID,
    .ops = &leopard_pllx_ops,
    .u.pll = 
	{
	    .enable_reg = CMU_NANDPLL,
	    .enable_mask = GPLL_ENABLE_MASK,
	    .value_mask = GPLL_VALUE_MASK,
	    .max_rate = NANDPLL_CLK_MAX_RATE,
	    .min_rate = NANDPLL_CLK_MIN_RATE,
	    .rate_step = NANDPLL_CLK_RATE_STEP,
	    .lock_delay = PLL_LOCK_DELAY,
	},    
    .flag = PLL_ALAWAY_ON,
};

static struct clk leopard_usbpll = {
    .name = CLK_NAME_USBPLL,
    .module_id = MODULE_INVALID_ID,
    .ops = &leopard_usbpll_ops,
    .u.pll = 
	{
	    .enable_reg = CMU_USBCLK,
	    .enable_mask = (0x1<<10),
	    .value_mask = 0,
	    .max_rate = 0,
	    .min_rate = 0,
	    .rate_step = 0,
	    .lock_delay = 0,
	},    
};

static struct clk leopard_ddrpll = {
    .name = CLK_NAME_DDRPLL,
    .module_id = MODULE_INVALID_ID,
    .ops = &leopard_pllx_ops,
    .u.pll = 
	{
	    .enable_reg = CMU_DDRPLL,
	    .enable_mask = GPLL_ENABLE_MASK,
	    .value_mask = GPLL_VALUE_MASK,
	    .max_rate = DDRPLL_CLK_MAX_RATE,
	    .min_rate = DDRPLL_CLK_MIN_RATE,
	    .rate_step = DDRPLL_CLK_RATE_STEP,
	    .lock_delay = PLL_LOCK_DELAY,
	},
    .flag = PLL_ALAWAY_ON,
};

static struct clk leopard_displaypll = {
    .name = CLK_NAME_DISPLAYPLL,
    .module_id = MODULE_INVALID_ID,
    .ops = &leopard_pllx_ops,
    .u.pll = 
	{
	    .enable_reg = CMU_DISPLAYPLL,
	    .enable_mask = GPLL_ENABLE_MASK,
	    .value_mask = GPLL_VALUE_MASK,
	    .max_rate = DISPLAYPLL_CLK_MAX_RATE,
	    .min_rate = DISPLAYPLL_CLK_MIN_RATE,
	    .rate_step = DISPLAYPLL_CLK_RATE_STEP,
	    .lock_delay = PLL_LOCK_DELAY,
	},
};

static unsigned long audiopll_rates[] = {45158400, 49152000, 0};
static struct clk leopard_audiopll = {
    .name = CLK_NAME_AUDIOPLL,
    .module_id = MODULE_INVALID_ID,
    .ops = &leopard_freq_point_pll_ops,
    .u.pll = 
	{
	    .enable_reg = CMU_AUDIOPLL,
	    .enable_mask = AUDIOPLL_ENABLE_MASK,
	    .value_mask = AUDIOPLL_VALUE_MASK,
	    .max_rate = 0,
	    .min_rate = 0,
	    .rate_step = 0,
	    .lock_delay = PLL_LOCK_DELAY,
	    .freq_point_array = audiopll_rates,
	},
};

static unsigned long tvout0pll_rates[] = {100800000, 297000000, ~0x0, 296000000, 0};
static struct clk leopard_tvout0pll = {
    .name = CLK_NAME_TVOUT0PLL,
    .module_id = MODULE_INVALID_ID,
    .ops = &leopard_freq_point_pll_ops,
    .u.pll = 
    {
	    .enable_reg = CMU_TVOUTPLL,
	    .enable_mask = TVOUT0PLL_ENABLE_MASK,
	    .value_mask = TVOUT0PLL_VALUE_MASK,
	    .max_rate = 0,
	    .min_rate = 0,
	    .rate_step = 0,
	    .lock_delay = PLL_LOCK_DELAY,
	    .freq_point_array = tvout0pll_rates,
	},
};

static unsigned long tvout1pll_rates[] = {80000000, 130000000, 171000000, 294000000, 216000000, 0};
static struct clk leopard_tvout1pll = {
    .name = CLK_NAME_TVOUT1PLL,
    .module_id = MODULE_INVALID_ID,
    .ops = &leopard_freq_point_pll_ops,
    .u.pll = 
	{
	    .enable_reg = CMU_TVOUTPLL,
	    .enable_mask = TVOUT1PLL_ENABLE_MASK,
	    .value_mask = TVOUT1PLL_VALUE_MASK,
	    .max_rate = 0,
	    .min_rate = 0,
	    .rate_step = 0,
	    .lock_delay = PLL_LOCK_DELAY,
	    .freq_point_array = tvout1pll_rates,
	},
};



static struct clk leopard_deepcolorpll = {
    .name = CLK_NAME_DEEPCOLORPLL,
    .module_id = MODULE_INVALID_ID,
    .ops = &leopard_deepcolorpll_ops,
    .u.pll = 
	{
	    .enable_reg = CMU_TVOUTPLL,
	    .enable_mask = DEEPCOLORPLL_ENABLE_MASK,
	    .value_mask = 0,
	    .max_rate = 0,
	    .min_rate = 0,
	    .rate_step = 0,
	    .lock_delay = PLL_LOCK_DELAY,
	},
};


static struct clk leopard_120Mpll = {
    .name = "120Mpll",
    .module_id = MODULE_INVALID_ID,
    .ops = &leopard_pllx_ops,
    .u.pll = 
	{
	    .enable_reg = CMU_120MPLL,
	    .enable_mask = PLL120M_ENABLE_MASK,
	    .value_mask = 0,
	    .max_rate = 0,
	    .min_rate = 0,
	    .rate_step = 0,
	    .lock_delay = PLL_LOCK_DELAY,
	},
	.rate = PLL120M_RATE,
	.flag = PLL_FIXED,
};

static struct clksel_rate ahb_clk_rates[] = 
{
    { .div = 1, .val = 0, .flags = 0 },
   	{ .div = 2, .val = 1, .flags = 0 },
   	{ .div = 3, .val = 2, .flags = 0 },
   	{ .div = 4, .val = 3, .flags = 0 },
    { .div = 6, .val = 4, .flags = 0 },
    { .div = 0}
}; 

static struct clksel_rate apb_clk_rates[] = 
{
    { .div = 2, .val = 0, .flags = 0 },
   	{ .div = 4, .val = 1, .flags = 0 },
   	{ .div = 6, .val = 2, .flags = 0 },
   	{ .div = 8, .val = 3, .flags = 0 },
    { .div = 12, .val = 4, .flags = 0 },
    { .div = 16, .val = 5, .flags = 0 },
    { .div = 0}
};

static struct clksel_rate leopard_cpuclk_rates[] = 
{
    {.div=0, .val = 1, .flags = 0},
};

static struct clk_mux_sel leopard_cpuclk_sel[] = {
    {.input = &leopard_hosc, .value = 1},
    {.input = &leopard_corepll, .value = 2},
    {.input = &leopard_vce_clk,.value = 3},
};

static struct clk leopard_cpuclk = 
{
    .name = CLK_NAME_CPUCLK,
    .module_id = MODULE_INVALID_ID,
    .ops = &leopard_cpuclk_ops,
    .clksel_reg = CMU_BUSCLK,
    .clksel_mask = CPUCLK_SEL_MASK,
    .clksel = leopard_cpuclk_sel,
    .rates = leopard_cpuclk_rates,
//    .parent = &leopard_corepll,
};

static struct clk_mux_sel leopard_devclk_sel[] = {
    {.input = &leopard_hosc, .value = 0},
    {.input = &leopard_devpll, .value = 1},
};

static struct clk leopard_dev_clk = {
    .name = CLK_NAME_DEVCLK,
    .module_id = MODULE_INVALID_ID,
    .ops = &leopard_devclk_ops,
    .clksel_reg = CMU_DEVPLL,
    .clksel_mask = DEVCLK_SEL_MASK,
    .clksel = leopard_devclk_sel,
    .rates = leopard_cpuclk_rates,
//    .parent = &leopard_devpll,
};

static struct clk_mux_sel leopard_hclk_sel[] = {
    {.input = &leopard_dev_clk, .value = 0},
    {.input = &leopard_ddr_clk, .value = 1},
};

static struct clk leopard_hclk = {
    .name = CLK_NAME_HCLK,
    .module_id = MODULE_INVALID_ID,
    .ops = &leopard_ahb_clk_ops,
    .clksel_reg = CMU_BUSCLK,
    .clksel_mask = AHBCLK_SEL_MASK,
    .div_mask = AHBCLK_DIV_MASK,
    .clksel = leopard_hclk_sel,
    .rates = ahb_clk_rates,
    .max_rate = AHBCLK_MAX_RATE,
    .min_rate = AHBCLK_MIN_RATE,
    .div        =  6,     
    .flag        = DIV_FIXED,		
};


static struct clk leopard_pclk = {
    .name = "pclk",
    .module_id = MODULE_INVALID_ID,
    .ops = &leopard_ahb_clk_ops,
    .clksel_reg = CMU_BUSCLK,
    .clksel_mask = 0,
    .clksel = NULL,
    .div_mask = APBCLK_DIV_MASK,
    .rates = apb_clk_rates,
    .max_rate = APBCLK_MAX_RATE,
    .min_rate = APBCLK_MIN_RATE,
    .div = 2,
    .flag = DIV_FIXED,
//    .parent = &leopard_hclk,
};

static struct clk_mux_sel leopard_nic_dcu_sel[] = {
    {.input = &leopard_dev_clk, .value = 0},
    {.input = &leopard_ddr_clk, .value = 1},

};

static struct clk leopard_nic_dcu_clk = {
    .name = CLK_NAME_NIC_DCU_CLK,
    .module_id = MODULE_INVALID_ID,
    .ops = &leopard_enable_only_ops,
    .clksel_reg = CMU_BUSCLK,
    .clksel_mask = NIC_DCU_SEL_MASK,
    .clksel = leopard_nic_dcu_sel,
    .div_mask = 0,
    .rates = NULL,
//    .parent = &leopard_hclk,
};

static struct clksel_rate lcd0_clk_rates[] = 
{
    { .div = 1, .val = 0, .flags = 0 },
   	{ .div = 2, .val = 1, .flags = 0 },
   	{ .div = 3, .val = 2, .flags = 0 },
   	{ .div = 4, .val = 3, .flags = 0 },
   	{ .div = 5, .val = 4, .flags = 0 },
   	{ .div = 6, .val = 5, .flags = 0 },
   	{ .div = 7, .val = 6, .flags = 0 },
   	{ .div = 8, .val = 7, .flags = 0 },
   	{ .div = 9, .val = 8, .flags = 0 },
   	{ .div = 10, .val = 9, .flags = 0 },
   	{ .div = 11, .val = 10, .flags = 0 },
   	{ .div = 12, .val = 11, .flags = 0 },
    { .div = 0}
};

static struct clk leopard_lcd0_clk = {
    .name      = CLK_NAME_LCD0_CLK,			
    .module_id = MODULE_CLK_LCD0,        
    .reset_id   = MODULE_INVALID_ID,       
    .ops       = &leopard_periph_clk_ops,	
    .clksel_reg = CMU_LCDCLK,			
    .clksel_mask = 0,       
    .div_mask    = LCD0CLK_DIV_MASK,       
    .clksel    = NULL,			
    .rates     = lcd0_clk_rates,         
    .flag     = 0,		
};
//
//static struct clk leopard_lcd1_clk = {
//    .name      = CLK_NAME_LCD1_CLK,			
//    .module_id = MODULE_CLK_LCD1,        
//    .reset_id   = MODULE_INVALID_ID,       
//    .ops       = &leopard_periph_clk_ops,	
//    .clksel_reg = CMU_LCDCLK,			
//    .clksel_mask = 0,       
//    .div_mask    = LCD1CLK_DIV_MASK,       
//    .clksel    = NULL,			
//    .rates     = lcd0_clk_rates,         
//    .flag     = 0,		
//};

static struct clksel_rate vce_clk_rates[] = 
{
    { .div = 1, .val = 0, .flags = 0 },
    { .div = 1, .val = 1, .flags = CLK_DIV_FLAG_ADD_STEP },
   	{ .div = 2, .val = 2, .flags = 0 },
   	{ .div = 2, .val = 3, .flags = CLK_DIV_FLAG_ADD_STEP },
   	{ .div = 3, .val = 4, .flags = 0 },
   	{ .div = 4, .val = 5, .flags = 0 },
   	{ .div = 6, .val = 6, .flags = 0 },
   	{ .div = 8, .val = 7, .flags = 0 },
    { .div = 0}
}; 

static struct clksel_rate vde_clk_rates[] = 
{
    { .div = 1, .val = 0, .flags = 0 },
    { .div = 1, .val = 1, .flags = CLK_DIV_FLAG_ADD_STEP },
   	{ .div = 2, .val = 2, .flags = 0 },
   	{ .div = 2, .val = 3, .flags = CLK_DIV_FLAG_ADD_STEP },
   	{ .div = 3, .val = 4, .flags = 0 },
   	{ .div = 4, .val = 5, .flags = 0 },
   	{ .div = 6, .val = 6, .flags = 0 },
   	{ .div = 8, .val = 7, .flags = 0 },
    { .div = 0}
}; 

//static struct clksel_rate gpu2d_clk_rates[] = 
//{
//    { .div = 1, .val = 0, .flags = 0 },
//   	{ .div = 2, .val = 1, .flags = 0 },
//   	{ .div = 3, .val = 2, .flags = 0 },
//   	{ .div = 4, .val = 3, .flags = 0 },
//    { .div = 0}
//}; 

static struct clksel_rate gpu_clk_rates[] = 
{
    { .div = 1, .val = 0, .flags = 0 },
    { .div = 1, .val = 1, .flags = CLK_DIV_FLAG_ADD_STEP},
   	{ .div = 2, .val = 2, .flags = 0 },
   	{ .div = 2, .val = 3, .flags = CLK_DIV_FLAG_ADD_STEP},
   	{ .div = 3, .val = 4, .flags = 0 },
   	{ .div = 4, .val = 5, .flags = 0 },
   	{ .div = 6, .val = 6, .flags = 0 },
   	{ .div = 8, .val = 7, .flags = 0 },
    { .div = 0}
}; 



static struct clksel_rate sd_clk_rates[] = 
{
    { .div = 1, .val = 0, .flags = 0 },
   	{ .div = 2, .val = 1, .flags = 0 },
   	{ .div = 3, .val = 2, .flags = 0 },
   	{ .div = 4, .val = 3, .flags = 0 },
   	{ .div = 5, .val = 4, .flags = 0 },
   	{ .div = 6, .val = 5, .flags = 0 },
   	{ .div = 7, .val = 6, .flags = 0 },
   	{ .div = 8, .val = 7, .flags = 0 },
   	{ .div = 9, .val = 8, .flags = 0 },
   	{ .div = 10, .val = 9, .flags = 0 },
   	{ .div = 11, .val = 10, .flags = 0 },
   	{ .div = 12, .val = 11, .flags = 0 },
   	{ .div = 13, .val = 12, .flags = 0 },
   	{ .div = 14, .val = 13, .flags = 0 },
   	{ .div = 15, .val = 14, .flags = 0 },
   	{ .div = 16, .val = 15, .flags = 0 },
   	{ .div = 17, .val = 16, .flags = 0 },
   	{ .div = 18, .val = 17, .flags = 0 },
   	{ .div = 19, .val = 18, .flags = 0 },
   	{ .div = 20, .val = 19, .flags = 0 },
   	{ .div = 21, .val = 20, .flags = 0 },
   	{ .div = 22, .val = 21, .flags = 0 },
   	{ .div = 23, .val = 22, .flags = 0 },
   	{ .div = 24, .val = 23, .flags = 0 },
   	{ .div = 25, .val = 24, .flags = 0 },
   	{ .div = 26, .val = 25, .flags = 0 },
   	{ .div = 27, .val = 26, .flags = 0 },
   	{ .div = 28, .val = 27, .flags = 0 },
   	{ .div = 29, .val = 28, .flags = 0 },
   	{ .div = 30, .val = 29, .flags = 0 },
   	{ .div = 31, .val = 30, .flags = 0 },
   	{ .div = 32, .val = 31, .flags = 0 },
    { .div = 0}
}; 

static struct clksel_rate sd_nic_clk_rates[] = 
{
    { .div = 1, .val = 0, .flags = 0 },
   	{ .div = 2, .val = 1, .flags = 0 },
   	{ .div = 4, .val = 2, .flags = 0 },
   	{ .div = 8, .val = 3, .flags = 0 },
   	{ .div = 0}
};

static struct clk_mux_sel leopard_vce_clk_sel[] = {
    {.input = &leopard_dev_clk, .value = 0},
    {.input = &leopard_displaypll, .value = 1},
    {.input = &leopard_nandpll, .value = 2},
    {.input = &leopard_ddrpll, .value = 3},
    {.input = &leopard_120Mpll, .value = 4},
};

static struct clk_mux_sel leopard_vde_clk_sel[] = {
    {.input = &leopard_dev_clk, .value = 0},
    {.input = &leopard_displaypll, .value = 1},
    {.input = &leopard_nandpll, .value = 2},
    {.input = &leopard_ddrpll, .value = 3},
};

//static struct clk_mux_sel leopard_gpu2d_clk_sel[] = {
//    {.input = &leopard_dev_clk, .value = 0},
//    {.input = &leopard_displaypll, .value = 1},
//    {.input = &leopard_nandpll, .value = 2},
//    {.input = &leopard_ddrpll, .value = 3},
//};

//static struct clk_mux_sel leopard_gpu2d_nic_clk_sel[] = {
//    {.input = &leopard_dev_clk, .value = 0},
//    {.input = &leopard_displaypll, .value = 1},
//    {.input = &leopard_nandpll, .value = 2},
//    {.input = &leopard_ddrpll, .value = 3},
//};


//static struct clk_mux_sel leopard_gpu3d_clk_sel[] = {
//    {.input = &leopard_dev_clk, .value = 0},
//    {.input = &leopard_displaypll, .value = 1},
//    {.input = &leopard_nandpll, .value = 2},
//    {.input = &leopard_ddrpll, .value = 3},
//};

static struct clk_mux_sel leopard_gpu_clk_sel[] = {
    {.input = &leopard_dev_clk, .value = 0},
    {.input = &leopard_displaypll, .value = 1},
    {.input = &leopard_nandpll, .value = 2},
    {.input = &leopard_ddrpll, .value = 3},
};

static struct clk_mux_sel leopard_sdclk_sel[] = {
    {.input = &leopard_dev_clk, .value = 0},
    {.input = &leopard_120Mpll, .value = 1},
    {.input = &leopard_nandpll, .value = 2},
};

static struct clk_mux_sel leopard_uartclk_sel[] = {
    {.input = &leopard_hosc, .value = 0},
    {.input = &leopard_devpll, .value = 1},
};

static struct clk_mux_sel leopard_nandcclk_sel[] = {
    {.input = &leopard_nandpll, .value = 0},
    {.input = &leopard_displaypll, .value = 1},
    {.input = &leopard_dev_clk, .value = 2},
    {.input = &leopard_ddrpll, .value = 3},
};

static struct clksel_rate nandc_clk_rates[] = 
{
    { .div = 1, .val = 0, .flags = 0 },
   	{ .div = 2, .val = 1, .flags = 0 },
   	{ .div = 4, .val = 2, .flags = 0 },
   	{ .div = 8, .val = 3, .flags = 0 },
    { .div = 0}
}; 

static struct clk_mux_sel leopard_gps2x_sel[] = {
    {.input = &leopard_dev_clk, .value = 0},
    {.input = &leopard_displaypll, .value = 1},
    {.input = &leopard_nandpll, .value = 2},
    {.input = &leopard_ddrpll, .value = 3},
    {.input = &leopard_corepll, .value = 4},
};

static struct clksel_rate gps2x_clk_rates[] = 
{
    { .div = 1, .val = 0, .flags = 0 },
   	{ .div = 2, .val = 1, .flags = 0 },
   	{ .div = 3, .val = 2, .flags = 0 },
   	{ .div = 4, .val = 3, .flags = 0 },
    { .div = 0}
}; 

static struct clk_mux_sel leopard_ethernet_nic_sel[] = {
    {.input = &leopard_120Mpll, .value = 0},
    {.input = &leopard_dev_clk, .value = 1},
    {.input = &leopard_nandpll, .value = 2},
};

static struct clksel_rate ethernet_nic_clk_rates[] = 
{
    { .div = 1, .val = 0, .flags = 0 },
   	{ .div = 2, .val = 1, .flags = 0 },
   	{ .div = 4, .val = 2, .flags = 0 },
   	{ .div = 8, .val = 3, .flags = 0 },
    { .div = 0}
}; 

static struct clk_mux_sel leopard_lcd_clk_sel[] = {
    {.input = &leopard_displaypll, .value = 0},
    {.input = &leopard_dev_clk, .value = 1},
    {.input = &leopard_tvout0pll, .value = 2},
};

static struct clksel_rate lcd_clk_rates[] = 
{
    { .div = 1, .val = 0, .flags = 0 },
   	{ .div = 7, .val = 1, .flags = 0 },
    { .div = 0}
}; 

static struct clk_mux_sel leopard_csi_clk_sel[] = {
    {.input = &leopard_displaypll, .value = 0},
    {.input = &leopard_dev_clk, .value = 1},
};

static struct clksel_rate csi_clk_rates[] = 
{
    { .div = 1, .val = 0, .flags = 0 },
   	{ .div = 2, .val = 1, .flags = 0 },
   	{ .div = 3, .val = 2, .flags = 0 },
   	{ .div = 4, .val = 3, .flags = 0 },
   	{ .div = 5, .val = 4, .flags = 0 },
   	{ .div = 6, .val = 5, .flags = 0 },
   	{ .div = 7, .val = 6, .flags = 0 },
   	{ .div = 8, .val = 7, .flags = 0 },
   	{ .div = 9, .val = 8, .flags = 0 },
   	{ .div = 10, .val = 9, .flags = 0 },
   	{ .div = 11, .val = 10, .flags = 0 },
   	{ .div = 12, .val = 11, .flags = 0 },
    { .div = 0}
}; 

/* 虚拟的de_clk, 为dex_clk的source*/
static struct clk_mux_sel leopard_de_clk_sel[] = {
    {.input = &leopard_displaypll, .value = 0},
    {.input = &leopard_dev_clk, .value = 1},
};

static struct clksel_rate de_clk_rates[] = 
{
    { .div = 1, .val = 0, .flags = 0 },
   	{ .div = 2, .val = 1, .flags = 0 },
   	{ .div = 3, .val = 2, .flags = 0 },
   	{ .div = 4, .val = 3, .flags = 0 },
   	{ .div = 5, .val = 4, .flags = 0 },
   	{ .div = 6, .val = 5, .flags = 0 },
   	{ .div = 7, .val = 6, .flags = 0 },
   	{ .div = 8, .val = 7, .flags = 0 },
   	{ .div = 9, .val = 8, .flags = 0 },
   	{ .div = 10, .val = 9, .flags = 0 },
   	{ .div = 11, .val = 10, .flags = 0 },
   	{ .div = 12, .val = 11, .flags = 0 },
    { .div = 0}
}; 

//static struct clk_mux_sel leopard_img5_clk_sel[] = {
//    {.input = &leopard_lcd1_clk, .value = 0},
//    {.input = &leopard_lcd0_clk, .value = 1},
//};

//static struct clk_mux_sel leopard_ispbp_sel[] = {
//    {.input = &leopard_displaypll, .value = 0},
//    {.input = &leopard_dev_clk, .value = 1},
//};
//
//static struct clksel_rate bisp_clk_rates[] = 
//{
//    { .div = 1, .val = 0, .flags = 0 },
//   	{ .div = 2, .val = 1, .flags = 0 },
//   	{ .div = 3, .val = 2, .flags = 0 },
//   	{ .div = 4, .val = 3, .flags = 0 },
//   	{ .div = 5, .val = 4, .flags = 0 },
//   	{ .div = 6, .val = 5, .flags = 0 },
//   	{ .div = 7, .val = 6, .flags = 0 },
//   	{ .div = 8, .val = 7, .flags = 0 },
//   	{ .div = 9, .val = 8, .flags = 0 },
//   	{ .div = 10, .val = 9, .flags = 0 },
//   	{ .div = 11, .val = 10, .flags = 0 },
//   	{ .div = 12, .val = 11, .flags = 0 },
//    { .div = 0}
//}; 

static struct clk_mux_sel leopard_si_sel[] = {
    {.input = &leopard_displaypll, .value = 0},
    {.input = &leopard_dev_clk, .value = 1},
};

static struct clksel_rate si_clk_rates[] = 
{
    { .div = 1, .val = 0, .flags = 0 },
   	{ .div = 2, .val = 1, .flags = 0 },
   	{ .div = 3, .val = 2, .flags = 0 },
   	{ .div = 4, .val = 3, .flags = 0 },
   	{ .div = 5, .val = 4, .flags = 0 },
   	{ .div = 6, .val = 5, .flags = 0 },
   	{ .div = 7, .val = 6, .flags = 0 },
   	{ .div = 8, .val = 7, .flags = 0 },
   	{ .div = 9, .val = 8, .flags = 0 },
   	{ .div = 10, .val = 9, .flags = 0 },
   	{ .div = 11, .val = 10, .flags = 0 },
   	{ .div = 12, .val = 11, .flags = 0 },
    { .div = 0}
}; 

static struct clk leopard_si_clk = {
    .name      = CLK_NAME_SI_CLK,			
    .module_id = MODULE_CLK_SI,        
    .reset_id   = MODULE_RESET_SI,       
    .ops       = &leopard_periph_clk_ops,	
    .clksel_reg = CMU_SICLK,			
    .clksel_mask = SICLK_SEL_MASK,       
    .div_mask    = SICLK_DIV_MASK,       
    .clksel    = leopard_si_sel,			
    .rates     = si_clk_rates,         
    .flag     = 0,		
};


static struct clksel_rate sensor_clk_rates[] = 
{
    { .div = 1, .val = 0, .flags = 0 },
   	{ .div = 2, .val = 1, .flags = 0 },
   	{ .div = 3, .val = 2, .flags = 0 },
   	{ .div = 4, .val = 3, .flags = 0 },
   	{ .div = 5, .val = 4, .flags = 0 },
   	{ .div = 6, .val = 5, .flags = 0 },
   	{ .div = 7, .val = 6, .flags = 0 },
   	{ .div = 8, .val = 7, .flags = 0 },
   	{ .div = 9, .val = 8, .flags = 0 },
   	{ .div = 10, .val = 9, .flags = 0 },
   	{ .div = 11, .val = 10, .flags = 0 },
   	{ .div = 12, .val = 11, .flags = 0 },
    { .div = 0}
}; 

static struct clk_mux_sel leopard_sensor_clkout_sel[] = {
    {.input = &leopard_hosc, .value = 0},
    {.input = &leopard_si_clk, .value = 1},
};

static struct clk_mux_sel leopard_tvout_sel[] = {
    {.input = &leopard_tvout0pll, .value = 0},
    {.input = &leopard_tvout1pll, .value = 1},
};
static struct clksel_rate tvout_clk_rates[] = 
{
    { .div = 1, .val = 0, .flags = 0 },
   	{ .div = 2, .val = 1, .flags = 0 },
   	{ .div = 4, .val = 2, .flags = 0 },
   	{ .div = 8, .val = 3, .flags = 0 },
    { .div = 0}
}; 

static struct clk leopard_tvout_mux_clk = {
    .name      = CLK_NAME_TVOUTMUX0_CLK,			
    .module_id = MODULE_INVALID_ID,        
    .reset_id   = MODULE_INVALID_ID,       
    .ops       = &leopard_mux_clk_ops,	
    .clksel_reg = CMU_TVOUTPLL,			
    .clksel_mask = TVOUTMUXCLK_SEL_MASK,       
    .div_mask    = 0,       
    .clksel    = leopard_tvout_sel,			
    .rates     = NULL,         
    .flag     = 0,		
};

static struct clk leopard_tvout0_clk = {
    .name      = CLK_NAME_TVOUT0_CLK,			
    .module_id = MODULE_INVALID_ID,        
    .reset_id   = MODULE_INVALID_ID,       
    .ops       = &leopard_periph_clk_ops,	
    .clksel_reg = CMU_TVOUTPLL,			
    .clksel_mask = 0,       
    .div_mask    = TVOUT0CLK_DIV_MASK,       
    .clksel    = NULL,			
    .rates     = tvout_clk_rates,         
    .flag     = 0,		
};

static struct clksel_rate audio_clk_rates[] = 
{
    { .div = 1, .val = 0, .flags = 0 },
   	{ .div = 2, .val = 1, .flags = 0 },
   	{ .div = 3, .val = 2, .flags = 0 },
   	{ .div = 4, .val = 3, .flags = 0 },
   	{ .div = 6, .val = 4, .flags = 0 },
   	{ .div = 8, .val = 5, .flags = 0 },
   	{ .div = 12, .val = 6, .flags = 0 },
   	{ .div = 16, .val = 7, .flags = 0 },
   	{ .div = 24, .val = 8, .flags = 0 },
    { .div = 0}
}; 

static struct clk leopard_i2stx_clk = {
    .name      = CLK_NAME_I2STX_CLK,			
    .module_id = MODULE_CLK_I2STX,        
    .reset_id   = MODULE_INVALID_ID,       
    .ops       = &leopard_periph_clk_ops,	
    .clksel_reg = CMU_AUDIOPLL,			
    .clksel_mask = 0,       
    .div_mask    = I2STXCLK_DIV_MASK,       
    .clksel    = NULL,			
    .rates     = audio_clk_rates,         
    .flag     = 0,		
};

static struct clk leopard_i2srx_clk = {
    .name      = CLK_NAME_I2SRX_CLK,			
    .module_id = MODULE_CLK_I2SRX,        
    .reset_id   = MODULE_INVALID_ID,       
    .ops       = &leopard_periph_clk_ops,	
    .clksel_reg = CMU_AUDIOPLL,			
    .clksel_mask = 0,       
    .div_mask    = I2SRXCLK_DIV_MASK,       
    .clksel    = NULL,			
    .rates     = audio_clk_rates,         
    .flag     = 0,		
};

static struct clk leopard_hdmia_clk = {
    .name      = CLK_NAME_HDMI_CLK,			
    .module_id = MODULE_CLK_HDMIA,        
    .reset_id   = MODULE_RESET_HDMI,       
    .ops       = &leopard_periph_clk_ops,	
    .clksel_reg = CMU_AUDIOPLL,			
    .clksel_mask = 0,       
    .div_mask    = HDMICLK_DIV_MASK,       
    .clksel    = NULL,			
    .rates     = audio_clk_rates,         
    .flag     = 0,		
};

static struct clk leopard_spdif_clk = {
    .name      = "spdif_clk",			
    .module_id = MODULE_CLK_SPDIF,        
    .reset_id   = MODULE_INVALID_ID,       
    .ops       = &leopard_periph_clk_ops,	
    .clksel_reg = CMU_AUDIOPLL,			
    .clksel_mask = 0,       
    .div_mask    = SPDIFCLK_DIV_MASK,       
    .clksel    = NULL,			
    .rates     = audio_clk_rates,         
    .flag     = 0,		
};

static struct clksel_rate lens_clk_rates[] = 
{
    { .div = 2, .val = 0, .flags = 0 },
   	{ .div = 4, .val = 1, .flags = 0 },
   	{ .div = 8, .val = 2, .flags = 0 },
   	{ .div = 16, .val = 3, .flags = 0 },
   	{ .div = 32, .val = 4, .flags = 0 },
   	{ .div = 64, .val = 5, .flags = 0 },
   	{ .div = 128, .val = 6, .flags = 0 },
   	{ .div = 256, .val = 7, .flags = 0 },   	
    { .div = 0}
}; 

static struct clksel_rate tls_clk_rates[] = 
{
    { .div = 1, .val = 0, .flags = 0 },
   	{ .div = 2, .val = 1, .flags = 0 },
   	{ .div = 4, .val = 2, .flags = 0 },
   	{ .div = 8, .val = 3, .flags = 0 },
   	{ .div = 16, .val = 4, .flags = 0 },
   	{ .div = 32, .val = 5, .flags = 0 },
   	{ .div = 64, .val = 6, .flags = 0 },
   	{ .div = 128, .val = 7, .flags = 0 },  
   	{ .div = 256, .val = 8, .flags = 0 }, 
   	{ .div = 512, .val = 9, .flags = 0 }, 
   	{ .div = 1024, .val = 10, .flags = 0 },  	
    { .div = 0}
}; 

static struct clk leopard_thermal_clk = {
    .name      = CLK_NAME_THERMAL_CLK,			
    .module_id = MODULE_CLK_THERMAL,        
    .reset_id   = MODULE_INVALID_ID,       
    .ops       = &leopard_periph_clk_ops,	
    .clksel_reg = CMU_TLSCLK,			
    .clksel_mask = 0,       
    .div_mask    = THERMALCLK_DIV_MASK,       
    .clksel    = NULL,			
    .rates     = tls_clk_rates,         
    .flag     = 0,		
};

static struct clk leopard_leakage_clk = {
    .name      = CLK_NAME_LEAKAGE_CLK,			
    .module_id = MODULE_CLK_LEAKAGE,        
    .reset_id   = MODULE_INVALID_ID,       
    .ops       = &leopard_periph_clk_ops,	
    .clksel_reg = CMU_TLSCLK,			
    .clksel_mask = 0,       
    .div_mask    = LEAKAGECLK_DIV_MASK,       
    .clksel    = NULL,			
    .rates     = tls_clk_rates,         
    .flag     = 0,		
};

static struct clk leopard_speed_clk = {
    .name      = CLK_NAME_SPEED_CLK,			
    .module_id = MODULE_CLK_SPEED,        
    .reset_id   = MODULE_INVALID_ID,       
    .ops       = &leopard_periph_clk_ops,	
    .clksel_reg = CMU_TLSCLK,			
    .clksel_mask = 0,       
    .div_mask    = SPEEDCLK_DIV_MASK,       
    .clksel    = NULL,			
    .rates     = tls_clk_rates,         
    .flag     = 0,		
};



static struct clk_mux_sel leopard_ddr_sel[] = {
    {.input = &leopard_ddrpll, .value = 0},
    {.input = &leopard_tvout0_clk, .value = 1},
};

static struct clk leopard_ddr_clk = {
    .name      = CLK_NAME_DDR_CLK,			
    .module_id = MODULE_CLK_DDRC,        
    .reset_id  = MODULE_RESET_DDR,       
    .ops       = &leopard_ddr_clk_ops,	
    .clksel_reg = CMU_DDRPLL,			
    .clksel_mask = DDRCLK_SEL_MASK,       
    .div_mask    = 0,       
    .clksel    = leopard_ddr_sel,			
    .rates     = NULL,         
    .div       = 2,     
    .flag     = DIV_FIXED,		
};

static struct clk_mux_sel leopard_pwm_sel[] = {
    {.input = &leopard_losc, .value = 0},
    {.input = &leopard_hosc, .value = 1},
};


static struct clk leopard_pwm0_clk = {
    .name        = CLK_NAME_PWM0_CLK,			
    .module_id   = MODULE_CLK_PWM0,        
    .reset_id    = MODULE_INVALID_ID,       
    .ops         = &leopard_periph_clk_ops,	
    .clksel_reg  = CMU_PWM0CLK,			
    .clksel_mask = PWMCLK_SEL_MASK,       
    .div_mask    = PWMCLK_DIV_MASK,       
    .clksel      = leopard_pwm_sel,			
    .rates       = NULL,         
    .flag        = DIV_ROUND_1_64,		
};

static struct clk leopard_pwm1_clk = {
    .name        = CLK_NAME_PWM1_CLK,			
    .module_id   = MODULE_CLK_PWM1,        
    .reset_id    = MODULE_INVALID_ID,       
    .ops         = &leopard_periph_clk_ops,	
    .clksel_reg  = CMU_PWM1CLK,			
    .clksel_mask = PWMCLK_SEL_MASK,       
    .div_mask    = PWMCLK_DIV_MASK,       
    .clksel      = leopard_pwm_sel,			
    .rates       = NULL,         
    .flag        = DIV_ROUND_1_64,		
};


static struct clk leopard_pwm2_clk = {
    .name        = "pwm2_clk",			
    .module_id   = MODULE_CLK_PWM2,        
    .reset_id    = 0,       
    .ops         = &leopard_periph_clk_ops,	
    .clksel_reg  = CMU_PWM2CLK,			
    .clksel_mask = PWMCLK_SEL_MASK,       
    .div_mask    = PWMCLK_DIV_MASK,       
    .clksel      = leopard_pwm_sel,			
    .rates       = NULL,         
    .flag        = DIV_ROUND_1_64,		
};


static struct clk leopard_pwm3_clk = {
    .name        = "pwm3_clk",			
    .module_id   = MODULE_CLK_PWM3,        
    .reset_id    = MODULE_INVALID_ID,       
    .ops         = &leopard_periph_clk_ops,	
    .clksel_reg  = CMU_PWM3CLK,			
    .clksel_mask = PWMCLK_SEL_MASK,       
    .div_mask    = PWMCLK_DIV_MASK,       
    .clksel      = leopard_pwm_sel,			
    .rates       = NULL,         
    .flag        = DIV_ROUND_1_64,		
};

//static struct clk leopard_img5_clk = {
//    .name        = CLK_NAME_IMG5_CLK,			
//    .module_id   = MODULE_INVALID_ID,        
//    .reset_id    = MODULE_INVALID_ID,       
//    .ops         = &leopard_enable_only_ops,	
//    .clksel_reg  = CMU_DECLK,			
//    .clksel_mask = IMG5CLK_SEL_MASK,       
//    .div_mask    = IMG5CLK_DIV_MASK,       
//    .clksel      = leopard_img5_clk_sel,			
//    .rates       = NULL,    
//    .div        =  1,     
//    .flag        = DIV_FIXED,		
//};

static struct clksel_rate dma_clk_rates[] = 
{
    { .div = 2, .val = 0, .flags = 0 },
   	{ .div = 4, .val = 1, .flags = 0 },
   	{ .div = 8, .val = 2, .flags = 0 },
    { .div = 0}
}; 

static struct clk leopard_dma_clk = {
    .name        = CLK_NAME_DMA_CLK,			
    .module_id   = MODULE_CLK_DMAC,        
    .reset_id    = MODULE_RESET_DMAC,       
    .ops         = &leopard_periph_clk_ops,	
    .clksel_reg  = CMU_DMACLK,			
    .clksel_mask = 0,       
    .div_mask    = DMACLK_DIV_MASK,       
    .clksel      = NULL,			
    .rates       = dma_clk_rates,         
    .flag        = 0,		
    .max_rate    = DMACLK_MAX_RATE,
};

static struct clk leopard_twd_clk = {
    .name        = CLK_NAME_TWD_CLK,			
    .module_id   = MODULE_INVALID_ID,        
    .reset_id    = MODULE_INVALID_ID,       
    .ops         = &leopard_inner_clk_ops,	
    .clksel_reg  = 0,			
    .clksel_mask = 0,       
    .div_mask    = 0,       
    .clksel      = NULL,			
    .rates       = 0,         
    .div        =  2,     
    .flag        = DIV_FIXED,		    
};

static struct clk leopard_l2_clk = {
    .name        = CLK_NAME_L2_CLK,			
    .module_id   = MODULE_INVALID_ID,        
    .reset_id    = MODULE_INVALID_ID,       
    .ops         = &leopard_inner_clk_ops,	
    .clksel_reg  = 0,			
    .clksel_mask = 0,       
    .div_mask    = 0,       
    .clksel      = NULL,			
    .rates       = 0,         
    .div        =  2,     
    .flag        = DIV_FIXED,		    
};

/*只有enable/disable 需要关注的clk*/
static struct clk leopard_hdmi24M_clk = {
    .name      = CLK_NAME_HDMI24M_CLK,			
    .module_id = MODULE_CLK_HDMI,        
    .reset_id   = MODULE_INVALID_ID,       
    .ops       = &leopard_enable_only_ops,	
    .clksel_reg = 0,			
    .clksel_mask = 0,       
    .div_mask    = 0,       
    .clksel    = NULL,			
    .rates     = NULL,         
    .div       = 1,
    .flag     = DIV_FIXED,		
};

static struct clk leopard_timer_clk = {
    .name      = CLK_NAME_TIMER_CLK,			
    .module_id = MODULE_CLK_TIMER,        
    .reset_id   = MODULE_INVALID_ID,       
    .ops       = &leopard_enable_only_ops,	
    .clksel_reg = 0,			
    .clksel_mask = 0,       
    .div_mask    = 0,       
    .clksel    = NULL,			
    .rates     = NULL,    
    .div       =1,     
    .flag     = DIV_FIXED,		
};

static struct clk leopard_irc_clk = {
    .name      = CLK_NAME_IRC_CLK,			
    .module_id = MODULE_CLK_IRC,        
    .reset_id   = MODULE_INVALID_ID,       
    .ops       = &leopard_enable_only_ops,	
    .clksel_reg = 0,			
    .clksel_mask = 0,       
    .div_mask    = 0,       
    .clksel    = NULL,			
    .rates     = NULL,      
    .div       = 120,   
    .flag     = DIV_FIXED,		
};

static struct clk leopard_i2c0_clk = {
    .name      = CLK_NAME_I2C0_CLK,			
    .module_id = MODULE_CLK_I2C0,        
    .reset_id   = MODULE_RESET_I2C0,       
    .ops       = &leopard_enable_only_ops,	
    .clksel_reg = 0,			
    .clksel_mask = 0,       
    .div_mask    = 0,       
    .clksel    = NULL,			
    .rates     = NULL,         
    .div       =1,     
    .flag     = DIV_FIXED,			
};


static struct clk leopard_i2c1_clk = {
    .name      = CLK_NAME_I2C1_CLK,			
    .module_id = MODULE_CLK_I2C1,        
    .reset_id   = MODULE_RESET_I2C1,       
    .ops       = &leopard_enable_only_ops,		
    .clksel_reg = 0,			
    .clksel_mask = 0,       
    .div_mask    = 0,       
    .clksel    = NULL,			
    .rates     = NULL,         
    .div       =1,     
    .flag     = DIV_FIXED,		
};


static struct clk leopard_i2c2_clk = {
    .name      = CLK_NAME_I2C2_CLK,			
    .module_id = MODULE_CLK_I2C2,        
    .reset_id   = MODULE_RESET_I2C2,       
    .ops       = &leopard_enable_only_ops,	
    .clksel_reg = 0,			
    .clksel_mask = 0,       
    .div_mask    = 0,       
    .clksel    = NULL,			
    .rates     = NULL,         
    .flag     = 0,	
    .div       =1,     
    .flag     = DIV_FIXED,			
};


//static struct clk leopard_tv24M_clk = {
//    .name      = CLK_NAME_TV24M_CLK,			
//    .module_id = MODULE_CLK_TV24M,        
//    .reset_id   = MODULE_INVALID_ID,       
//    .ops       = &leopard_enable_only_ops,	
//    .clksel_reg = 0,			
//    .clksel_mask = 0,       
//    .div_mask    = 0,       
//    .clksel    = NULL,			
//    .rates     = NULL,         
//    .div       =1,     
//    .flag     = DIV_FIXED,			
//};


static struct clk leopard_pcm0_clk = {
    .name      = CLK_NAME_PCM0_CLK,			
    .module_id = MODULE_CLK_PCM0,        
    .reset_id   = MODULE_RESET_PCM0,       
    .ops       = &leopard_enable_only_ops,	
    .clksel_reg = 0,			
    .clksel_mask = 0,       
    .div_mask    = 0,       
    .clksel    = NULL,			
    .rates     = NULL,         
    .div       = 2,     
    .flag     = DIV_FIXED,		
};


static struct clk leopard_pcm1_clk = {
    .name      = CLK_NAME_PCM1_CLK,			
    .module_id = MODULE_CLK_PCM1,        
    .reset_id   = MODULE_RESET_PCM1,       
    .ops       = &leopard_enable_only_ops,		
    .clksel_reg = 0,			
    .clksel_mask = 0,       
    .div_mask    = 0,       
    .clksel    = NULL,			
    .rates     = NULL,         
    .div       = 2,     
    .flag     = DIV_FIXED,		
};

//static struct clk leopard_cvbs_clk = {
//    .name      = CLK_NAME_CVBS_CLK,			
//    .module_id = MODULE_CLK_CVBS,        
//    .reset_id   = MODULE_INVALID_ID,       
//    .ops       = &leopard_enable_only_ops,		
//    .clksel_reg = 0,			
//    .clksel_mask = 0,       
//    .div_mask    = 0,       
//    .clksel    = NULL,			
//    .rates     = NULL,         
//    .div       = 16,     
//    .flag     = DIV_FIXED,		
//};


//static struct clk leopard_lvds_clk = {
//    .name      = CLK_NAME_LVDS_CLK,			
//    .module_id = MODULE_CLK_LVDS,        
//    .reset_id   = MODULE_INVALID_ID,       
//    .ops       = &leopard_enable_only_ops,			
//    .clksel_reg = 0,			
//    .clksel_mask = 0,       
//    .div_mask    = 0,       
//    .clksel    = NULL,			
//    .rates     = NULL,         
//    .flag      = 0,		
//    .div       =  1,
//    .flag    = DIV_FIXED,
//};


static struct clk leopard_vce_clk = {						
		.name      = CLK_NAME_VCE_CLK,		
		.module_id = MODULE_CLK_VCE,        
		.reset_id   = MODULE_RESET_VCE,      
		.ops       = &leopard_periph_clk_ops,	
		.clksel_reg = CMU_VCECLK,			
		.clksel_mask = (0x7<<4),       
		.div_mask    = VCECLK_DIV_MASK,       
		.clksel    = leopard_vce_clk_sel,			
		.rates     = vce_clk_rates,         
//		.parent = &leopard_dev_clk,			
	
};

/* add spi ctrl. only can enable & disable */
static struct clk leopard_spi0_clk = {
    .name      = CLK_NAME_SPI0_CLK,			
    .module_id = MODULE_CLK_SPI0,        
    .reset_id   = MODULE_RESET_SPI0,       
    .ops       = &leopard_enable_only_ops,		
    .clksel_reg = 0,			
    .clksel_mask = 0,       
    .div_mask    = 0,       
    .clksel    = NULL,			
    .rates     = NULL,         
    .div       = 1,     
    .flag     = DIV_FIXED,		
};

static struct clk leopard_spi1_clk = {
    .name      = CLK_NAME_SPI1_CLK,			
    .module_id = MODULE_CLK_SPI1,        
    .reset_id   = MODULE_RESET_SPI1,       
    .ops       = &leopard_enable_only_ops,		
    .clksel_reg = 0,			
    .clksel_mask = 0,       
    .div_mask    = 0,       
    .clksel    = NULL,			
    .rates     = NULL,         
    .div       = 1,     
    .flag     = DIV_FIXED,		
};

static struct clk leopard_spi2_clk = {
    .name      = CLK_NAME_SPI2_CLK,			
    .module_id = MODULE_CLK_SPI2,        
    .reset_id   = MODULE_RESET_SPI2,       
    .ops       = &leopard_enable_only_ops,		
    .clksel_reg = 0,			
    .clksel_mask = 0,       
    .div_mask    = 0,       
    .clksel    = NULL,			
    .rates     = NULL,         
    .div       = 1,     
    .flag     = DIV_FIXED,		
};

static struct clk leopard_spi3_clk = {
    .name      = CLK_NAME_SPI3_CLK,			
    .module_id = MODULE_CLK_SPI3,        
    .reset_id   = MODULE_RESET_SPI3,       
    .ops       = &leopard_enable_only_ops,		
    .clksel_reg = 0,			
    .clksel_mask = 0,       
    .div_mask    = 0,       
    .clksel    = NULL,			
    .rates     = NULL,         
    .div       = 1,     
    .flag     = DIV_FIXED,		
};



#define PERIPH_CLK(_name, _mod_id, _reset_id,_clk_sel_reg, _clk_sel_mask, _clksel, _clk_div_mask,_clkrates, _flags) \
	{						\
		.name      = _name,			\
        .module_id = _mod_id,        \
        .reset_id   = _reset_id,       \
		.ops       = &leopard_periph_clk_ops,	\
		.clksel_reg = _clk_sel_reg,			\
		.clksel_mask = _clk_sel_mask,       \
		.div_mask    = _clk_div_mask,       \
		.clksel    = _clksel,			\
		.rates     = _clkrates,         \
		.flag     = _flags,			\
	}

#define MUX_CLK(_name, _mod_id, _reset_id,_clk_sel_reg, _clk_sel_mask, _clksel, _clk_div_mask,_clkrates, _flags) \
	{						\
		.name      = _name,			\
        .module_id = _mod_id,        \
        .reset_id   = _reset_id,       \
		.ops       = &leopard_mux_clk_ops,	\
		.clksel_reg = _clk_sel_reg,			\
		.clksel_mask = _clk_sel_mask,       \
		.div_mask    = _clk_div_mask,       \
		.clksel    = _clksel,			\
		.rates     = _clkrates,         \
		.flag     = _flags,			\
	}

#define SHARED_CLK(_name, _mod_id, _reset_id, _clk_sel_reg, _clk_sel_mask, _clksel, _clk_div_mask, _clkrates, prior, _flags) \
	{						\
		.name      = _name,			\
		.module_id = _mod_id,        \
		.reset_id   = _reset_id,      \
		.ops       = &leopard_clk_shared_bus_ops,	\
		.clksel_reg = _clk_sel_reg,			\
		.clksel_mask = _clk_sel_mask,       \
		.div_mask    = _clk_div_mask,       \
		.clksel    = _clksel,			\
		.rates     = _clkrates,         \
		.u.shared_bus_user =        \
		{                           \
		    .priority = prior,       \            
		},                           \  
		.flag     = _flags,          \    
	}
	
struct clk* leopard_ptr_clks[] = {
    &leopard_losc,
    &leopard_hosc,
    /*pll*/
    &leopard_corepll,
    &leopard_devpll,
    &leopard_nandpll,
    &leopard_ddrpll,
    &leopard_displaypll,
    &leopard_audiopll,
    &leopard_tvout0pll,
    &leopard_tvout1pll,
    &leopard_deepcolorpll,
    &leopard_120Mpll,
    &leopard_usbpll,
    /*cpu & AMBA bus*/
    &leopard_cpuclk,
    &leopard_dev_clk,
    &leopard_hclk,
    &leopard_pclk,
    &leopard_twd_clk,
    /*else*/
    &leopard_vce_clk,
    &leopard_lcd0_clk,
    
    &leopard_tvout_mux_clk,
    &leopard_tvout0_clk,
    &leopard_i2stx_clk,
    &leopard_i2srx_clk,
    &leopard_hdmia_clk,
    &leopard_spdif_clk,
    /*新增tls clk*/
    &leopard_thermal_clk,
    &leopard_leakage_clk,
    &leopard_speed_clk,
    &leopard_ddr_clk,
    &leopard_pwm0_clk,
    &leopard_pwm1_clk,
    &leopard_pwm2_clk,
    &leopard_pwm3_clk,
    &leopard_dma_clk,
    &leopard_hdmi24M_clk,
    &leopard_timer_clk,
    &leopard_irc_clk,
    &leopard_i2c0_clk,
    &leopard_i2c1_clk,
    &leopard_i2c2_clk,
 //   &leopard_tv24M_clk,
    &leopard_pcm0_clk,
    &leopard_pcm1_clk,
//    &leopard_lvds_clk,
    &leopard_nic_dcu_clk,
//    &leopard_img5_clk,
 //   &leopard_cvbs_clk,
	&leopard_l2_clk,
	&leopard_si_clk,

	/*spix*/
	&leopard_spi0_clk,
	&leopard_spi1_clk,
	&leopard_spi2_clk,
	&leopard_spi3_clk,
};

/*
  VCE/VDE/GPU2D/GPU3D虽然有不同的source，但是在系统运行中基本固定为dev_clk.
  除非出现了bug，此时可以直接修改&parent即可，所以不为上述clk设置clk_sel.
*/
struct clk leopard_list_clks[] = {
    /*share bus*/
//    SHARED_CLK(CLK_NAME_VDE_CLK, MODULE_CLK_VDE, MODULE_RESET_VDE,&leopard_dev_clk, CMU_VDECLK, VDECLK_SEL_MASK, NULL, VDECLK_DIV_MASK, vde_clk_rates),
//    SHARED_CLK(CLK_NAME_GPU2D_CLK, MODULE_CLK_GPU2D, MODULE_RESET_GPU2D,&leopard_dev_clk, CMU_GPU2DCLK, GPU2DCLK_SEL_MASK, NULL, GPU2DCLK_DIV_MASK, gpu2d_clk_rates),
//    SHARED_CLK(CLK_NAME_GPU2D_NIC_CLK, MODULE_CLK_GPU2DLP, MODULE_INVALID_ID,&leopard_dev_clk,CMU_GPU2DCLK, GPU2D_NIC_CLK_SEL_MASK, NULL, GPU2D_NIC_CLK_DIV_MASK, gpu2d_clk_rates),
 
#if 0 
    SHARED_CLK(CLK_NAME_GPU_CLK, MODULE_CLK_GPU, MODULE_RESET_GPU, CMU_GPUCLK, GPU_CORECLK_SEL_MASK, leopard_gpu_clk_sel, GPU_CORECLK_DIV_MASK, gpu_clk_rates, 1, TYPE_GPU),
    SHARED_CLK(CLK_NAME_VDE_CLK,   MODULE_CLK_VDE,   MODULE_RESET_VDE,   CMU_VDECLK,   VDECLK_SEL_MASK,   leopard_vde_clk_sel,   VDECLK_DIV_MASK,   vde_clk_rates,   0, 0),
#else

    /*去掉 shared clk 机制，全部按照PERIPH_CLK来处理*/
    PERIPH_CLK(CLK_NAME_GPU_CLK, MODULE_CLK_GPU, MODULE_RESET_GPU, CMU_GPUCLK, GPU_CORECLK_SEL_MASK, leopard_gpu_clk_sel, GPU_CORECLK_DIV_MASK, gpu_clk_rates, TYPE_GPU),
    PERIPH_CLK(CLK_NAME_VDE_CLK,   MODULE_CLK_VDE,   MODULE_RESET_VDE,   CMU_VDECLK,   VDECLK_SEL_MASK,   leopard_vde_clk_sel,   VDECLK_DIV_MASK,   vde_clk_rates,  0),
#endif
 //   SHARED_CLK(CLK_NAME_GPU3D_NIC_CLK, MODULE_CLK_GPU3DLP, MODULE_INVALID_ID,&leopard_dev_clk, CMU_GPU3DCLK, GPU3D_NIC_CLK_SEL_MASK,NULL, GPU3D_NIC_CLK_DIV_MASK, gpu2d_clk_rates),
    
    /*periph*/
//    PERIPH_CLK(CLK_NAME_VDE_CLK, MODULE_CLK_VDE, MODULE_RESET_VDE,CMU_VDECLK, VDECLK_SEL_MASK, leopard_vde_clk_sel, VDECLK_DIV_MASK, vde_clk_rates, 0),
    PERIPH_CLK(CLK_NAME_NANDC_CLK, MODULE_CLK_NANDC, MODULE_RESET_NANDC, CMU_NANDCCLK, (0x3<<4), leopard_nandcclk_sel, 0x3, nandc_clk_rates, 0),
    PERIPH_CLK(CLK_NAME_NANDC_NIC_CLK, MODULE_CLK_NANDC, MODULE_INVALID_ID, CMU_NANDCCLK, (0x3<<4), leopard_nandcclk_sel, (0x3<<8), nandc_clk_rates, 0),

     /*
      注意sd0/sd1/sd2是指在sdx_clk 的div128之前的clk，真正的clk还需两次分频才能得到，
      需要驱动进行管理！！
     */
    PERIPH_CLK(CLK_NAME_SD0_CLK, MODULE_CLK_SD0, MODULE_RESET_SD0, CMU_SD0CLK, SDCLK_SEL_MASK, leopard_sdclk_sel, SDCLK_DIV_MASK, sd_clk_rates, 0),
    PERIPH_CLK(CLK_NAME_SD1_CLK, MODULE_CLK_SD1, MODULE_RESET_SD1, CMU_SD1CLK, SDCLK_SEL_MASK, leopard_sdclk_sel, SDCLK_DIV_MASK, sd_clk_rates, 0),
    PERIPH_CLK(CLK_NAME_SD2_CLK, MODULE_CLK_SD2, MODULE_RESET_SD2, CMU_SD2CLK, SDCLK_SEL_MASK, leopard_sdclk_sel, SDCLK_DIV_MASK, sd_clk_rates, 0),
 
    /*新增nic clk*/
    PERIPH_CLK(CLK_NAME_SD0_NIC_CLK, MODULE_INVALID_ID, MODULE_INVALID_ID, CMU_SD0CLK, 0, NULL, SD_NIC_CLK_DIV_MASK, sd_nic_clk_rates, 0),
    PERIPH_CLK(CLK_NAME_SD1_NIC_CLK, MODULE_INVALID_ID, MODULE_INVALID_ID, CMU_SD1CLK, 0, NULL, SD_NIC_CLK_DIV_MASK, sd_nic_clk_rates, 0),
    PERIPH_CLK(CLK_NAME_SD2_NIC_CLK, MODULE_INVALID_ID, MODULE_INVALID_ID, CMU_SD2CLK, 0, NULL, SD_NIC_CLK_DIV_MASK, sd_nic_clk_rates, 0),   
    
    PERIPH_CLK(CLK_NAME_UART0_CLK, MODULE_CLK_UART0, MODULE_RESET_UART0, CMU_UART0CLK, UARTCLK_SEL_MASK, leopard_uartclk_sel, UARTCLK_DIV_MASK, NULL, DIV_UART),
    PERIPH_CLK(CLK_NAME_UART1_CLK, MODULE_CLK_UART1, MODULE_RESET_UART1, CMU_UART1CLK, UARTCLK_SEL_MASK, leopard_uartclk_sel, UARTCLK_DIV_MASK, NULL, DIV_UART),
    PERIPH_CLK(CLK_NAME_UART2_CLK, MODULE_CLK_UART2, MODULE_RESET_UART2, CMU_UART2CLK, UARTCLK_SEL_MASK, leopard_uartclk_sel, UARTCLK_DIV_MASK, NULL, DIV_UART),
    PERIPH_CLK(CLK_NAME_UART3_CLK, MODULE_CLK_UART3, MODULE_RESET_UART3, CMU_UART3CLK, UARTCLK_SEL_MASK, leopard_uartclk_sel, UARTCLK_DIV_MASK, NULL, DIV_UART),
    PERIPH_CLK(CLK_NAME_UART4_CLK, MODULE_CLK_UART4, MODULE_RESET_UART4, CMU_UART4CLK, UARTCLK_SEL_MASK, leopard_uartclk_sel, UARTCLK_DIV_MASK, NULL, DIV_UART),
    PERIPH_CLK(CLK_NAME_UART5_CLK, MODULE_CLK_UART5, MODULE_RESET_UART5, CMU_UART5CLK, UARTCLK_SEL_MASK, leopard_uartclk_sel, UARTCLK_DIV_MASK, NULL, DIV_UART),
    PERIPH_CLK(CLK_NAME_ETHERNET_CLK, MODULE_CLK_ETHERNET, MODULE_RESET_ETHERNET, CMU_ETHERNETCLK, (0x3<<4), leopard_ethernet_nic_sel, 0xf, ethernet_nic_clk_rates, 0),


   
    PERIPH_CLK(CLK_NAME_LCD_CLK, MODULE_INVALID_ID, MODULE_RESET_LCD, CMU_LCDCLK, LCDCLK_SEL_MASK, leopard_lcd_clk_sel, LCDCLK_DIV_MASK, lcd_clk_rates, 0),

    MUX_CLK(CLK_NAME_DE_CLK,  MODULE_CLK_DE, MODULE_RESET_DE, CMU_DECLK, DECLK_SEL_MASK, leopard_de_clk_sel, 0, NULL, 0),
    PERIPH_CLK(CLK_NAME_DE1_CLK,  MODULE_CLK_DE, MODULE_RESET_DE, CMU_DECLK, 0, NULL, DE1CLK_DIV_MASK,de_clk_rates, 0),
    PERIPH_CLK(CLK_NAME_DE2_CLK,  MODULE_CLK_DE, MODULE_RESET_DE, CMU_DECLK, 0, NULL, DE2CLK_DIV_MASK,de_clk_rates, 0),
    PERIPH_CLK(CLK_NAME_DE3_CLK,  MODULE_CLK_DE, MODULE_RESET_DE, CMU_DECLK, 0, NULL, DE3CLK_DIV_MASK,de_clk_rates, 0),
    PERIPH_CLK(CLK_NAME_DE_WB_CLK,  MODULE_CLK_DE, MODULE_RESET_DE, CMU_DECLK, 0, NULL, DE_WB_CLK_DIV_MASK,de_clk_rates, 0),    
//    PERIPH_CLK(CLK_NAME_SENSOR_OUT0_CLK,  MODULE_INVALID_ID, MODULE_INVALID_ID, CMU_SENSORCLK, 0, NULL, SENSOROUT0CLK_DIV_MASK, sensor_clk_rates, 0),  
//    PERIPH_CLK(CLK_NAME_SENSOR_OUT1_CLK,  MODULE_INVALID_ID, MODULE_INVALID_ID, CMU_SENSORCLK, 0, NULL, SENSOROUT1CLK_DIV_MASK, sensor_clk_rates, 0),
    PERIPH_CLK(CLK_NAME_SENSOR_OUT_CLK,  MODULE_INVALID_ID, MODULE_INVALID_ID, CMU_SENSORCLK, SENSOROUTCLK_SEL_MASK, leopard_sensor_clkout_sel, SENSOROUTCLK_DIV_MASK, sensor_clk_rates, 0),

};

/*only do clk_init*/
__init int leopard_init_clocks(void)
{
    int i=0;
    struct clk* c = NULL;
    
    for(i=0;i<ARRAY_SIZE(leopard_ptr_clks);i++)
    {
        c = leopard_ptr_clks[i];
        clk_init(c);
        if (!c->lookup.dev_id && !c->lookup.con_id)
        {
		    c->lookup.dev_id = c->name;
	    }
	    c->lookup.clk = c;
	    clkdev_add(&c->lookup);
    }
    c = NULL;
    for(i=0; i<ARRAY_SIZE(leopard_list_clks); i++)
    {
        c = &(leopard_list_clks[i]);
        if(c == NULL)
        {
            printk("\n[zjl] leopard_list_clks[%d] error at %s %d",i, __FUNCTION__, __LINE__);
        }
        
        clk_init(c);
        if (!c->lookup.dev_id && !c->lookup.con_id)
        {
		    c->lookup.dev_id = c->name;
	    }
	    c->lookup.clk = c;
	    clkdev_add(&c->lookup);
    }

    return 0;
}


__init int leopard_arch_init_clocks(void)
{
    int i=0;
    struct clk* c = NULL;
    
    asoc_clock_debugfs_init();
    for(i=0;i<ARRAY_SIZE(leopard_ptr_clks);i++)
    {
        c = leopard_ptr_clks[i];
        if(c != &leopard_cpuclk)
        {
            asoc_clock_debugfs_add_clk(c);
        }
    }
    c = NULL;
    for(i=0; i<ARRAY_SIZE(leopard_list_clks); i++)
    {
        c = &(leopard_list_clks[i]);
        if(c == NULL)
        {
            printk("\n[zjl] leopard_list_clks[%d] error at %s %d",i, __FUNCTION__, __LINE__);
        }
	    asoc_clock_debugfs_add_clk(c);
    }

    return 0;
}

//arch_initcall(leopard_arch_init_clocks);