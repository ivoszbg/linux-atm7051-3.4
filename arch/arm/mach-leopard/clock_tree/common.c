#include <linux/kernel.h>
#include <linux/clk.h>
#include <mach/clock.h>
#include <mach/hardware.h>


static __initdata struct leopard_clk_init_table leopard_clk_init_table[] = {
    /* name                        parent              rate       enabled */
    { "corepll",           NULL,              0,     false},
    { "cpuclk",            "corepll",         0,     false},
    { "dma_clk",           "cpuclk",          0,     false},
    { "smp_twd",           "cpuclk",          0,     false},  
    { "l2_clk",            "cpuclk",          0,     false},
    
    /*devpll sub clock tree*/
    { "devpll",            NULL,      654000000,     true},
    { "devclk",            "devpll",  0,     true},
 
    { "hclk",              "devclk",  0,     true},
    { "pclk",              "hclk",    0,     true},
    
//    { "lcd_clk",            "devclk", 0,  false},
//    { "lcd0_clk",           "lcd_clk",     0,  false},
    
//    { "vce_clk",           "devclk",  0,     false},
//    { "vde_clk",           "devclk",  0,     false},
//    { "gpu2d_clk",         "devclk",  0,     false},
//    { "gpu2d_nic_clk",     "devclk",  0,     false},
      { "gpu_clk",         "devclk",  0,     false},
      { "vce_clk",           "devclk",  0,     false},
      { "vde_clk",           "devclk",  0,     false},
      { "si_clk",            "devclk",        0,      false},
      { "sd0_clk",           "devclk",  0,     false}, 
      { "sd0_nic_clk",       "devclk",  100000000,     false},
      { "sd1_clk",           "devclk",  0,     false}, 
      { "sd1_nic_clk",       "devclk",  100000000,     false}, 
      { "sd2_clk",           "devclk",  0,     false}, 
      { "sd2_nic_clk",       "devclk",  100000000,     false},
//    { "gpu3d_nic_clk",     "devclk",  0,     false},
//    { "nic_dcu_clk",       "ddr_clk",  0,     false},
    
    /*120MPLL sub clock tree*/
    { "120Mpll",           NULL,              0,     false},
    //{ "sd0_clk",           "120Mpll",  0,     false}, 
    //{ "sd1_clk",           "120Mpll",  0,     false},
    //{ "sd2_clk",           "120Mpll",  0,     false},  
    
    {"usbpll",             NULL,               0,    false},
    
    /*hosc sub clock tree*/
    { "uart0_clk",          "hosc",             0,      false},  
    { "uart1_clk",          "hosc",             0,      false},  
    { "uart2_clk",          "hosc",             0,      false},  
    { "uart3_clk",          "hosc",             0,      false},  
    { "uart4_clk",          "hosc",             0,      false},  
    { "uart5_clk",          "hosc",             0,      false},  
 //   { "lens_clk",           "hosc",             0,        false},
    { "hdmi24M_clk",        "hosc",             0,      false},
    { "timer_clk",          "hosc",             0,      false},
    { "irc_clk",            "hosc",             0,      false},
    { "i2c0_clk",           "hosc",             0,      false},
    { "i2c1_clk",           "hosc",             0,      false},    
    { "i2c2_clk",           "hosc",             0,      false},   
    { "sensor_out_clk",     "hosc",             0,      false},
    
//    { "tv24M_clk",          "hosc",             0,            false},            
 //   { "sensor_clk",         "hosc",             0,            false}, /*sensor_clkÊÇÐéÄâclk*/   
//    { "sensor_out0_clk",    "sensor_clk", 0,          false},
 //   { "sensor_out1_clk",    "sensor_clk", 0,          false},
    
    /*tvout0pll*/
    { "tvout0pll",          NULL,                0, false},
    { "tvout_mux0_clk",  "tvout0pll",      0,         false},
    { "tvout0_clk",      "tvout_mux0_clk", 0, false},
    { "deepcolorpll",    "tvout_mux0_clk", 0, false},       
    
    { "tvout1pll",         NULL,          0, false},
//    { "cvbs_clk",         "tvout1pll",     0, false},
    
    /*displaypll*/
    { "displaypll",         NULL,                 0,  true},
 //   { "gpu2d_clk",         "displaypll",  0,     false},
//    { "gpu2d_nic_clk",     "displaypll",  0,     false},
    { "lcd_clk",            "displaypll", 0,  false},
    { "lcd0_clk",           "lcd_clk",     0,  false},
//    { "lcd1_clk",           "lcd_clk",     0,  false},
 //   { "img5_clk",           "lcd0_clk",    0,  false},
//    { "vce_clk",           "displaypll",  0,     false},
//    { "vde_clk",           "displaypll",  0,     false},
 //   { "csi_clk",            "displaypll", 0,  false},    

//    { "de_clk",             "displaypll", 0,  false},  /*de_clkÊÇÐéÄâclk*/ 
    { "de_clk",             "devclk", 0,  false},  /*de_clkÊÇÐéÄâclk*/     
    { "de1_clk",            "de_clk",     0,  false},    
    { "de2_clk",            "de_clk",     0,  false},    
    { "de3_clk",            "de_clk",     0,  false},    
    { "de_wb_clk",          "de_clk",     0,  false},    
//    { "bisp_clk",           "displaypll", 0,  false},    
    
    /*audiopll*/
    { "audiopll",           NULL,                 0,   false},
    { "i2stx_clk",          "audiopll",   0,   false},
    { "i2srx_clk",          "audiopll",   0,   false},
    { "hdmia_clk",          "audiopll",   0,   false},
    { "spdif_clk",          "audiopll",   0,   false},
    { "pcm0_clk",           "audiopll",   0,   false},
    { "pcm1_clk",           "audiopll",   0,   false},
    
    /*ddr clk*/
    { "ddrpll",             NULL,                 0,  true},
    { "ddr_clk",            "ddrpll",    0,   true},
//    { "hclk",               "ddr_clk",  0,     true},
//    { "pclk",               "hclk",    0,     true},
    { NULL,     NULL,       0,      0},
};

static int leopard_clk_init_one_from_table(struct leopard_clk_init_table *table)
{
    struct clk *c;
    struct clk *p;

    int ret = 0;    

    c = clk_get_by_name(table->name);

    if (!c) {
        pr_warning("Unable to initialize clock %s\n",
            table->name);
        return -ENODEV;
    }

    if (table->parent) {
        p = clk_get_by_name(table->parent);
        if (!p) {
            pr_warning("Unable to find parent %s of clock %s\n",
                table->parent, table->name);
            return -ENODEV;
        }
        if (c->parent != p) {
            ret = clk_set_parent(c, p);
            if (ret) {
                pr_warning("Unable to set parent %s of clock %s: %d\n",
                    table->parent, table->name, ret);
                return -EINVAL;
            }
        }
    }
    if (table->rate && (table->rate != clk_get_rate(c))) {
        ret = clk_set_rate(c, table->rate);
        if (ret) {
            pr_warning("Unable to set clock %s to rate %lu: %d\n",
                table->name, table->rate, ret);
            return -EINVAL;
        }
    }
    if (table->enabled) {
        ret = clk_enable(c);
        if (ret) {
            pr_warning("Unable to enable clock %s: %d\n",
                table->name, ret);
            return -EINVAL;
        }
    }
    return 0;
}


void leopard_clk_init_from_table(struct leopard_clk_init_table *table)
{
    for (; table->name; table++)
        leopard_clk_init_one_from_table(table);
}


int __init leopard_cmu_init_early(void)
{
    leopard_init_clocks();
    leopard_clk_init_from_table(leopard_clk_init_table);
   
    recalculate_root_clocks();
    return 0;
}
