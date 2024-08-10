/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2012 Actions Semi Inc.
*/
/******************************************************************************/
#include <mach/hardware.h>
#include <asm/memory.h>
#include <asm/hardware/cache-l2x0.h>
#include <asm/asm-offsets.h>
#include <asm/assembler.h>
#include <linux/linkage.h>
#include <linux/threads.h>

    .text

    //.equ pmu_type,  0
#ifdef CONFIG_THUMB2_KERNEL
	ENTRY(leopard_finish_suspend)
	ldr r0, =0x0
	ENDPROC(leopard_finish_suspend)

	ENTRY(leopard_cpu_resume)
	ldr r0, =0x0
	ENDPROC(leopard_cpu_resume)


#else	/* CONFIG_THUMB2_KERNEL */

ENTRY(leopard_finish_suspend)
    stmfd	sp!, {lr}

   mov r8,r1  @i2c base addr
    /*Ç¿ÖÆÇÐjtag*/
   ldr r0, =IO_ADDRESS(0xb01c0048)
   ldr r1, =0x759c0
   str r1, [r0]

   
#if 1 
    bl leopard_clean_dcache_all
    
    /*
	 * Clean & invalid L2 cache, and sync L2 cache.
	 */
    ldr r0, =IO_ADDRESS(0xB0022000)
    ldr r1, =0x0000ffff
    str	r1, [r0, #L2X0_CLEAN_INV_WAY]
wait:
	ldr	r2, [r0, #L2X0_CLEAN_INV_WAY]
	cmp r2, #0
	bne	wait    
    
    ldr r1, =0x0
    str	r1, [r0, #L2X0_CACHE_SYNC]
sync:
	ldr	r1, [r0, #L2X0_CACHE_SYNC]
	ands	r1, r1, #0x1
	bne	sync
	
	bl leopard_flush_dcache_all
	
#endif
   
#if 1 
    /*set ddr to self refresh*/
1:  
    ldr r0, =IO_ADDRESS(DCU_BASE)  @ Load DDR Control base address 0xB0230000 
    ldr r1, [r0, #0x7c]  @ Load DCU_CMD(0x007c)
    orr r1, r1, #0x20
    str r1, [r0, #0x7c]  @Set SREN(bit5)
    
wait_for_ok:
    ldr r2, [r0, #0x7c]
    cmp r1,r2
    beq  wait_for_ok
    
    /* diable DCU VDD detect */
    ldr r1, [r0, #0x4]  @ Load DCU_FEA(0x0004)
    bic r1, #0x10000000   @ clear bit 28
    str r1, [r0, #0x4]  
#endif

#if 0    /*not need*/
   /*enable hosc*/
    ldr r0, =IO_ADDRESS(CMU_BASE)
    ldr r1, [r0]
    and r1, r1, #(1<<8)
    str r1, [r0]
    
#endif 

    /*switch corepll to hosc!*/
    ldr r0, =IO_ADDRESS(CMU_BASE)
    ldr r1, [r0, #0x1c] @load CMU_BUSCLK 
    bic r1, r1, #0x3    @clear bit[1:0]  
    orr r1, r1, #0x1
    str r1, [r0, #0x1c]
    nop
    nop
    nop
    nop


    cmp r8,#0  /*i2c base8*/
    beq enter_s2
		
i2c_enter_s2:    
    /*enter to S2 state*/ /* enable i2c0 */

	ldr r0, =IO_ADDRESS(CMU_DEVCLKEN1)
	ldr r1, [r0]
	orr r1, r1, #0x4000
	str r1, [r0]

	/* reset i2c0 module */
	/*ldr r0, =IO_ADDRESS(I2C0_BASE)*/	/*gl5307 using i2c0*/
	mov r0, r8  /*i2c base addr*/
	ldr r4,=0x0
	ldr r3, =0x30 @timeout 2us at 24M clk
	
	ldr r2,=0x80
	str r2, [r0]
	
	/*delay 2us*/
wait_for_i2c_reset:   
	sub  r3,r3, #0x1
	cmp  r3,r4
	bne  wait_for_i2c_reset
    
	ldr r2, =0x4 		/*set clk*/
	str r2, [r0, #0x04]
	
	ldr  r2, =0xCA 		/* set client address. */
	str  r2, [r0, #0x10]
	
	ldr  r2, =0x3		/* set data count. */
	str  r2, [r0, #0x24]	
	
	ldr  r2, =0x01		/* PMU_SYS_CTL1*/
	str  r2, [r0, #0x10]
	
	ldr r2, =0x00		/* reg value: 0x001e*/
	str  r2, [r0, #0x10]
	ldr r2, =0x1E
	str  r2, [r0, #0x10]
	
	ldr r2, =0x8f03		/* I2C_CMD_EXEC | I2C_CMD_MSS | I2C_CMD_SE | I2C_CMD_NS | I2C_CMD_DE | I2C_CMD_AS(1) | I2C_CMD_SBE*/
	str  r2, [r0, #0x18]
	
	ldr r4,=0x0
    ldr r3, =100000 @50000 rounds 
	
wait_for_finish:   
	ldr  r1, [r0, #0x20]	/*wait for transfer complete*/
	ands r1, r1, #0x1		/* check transfer finish*/
	beq  done
	sub  r3,r3, #0x1
	cmp  r3,r4
	bne  wait_for_finish
	

done:
	nop

	nop
	nop
	nop
	nop
  
	b   i2c_enter_s2
		
enter_s2:    

    /*enter to S2 state*/
//1:
//	b 1b
    /*reset spi module*/
reset_spi:
    ldr r0, =IO_ADDRESS(SPI1_BASE)  @Load SPI1 Control base address 0xB0254000
    ldr r4,=0x0
    ldr r3, =0x30 @timeout 2us at 24M clk
    
    ldr r2,=0xd0
    str r2, [r0]           

    /*delay 2us*/
wait_for_reset:   
    sub  r3,r3, #0x1
    cmp  r3,r4
    bne  wait_for_reset
    
    ldr  r2, =0x401d0 
    str  r2, [r0]
    
    /*start transfer*/
    ldr r3, =0x1d4c0      @timeout 5ms at 24M clk
    ldr r0, =IO_ADDRESS(SPI1_BASE)  @Load SPI1 Control base address 0xB0254000
    ldr r1, =0x30
    str r1, [r0, #0x8]   @clear SPI FIFO
    
    ldr r1, =0x401c0
    str r1, [r0]         @16bit mode 3,SPI CS- low
    
    ldr r1, =0x8008      @(1 << 3) | 0x8000
    str r1, [r0, #0x10]  @write gl5302_PMU_SYS_CTL1 register number
    
wait_for_spi:
    subs  r3,r3,#0x1
    beq  reset_spi
    
    ldr r1, [r0, #0x8]
    ands r1, r1, #0x4
    beq  wait_for_spi
    nop
    
    ldr r1, [r0, #0x8]  @clear Transfer Complete Bit
    str r1, [r0, #0x8]
    
    ldr r1, =0x30
    str r1, [r0, #0x8] @clear SPI FIFO
    
    ldr r3, =0x1d4c0      @timeout 5ms at 24M clk
    ldr r1, =0x1e         @ write reg value 
    str r1, [r0, #0x10] 
wait_for_spi_1:
    subs  r3,r3,#0x1
    beq  reset_spi
    
    ldr r1, [r0, #0x8]
    ands r1, r1, #0x4
    beq  wait_for_spi_1
    nop
    
    ldr r1, [r0, #0x8]  @clear Transfer Complete Bit
    str r1, [r0, #0x8]    
    ldr r1, =0x30
    str r1, [r0, #0x8] @clear SPI FIFO
#if 1    
    ldr r1, =0x401d0
    str r1, [r0]         @16bit mode 3,SPI CS- high
      
    nop
    nop
    nop
    nop
  
    b   enter_s2
    
    
    
#else           
    /*wait for power down*/
wait_for_pd:
    b    wait_for_pd

    nop
    nop
    nop 
#endif 
ENDPROC(leopard_finish_suspend)

ENTRY(leopard_cpu_resume)
    /*enable L2 cache*/
    
    /* FIXME: map execption vector & init uart for Actions FPGA board */
#if 0
	movw	r0, 0x7ffc
	movt	r0, 0x0000
	movw	r6, 0x0c12
	movt	r6, 0xffff
	str	r6, [r0]

	movw	r0, 0x0074
	movt	r0, 0xb01c
	movw	r6, 0x0002
	movt	r6, 0x0000
	str	r6, [r0]

	movw	r0, 0x004c
	movt	r0, 0xb01c
	ldr	r6, [r0]
	and	r6, r6, #0xff07ffff
	orr	r6, r6, #0x0
	str	r6, [r0]

	movw	r0, 0x0000
	movt	r0, 0xb016
	movw	r6, 0x8003
	movt	r6, 0x0004
	str	r6, [r0]
#endif
    
#if 0   /*for debug , don't remove*/
     /* enable ks_out1(GPIOB7) as gpio input */
    ldr     r0, =0xb01c0010
    ldr	    r6, [r0]
	orr	r6, r6, #0x00000080
	str	r6, [r0]

    /* MFP_CTL2 */
    ldr     r0, =0xb01c0048
    ldr	    r6, [r0]
	and	r6, r6, #0xffff3fff
	orr	r6, r6, #0x000b4000
	str	r6, [r0]
	
	/*devclk enable*/
	ldr     r0, =0xb01500a4
	ldr     r6, [r0]
	orr     r6, #(0x1<<21)
	str     r6, [r0]
	
	/* UART5_CTL */
    ldr     r0, =0xb016a000
    ldr     r6, =0x00048003
    str	r6, [r0]

    /* CMU_UART5CLK, use 24M HOSC, 38400 */
    ldr     r0, =0xb01500b8
    ldr     r6, =0x19
    str	r6, [r0]
#endif 
    
    /*goto cpu_resume*/
    b	cpu_resume	
ENDPROC(leopard_cpu_resume)

#endif /* !CONFIG_THUMB2_KERNEL */


ENTRY(leopard_flush_dcache_all)
	dmb					@ ensure ordering with previous memory accesses
	mrc	p15, 1, r0, c0, c0, 1		@ read clidr
	ands	r3, r0, #0x7000000		@ extract loc from clidr
	mov	r3, r3, lsr #23			@ left align loc bit field
	beq	finished			@ if loc is 0, then no need to clean
	mov	r10, #0				@ start clean at cache level 0
loop1:
	add	r2, r10, r10, lsr #1		@ work out 3x current cache level
	mov	r1, r0, lsr r2			@ extract cache type bits from clidr
	and	r1, r1, #7			@ mask of the bits for current cache only
	cmp	r1, #2				@ see what cache we have at this level
	blt	skip				@ skip if no cache, or just i-cache
#ifdef CONFIG_PREEMPT
	save_and_disable_irqs_notrace r9	@ make cssr&csidr read atomic
#endif
	mcr	p15, 2, r10, c0, c0, 0		@ select current cache level in cssr
	isb					@ isb to sych the new cssr&csidr
	mrc	p15, 1, r1, c0, c0, 0		@ read the new csidr
#ifdef CONFIG_PREEMPT
	restore_irqs_notrace r9
#endif
	and	r2, r1, #7			@ extract the length of the cache lines
	add	r2, r2, #4			@ add 4 (line length offset)
	ldr	r4, =0x3ff
	ands	r4, r4, r1, lsr #3		@ find maximum number on the way size
	clz	r5, r4				@ find bit position of way size increment
	ldr	r7, =0x7fff
	ands	r7, r7, r1, lsr #13		@ extract max number of the index size
loop2:
	mov	r9, r4				@ create working copy of max way size
loop3:
 ARM(	orr	r11, r10, r9, lsl r5	)	@ factor way and cache number into r11
 THUMB(	lsl	r6, r9, r5		)
 THUMB(	orr	r11, r10, r6		)	@ factor way and cache number into r11
 ARM(	orr	r11, r11, r7, lsl r2	)	@ factor index number into r11
 THUMB(	lsl	r6, r7, r2		)
 THUMB(	orr	r11, r11, r6		)	@ factor index number into r11
	mcr	p15, 0, r11, c7, c14, 2		@ clean & invalidate by set/way
	subs	r9, r9, #1			@ decrement the way
	bge	loop3
	subs	r7, r7, #1			@ decrement the index
	bge	loop2
skip:
	add	r10, r10, #2			@ increment cache number
	cmp	r3, r10
	bgt	loop1
finished:
	mov	r10, #0				@ swith back to cache level 0
	mcr	p15, 2, r10, c0, c0, 0		@ select current cache level in cssr
	dsb
	isb
	mov	pc, lr
ENDPROC(leopard_flush_dcache_all)


ENTRY(leopard_clean_dcache_all)
	dmb					@ ensure ordering with previous memory accesses
	mrc	p15, 1, r0, c0, c0, 1		@ read clidr
	ands	r3, r0, #0x7000000		@ extract loc from clidr
	mov	r3, r3, lsr #23			@ left align loc bit field
	beq	finished_			@ if loc is 0, then no need to clean
	mov	r10, #0				@ start clean at cache level 0
loop1_:
	add	r2, r10, r10, lsr #1		@ work out 3x current cache level
	mov	r1, r0, lsr r2			@ extract cache type bits from clidr
	and	r1, r1, #7			@ mask of the bits for current cache only
	cmp	r1, #2				@ see what cache we have at this level
	blt	skip_				@ skip if no cache, or just i-cache
#ifdef CONFIG_PREEMPT
	save_and_disable_irqs_notrace r9	@ make cssr&csidr read atomic
#endif
	mcr	p15, 2, r10, c0, c0, 0		@ select current cache level in cssr
	isb					@ isb to sych the new cssr&csidr
	mrc	p15, 1, r1, c0, c0, 0		@ read the new csidr
#ifdef CONFIG_PREEMPT
	restore_irqs_notrace r9
#endif
	and	r2, r1, #7			@ extract the length of the cache lines
	add	r2, r2, #4			@ add 4 (line length offset)
	ldr	r4, =0x3ff
	ands	r4, r4, r1, lsr #3		@ find maximum number on the way size
	clz	r5, r4				@ find bit position of way size increment
	ldr	r7, =0x7fff
	ands	r7, r7, r1, lsr #13		@ extract max number of the index size
loop2_:
	mov	r9, r4				@ create working copy of max way size
loop3_:
 ARM(	orr	r11, r10, r9, lsl r5	)	@ factor way and cache number into r11
 THUMB(	lsl	r6, r9, r5		)
 THUMB(	orr	r11, r10, r6		)	@ factor way and cache number into r11
 ARM(	orr	r11, r11, r7, lsl r2	)	@ factor index number into r11
 THUMB(	lsl	r6, r7, r2		)
 THUMB(	orr	r11, r11, r6		)	@ factor index number into r11
	mcr	p15, 0, r11, c7, c10, 2		@ clean & invalidate by set/way
	subs	r9, r9, #1			@ decrement the way
	bge	loop3_
	subs	r7, r7, #1			@ decrement the index
	bge	loop2_
skip_:
	add	r10, r10, #2			@ increment cache number
	cmp	r3, r10
	bgt	loop1_
finished_:
	mov	r10, #0				@ swith back to cache level 0
	mcr	p15, 2, r10, c0, c0, 0		@ select current cache level in cssr
	dsb
	isb
	mov	pc, lr
ENDPROC(leopard_clean_dcache_all)


ENTRY(leopard_clean_l2_cache_all)
	stmfd	sp!, {r0-r11,lr}		@ save registers

    bl leopard_clean_dcache_all
    
    /*
	 * Clean & invalid L2 cache, and sync L2 cache.
	 */
    ldr r0, =IO_ADDRESS(0xB0022000)
    ldr r1, =0x0000ffff
    str	r1, [r0, #L2X0_CLEAN_INV_WAY]
1:
	ldr	r2, [r0, #L2X0_CLEAN_INV_WAY]
	cmp r2, #0
	bne	1b    
    
    ldr r1, =0x0
    str	r1, [r0, #L2X0_CACHE_SYNC]
2:
	ldr	r1, [r0, #L2X0_CACHE_SYNC]
	ands	r1, r1, #0x1
	bne	2b
	
	bl leopard_flush_dcache_all
	
	ldmfd	sp!, {r0-r11, pc}		@ return
ENDPROC(leopard_clean_l2_cache_all)
