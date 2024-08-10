/*
 * arch/arm/mach-leopard/sleep.S
 *
 * Secondary CPU startup routine source file
 *
 * Copyright 2012 Actions Semi Inc.
 * Author: Actions Semi, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/linkage.h>
#include <linux/init.h>

ENTRY(leopard_do_wfi)
	#stmfd	sp!, {lr}
	/* Drain interconnect write buffers. */
	#bl leopard_bus_sync ?

	/*
	 * Execute an ISB instruction to ensure that all of the
	 * CP15 register changes have been committed.
	 */
	isb

	/*
	 * Execute a barrier instruction to ensure that all cache,
	 * TLB and branch predictor maintenance operations issued
	 * by any CPU in the cluster have completed.
	 */
	dsb
	dmb

	/*
	 * Execute a WFI instruction and wait until the
	 * STANDBYWFI output is asserted to indicate that the
	 * CPU is in idle and low power state. CPU can specualatively
	 * prefetch the instructions so add NOPs after WFI. Sixteen
	 * NOPs as per Cortex-A9 pipeline.
	 */
	wfi 				@ Wait For Interrupt
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop

	#ldmfd	sp!, {pc}
	mov	pc, lr
ENDPROC(leopard_do_wfi)


