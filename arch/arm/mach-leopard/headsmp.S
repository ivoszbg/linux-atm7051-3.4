/*
 * arch/arm/mach-leopard/headsmp.S
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

	__CPUINIT

/*
 * Leopard specific entry point for secondary CPUs.
 */
/*
 * Leopard specific entry point for secondary CPU to jump from ROM
 * code.  This routine also provides a holding flag into which
 * secondary core is held until we're ready for it to initialise.
 */
ENTRY(asoc_secondary_startup)
	mrc	p15, 0, r0, c0, c0, 5
	and	r0, r0, #0xf
	adr	r4, 1f
	ldmia	r4, {r5, r6}
	sub	r4, r4, r5
	add	r6, r6, r4
pen:	ldr	r7, [r6]
	cmp	r7, r0
	bne	pen

	/*
	 * we've been released from the holding pen: secondary_stack
	 * should now contain the SVC stack for this core
	 */
	b	secondary_startup

1:	.long	.
	.long	pen_release
