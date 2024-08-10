/*
 * Hibernation support specific for ARM
 *
 * Based on work by:
 *
 * Ubuntu project, hibernation support for mach-dove,
 * Copyright (C) 2010 Nokia Corporation (Hiroshi Doyu)
 * Copyright (C) 2010 Texas Instruments, Inc. (Teerth Reddy et al.)
 *	https://lkml.org/lkml/2010/6/18/4
 *	https://lists.linux-foundation.org/pipermail/linux-pm/2010-June/027422.html
 *	https://patchwork.kernel.org/patch/96442/
 *
 * Copyright (C) 2006 Rafael J. Wysocki <rjw@sisk.pl>
 *
 * License terms: GNU General Public License (GPL) version 2
 */

#include <linux/linkage.h>
#include <linux/init.h>

#include <asm/assembler.h>
#include <asm/cp15.h>
#include <asm/domain.h>
#include <asm/ptrace.h>
#include <asm/asm-offsets.h>
#include <asm/memory.h>
#include <asm/page.h>
#include <asm/assembler.h>

/*
 * Save the current CPU state before suspend / poweroff.
 * cpu_suspend() allocates space on the stack to save all necessary
 * information. This has two consequences:
 *	- swsusp_save() has to be called without changing anything on
 *	  the stack. One cannot just return into it.
 *	- should swsusp_save() fail for some reason, the previous value
 *	  of sp has to be restored from a safe place.
 */
ENTRY(swsusp_arch_suspend)
	mrs	r1, cpsr
	mrs	r2, spsr
	stmfd	sp!, {r1-r11,lr}		@ save registers
	ldr	r0, =.Ltemp_sp
	str	sp, [r0]			@ temp
	ldr	r1, =(PHYS_OFFSET - PAGE_OFFSET)
	adr	r3, .Lresume_post_mmu		@ resume here
	bl	hibernate_cpu_suspend			@ snapshot state (to stack)
	subs	r1, r0, #0
	beq	resumeSuccess
	
	ldr	r1, =sleep_save_sp
	ldr	r0, =__swsusp_save_sp
	ldr	r2, [r1]
	str	r2, [r0]
	ldr	sp, =(__swsusp_resume_stk + PAGE_SIZE / 2)
	bl	swsusp_save			@ write snapshot
resumeSuccess:
	ldr	r1, =.Ltemp_sp
	ldr	sp, [r1]			@ restore stack
	ldmfd	sp!, {r1-r11, pc}		@ return
ENDPROC(swsusp_arch_suspend)

/*
 * Restore the memory image from the pagelists, and load the CPU registers
 * from saved state.
 */
ENTRY(swsusp_arch_resume)
	setmode PSR_I_BIT | PSR_F_BIT | SVC_MODE, r2
	/*
	 * Switch stack to a nosavedata region to make sure image restore
	 * doesn't clobber it underneath itself.
	 * Note that this effectively nukes "current"; from here on, the
	 * executing code runs context-less and no functions can be called
	 * that have side effects beyond accessing global variables.
	 */
	ldr	sp, =(__swsusp_resume_stk + PAGE_SIZE / 2)
	b	__swsusp_arch_restore_image
.ltorg
.align 5
	/*
	 * Execution returns here via resuming the saved context.
	 * MMU is active again and CPU core state has been restored, all
	 * that remains to be done now is to restore the CPU registers.
	 */
.Lresume_post_mmu:
	ldmfd	sp!, {r1-r11}
	msr	cpsr, r1
	msr	spsr, r2
	bl	cpu_init			@ reinitialize other modes
	ldmfd	sp!, {lr}
	b	__swsusp_arch_resume_finish	@ cleanup
ENDPROC(swsusp_arch_resume)

.data
.Ltemp_sp:
	.long 0
