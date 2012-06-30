/*
 * relocate_kernel.S - put the kernel image in place to boot
 */

#include <asm/kexec.h>
#include <linux/linkage.h>
#include <asm/assembler.h>
 
#include <linux/init.h>
#include <linux/linkage.h>
#include <asm/asm-offsets.h>
#include <asm/hwcap.h>
#include <asm/pgtable-hwdef.h>
#include <asm/pgtable.h>

ENTRY(omap_smc1)
	stmfd   sp!, {r2-r12, lr}
	mov	r12, r0
	mov 	r0, r1
	dsb
	smc	#0
	ldmfd   sp!, {r2-r12, pc}
END(omap_smc1)

ENTRY(kexec_cpu_proc_fin)
	mrc	p15, 0, r0, c1, c0, 0		@ ctrl register
	bic	r0, r0, #0x1000			@ ...i............
	bic	r0, r0, #0x0006			@ .............ca.
	mcr	p15, 0, r0, c1, c0, 0		@ disable caches
	mov	pc, lr
ENDPROC(kexec_cpu_proc_fin)

/*
 *	kexec_cpu_reset(loc)
 *
 *	Perform a soft reset of the system.  Put the CPU into the
 *	same state as it would be if it had been reset, and branch
 *	to what would be the reset vector.
 *
 *	- loc   - location to jump to for soft reset
 */
	.align	5
ENTRY(kexec_cpu_reset)

	mrc	p15, 0, r1, c1, c0, 0		@ ctrl register
	bic	r1, r1, #0x0001			@ ...............m
	mcr	p15, 0, r1, c1, c0, 0		@ disable MMU
	mcr	p15, 0, ip, c8, c7, 0		@ invalidate I and D TLBs
	mov	pc, r0
ENDPROC(kexec_cpu_reset)

/*
 * void call_with_stack(void (*fn)(void *), void *arg, void *sp)
 *
 * Change the stack to that pointed at by sp, then invoke fn(arg) with
 * the new stack.
 */
ENTRY(call_with_stack)
	str	sp, [r2, #-4]!
	str	lr, [r2, #-4]!

	mov	sp, r2
	mov	r2, r0
	mov	r0, r1

	adr	lr, BSYM(1f)
	mov	pc, r2

1:	ldr	lr, [sp]
	ldr	sp, [sp, #4]
	mov	pc, lr
ENDPROC(call_with_stack)

	.align 8
	.balign 32
	.globl relocate_new_kernel
relocate_new_kernel:

	# Stop MPU WatchDog -- 0x4a314048
	movw r2,#0x4048
	movt r2,#0x4A31
	# WSPR - Start Stop register
	movw r3,#0xAAAA
	movt r3,#0x0000
	str r3,[r2, #0x48]

	@ reset
	mrc	p15, 0, r1, c1, c0, 0		@ ctrl register
	bic	r1, r1, #0x1			@ ...............m
	mcr	p15, 0, r1, c1, c0, 0		@ disable MMU
	isb

	ldr	r0,kexec_indirection_page
	ldr	r1,kexec_start_address


0:	/* top, read another word for the indirection page */
	ldr	r3, [r0],#4

	/* Is it a destination page. Put destination address to r4 */
	tst	r3,#1,0
	beq	1f
	bic	r4,r3,#1
	b	0b
1:
	/* Is it an indirection page */
	tst	r3,#2,0
	beq	1f
	bic	r0,r3,#2
	b	0b
1:

	/* are we done ? */
	tst	r3,#4,0
	beq	1f
	b	2f

1:
	/* is it source ? */
	tst	r3,#8,0
	beq	0b
	bic r3,r3,#8
	mov r6,#1024
9:
	ldr r5,[r3],#4
	str r5,[r4],#4
	subs r6,r6,#1
	bne 9b
	b 0b

2:
	/* Jump to relocated kernel */
	mov lr,r1
	mov r0,#0
	ldr r1,kexec_mach_type
	ldr r2,kexec_boot_atags

	movw	r3, #:lower16:0x4a008920
	movt	r3, #:upper16:0x4a008920
	movw	r4, #:lower16:0x00070000
	movt	r4, #:upper16:0x00070000
	str	r4, [r3]

	movw	r3, #:lower16:0x4a008900
	movt	r3, #:upper16:0x4a008900
	movw	r4, #:lower16:0x00000003
	str	r4, [r3]

	movw	r3, #:lower16:0x4a008f20
	movt	r3, #:upper16:0x4a008f20
	movw	r4, #:lower16:0x00070000
	movt	r4, #:upper16:0x00070000
	str	r4, [r3]

	movw	r3, #:lower16:0x4a008f28
	movt	r3, #:upper16:0x4a008f28
	movw	r4, #:lower16:0x00030000
	movt	r4, #:upper16:0x00030000
	str	r4, [r3]

	movw	r3, #:lower16:0x4a008f00
	movt	r3, #:upper16:0x4a008f00
	movw	r4, #:lower16:0x00000103
	movt	r4, #:upper16:0x00000103
	str	r4, [r3]

	movw	r3, #:lower16:0x4a004420
	movt	r3, #:upper16:0x4a004420
	movw	r4, #:lower16:0x00070000
	movt	r4, #:upper16:0x00070000
	str	r4, [r3]

	movw	r3, #:lower16:0x4a004400
	movt	r3, #:upper16:0x4a004400
	movw	r4, #:lower16:0x00000103
	movt	r4, #:upper16:0x00000103
	str	r4, [r3]

	movw	r3, #:lower16:0x4a004500
	movt	r3, #:upper16:0x4a004500
	movw	r4, #:lower16:0x00000103
	movt	r4, #:upper16:0x00000103
	str	r4, [r3]

	movw	r3, #:lower16:0x4a004528
	movt	r3, #:upper16:0x4a004528
	movw	r4, #:lower16:0x00070000
	movt	r4, #:upper16:0x00070000
	str	r4, [r3]

	movw	r3, #:lower16:0x4a004530
	movt	r3, #:upper16:0x4a004530
	movw	r4, #:lower16:0x00030000
	movt	r4, #:upper16:0x00030000
	str	r4, [r3]

	movw	r3, #:lower16:0x4a004538
	movt	r3, #:upper16:0x4a004538
	movw	r4, #:lower16:0x00030000
	movt	r4, #:upper16:0x00030000
	str	r4, [r3]

	movw	r3, #:lower16:0x4a004540
	movt	r3, #:upper16:0x4a004540
	movw	r4, #:lower16:0x00030000
	movt	r4, #:upper16:0x00030000
	str	r4, [r3]

	movw	r3, #:lower16:0x4a004548
	movt	r3, #:upper16:0x4a004548
	movw	r4, #:lower16:0x00030000
	movt	r4, #:upper16:0x00030000
	str	r4, [r3]

	movw	r3, #:lower16:0x4a004550
	movt	r3, #:upper16:0x4a004550
	movw	r4, #:lower16:0x00030000
	movt	r4, #:upper16:0x00030000
	str	r4, [r3]

	movw	r3, #:lower16:0x4a004558
	movt	r3, #:upper16:0x4a004558
	movw	r4, #:lower16:0x00030000
	movt	r4, #:upper16:0x00030000
	str	r4, [r3]

	movw	r3, #:lower16:0x4a004560
	movt	r3, #:upper16:0x4a004560
	movw	r4, #:lower16:0x00030000
	movt	r4, #:upper16:0x00030000
	str	r4, [r3]

	movw	r3, #:lower16:0x4a004568
	movt	r3, #:upper16:0x4a004568
	movw	r4, #:lower16:0x00030000
	movt	r4, #:upper16:0x00030000
	str	r4, [r3]

	movw	r3, #:lower16:0x4a004570
	movt	r3, #:upper16:0x4a004570
	movw	r4, #:lower16:0x00030000
	movt	r4, #:upper16:0x00030000
	str	r4, [r3]

	movw	r3, #:lower16:0x4a004578
	movt	r3, #:upper16:0x4a004578
	movw	r4, #:lower16:0x00030000
	movt	r4, #:upper16:0x00030000
	str	r4, [r3]

	movw	r3, #:lower16:0x4a004580
	movt	r3, #:upper16:0x4a004580
	movw	r4, #:lower16:0x00030000
	movt	r4, #:upper16:0x00030000
	str	r4, [r3]

	movw	r3, #:lower16:0x4a004588
	movt	r3, #:upper16:0x4a004588
	movw	r4, #:lower16:0x00030000
	movt	r4, #:upper16:0x00030000
	str	r4, [r3]

	movw	r3, #:lower16:0x4a009400
	movt	r3, #:upper16:0x4a009400
	movw	r4, #:lower16:0x004c8103
	movt	r4, #:upper16:0x004c8103
	str	r4, [r3]

	movw	r3, #:lower16:0x4a009428
	movt	r3, #:upper16:0x4a009428
	movw	r4, #:lower16:0x00030000
	movt	r4, #:upper16:0x00030000
	str	r4, [r3]

	movw	r3, #:lower16:0x4a009430
	movt	r3, #:upper16:0x4a009430
	movw	r4, #:lower16:0x00030000
	movt	r4, #:upper16:0x00030000
	str	r4, [r3]

	movw	r3, #:lower16:0x4a009438
	movt	r3, #:upper16:0x4a009438
	movw	r4, #:lower16:0x00030000
	movt	r4, #:upper16:0x00030000
	str	r4, [r3]

	movw	r3, #:lower16:0x4a009440
	movt	r3, #:upper16:0x4a009440
	movw	r4, #:lower16:0x00030000
	movt	r4, #:upper16:0x00030000
	str	r4, [r3]

	movw	r3, #:lower16:0x4a009448
	movt	r3, #:upper16:0x4a009448
	movw	r4, #:lower16:0x00030000
	movt	r4, #:upper16:0x00030000
	str	r4, [r3]

	movw	r3, #:lower16:0x4a009450
	movt	r3, #:upper16:0x4a009450
	movw	r4, #:lower16:0x00030000
	movt	r4, #:upper16:0x00030000
	str	r4, [r3]

	movw	r3, #:lower16:0x4a009460
	movt	r3, #:upper16:0x4a009460
	movw	r4, #:lower16:0x00000001
	str	r4, [r3]

	movw	r3, #:lower16:0x4a009468
	movt	r3, #:upper16:0x4a009468
	movw	r4, #:lower16:0x00000001
	str	r4, [r3]

	movw	r3, #:lower16:0x4a009470
	movt	r3, #:upper16:0x4a009470
	movw	r4, #:lower16:0x00000001
	str	r4, [r3]

	movw	r3, #:lower16:0x4a009478
	movt	r3, #:upper16:0x4a009478
	movw	r4, #:lower16:0x00000001
	str	r4, [r3]

	movw	r3, #:lower16:0x4a009480
	movt	r3, #:upper16:0x4a009480
	movw	r4, #:lower16:0x00000001
	str	r4, [r3]

	movw	r3, #:lower16:0x4a009488
	movt	r3, #:upper16:0x4a009488
	movw	r4, #:lower16:0x00000002
	str	r4, [r3]

	movw	r3, #:lower16:0x4a0094a0
	movt	r3, #:upper16:0x4a0094a0
	movw	r4, #:lower16:0x00000002
	str	r4, [r3]

	movw	r3, #:lower16:0x4a0094a8
	movt	r3, #:upper16:0x4a0094a8
	movw	r4, #:lower16:0x00030000
	movt	r4, #:upper16:0x00030000
	str	r4, [r3]

	movw	r3, #:lower16:0x4a0094b0
	movt	r3, #:upper16:0x4a0094b0
	movw	r4, #:lower16:0x00030000
	movt	r4, #:upper16:0x00030000
	str	r4, [r3]

	movw	r3, #:lower16:0x4a0094b8
	movt	r3, #:upper16:0x4a0094b8
	movw	r4, #:lower16:0x00030000
	movt	r4, #:upper16:0x00030000
	str	r4, [r3]

	movw	r3, #:lower16:0x4a0094e0
	movt	r3, #:upper16:0x4a0094e0
	movw	r4, #:lower16:0x00000002
	str	r4, [r3]

	movw	r3, #:lower16:0x4a0094f0
	movt	r3, #:upper16:0x4a0094f0
	movw	r4, #:lower16:0x00000002
	str	r4, [r3]

	movw	r3, #:lower16:0x4a0094f8
	movt	r3, #:upper16:0x4a0094f8
	movw	r4, #:lower16:0x00030000
	movt	r4, #:upper16:0x00030000
	str	r4, [r3]

	movw	r3, #:lower16:0x4a009500
	movt	r3, #:upper16:0x4a009500
	movw	r4, #:lower16:0x00030000
	movt	r4, #:upper16:0x00030000
	str	r4, [r3]

	movw	r3, #:lower16:0x4a009508
	movt	r3, #:upper16:0x4a009508
	movw	r4, #:lower16:0x00030000
	movt	r4, #:upper16:0x00030000
	str	r4, [r3]

	movw	r3, #:lower16:0x4a009328
	movt	r3, #:upper16:0x4a009328
	movw	r4, #:lower16:0x01040002
	movt	r4, #:upper16:0x01040002
	str	r4, [r3]

	movw	r3, #:lower16:0x4a009330
	movt	r3, #:upper16:0x4a009330
	movw	r4, #:lower16:0x01040002
	movt	r4, #:upper16:0x01040002
	str	r4, [r3]

	movw	r3, #:lower16:0x4a009520
	movt	r3, #:upper16:0x4a009520
	movw	r4, #:lower16:0x00000002
	str	r4, [r3]

	movw	r3, #:lower16:0x4a009528
	movt	r3, #:upper16:0x4a009528
	movw	r4, #:lower16:0x00000002
	str	r4, [r3]

	movw	r3, #:lower16:0x4a009560
	movt	r3, #:upper16:0x4a009560
	movw	r4, #:lower16:0x00000002
	str	r4, [r3]

	movw	r3, #:lower16:0x4a009540
	movt	r3, #:upper16:0x4a009540
	movw	r4, #:lower16:0x00000002
	str	r4, [r3]

	movw	r3, #:lower16:0x4a009548
	movt	r3, #:upper16:0x4a009548
	movw	r4, #:lower16:0x00030000
	movt	r4, #:upper16:0x00030000
	str	r4, [r3]

	movw	r3, #:lower16:0x4a009550
	movt	r3, #:upper16:0x4a009550
	movw	r4, #:lower16:0x00000002
	str	r4, [r3]

	movw	r3, #:lower16:0x4a009558
	movt	r3, #:upper16:0x4a009558
	movw	r4, #:lower16:0x00030000
	movt	r4, #:upper16:0x00030000
	str	r4, [r3]

	movw	r3, #:lower16:0x4a307838
	movt	r3, #:upper16:0x4a307838
	movw	r4, #:lower16:0x00000001
	str	r4, [r3]

	movw	r3, #:lower16:0x4a307840
	movt	r3, #:upper16:0x4a307840
	movw	r4, #:lower16:0x01000002
	movt	r4, #:upper16:0x01000002
	str	r4, [r3]

	movw	r3, #:lower16:0x4a307878
	movt	r3, #:upper16:0x4a307878
	movw	r4, #:lower16:0x00000002
	str	r4, [r3]

	movw	r3, #:lower16:0x4a008a00
	movt	r3, #:upper16:0x4a008a00
	movw	r4, #:lower16:0x00000100
	movt	r4, #:upper16:0x00000100
	str	r4, [r3]

	movw	r3, #:lower16:0x4a008b00
	movt	r3, #:upper16:0x4a008b00
	movw	r4, #:lower16:0x00000703
	movt	r4, #:upper16:0x00000703
	str	r4, [r3]

	movw	r3, #:lower16:0x4a008b30
	movt	r3, #:upper16:0x4a008b30
	movw	r4, #:lower16:0x00000001
	str	r4, [r3]

	movw	r3, #:lower16:0x4a008b38
	movt	r3, #:upper16:0x4a008b38
	movw	r4, #:lower16:0x00000001
	str	r4, [r3]

	movw	r3, #:lower16:0x4a008c00
	movt	r3, #:upper16:0x4a008c00
	movw	r4, #:lower16:0x00000003
	str	r4, [r3]

	movw	r3, #:lower16:0x4a008828
	movt	r3, #:upper16:0x4a008828
	movw	r4, #:lower16:0x00000001
	str	r4, [r3]

	movw	r3, #:lower16:0x4a008e20
	movt	r3, #:upper16:0x4a008e20
	movw	r4, #:lower16:0x00000001
	str	r4, [r3]

	movw	r3, #:lower16:0x4a008e28
	movt	r3, #:upper16:0x4a008e28
	movw	r4, #:lower16:0x00000001
	str	r4, [r3]

	movw	r3, #:lower16:0x4a008e40
	movt	r3, #:upper16:0x4a008e40
	movw	r4, #:lower16:0x00000001
	str	r4, [r3]

	movw	r3, #:lower16:0x4a307830
	movt	r3, #:upper16:0x4a307830
	movw	r4, #:lower16:0x00000002
	str	r4, [r3]

	movw	r3, #:lower16:0x4a009000
	movt	r3, #:upper16:0x4a009000
	movw	r4, #:lower16:0x00000003
	str	r4, [r3]

	movw	r3, #:lower16:0x4a009020
	movt	r3, #:upper16:0x4a009020
	movw	r4, #:lower16:0x00070000
	movt	r4, #:upper16:0x00070000
	str	r4, [r3]

	movw	r3, #:lower16:0x4a009028
	movt	r3, #:upper16:0x4a009028
	movw	r4, #:lower16:0x00070000
	movt	r4, #:upper16:0x00070000
	str	r4, [r3]

	movw	r3, #:lower16:0x4a009000
	movt	r3, #:upper16:0x4a009000
	movw	r4, #:lower16:0x00000003
	str	r4, [r3]

	movw	r3, #:lower16:0x4a009100
	movt	r3, #:upper16:0x4a009100
	movw	r4, #:lower16:0x00000f02
	movt	r4, #:upper16:0x00000f02
	str	r4, [r3]

	movw	r3, #:lower16:0x4a009120
	movt	r3, #:upper16:0x4a009120
	movw	r4, #:lower16:0x00040f02
	movt	r4, #:upper16:0x00040f02
	str	r4, [r3]

	movw	r3, #:lower16:0x4a009128
	movt	r3, #:upper16:0x4a009128
	movw	r4, #:lower16:0x00070000
	movt	r4, #:upper16:0x00070000
	str	r4, [r3]

	movw	r3, #:lower16:0x4a009100
	movt	r3, #:upper16:0x4a009100
	movw	r4, #:lower16:0x00000f02
	movt	r4, #:upper16:0x00000f02
	str	r4, [r3]

	movw	r3, #:lower16:0x4a009200
	movt	r3, #:upper16:0x4a009200
	movw	r4, #:lower16:0x00000f02
	movt	r4, #:upper16:0x00000f02
	str	r4, [r3]

	movw	r3, #:lower16:0x4a009220
	movt	r3, #:upper16:0x4a009220
	movw	r4, #:lower16:0x01040002
	movt	r4, #:upper16:0x01040002
	str	r4, [r3]

	movw	r3, #:lower16:0x4a009200
	movt	r3, #:upper16:0x4a009200
	movw	r4, #:lower16:0x00000302
	movt	r4, #:upper16:0x00000302
	str	r4, [r3]

	movw	r3, #:lower16:0x4a009338
	movt	r3, #:upper16:0x4a009338
	movw	r4, #:lower16:0x00070000
	movt	r4, #:upper16:0x00070000
	str	r4, [r3]

	movw	r3, #:lower16:0x4a009340
	movt	r3, #:upper16:0x4a009340
	movw	r4, #:lower16:0x00070000
	movt	r4, #:upper16:0x00070000
	str	r4, [r3]

	movw	r3, #:lower16:0x4a009358
	movt	r3, #:upper16:0x4a009358
	movw	r4, #:lower16:0x00070000
	movt	r4, #:upper16:0x00070000
	str	r4, [r3]

	movw	r3, #:lower16:0x4a009360
	movt	r3, #:upper16:0x4a009360
	movw	r4, #:lower16:0x01000101
	movt	r4, #:upper16:0x01000101
	str	r4, [r3]

	movw	r3, #:lower16:0x4a009368
	movt	r3, #:upper16:0x4a009368
	movw	r4, #:lower16:0x00030000
	movt	r4, #:upper16:0x00030000
	str	r4, [r3]

	movw	r3, #:lower16:0x4a0093d0
	movt	r3, #:upper16:0x4a0093d0
	movw	r4, #:lower16:0x00070000
	movt	r4, #:upper16:0x00070000
	str	r4, [r3]

	movw	r3, #:lower16:0x4a0093e0
	movt	r3, #:upper16:0x4a0093e0
	movw	r4, #:lower16:0x00070000
	movt	r4, #:upper16:0x00070000
	str	r4, [r3]

// PRM_PSCON_COUNT
	movw	r3, #:lower16:0x48243204
	movt	r3, #:upper16:0x48243204
	movw	r4, #:lower16:0x00000017
	movt	r4, #:upper16:0x00000017
	str	r4, [r3]

// PRCM CPU0 Registers:
	movw	r3, #:lower16:0x48243400
	movt	r3, #:upper16:0x48243400
	movw	r4, #:lower16:0x00030106
	movt	r4, #:upper16:0x00030106
	str	r4, [r3]

	movw	r3, #:lower16:0x48243404
	movt	r3, #:upper16:0x48243404
	movw	r4, #:lower16:0x00000037
	movt	r4, #:upper16:0x00000037
	str	r4, [r3]

	movw	r3, #:lower16:0x48243408
	movt	r3, #:upper16:0x48243408
	movw	r4, #:lower16:0x00000101
	movt	r4, #:upper16:0x00000101
	str	r4, [r3]

	movw	r3, #:lower16:0x4824340c
	movt	r3, #:upper16:0x4824340c
	movw	r4, #:lower16:0x00000000
	movt	r4, #:upper16:0x00000000
	str	r4, [r3]

	movw	r3, #:lower16:0x48243410
	movt	r3, #:upper16:0x48243410
	movw	r4, #:lower16:0x00000000
	movt	r4, #:upper16:0x00000000
	str	r4, [r3]

	movw	r3, #:lower16:0x48243414
	movt	r3, #:upper16:0x48243414
	movw	r4, #:lower16:0x00000000
	movt	r4, #:upper16:0x00000000
	str	r4, [r3]

	movw	r3, #:lower16:0x48243418
	movt	r3, #:upper16:0x48243418
	movw	r4, #:lower16:0x00000000
	movt	r4, #:upper16:0x00000000
	str	r4, [r3]

// PRCM CPU1 Registers:
	movw	r3, #:lower16:0x48243800
	movt	r3, #:upper16:0x48243800
	movw	r4, #:lower16:0x00030106
	movt	r4, #:upper16:0x00030106
	str	r4, [r3]

	movw	r3, #:lower16:0x48243804
	movt	r3, #:upper16:0x48243804
	movw	r4, #:lower16:0x00000037
	movt	r4, #:upper16:0x00000037
	str	r4, [r3]

	movw	r3, #:lower16:0x48243808
	movt	r3, #:upper16:0x48243808
	movw	r4, #:lower16:0x00000101
	movt	r4, #:upper16:0x00000101
	str	r4, [r3]

	movw	r3, #:lower16:0x4824380c
	movt	r3, #:upper16:0x4824380c
	movw	r4, #:lower16:0x00000000
	movt	r4, #:upper16:0x00000000
	str	r4, [r3]

	movw	r3, #:lower16:0x48243810
	movt	r3, #:upper16:0x48243810
	movw	r4, #:lower16:0x00000000
	movt	r4, #:upper16:0x00000000
	str	r4, [r3]

	movw	r3, #:lower16:0x48243814
	movt	r3, #:upper16:0x48243814
	movw	r4, #:lower16:0x00000000
	movt	r4, #:upper16:0x00000000
	str	r4, [r3]

	movw	r3, #:lower16:0x48243818
	movt	r3, #:upper16:0x48243818
	movw	r4, #:lower16:0x00000000
	movt	r4, #:upper16:0x00000000
	str	r4, [r3]


// CPU0 IRQ ACCESS REGISTERS: Permit access to all IRQs
	movw	r3, #:lower16:0x48281010
	movt	r3, #:upper16:0x48281010
	movw	r4, #:lower16:0xFFFFFEFF
	movt	r4, #:upper16:0xFFFFFEFF
	str	r4, [r3]

	movw	r3, #:lower16:0x48281014
	movt	r3, #:upper16:0x48281014
	movw	r4, #:lower16:0xFFFFFFFF
	movt	r4, #:upper16:0xFFFFFFFF
	str	r4, [r3]

	movw	r3, #:lower16:0x48281018
	movt	r3, #:upper16:0x48281018
	movw	r4, #:lower16:0xFFFFFFFF
	movt	r4, #:upper16:0xFFFFFFFF
	str	r4, [r3]

	movw	r3, #:lower16:0x4828101C
	movt	r3, #:upper16:0x4828101C
	movw	r4, #:lower16:0xFFFFFFFF
	movt	r4, #:upper16:0xFFFFFFFF
	str	r4, [r3]


// CPU1 IRQ ACCESS REGISTERS: Permit access to all IRQs
	movw	r3, #:lower16:0x48281410
	movt	r3, #:upper16:0x48281410
	movw	r4, #:lower16:0xFFFFFEFF
	movt	r4, #:upper16:0xFFFFFEFF
	str	r4, [r3]

	movw	r3, #:lower16:0x48281414
	movt	r3, #:upper16:0x48281414
	movw	r4, #:lower16:0xFFFFFFFF
	movt	r4, #:upper16:0xFFFFFFFF
	str	r4, [r3]

	movw	r3, #:lower16:0x48281418
	movt	r3, #:upper16:0x48281418
	movw	r4, #:lower16:0xFFFFFFFF
	movt	r4, #:upper16:0xFFFFFFFF
	str	r4, [r3]

	movw	r3, #:lower16:0x4828141C
	movt	r3, #:upper16:0x4828141C
	movw	r4, #:lower16:0xFFFFFFFF
	movt	r4, #:upper16:0xFFFFFFFF
	str	r4, [r3]


// AUX_CORE_BOOT_0: Reset
	movw	r3, #:lower16:0x48281800
	movt	r3, #:upper16:0x48281800
	movw	r4, #:lower16:0x00000000
	movt	r4, #:upper16:0x00000000
	str	r4, [r3]

// AUX_CORE_BOOT_1: Reset
	movw	r3, #:lower16:0x48281804
	movt	r3, #:upper16:0x48281804
	movw	r4, #:lower16:0x00000000
	movt	r4, #:upper16:0x00000000
	str	r4, [r3]


// Those values have been dumped on DevEd RAZR at early boot

// MPU_CM1
	movw	r3, #:lower16:0x4a004300
	movt	r3, #:upper16:0x4a004300
	movw	r4, #:lower16:0x00000103
	movt	r4, #:upper16:0x00000103
	str	r4, [r3]

	movw	r3, #:lower16:0x4a004304
	movt	r3, #:upper16:0x4a004304
	movw	r4, #:lower16:0x0000b0f0
	movt	r4, #:upper16:0x0000b0f0
	str	r4, [r3]

	movw	r3, #:lower16:0x4a004308
	movt	r3, #:upper16:0x4a004308
	movw	r4, #:lower16:0x04000038
	movt	r4, #:upper16:0x04000038
	str	r4, [r3]

	movw	r3, #:lower16:0x4a004320
	movt	r3, #:upper16:0x4a004320
	movw	r4, #:lower16:0x00000001
	movt	r4, #:upper16:0x00000001
	str	r4, [r3]


// Invalidate All SCU Registers in Secure State
	movw	r3, #:lower16:0x4824000C
	movt	r3, #:upper16:0x4824000C
	movw	r4, #:lower16:0x00000000
	movt	r4, #:upper16:0x00000000
	str	r4, [r3]

// SCU
	movw r3, #:lower16:0x48240000
	movt r3, #:upper16:0x48240000
	movw r4, #:lower16:0x00000001
	movt r4, #:upper16:0x00000001
	str r4, [r3]
	
	movw r3, #:lower16:0x48240008
	movt r3, #:upper16:0x48240008
	movw r4, #:lower16:0x03030000
	movt r4, #:upper16:0x03030000
	str r4, [r3]
	
	movw r3, #:lower16:0x48240040
	movt r3, #:upper16:0x48240040
	movw r4, #:lower16:0x00000000
	movt r4, #:upper16:0x00000000
	str r4, [r3]
	
	movw r3, #:lower16:0x48240044
	movt r3, #:upper16:0x48240044
	movw r4, #:lower16:0x00000000
	movt r4, #:upper16:0x00000000
	str r4, [r3]
	
	movw r3, #:lower16:0x48240050
	movt r3, #:upper16:0x48240050
	movw r4, #:lower16:0x0000000f
	movt r4, #:upper16:0x0000000f
	str r4, [r3]
	
	movw r3, #:lower16:0x48240054
	movt r3, #:upper16:0x48240054
	movw r4, #:lower16:0x00000ff0
	movt r4, #:upper16:0x00000ff0
	str r4, [r3]
	
//GIC_Intr_Distributor:
	movw r3, #:lower16:0x48241000
	movt r3, #:upper16:0x48241000
	movw r4, #:lower16:0x00000001
	movt r4, #:upper16:0x00000001
	str r4, [r3]
	
	movw r3, #:lower16:0x48241080
	movt r3, #:upper16:0x48241080
	movw r4, #:lower16:0x00000000
	movt r4, #:upper16:0x00000000
	str r4, [r3]
	
	movw r3, #:lower16:0x48241084
	movt r3, #:upper16:0x48241084
	movw r4, #:lower16:0x00000000
	movt r4, #:upper16:0x00000000
	str r4, [r3]
	
	movw r3, #:lower16:0x48241088
	movt r3, #:upper16:0x48241088
	movw r4, #:lower16:0x00000000
	movt r4, #:upper16:0x00000000
	str r4, [r3]
	
	movw r3, #:lower16:0x4824108c
	movt r3, #:upper16:0x4824108c
	movw r4, #:lower16:0x00000000
	movt r4, #:upper16:0x00000000
	str r4, [r3]
	
	movw r3, #:lower16:0x48241090
	movt r3, #:upper16:0x48241090
	movw r4, #:lower16:0x00000000
	movt r4, #:upper16:0x00000000
	str r4, [r3]
	
	movw r3, #:lower16:0x48241094
	movt r3, #:upper16:0x48241094
	movw r4, #:lower16:0x00000000
	movt r4, #:upper16:0x00000000
	str r4, [r3]
	
	movw r3, #:lower16:0x48241098
	movt r3, #:upper16:0x48241098
	movw r4, #:lower16:0x00000000
	movt r4, #:upper16:0x00000000
	str r4, [r3]
	
	movw r3, #:lower16:0x4824109c
	movt r3, #:upper16:0x4824109c
	movw r4, #:lower16:0x00000000
	movt r4, #:upper16:0x00000000
	str r4, [r3]
	
	movw r3, #:lower16:0x48241100
	movt r3, #:upper16:0x48241100
	movw r4, #:lower16:0x0000ffff
	movt r4, #:upper16:0x0000ffff
	str r4, [r3]
	
	movw r3, #:lower16:0x48241104
	movt r3, #:upper16:0x48241104
	movw r4, #:lower16:0x00000000
	movt r4, #:upper16:0x00000000
	str r4, [r3]
	
	movw r3, #:lower16:0x48241108
	movt r3, #:upper16:0x48241108
	movw r4, #:lower16:0x00000020
	movt r4, #:upper16:0x00000020
	str r4, [r3]
	
	movw r3, #:lower16:0x4824110c
	movt r3, #:upper16:0x4824110c
	movw r4, #:lower16:0x00000000
	movt r4, #:upper16:0x00000000
	str r4, [r3]
	
	movw r3, #:lower16:0x48241110
	movt r3, #:upper16:0x48241110
	movw r4, #:lower16:0x00000000
	movt r4, #:upper16:0x00000000
	str r4, [r3]
	
	movw r3, #:lower16:0x48241114
	movt r3, #:upper16:0x48241114
	movw r4, #:lower16:0x00000000
	movt r4, #:upper16:0x00000000
	str r4, [r3]
	
	movw r3, #:lower16:0x48241118
	movt r3, #:upper16:0x48241118
	movw r4, #:lower16:0x00000000
	movt r4, #:upper16:0x00000000
	str r4, [r3]
	
	movw r3, #:lower16:0x4824111c
	movt r3, #:upper16:0x4824111c
	movw r4, #:lower16:0x00000000
	movt r4, #:upper16:0x00000000
	str r4, [r3]
	
	movw r3, #:lower16:0x48241180
	movt r3, #:upper16:0x48241180
	movw r4, #:lower16:0x0000ffff
	movt r4, #:upper16:0x0000ffff
	str r4, [r3]
	
	movw r3, #:lower16:0x48241184
	movt r3, #:upper16:0x48241184
	movw r4, #:lower16:0x00000000
	movt r4, #:upper16:0x00000000
	str r4, [r3]
	
	movw r3, #:lower16:0x48241188
	movt r3, #:upper16:0x48241188
	movw r4, #:lower16:0x00000020
	movt r4, #:upper16:0x00000020
	str r4, [r3]
	
	movw r3, #:lower16:0x4824118c
	movt r3, #:upper16:0x4824118c
	movw r4, #:lower16:0x00000000
	movt r4, #:upper16:0x00000000
	str r4, [r3]
	
	movw r3, #:lower16:0x48241190
	movt r3, #:upper16:0x48241190
	movw r4, #:lower16:0x00000000
	movt r4, #:upper16:0x00000000
	str r4, [r3]
	
	movw r3, #:lower16:0x48241194
	movt r3, #:upper16:0x48241194
	movw r4, #:lower16:0x00000000
	movt r4, #:upper16:0x00000000
	str r4, [r3]
	
	movw r3, #:lower16:0x48241198
	movt r3, #:upper16:0x48241198
	movw r4, #:lower16:0x00000000
	movt r4, #:upper16:0x00000000
	str r4, [r3]
	
	movw r3, #:lower16:0x4824119c
	movt r3, #:upper16:0x4824119c
	movw r4, #:lower16:0x00000000
	movt r4, #:upper16:0x00000000
	str r4, [r3]
	
	movw r3, #:lower16:0x48241200
	movt r3, #:upper16:0x48241200
	movw r4, #:lower16:0x00000000
	movt r4, #:upper16:0x00000000
	str r4, [r3]
	
	movw r3, #:lower16:0x48241204
	movt r3, #:upper16:0x48241204
	movw r4, #:lower16:0x00000100
	movt r4, #:upper16:0x00000100
	str r4, [r3]
	
	movw r3, #:lower16:0x48241208
	movt r3, #:upper16:0x48241208
	movw r4, #:lower16:0x00200020
	movt r4, #:upper16:0x00200020
	str r4, [r3]
	
	movw r3, #:lower16:0x4824120c
	movt r3, #:upper16:0x4824120c
	movw r4, #:lower16:0x00000000
	movt r4, #:upper16:0x00000000
	str r4, [r3]
	
	movw r3, #:lower16:0x48241210
	movt r3, #:upper16:0x48241210
	movw r4, #:lower16:0x00000000
	movt r4, #:upper16:0x00000000
	str r4, [r3]
	
	movw r3, #:lower16:0x48241214
	movt r3, #:upper16:0x48241214
	movw r4, #:lower16:0x00000000
	movt r4, #:upper16:0x00000000
	str r4, [r3]
	
	movw r3, #:lower16:0x48241218
	movt r3, #:upper16:0x48241218
	movw r4, #:lower16:0x00000000
	movt r4, #:upper16:0x00000000
	str r4, [r3]
	
	movw r3, #:lower16:0x4824121c
	movt r3, #:upper16:0x4824121c
	movw r4, #:lower16:0x00000000
	movt r4, #:upper16:0x00000000
	str r4, [r3]
	
	movw r3, #:lower16:0x48241220
	movt r3, #:upper16:0x48241220
	movw r4, #:lower16:0x00000000
	movt r4, #:upper16:0x00000000
	str r4, [r3]
	
	movw r3, #:lower16:0x48241224
	movt r3, #:upper16:0x48241224
	movw r4, #:lower16:0x00000000
	movt r4, #:upper16:0x00000000
	str r4, [r3]
	
	movw r3, #:lower16:0x48241228
	movt r3, #:upper16:0x48241228
	movw r4, #:lower16:0x00000000
	movt r4, #:upper16:0x00000000
	str r4, [r3]
	
	movw r3, #:lower16:0x4824122c
	movt r3, #:upper16:0x4824122c
	movw r4, #:lower16:0x00000000
	movt r4, #:upper16:0x00000000
	str r4, [r3]
	
	movw r3, #:lower16:0x48241230
	movt r3, #:upper16:0x48241230
	movw r4, #:lower16:0x00000000
	movt r4, #:upper16:0x00000000
	str r4, [r3]
	
	movw r3, #:lower16:0x48241234
	movt r3, #:upper16:0x48241234
	movw r4, #:lower16:0x00000000
	movt r4, #:upper16:0x00000000
	str r4, [r3]
	
	movw r3, #:lower16:0x48241238
	movt r3, #:upper16:0x48241238
	movw r4, #:lower16:0x00000000
	movt r4, #:upper16:0x00000000
	str r4, [r3]
	
	movw r3, #:lower16:0x4824123c
	movt r3, #:upper16:0x4824123c
	movw r4, #:lower16:0x00000000
	movt r4, #:upper16:0x00000000
	str r4, [r3]
	
	movw r3, #:lower16:0x48241280
	movt r3, #:upper16:0x48241280
	movw r4, #:lower16:0x00000000
	movt r4, #:upper16:0x00000000
	str r4, [r3]
	
	movw r3, #:lower16:0x48241284
	movt r3, #:upper16:0x48241284
	movw r4, #:lower16:0x00000100
	movt r4, #:upper16:0x00000100
	str r4, [r3]
	
	movw r3, #:lower16:0x48241288
	movt r3, #:upper16:0x48241288
	movw r4, #:lower16:0x00200020
	movt r4, #:upper16:0x00200020
	str r4, [r3]
	
	movw r3, #:lower16:0x4824128c
	movt r3, #:upper16:0x4824128c
	movw r4, #:lower16:0x00000000
	movt r4, #:upper16:0x00000000
	str r4, [r3]
	
	movw r3, #:lower16:0x48241290
	movt r3, #:upper16:0x48241290
	movw r4, #:lower16:0x00000000
	movt r4, #:upper16:0x00000000
	str r4, [r3]
	
	movw r3, #:lower16:0x48241294
	movt r3, #:upper16:0x48241294
	movw r4, #:lower16:0x00000000
	movt r4, #:upper16:0x00000000
	str r4, [r3]
	
	movw r3, #:lower16:0x48241298
	movt r3, #:upper16:0x48241298
	movw r4, #:lower16:0x00000000
	movt r4, #:upper16:0x00000000
	str r4, [r3]
	
	movw r3, #:lower16:0x4824129c
	movt r3, #:upper16:0x4824129c
	movw r4, #:lower16:0x00000000
	movt r4, #:upper16:0x00000000
	str r4, [r3]
	
	movw r3, #:lower16:0x48241c00
	movt r3, #:upper16:0x48241c00
	movw r4, #:lower16:0xaaaaaaaa
	movt r4, #:upper16:0xaaaaaaaa
	str r4, [r3]
	
	movw r3, #:lower16:0x48241c04
	movt r3, #:upper16:0x48241c04
	movw r4, #:lower16:0x7dc00000
	movt r4, #:upper16:0x7dc00000
	str r4, [r3]
	
	movw r3, #:lower16:0x48241c08
	movt r3, #:upper16:0x48241c08
	movw r4, #:lower16:0x15555555
	movt r4, #:upper16:0x15555555
	str r4, [r3]
	
	movw r3, #:lower16:0x48241c10
	movt r3, #:upper16:0x48241c10
	movw r4, #:lower16:0x55555555
	movt r4, #:upper16:0x55555555
	str r4, [r3]
	
	movw r3, #:lower16:0x48241c14
	movt r3, #:upper16:0x48241c14
	movw r4, #:lower16:0x55555555
	movt r4, #:upper16:0x55555555
	str r4, [r3]
	
	movw r3, #:lower16:0x48241c18
	movt r3, #:upper16:0x48241c18
	movw r4, #:lower16:0x55555555
	movt r4, #:upper16:0x55555555
	str r4, [r3]
	
	movw r3, #:lower16:0x48241c1c
	movt r3, #:upper16:0x48241c1c
	movw r4, #:lower16:0x55555555
	movt r4, #:upper16:0x55555555
	str r4, [r3]
	
	movw r3, #:lower16:0x48241c20
	movt r3, #:upper16:0x48241c20
	movw r4, #:lower16:0x55555555
	movt r4, #:upper16:0x55555555
	str r4, [r3]
	
	movw r3, #:lower16:0x48241c24
	movt r3, #:upper16:0x48241c24
	movw r4, #:lower16:0x55554555
	movt r4, #:upper16:0x55554555
	str r4, [r3]
	
	movw r3, #:lower16:0x48241c28
	movt r3, #:upper16:0x48241c28
	movw r4, #:lower16:0x00000000
	movt r4, #:upper16:0x00000000
	str r4, [r3]
	
	movw r3, #:lower16:0x48241c2c
	movt r3, #:upper16:0x48241c2c
	movw r4, #:lower16:0x00000000
	movt r4, #:upper16:0x00000000
	str r4, [r3]
	
	movw r3, #:lower16:0x48241c30
	movt r3, #:upper16:0x48241c30
	movw r4, #:lower16:0x00000000
	movt r4, #:upper16:0x00000000
	str r4, [r3]
	
	movw r3, #:lower16:0x48241c34
	movt r3, #:upper16:0x48241c34
	movw r4, #:lower16:0x00000000
	movt r4, #:upper16:0x00000000
	str r4, [r3]
	
	movw r3, #:lower16:0x48241c38
	movt r3, #:upper16:0x48241c38
	movw r4, #:lower16:0x00000000
	movt r4, #:upper16:0x00000000
	str r4, [r3]
	
	movw r3, #:lower16:0x48241c3c
	movt r3, #:upper16:0x48241c3c
	movw r4, #:lower16:0x00000000
	movt r4, #:upper16:0x00000000
	str r4, [r3]
	
//GIC_Proc_Interface (GIC CPU Interface):
	movw r3, #:lower16:0x48240100
	movt r3, #:upper16:0x48240100
	movw r4, #:lower16:0x00000001
	movt r4, #:upper16:0x00000001
	str r4, [r3]
	
	movw r3, #:lower16:0x48240104
	movt r3, #:upper16:0x48240104
	movw r4, #:lower16:0x000000f0
	movt r4, #:upper16:0x000000f0
	str r4, [r3]
	
	movw r3, #:lower16:0x48240108
	movt r3, #:upper16:0x48240108
	movw r4, #:lower16:0x00000003
	movt r4, #:upper16:0x00000003
	str r4, [r3]
	
	movw r3, #:lower16:0x4824011c
	movt r3, #:upper16:0x4824011c
	movw r4, #:lower16:0x00000000
	movt r4, #:upper16:0x00000000
	str r4, [r3]
	
//Timers interface:
	movw r3, #:lower16:0x48240600
	movt r3, #:upper16:0x48240600
	movw r4, #:lower16:0x00000000
	movt r4, #:upper16:0x00000000
	str r4, [r3]
	
	movw r3, #:lower16:0x48240604
	movt r3, #:upper16:0x48240604
	movw r4, #:lower16:0x00000000
	movt r4, #:upper16:0x00000000
	str r4, [r3]
	
	movw r3, #:lower16:0x48240608
	movt r3, #:upper16:0x48240608
	movw r4, #:lower16:0x00000000
	movt r4, #:upper16:0x00000000
	str r4, [r3]
	
	movw r3, #:lower16:0x4824060c
	movt r3, #:upper16:0x4824060c
	movw r4, #:lower16:0x00000000
	movt r4, #:upper16:0x00000000
	str r4, [r3]
	
	movw r3, #:lower16:0x48240620
	movt r3, #:upper16:0x48240620
	movw r4, #:lower16:0x00000000
	movt r4, #:upper16:0x00000000
	str r4, [r3]
	
	movw r3, #:lower16:0x48240624
	movt r3, #:upper16:0x48240624
	movw r4, #:lower16:0x00000000
	movt r4, #:upper16:0x00000000
	str r4, [r3]
	
	movw r3, #:lower16:0x48240628
	movt r3, #:upper16:0x48240628
	movw r4, #:lower16:0x00000000
	movt r4, #:upper16:0x00000000
	str r4, [r3]
	
	movw r3, #:lower16:0x4824062c
	movt r3, #:upper16:0x4824062c
	movw r4, #:lower16:0x00000000
	movt r4, #:upper16:0x00000000
	str r4, [r3]
	
	movw r3, #:lower16:0x48240630
	movt r3, #:upper16:0x48240630
	movw r4, #:lower16:0x00000000
	movt r4, #:upper16:0x00000000
	str r4, [r3]

	@ gpmc
	movw	r3, #:lower16:0x4a008828
	movt	r3, #:upper16:0x4a008828
	movw	r4, #:lower16:0x00000001
	movt	r4, #:upper16:0x00000001
	str	r4, [r3]

	@ pulse
//	movw	r3, #:lower16:0x4a310000
//	movt	r3, #:upper16:0x4a310000
//	mov	r4, #0x10000000
//	str	r4, [r3, #0x190]
//	str	r4, [r3, #0x194]

	mov pc,lr

	.globl kexec_start_address
kexec_start_address:
	.long	0x0

	.globl kexec_indirection_page
kexec_indirection_page:
	.long	0x0

	.globl kexec_mach_type
kexec_mach_type:
	.long	0x0

	/* phy addr of the atags for the new kernel */
	.globl kexec_boot_atags
kexec_boot_atags:
	.long	0x0

relocate_new_kernel_end:

	.globl relocate_new_kernel_size
relocate_new_kernel_size:
	.long relocate_new_kernel_end - relocate_new_kernel


