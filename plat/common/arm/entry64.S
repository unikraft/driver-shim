/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Authors: Wei Chen <wei.chen@arm.com>
 *
 * Copyright (c) 2018, Arm Ltd. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * THIS HEADER MAY NOT BE EXTRACTED OR MODIFIED IN ANY WAY.
 */
#include <uk/config.h>
#include <uk/arch/limits.h>
#include <uk/asm.h>
#include <arm/mm.h>
#include <arm/cpu_defs.h>
#include <uk/plat/common/sections.h>
#include <uk/config.h>

.global page_table_size
.data
page_table_size:
	.dword 0x0

.global boot_ticks
boot_ticks:
	.dword 0x0

/*
 * The registers used by _libplat_start:
 * x0 - FDT pointer
 */

.text
ENTRY(_libplat_entry)
#ifdef CONFIG_FPSIMD
	/* Enable fp/simd support */
	ldr        x0, =(3 << 20)
	msr        cpacr_el1, x0
	isb
#endif
	bl drop_from_el3

#ifdef CONFIG_ARM64_EARLY_BOOTTICKS
	ldr x24, =boot_ticks
	mrs x23, cntvct_el0
	str x23, [x24]
#endif /* CONFIG_ARM64_EARLY_BOOTTICKS */

	/* Calculate the image size */
	ldr x25, =_dtb
	ldr x26, =_end
	mov x15, x25
	sub x15, x26, x15

	/* Round up the size to 2MB */
	mov x17, #(IMAGE_ROUNDUP_SIZE - 1)
	add x15, x15, x17
	lsr x15, x15, #IMAGE_ROUNDUP_SHIFT
	lsl x15, x15, #IMAGE_ROUNDUP_SHIFT

	/*
	 * How many bytes would be used for L3_TABLE
	 * ((x15 >> 21) << 12)
	 */
#ifdef CONFIG_PLAT_KVM
	lsr x17, x15, #9
#else
	bl fetch_p3_table_size
#endif

	/* Total bytes for pagetable */
	add x17, x17, #L0_TABLE_SIZE
	add x17, x17, #L1_TABLE_SIZE
	add x17, x17, #L2_TABLE_SIZE

	/*
	 * We will disable MMU and CACHE before pagetable is ready. This
	 * means we will change memory with cache disabled, so we need to
	 * invalidate the cache to ensure there is no stall data in it.
	 * But we don't know the size of the RAM either. And it would be
	 * expensive to invalidate the whole cache. In this case, just
	 * just need to invalidate what we are going to use:
	 * DTB, TEXT, DATA, BSS, pagetables and bootstack.
	 */
	add x27, x26, x17
#ifdef CONFIG_PLAT_KVM
	add x27, x27, #__STACK_SIZE
#else
	bl get_stack_size
#endif
	sub x1, x27, x25
	bl clean_and_invalidate_dcache_range

	/* Disable the MMU and D-Cache. */
	dsb sy
	mrs x2, sctlr_el1
	mov x3, #SCTLR_M|SCTLR_C
	bic x2, x2, x3
	msr sctlr_el1, x2
	isb

	/*
	 * Clean the boot stack and page table. As _end, PAGE_TABLE_SIZE and
	 * BOOT_STACK_SIZE are page_size alignment, the boot stack can be
	 * 64-bytes alignment too. Execute 4 stp consecutively without boundary
	 * check would be safe here.
	 */
#ifdef CONFIG_PLAT_KVM
1:
	stp xzr, xzr, [x26], #16
	stp xzr, xzr, [x26], #16
	stp xzr, xzr, [x26], #16
	stp xzr, xzr, [x26], #16
	cmp x26, x27
	b.lo 1b

	mov sp, x27
#endif

	/* Set the context id */
	msr contextidr_el1, xzr

#ifndef CONFIG_PLAT_KVM
	ldr x25, =__sbss_start
	ldr x26, =_end
2:
	stp xzr, xzr, [x25], #16
	stp xzr, xzr, [x25], #16
	stp xzr, xzr, [x25], #16
	stp xzr, xzr, [x25], #16
	cmp x25, x26
	b.lo 2b
#endif

	/* Save page table size for later usage */
	ldr x26, =page_table_size
	str x17, [x26]

	/* Create a pagetable to do PA == VA mapping */
	bl create_pagetables

#ifndef CONFIG_PLAT_KVM
	bl get_stack_ptr
	mov sp, x18
#endif

	/* Setup exception vector table address before enable MMU */
	ldr x29, =vector_table
	msr VBAR_EL1, x29

	/* Enable the mmu */
	bl start_mmu

	/* Load dtb address to x0 as a parameter */
	ldr x0, =_dtb
	b _libplat_start
END(_libplat_entry)

ENTRY(_libplat_newstack)
	/* Setup new stack */
	mov sp, x0

	/* Setup parameter for _libplat_entry2 */
	mov x0, x2

	/* Branch to _libplat_entry2 */
	br x1
END(_libplat_newstack)

/*
 * If we are started in EL2, configure the required hypervisor
 * registers and drop to EL1.
 */
drop_to_el1:
    mrs x1, CurrentEL
	cmp x1, #0xC
	b.eq drop_from_el3
    cmp x1, #0x4
    b.eq drop_from_el2 
    ret

drop_from_el2:
    /* Configure the Hypervisor */
    mov x2, #(HCR_RW)
    msr hcr_el2, x2
    /* Load the Virtualization Process ID Register */
    mrs x2, midr_el1
    msr vpidr_el2, x2
    /* Load the Virtualization Multiprocess ID Register */
    mrs x2, mpidr_el1
    msr vmpidr_el2, x2
    /* Set the bits that need to be 1 in sctlr_el1 */
    ldr x2, .Lsctlr_res1
    msr sctlr_el1, x2
    /* Don't trap to EL2 for exceptions */
    mov x2, #CPTR_RES1
    msr cptr_el2, x2
    /* Don't trap to EL2 for CP15 traps */
    msr hstr_el2, xzr
	mov x3, #(CNTHCTL_EL1PCTEN | CNTHCTL_EL1PCEN)
    /* Enable access to the physical timers at EL1 */
    mrs x2, cnthctl_el2
    orr x2, x2, x3
    msr cnthctl_el2, x2
    /* Set the counter offset to a known value */
    msr cntvoff_el2, xzr
    /* Hypervisor trap functions */
    adr x2, hyp_vectors
    msr vbar_el2, x2
    mov x2, #(PSR_F | PSR_I | PSR_A | PSR_D | PSR_M_EL1h)
    msr spsr_el2, x2
#ifdef CONFIG_ARM_GIC_V3
    /* Configure GICv3 CPU interface */
    mrs x2, id_aa64pfr0_el1
    /* Extract GIC bits from the register */
    ubfx    x2, x2, #ID_AA64PFR0_GIC_SHIFT, #ID_AA64PFR0_GIC_BITS
    /* GIC[3:0] == 0001 - GIC CPU interface via special regs. supported */
    cmp x2, #(ID_AA64PFR0_GIC_CPUIF_EN >> ID_AA64PFR0_GIC_SHIFT)
    b.ne    2f
    mrs x2, icc_sre_el2
    orr x2, x2, #ICC_SRE_EL2_EN /* Enable access from insecure EL1 */
    orr x2, x2, #ICC_SRE_EL2_SRE    /* Enable system registers */
    msr icc_sre_el2, x2
2:
#endif /* CONFIG_ARM_GIC_V3 */
    /* Set the address to return to our return address */
    msr elr_el2, x30
    isb
    eret

drop_from_el3:
	mrs x0, scr_el3
    /* Lower EL is 64bits */
 	orr x0, x0, #(1 << 1)   // IRQ=1 IRQs routed to EL3
 	orr x0, x0, #(1 << 2)   // FIQ=1 FIQs routed to EL3
 	orr x0, x0, #(1 << 3)   // EA=1  SError routed to EL3
 	orr x0, x0, #(1 << 8)   // HCE=1 HVC instructions are enabled
 	orr x0, x0, #(1 << 10)  // RW=1  Next EL down uses AArch64
 	orr x0, x0, #(1 << 11)  // ST=1  Secure EL1 can access timers
	msr scr_el3, x0

	/* Configure the spsr_el3 */
	mov x0, xzr
	mov x0, #(PSR_F | PSR_I | PSR_A | PSR_D | PSR_M_EL1h)
	msr spsr_el3, x0

	msr elr_el3, x30
	eret

.Lsctlr_res1:
	.quad SCTLR_RES1

#define VECT_EMPTY  \
    .align 7;   \
    1:  b   1b


    .align 11
hyp_vectors:
    VECT_EMPTY  /* Synchronous EL2t */
    VECT_EMPTY  /* IRQ EL2t */
    VECT_EMPTY  /* FIQ EL2t */
    VECT_EMPTY  /* Error EL2t */
    VECT_EMPTY  /* Synchronous EL2h */
    VECT_EMPTY  /* IRQ EL2h */
    VECT_EMPTY  /* FIQ EL2h */
    VECT_EMPTY  /* Error EL2h */
    VECT_EMPTY  /* Synchronous 64-bit EL1 */
    VECT_EMPTY  /* IRQ 64-bit EL1 */
    VECT_EMPTY  /* FIQ 64-bit EL1 */
    VECT_EMPTY  /* Error 64-bit EL1 */
    VECT_EMPTY  /* Synchronous 32-bit EL1 */
    VECT_EMPTY  /* IRQ 32-bit EL1 */
    VECT_EMPTY  /* FIQ 32-bit EL1 */
    VECT_EMPTY  /* Error 32-bit EL1 */
