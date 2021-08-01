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
#ifndef __CPU_ARM_64_DEFS_H__
#define __CPU_ARM_64_DEFS_H__

#define MRS_REG(op0, op1, crn, crm, op2)                \
    (((op0) << MRS_Op0_SHIFT) | ((op1) << MRS_Op1_SHIFT) |      \
     ((crn) << MRS_CRn_SHIFT) | ((crm) << MRS_CRm_SHIFT) |      \
     ((op2) << MRS_Op2_SHIFT))

#define TLBI_REG(reg) 

/* CNTHCTL_EL2 - Counter-timer Hypervisor Control register */
#define CNTHCTL_EVNTI_MASK  (0xf << 4) /* Bit to trigger event stream */
#define CNTHCTL_EVNTDIR     (1 << 3) /* Control transition trigger bit */
#define CNTHCTL_EVNTEN      (1 << 2) /* Enable event stream */
#define CNTHCTL_EL1PCEN     (1 << 1) /* Allow EL0/1 physical timer access */
#define CNTHCTL_EL1PCTEN    (1 << 0) /*Allow EL0/1 physical counter access*/

/*
 * Architecture feature trap register
 */
#define CPTR_RES0   0x7fefc800
#define CPTR_RES1   0x000033ff
#define CPTR_TFP    0x00000400
#define CPTR_TTA    0x00100000
#define CPTR_TCPAC  0x80000000

/*
 * Secure Configuration register
 */
#define SCR_NS	    (_AC(1,UL) << 0)
#define SCR_IRQ     (_AC(1,UL) << 1)
#define SCR_FIQ     (_AC(1,UL) << 2)
#define SCR_EA      (_AC(1,UL) << 3)
#define SCR_SMD     (_AC(1,UL) << 7)
#define SCR_HCE     (_AC(1,UL) << 8)
#define SCR_SIF     (_AC(1,UL) << 9)
#define SCR_RW	    (_AC(1,UL) << 10)

/* Hyp Configuration Register (HCR) bits */
#define HCR_FWB     (_AC(1,UL) << 46)
#define HCR_API     (_AC(1,UL) << 41)
#define HCR_APK     (_AC(1,UL) << 40)
#define HCR_TEA     (_AC(1,UL) << 37)
#define HCR_TERR    (_AC(1,UL) << 36)
#define HCR_TLOR    (_AC(1,UL) << 35)
#define HCR_E2H     (_AC(1,UL) << 34)
#define HCR_ID      (_AC(1,UL) << 33)
#define HCR_CD      (_AC(1,UL) << 32)
#define HCR_RW_SHIFT    31
#define HCR_RW      (_AC(1,UL) << HCR_RW_SHIFT)
#define HCR_TRVM    (_AC(1,UL) << 30)
#define HCR_HCD     (_AC(1,UL) << 29)
#define HCR_TDZ     (_AC(1,UL) << 28)
#define HCR_TGE     (_AC(1,UL) << 27)
#define HCR_TVM     (_AC(1,UL) << 26)
#define HCR_TTLB    (_AC(1,UL) << 25)
#define HCR_TPU     (_AC(1,UL) << 24)
#define HCR_TPC     (_AC(1,UL) << 23)
#define HCR_TSW     (_AC(1,UL) << 22)
#define HCR_TAC     (_AC(1,UL) << 21)
#define HCR_TIDCP   (_AC(1,UL) << 20)
#define HCR_TSC     (_AC(1,UL) << 19)
#define HCR_TID3    (_AC(1,UL) << 18)
#define HCR_TID2    (_AC(1,UL) << 17)
#define HCR_TID1    (_AC(1,UL) << 16)
#define HCR_TID0    (_AC(1,UL) << 15)
#define HCR_TWE     (_AC(1,UL) << 14)
#define HCR_TWI     (_AC(1,UL) << 13)
#define HCR_DC      (_AC(1,UL) << 12)
#define HCR_BSU     (3 << 10)
#define HCR_BSU_IS  (_AC(1,UL << 10)
#define HCR_FB      (_AC(1,UL << 9)
#define HCR_VSE     (_AC(1,UL << 8)
#define HCR_VI      (_AC(1,UL << 7)
#define HCR_VF      (_AC(1,UL << 6)
#define HCR_AMO     (_AC(1,UL << 5)
#define HCR_IMO     (_AC(1,UL << 4)
#define HCR_FMO     (_AC(1,UL << 3)
#define HCR_PTW     (_AC(1,UL << 2)
#define HCR_SWIO    (_AC(1,UL << 1)
#define HCR_VM      (_AC(1,UL << 0)
/*
 * Power State Coordination Interface (PSCI v0.2) function codes
 */
#define PSCI_FNID_VERSION		0x84000000
#define PSCI_FNID_CPU_SUSPEND		0xc4000001
#define PSCI_FNID_CPU_OFF		0x84000002
#define PSCI_FNID_CPU_ON		0xc4000003
#define PSCI_FNID_AFFINITY_INFO		0xc4000004
#define PSCI_FNID_MIGRATE		0xc4000005
#define PSCI_FNID_MIGRATE_INFO_TYPE	0x84000006
#define PSCI_FNID_MIGRATE_INFO_UP_CPU	0xc4000007
#define PSCI_FNID_SYSTEM_OFF		0x84000008
#define PSCI_FNID_SYSTEM_RESET		0x84000009

/*
 * CTR_EL0, Cache Type Register
 * Provides information about the architecture of the caches.
 */
#define CTR_DMINLINE_SHIFT	16
#define CTR_DMINLINE_WIDTH	4
#define CTR_IMINLINE_MASK	0xf
#define CTR_BYTES_PER_WORD	4

/* Registers and Bits definitions for MMU */
/* MAIR_EL1 - Memory Attribute Indirection Register */
#define MAIR_ATTR_MASK(idx)	(0xff << ((n)* 8))
#define MAIR_ATTR(attr, idx)	((attr) << ((idx) * 8))

/* Device-nGnRnE memory */
#define MAIR_DEVICE_nGnRnE	0x00
/* Device-nGnRE memory */
#define MAIR_DEVICE_nGnRE	0x04
/* Device-GRE memory */
#define MAIR_DEVICE_GRE		0x0C
/* Outer Non-cacheable + Inner Non-cacheable */
#define MAIR_NORMAL_NC		0x44
/* Outer + Inner Write-through non-transient */
#define MAIR_NORMAL_WT		0xbb
/* Outer + Inner Write-back non-transient */
#define MAIR_NORMAL_WB		0xff

/*
 * Memory types, these values are the indexs of the attributes
 * that defined in MAIR_EL1.
 */
#define DEVICE_nGnRnE	0
#define DEVICE_nGnRE	1
#define DEVICE_GRE	2
#define NORMAL_NC	3
#define NORMAL_WT	4
#define NORMAL_WB	5

#define MAIR_INIT_ATTR	\
		(MAIR_ATTR(MAIR_DEVICE_nGnRnE, DEVICE_nGnRnE) | \
		MAIR_ATTR(MAIR_DEVICE_nGnRE, DEVICE_nGnRE) |   \
		MAIR_ATTR(MAIR_DEVICE_GRE, DEVICE_GRE) |       \
		MAIR_ATTR(MAIR_NORMAL_NC, NORMAL_NC) |         \
		MAIR_ATTR(MAIR_NORMAL_WB, NORMAL_WT) |         \
		MAIR_ATTR(MAIR_NORMAL_WT, NORMAL_WB))

/* TCR_EL1 - Translation Control Register */
#define TCR_ASID_16	(1 << 36)

#define TCR_IPS_SHIFT	32
#define TCR_IPS_32BIT	(0 << TCR_IPS_SHIFT)
#define TCR_IPS_36BIT	(1 << TCR_IPS_SHIFT)
#define TCR_IPS_40BIT	(2 << TCR_IPS_SHIFT)
#define TCR_IPS_42BIT	(3 << TCR_IPS_SHIFT)
#define TCR_IPS_44BIT	(4 << TCR_IPS_SHIFT)
#define TCR_IPS_48BIT	(5 << TCR_IPS_SHIFT)

#define TCR_TG1_SHIFT	30
#define TCR_TG1_16K	(1 << TCR_TG1_SHIFT)
#define TCR_TG1_4K	(2 << TCR_TG1_SHIFT)
#define TCR_TG1_64K	(3 << TCR_TG1_SHIFT)

#define TCR_TG0_SHIFT	14
#define TCR_TG0_4K	(0 << TCR_TG0_SHIFT)
#define TCR_TG0_64K	(1 << TCR_TG0_SHIFT)
#define TCR_TG0_16K	(2 << TCR_TG0_SHIFT)

#define TCR_SH1_SHIFT	28
#define TCR_SH1_IS	(0x3 << TCR_SH1_SHIFT)
#define TCR_ORGN1_SHIFT	26
#define TCR_ORGN1_WBWA	(0x1 << TCR_ORGN1_SHIFT)
#define TCR_IRGN1_SHIFT	24
#define TCR_IRGN1_WBWA	(0x1 << TCR_IRGN1_SHIFT)
#define TCR_SH0_SHIFT	12
#define TCR_SH0_IS	(0x3 << TCR_SH0_SHIFT)
#define TCR_ORGN0_SHIFT	10
#define TCR_ORGN0_WBWA	(0x1 << TCR_ORGN0_SHIFT)
#define TCR_IRGN0_SHIFT	8
#define TCR_IRGN0_WBWA	(0x1 << TCR_IRGN0_SHIFT)

#define TCR_CACHE_ATTRS ((TCR_IRGN0_WBWA | TCR_IRGN1_WBWA) | \
			(TCR_ORGN0_WBWA | TCR_ORGN1_WBWA))

#define TCR_SMP_ATTRS	(TCR_SH0_IS | TCR_SH1_IS)

#define TCR_T1SZ_SHIFT	16
#define TCR_T0SZ_SHIFT	0
#define TCR_T1SZ(x)	((x) << TCR_T1SZ_SHIFT)
#define TCR_T0SZ(x)	((x) << TCR_T0SZ_SHIFT)
#define TCR_TxSZ(x)	(TCR_T1SZ(x) | TCR_T0SZ(x))

/*
 * As we use VA == PA mapping, so the VIRT_BITS must be the same
 * as PA_BITS. We can get PA_BITS from ID_AA64MMFR0_EL1.PARange.
 * So the TxSZ will be calculate dynamically.
 */
#define TCR_INIT_FLAGS	(TCR_ASID_16 | TCR_TG0_4K | \
			TCR_CACHE_ATTRS | TCR_SMP_ATTRS)

/* SCTLR_EL1 - System Control Register */
#define SCTLR_M		(_AC(1, UL) << 0)	/* MMU enable */
#define SCTLR_A		(_AC(1, UL) << 1)	/* Alignment check enable */
#define SCTLR_C		(_AC(1, UL) << 2)	/* Data/unified cache enable */
#define SCTLR_SA	(_AC(1, UL) << 3)	/* Stack alignment check enable */
#define SCTLR_SA0	(_AC(1, UL) << 4)	/* Stack Alignment Check Enable for EL0 */
#define SCTLR_CP15BEN	(_AC(1, UL) << 5)	/* System instruction memory barrier enable */
#define SCTLR_ITD	(_AC(1, UL) << 7)	/* IT disable */
#define SCTLR_SED	(_AC(1, UL) << 8)	/* SETEND instruction disable */
#define SCTLR_UMA	(_AC(1, UL) << 9)	/* User mask access */
#define SCTLR_I		(_AC(1, UL) << 12)	/* Instruction access Cacheability control */
#define SCTLR_DZE	(_AC(1, UL) << 14)	/* Traps EL0 DC ZVA instructions to EL1 */
#define SCTLR_UCT	(_AC(1, UL) << 15)	/* Traps EL0 accesses to the CTR_EL0 to EL1 */
#define SCTLR_nTWI	(_AC(1, UL) << 16)	/* Don't trap EL0 WFI to EL1 */
#define SCTLR_nTWE	(_AC(1, UL) << 18)	/* Don't trap EL0 WFE to EL1 */
#define SCTLR_WXN	(_AC(1, UL) << 19)	/* Write permission implies XN */
#define SCTLR_EOE	(_AC(1, UL) << 24)	/* Endianness of data accesses at EL0 */
#define SCTLR_EE	(_AC(1, UL) << 25)	/* Endianness of data accesses at EL1 */
#define SCTLR_UCI	(_AC(1, UL) << 26)	/* Traps EL0 cache instructions to EL1 */

/* Reserve to 1 */
#define SCTLR_RES1_B11	(_AC(1, UL) << 11)
#define SCTLR_RES1_B20	(_AC(1, UL) << 20)
#define SCTLR_RES1_B22	(_AC(1, UL) << 22)
#define SCTLR_RES1_B23	(_AC(1, UL) << 23)
#define SCTLR_RES1_B28	(_AC(1, UL) << 28)
#define SCTLR_RES1_B29	(_AC(1, UL) << 29)

/* Reserve to 0 */
#define SCTLR_RES0_B6	(_AC(1, UL) << 6)
#define SCTLR_RES0_B10	(_AC(1, UL) << 10)
#define SCTLR_RES0_B13	(_AC(1, UL) << 13)
#define SCTLR_RES0_B17	(_AC(1, UL) << 17)
#define SCTLR_RES0_B21	(_AC(1, UL) << 21)
#define SCTLR_RES0_B27	(_AC(1, UL) << 27)
#define SCTLR_RES0_B30	(_AC(1, UL) << 30)
#define SCTLR_RES0_B31	(_AC(1, UL) << 31)

#define SCTLR_RES1		(0x30D00800)

#ifdef CONFIG_PLAT_KVM
/* Bits to set */
#define SCTLR_SET_BITS	\
		(SCTLR_UCI | SCTLR_nTWE | SCTLR_nTWI | SCTLR_UCT | \
		SCTLR_DZE | SCTLR_I | SCTLR_SED | SCTLR_SA0 | SCTLR_SA | \
		SCTLR_C | SCTLR_M | SCTLR_CP15BEN | SCTLR_RES1_B11 | \
		SCTLR_RES1_B20 | SCTLR_RES1_B22 | SCTLR_RES1_B23 | \
		SCTLR_RES1_B28 | SCTLR_RES1_B29)
#else
/* Bits to set */
#define SCTLR_SET_BITS	\
		(SCTLR_UCI | SCTLR_UCT | \
		SCTLR_DZE | SCTLR_I | SCTLR_SED | SCTLR_SA0 | SCTLR_SA | \
		SCTLR_C | SCTLR_M | SCTLR_CP15BEN | SCTLR_RES1_B11 | \
		SCTLR_RES1_B20 | SCTLR_RES1_B22 | SCTLR_RES1_B23 | \
		SCTLR_RES1_B28 | SCTLR_RES1_B29)
#endif

/* Bits to clear */
#define SCTLR_CLEAR_BITS \
		(SCTLR_EE | SCTLR_EOE | SCTLR_WXN | SCTLR_UMA | \
		SCTLR_ITD | SCTLR_A | SCTLR_RES0_B6 | SCTLR_RES0_B10 | \
		SCTLR_RES0_B13 | SCTLR_RES0_B17 | SCTLR_RES0_B21 | \
		SCTLR_RES0_B27 | SCTLR_RES0_B30 | SCTLR_RES0_B31)

/* SPSR_EL3 */
/*
 * When the exception is taken in AArch64:
 * M[3:2] is the exception level
 *
 * M[0] is the SP select:
 *          0: always SP0
 *          1: current ELs SP
 *
 */

/* SPSR_EL1 */
/*
 * When the exception is taken in AArch64:
 * M[3:2] is the exception level
 *
 * M[0] is the SP select:
 *          0: always SP0
 *          1: current ELs SP
 *
 */
#define PSR_M_EL0t  0x00000000
#define PSR_M_EL1t  0x00000004
#define PSR_M_EL1h  0x00000005
#define PSR_M_EL2t  0x00000008
#define PSR_M_EL2h  0x00000009
#define PSR_M_64    0x00000000
#define PSR_M_32    0x00000010
#define PSR_M_MASK  0x0000000f
#define PSR_T       0x00000020
#define PSR_AARCH32 0x00000010
#define PSR_F       0x00000040
#define PSR_I       0x00000080
#define PSR_A       0x00000100
#define PSR_D       0x00000200
#define PSR_DAIF    (PSR_D | PSR_A | PSR_I | PSR_F)
#define PSR_IL      0x00100000
#define PSR_SS      0x00200000
#define PSR_V       0x10000000
#define PSR_C       0x20000000
#define PSR_Z       0x40000000
#define PSR_N       0x80000000
#define PSR_FLAGS   0xf0000000

#define MIDR_EL1_IMPL       0xff000000       // Implementor
#define MIDR_EL1_VARIANT    0x00f00000         // CPU Variant
#define MIDR_EL1_ARCH       0x000f0000       // Architecture
#define MIDR_EL1_PARTNUM    0x0000fff0     		 // PartNum
#define MIDR_EL1_REVISION   0x0000000f     		// Revision
#define MPIDR_AFF3      (0x000000ff00000000)
#define MPIDR_U         (0x0000000040000000)       // 1 = Uni-Processor System
#define MPIDR_MT        (0x0000000001000000)       // 1 = SMT(AFF0 is logical)
#define MPIDR_AFF2      (0x0000000000ff0000)
#define MPIDR_AFF1      (0x000000000000ff00)
#define MPIDR_AFF0      (0x00000000000000ff)
/*
 * Definitions for Block and Page descriptor attributes
 */
/* Level 0 table, 512GiB per entry */
#define L0_SHIFT	39
#define L0_SIZE		(1ul << L0_SHIFT)
#define L0_OFFSET	(L0_SIZE - 1ul)
#define L0_INVAL	0x0 /* An invalid address */
	/* 0x1 Level 0 doesn't support block translation */
	/* 0x2 also marks an invalid address */
#define L0_TABLE	0x3 /* A next-level table */

/* Level 1 table, 1GiB per entry */
#define L1_SHIFT	30
#define L1_SIZE 	(1 << L1_SHIFT)
#define L1_OFFSET 	(L1_SIZE - 1)
#define L1_INVAL	L0_INVAL
#define L1_BLOCK	0x1
#define L1_TABLE	L0_TABLE

/* Level 2 table, 2MiB per entry */
#define L2_SHIFT	21
#define L2_SIZE 	(1 << L2_SHIFT)
#define L2_OFFSET 	(L2_SIZE - 1)
#define L2_INVAL	L1_INVAL
#define L2_BLOCK	L1_BLOCK
#define L2_TABLE	L1_TABLE

#define L2_BLOCK_MASK	_AC(0xffffffe00000, UL)

/* Level 3 table, 4KiB per entry */
#define L3_SHIFT	12
#define L3_SIZE 	(1 << L3_SHIFT)
#define L3_OFFSET 	(L3_SIZE - 1)
#define L3_SHIFT	12
#define L3_INVAL	0x0
	/* 0x1 is reserved */
	/* 0x2 also marks an invalid address */
#define L3_PAGE		0x3

#define L0_ENTRIES_SHIFT 9
#define L0_ENTRIES	(1 << L0_ENTRIES_SHIFT)
#define L0_ADDR_MASK	(L0_ENTRIES - 1)

#define Ln_ENTRIES_SHIFT 9
#define Ln_ENTRIES	(1 << Ln_ENTRIES_SHIFT)
#define Ln_ADDR_MASK	(Ln_ENTRIES - 1)
#define Ln_TABLE_MASK	((1 << 12) - 1)
#define Ln_TABLE	0x3
#define Ln_BLOCK	0x1

/*
 * Hardware page table definitions.
 */
/* TODO: Add the upper attributes */
#define ATTR_MASK_H	_AC(0xfff0000000000000, UL)
#define ATTR_MASK_L	_AC(0x0000000000000fff, UL)
#define ATTR_MASK	(ATTR_MASK_H | ATTR_MASK_L)
/* Bits 58:55 are reserved for software */
#define ATTR_SW_MANAGED	(_AC(1, UL) << 56)
#define ATTR_SW_WIRED	(_AC(1, UL) << 55)
#define ATTR_UXN	(_AC(1, UL) << 54)
#define ATTR_PXN	(_AC(1, UL) << 53)
#define ATTR_XN		(ATTR_PXN | ATTR_UXN)
#define ATTR_CONTIGUOUS	(_AC(1, UL) << 52)
#define ATTR_DBM	(_AC(1, UL) << 51)
#define ATTR_nG		(1 << 11)
#define ATTR_AF		(1 << 10)
#define ATTR_SH(x)	((x) << 8)
#define ATTR_SH_MASK	ATTR_SH(3)
#define ATTR_SH_NS	0		/* Non-shareable */
#define ATTR_SH_OS	2		/* Outer-shareable */
#define ATTR_SH_IS	3		/* Inner-shareable */
#define ATTR_AP_RW_BIT	(1 << 7)
#define ATTR_AP(x)	((x) << 6)
#define ATTR_AP_MASK	ATTR_AP(3)
#define ATTR_AP_RW	(0 << 1)
#define ATTR_AP_RO	(1 << 1)
#define ATTR_AP_USER	(1 << 0)
#define ATTR_NS		(1 << 5)
#define ATTR_IDX(x)	((x) << 2)
#define ATTR_IDX_MASK	(7 << 2)

#define ATTR_DEFAULT	(ATTR_AF | ATTR_SH(ATTR_SH_IS))

#define ATTR_DESCR_MASK	3

/*
 * Define the attributes of pagetable descriptors
 */
#define SECT_ATTR_DEFAULT	\
		(Ln_BLOCK | ATTR_DEFAULT)
#define SECT_ATTR_NORMAL	\
		(SECT_ATTR_DEFAULT | ATTR_XN | \
		ATTR_IDX(NORMAL_WB))
#define SECT_ATTR_NORMAL_RO	\
		(SECT_ATTR_DEFAULT | ATTR_XN | \
		ATTR_AP_RW_BIT | ATTR_IDX(NORMAL_WB))
#define SECT_ATTR_NORMAL_EXEC	\
		(SECT_ATTR_DEFAULT | ATTR_UXN | \
		ATTR_AP_RW_BIT | ATTR_IDX(NORMAL_WB))
#define SECT_ATTR_DEVICE_nGnRE	\
		(SECT_ATTR_DEFAULT | ATTR_XN | \
		ATTR_IDX(DEVICE_nGnRnE))

/* ID_AA64DFR0_EL1 */
#define ID_AA64DFR0_EL1         MRS_REG(3, 0, 0, 5, 0)
#define ID_AA64DFR0_DebugVer_SHIFT  0
#define ID_AA64DFR0_DebugVer_MASK   (_AC(0xf, UL) << ID_AA64DFR0_DebugVer_SHIFT)
#define ID_AA64DFR0_DebugVer_VAL(x) ((x) & ID_AA64DFR0_DebugVer_MASK)
#define  ID_AA64DFR0_DebugVer_8     (_AC(0x6, UL) << ID_AA64DFR0_DebugVer_SHIFT)
#define  ID_AA64DFR0_DebugVer_8_VHE (_AC(0x7, UL) << ID_AA64DFR0_DebugVer_SHIFT)
#define  ID_AA64DFR0_DebugVer_8_2   (_AC(0x8, UL) << ID_AA64DFR0_DebugVer_SHIFT)
#define ID_AA64DFR0_TraceVer_SHIFT  4
#define ID_AA64DFR0_TraceVer_MASK   (_AC(0xf, UL) << ID_AA64DFR0_TraceVer_SHIFT)
#define ID_AA64DFR0_TraceVer_VAL(x) ((x) & ID_AA64DFR0_TraceVer_MASK)
#define  ID_AA64DFR0_TraceVer_NONE  (_AC(0x0, UL) << ID_AA64DFR0_TraceVer_SHIFT)
#define  ID_AA64DFR0_TraceVer_IMPL  (__AC(0x1, UL) << ID_AA64DFR0_TraceVer_SHIFT)
#define ID_AA64DFR0_PMUVer_SHIFT    8
#define ID_AA64DFR0_PMUVer_MASK     (0xfUL << ID_AA64DFR0_PMUVer_SHIFT)
#define ID_AA64DFR0_PMUVer_VAL(x)   ((x) & ID_AA64DFR0_PMUVer_MASK)
#define  ID_AA64DFR0_PMUVer_NONE    (0x0UL << ID_AA64DFR0_PMUVer_SHIFT)
#define  ID_AA64DFR0_PMUVer_3       (0x1UL << ID_AA64DFR0_PMUVer_SHIFT)
#define  ID_AA64DFR0_PMUVer_3_1     (0x4UL << ID_AA64DFR0_PMUVer_SHIFT)
#define  ID_AA64DFR0_PMUVer_IMPL    (0xfUL << ID_AA64DFR0_PMUVer_SHIFT)
#define ID_AA64DFR0_BRPs_SHIFT      12
#define ID_AA64DFR0_BRPs_MASK       (0xfUL << ID_AA64DFR0_BRPs_SHIFT)
#define ID_AA64DFR0_BRPs_VAL(x) \
    ((((x) >> ID_AA64DFR0_BRPs_SHIFT) & 0xf) + 1)
#define ID_AA64DFR0_WRPs_SHIFT      20
#define ID_AA64DFR0_WRPs_MASK       (0xfUL << ID_AA64DFR0_WRPs_SHIFT)
#define ID_AA64DFR0_WRPs_VAL(x) \
    ((((x) >> ID_AA64DFR0_WRPs_SHIFT) & 0xf) + 1)
#define ID_AA64DFR0_CTX_CMPs_SHIFT  28
#define ID_AA64DFR0_CTX_CMPs_MASK   (0xfUL << ID_AA64DFR0_CTX_CMPs_SHIFT)
#define ID_AA64DFR0_CTX_CMPs_VAL(x) \
    ((((x) >> ID_AA64DFR0_CTX_CMPs_SHIFT) & 0xf) + 1)
#define ID_AA64DFR0_PMSVer_SHIFT    32
#define ID_AA64DFR0_PMSVer_MASK     (0xfUL << ID_AA64DFR0_PMSVer_SHIFT)
#define ID_AA64DFR0_PMSVer_VAL(x)   ((x) & ID_AA64DFR0_PMSVer_MASK)
#define  ID_AA64DFR0_PMSVer_NONE    (0x0UL << ID_AA64DFR0_PMSVer_SHIFT)
#define  ID_AA64DFR0_PMSVer_V1      (0x1UL << ID_AA64DFR0_PMSVer_SHIFT)


/* ID_AA64PFR0_EL1 */
#define ID_AA64PFR0_EL1         MRS_REG(3, 0, 0, 4, 0)
#define ID_AA64PFR0_EL0_SHIFT       0
#define ID_AA64PFR0_EL0_MASK        (0xfUL << ID_AA64PFR0_EL0_SHIFT)
#define ID_AA64PFR0_EL0_VAL(x)      ((x) & ID_AA64PFR0_EL0_MASK)
#define  ID_AA64PFR0_EL0_64     (UL(0x1) << ID_AA64PFR0_EL0_SHIFT)
#define  ID_AA64PFR0_EL0_64_32      (0x2UL << ID_AA64PFR0_EL0_SHIFT)
#define ID_AA64PFR0_EL1_SHIFT       4
#define ID_AA64PFR0_EL1_MASK        (0xfUL << ID_AA64PFR0_EL1_SHIFT)
#define ID_AA64PFR0_EL1_VAL(x)      ((x) & ID_AA64PFR0_EL1_MASK)
#define  ID_AA64PFR0_EL1_64     (UL(0x1) << ID_AA64PFR0_EL1_SHIFT)
#define  ID_AA64PFR0_EL1_64_32      (0x2UL << ID_AA64PFR0_EL1_SHIFT)
#define ID_AA64PFR0_EL2_SHIFT       8
#define ID_AA64PFR0_EL2_MASK        (0xfUL << ID_AA64PFR0_EL2_SHIFT)
#define ID_AA64PFR0_EL2_VAL(x)      ((x) & ID_AA64PFR0_EL2_MASK)
#define  ID_AA64PFR0_EL2_NONE       (0x0UL << ID_AA64PFR0_EL2_SHIFT)
#define  ID_AA64PFR0_EL2_64     (0x1UL << ID_AA64PFR0_EL2_SHIFT)
#define  ID_AA64PFR0_EL2_64_32      (0x2UL << ID_AA64PFR0_EL2_SHIFT)
#define ID_AA64PFR0_EL3_SHIFT       12
#define ID_AA64PFR0_EL3_MASK        (0xfUL << ID_AA64PFR0_EL3_SHIFT)
#define ID_AA64PFR0_EL3_VAL(x)      ((x) & ID_AA64PFR0_EL3_MASK)
#define  ID_AA64PFR0_EL3_NONE       (0x0UL << ID_AA64PFR0_EL3_SHIFT)
#define  ID_AA64PFR0_EL3_64     (0x1UL << ID_AA64PFR0_EL3_SHIFT)
#define  ID_AA64PFR0_EL3_64_32      (0x2UL << ID_AA64PFR0_EL3_SHIFT)
#define ID_AA64PFR0_FP_SHIFT        16
#define ID_AA64PFR0_FP_MASK     (0xfUL << ID_AA64PFR0_FP_SHIFT)
#define ID_AA64PFR0_FP_VAL(x)       ((x) & ID_AA64PFR0_FP_MASK)
#define  ID_AA64PFR0_FP_IMPL        (0x0UL << ID_AA64PFR0_FP_SHIFT)
#define  ID_AA64PFR0_FP_HP      (0x1UL << ID_AA64PFR0_FP_SHIFT)
#define  ID_AA64PFR0_FP_NONE        (0xfUL << ID_AA64PFR0_FP_SHIFT)
#define ID_AA64PFR0_AdvSIMD_SHIFT   20
#define ID_AA64PFR0_AdvSIMD_MASK    (0xfUL << ID_AA64PFR0_AdvSIMD_SHIFT)
#define ID_AA64PFR0_AdvSIMD_VAL(x)  ((x) & ID_AA64PFR0_AdvSIMD_MASK)
#define  ID_AA64PFR0_AdvSIMD_IMPL   (0x0UL << ID_AA64PFR0_AdvSIMD_SHIFT)
#define  ID_AA64PFR0_AdvSIMD_HP     (0x1UL << ID_AA64PFR0_AdvSIMD_SHIFT)
#define  ID_AA64PFR0_AdvSIMD_NONE   (0xfUL << ID_AA64PFR0_AdvSIMD_SHIFT)
#define ID_AA64PFR0_GIC_BITS        0x4 /* Number of bits in GIC field */
#define ID_AA64PFR0_GIC_SHIFT       24
#define ID_AA64PFR0_GIC_MASK        (0xfUL << ID_AA64PFR0_GIC_SHIFT)
#define ID_AA64PFR0_GIC_VAL(x)      ((x) & ID_AA64PFR0_GIC_MASK)
#define  ID_AA64PFR0_GIC_CPUIF_NONE (0x0UL << ID_AA64PFR0_GIC_SHIFT)
#define  ID_AA64PFR0_GIC_CPUIF_EN   (0x1UL << ID_AA64PFR0_GIC_SHIFT)
#define ID_AA64PFR0_RAS_SHIFT       28
#define ID_AA64PFR0_RAS_MASK        (0xfUL << ID_AA64PFR0_RAS_SHIFT)
#define ID_AA64PFR0_RAS_VAL(x)      ((x) & ID_AA64PFR0_RAS_MASK)
#define  ID_AA64PFR0_RAS_NONE       (0x0UL << ID_AA64PFR0_RAS_SHIFT)
#define  ID_AA64PFR0_RAS_V1     (0x1UL << ID_AA64PFR0_RAS_SHIFT)
#define ID_AA64PFR0_SVE_SHIFT       32
#define ID_AA64PFR0_SVE_MASK        (0xfUL << ID_AA64PFR0_SVE_SHIFT)
#define ID_AA64PFR0_SVE_VAL(x)      ((x) & ID_AA64PFR0_SVE_MASK)
#define  ID_AA64PFR0_SVE_NONE       (0x0UL << ID_AA64PFR0_SVE_SHIFT)
#define  ID_AA64PFR0_SVE_IMPL       (0x1UL << ID_AA64PFR0_SVE_SHIFT)

/* ICC_SRE_EL1 */
#define ICC_SRE_EL1_SRE     (1U << 0)
/* ICC_SRE_EL2 */
#define ICC_SRE_EL2_SRE     (1U << 0)
#define ICC_SRE_EL2_EN      (1U << 3)
#endif /* __CPU_ARM_64_DEFS_H__ */
