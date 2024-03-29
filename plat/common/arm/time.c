/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Authors: Wei Chen <Wei.Chen@arm.com>
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
#include <stdlib.h>
#include <libfdt.h>
#include <ofw/fdt.h>
#include <uk/assert.h>
#include <uk/plat/time.h>
#include <uk/plat/lcpu.h>
#include <uk/plat/irq.h>
#include <uk/bitops.h>
#include <uk/plat/common/cpu.h>
#include <ofw/gic_fdt.h>
#include <uk/plat/common/irq.h>
#include <gic/gic-v2.h>
#include <arm/time.h>
#include <arm/config.h>

#if 0
/* TODO: For now this file is KVM dependent. As soon as we have more
 * Arm platforms that are using this file, we need to introduce a
 * portable way to handover the DTB entry point to common platform code */
#include <kvm/config.h>
#endif /* Disable KVM Config */

static const char * const arch_timer_list[] = {
	"arm,armv8-timer",
	"arm,armv7-timer",
	NULL
};

void generic_timer_mask_irq(void)
{
	set_el0(cntv_ctl, get_el0(cntv_ctl) | GT_TIMER_MASK_IRQ);

	/* Ensure the write of sys register is visible */
	isb();
}

void generic_timer_unmask_irq(void)
{
	set_el0(cntv_ctl, get_el0(cntv_ctl) & ~GT_TIMER_MASK_IRQ);

	/* Ensure the write of sys register is visible */
	isb();
}

uint32_t generic_timer_get_frequency(int fdt_timer)
{
	int len;
	const uint64_t *fdt_freq;

	/*
	* On a few platforms the frequency is not configured correctly
	* by the firmware. A property in the DT (clock-frequency) has
	* been introduced to workaround those firmware.
	*/
	fdt_freq = fdt_getprop(ukplat_dtb_get(),
			fdt_timer, "clock-frequency", &len);
	if (!fdt_freq || (len <= 0)) {
		uk_pr_info("No clock-frequency found, reading from register directly.\n");

		/* No workaround, get from register directly */
		return get_el0(cntfrq);
	}

	return fdt32_to_cpu(fdt_freq[0]);
}

unsigned long sched_have_pending_events;

void time_block_until(__snsec until)
{
	while ((__snsec) ukplat_monotonic_clock() < until) {
		generic_timer_cpu_block_until(until);
		if (__uk_test_and_clear_bit(0, &sched_have_pending_events))
			break;
	}
}

/* must be called before interrupts are enabled */
void ukplat_time_init(void)
{
	int rc, irq, fdt_timer;
	uint32_t irq_type, hwirq;
	uint32_t trigger_type;
	const void *fdt_base;

	/*
	 * Monotonic time begins at boot_ticks (first read of counter
	 * before calibration).
	 */
	generic_timer_update_boot_ticks();

	fdt_base = ukplat_dtb_get();

	/* Currently, we only support 1 timer per system */
	fdt_timer = fdt_node_offset_by_compatible_list(fdt_base,
				-1, arch_timer_list);
	if (fdt_timer < 0)
		UK_CRASH("Could not find arch timer!\n");

	rc = generic_timer_init(fdt_timer);
	if (rc < 0)
		UK_CRASH("Failed to initialize platform time\n");

	rc = gic_get_irq_from_dtb(fdt_base, fdt_timer, 2,
			&irq_type, &hwirq, &trigger_type);
	if (rc < 0)
		UK_CRASH("Failed to find IRQ number from DTB\n");

	irq = gic_irq_translate(irq_type, hwirq);
	if (irq < 0 || irq >= __MAX_IRQ)
		UK_CRASH("Failed to translate IRQ number, type=%u, hwirq=%u\n",
			irq_type, hwirq);
	uk_pr_info("Registering timer interrupt %d\n", irq);

	rc = ukplat_irq_register(irq, generic_timer_irq_handler, NULL);
	if (rc < 0)
		UK_CRASH("Failed to register timer interrupt handler\n");

	/*
	 * Mask IRQ before scheduler start working. Otherwise we will get
	 * unexpected timer interrupts when system is booting.
	 */
	generic_timer_mask_irq();

	/* Enable timer */
	generic_timer_enable();
}
