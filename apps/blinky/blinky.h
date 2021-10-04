/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Unikraft Monkey Animation
 *
 * Authors: Simon Kuenzer <simon.kuenzer@neclab.eu>
 *
 *
 * Copyright (c) 2020, NEC Laboratories Europe GmbH, NEC Corporation.
 *                     All rights reserved.
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
 */

#ifndef BLINKY_H
#define BLINKY_H
#endif

#include <uk/alloc.h>
#include <uk/assert.h>
#include <uk/gpio_interface.h>

/* The following constants define the interrupt types that can be set for each
 * GPIO pin as defines in the Xilinx PS GPIO dirver */
/* Definitions for driver GPIOPS */
#define XPAR_XGPIOPS_NUM_INSTANCES 1

/* Definitions for peripheral PSU_GPIO_0 */
#define XPAR_PSU_GPIO_0_DEVICE_ID 0
#define XPAR_PSU_GPIO_0_BASEADDR 0xFF0A0000
#define XPAR_PSU_GPIO_0_HIGHADDR 0xFF0AFFFF


#define XGPIOPS_IRQ_TYPE_EDGE_RISING	0x00U  /**< Interrupt on Rising edge */
#define XGPIOPS_IRQ_TYPE_EDGE_FALLING	0x01U  /**< Interrupt Falling edge */
#define XGPIOPS_IRQ_TYPE_EDGE_BOTH		0x02U  /**< Interrupt on both edges */
#define XGPIOPS_IRQ_TYPE_LEVEL_HIGH		0x03U  /**< Interrupt on high level */
#define XGPIOPS_IRQ_TYPE_LEVEL_LOW		0x04U  /**< Interrupt on low level */
/*@}*/

#define XGPIOPS_BANK_MAX_PINS	(u32)32 /**< Max pins in a GPIO bank */
#define XGPIOPS_BANK0			0x00U  /**< GPIO Bank 0 */
#define XGPIOPS_BANK1			0x01U  /**< GPIO Bank 1 */
#define XGPIOPS_BANK2			0x02U  /**< GPIO Bank 2 */
#define XGPIOPS_BANK3			0x03U  /**< GPIO Bank 3 */

#ifdef XPAR_PSU_GPIO_0_BASEADDR
#define XGPIOPS_BANK4			0x04U  /**< GPIO Bank 4 */
#define XGPIOPS_BANK5			0x05U  /**< GPIO Bank 5 */
#endif

#define XGPIOPS_MAX_BANKS_ZYNQMP	0x06U  /**< Max banks in a Zynq
									            Ultrascale+ MP GPIO device */

#define XGPIOPS_MAX_BANKS		0x04U  /**< Max banks in a Zynq GPIO device */

#define XGPIOPS_DEVICE_MAX_PIN_NUM_ZYNQMP	(u32)174 /**< Max pins in the
						  *	Zynq Ultrascale+ MP GPIO device
					      * 0 - 25,  Bank 0
					      * 26 - 51, Bank 1
					      *	52 - 77, Bank 2
					      *	78 - 109, Bank 3
					      *	110 - 141, Bank 4
					      *	142 - 173, Bank 5
					      */

#define XGPIOPS_DEVICE_MAX_PIN_NUM	(u32)118 /**< Max pins in the Zynq GPIO device
					      * 0 - 31,  Bank 0
					      * 32 - 53, Bank 1
					      *	54 - 85, Bank 2
					      *	86 - 117, Bank 3
					      */

