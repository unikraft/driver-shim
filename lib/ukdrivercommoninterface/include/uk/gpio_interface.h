/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Authors: Daniela Andreea Dumitrache <andreeadumitrache29@gmail.com>
 *
 * Copyright (c) 2021, NEC Europe Ltd., NEC Corporation. All rights reserved.
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

#ifndef __UK_COMMON_DRIVER_INTERFACE_H__
#define __UK_COMMON_DRIVER_INTERFACE_H__

#include <uk/assert.h>
#include <uk_gpio_data.h>
#include <uk/essentials.h>
#include <errno.h>

#ifdef __cplusplus
extern "C" {
#endif

/**************************** Type Definitions *******************************/

/****************************************************************************/
/**
 * This handler data type allows the user to define a callback function to
 * handle the interrupts for the GPIO device. The application using this
 * driver is expected to define a handler of this type, to support interrupt
 * driven mode. The handler executes in an interrupt context such that minimal
 * processing should be performed.
 *
 * @param	CallBackRef is a callback reference passed in by the upper layer
 *		when setting the callback functions for a GPIO bank. It is
 *		passed back to the upper layer when the callback is invoked. Its
 *		type is not important to the driver component, so it is a void
 *		pointer.
 * @param	bank is the bank for which the interrupt status has changed.
 * @param	Status is the Interrupt status of the GPIO bank.
 *
 *****************************************************************************/
typedef void (*uk_gpio_handler) (void *callback_ref, uin32_t bank, uin32_t status);


/**
 * This typedef contains configuration information for a device.
 */
typedef struct {
	uin16_t deviceId;		/**< Unique ID of device */
	uin32_t baseAddr;		/**< Register base address */
} uk_gpio_config;

/**
 * The GPIO driver instance data. The user is required to allocate a
 * variable of this type for the GPIO device in the system. A pointer
 * to a variable of this type is then passed to the driver API functions.
 */
typedef struct {
	uk_gpio_config gpio_config;	    /**< Device configuration */
	uin32_t ready;			        /**< Device is initialized and ready */
	uk_gpio_handler handler;	    /**< Status handlers for all banks */
	void *callback_ref; 		    /**< Callback ref for bank handlers */
	uin32_t platform;			    /**< Platform data */
	uin32_t maxPinNum;			    /**< Max pins in the GPIO device */
	uin8_t maxbanks;			    /**< Max banks in a GPIO device */
} uk_gpio_data;


/************************** Function Prototypes ******************************/

/* Initialization function */
s32 uk_gpio_initialize(uk_gpio_data *ptr, uk_gpio_config *configPtr,
			   uin32_t effectiveAddr);

/* Bank APIs */
uin32_t uk_gpio_read(uk_gpio_data *ptr, uin8_t bank);
void uk_gpio_write(uk_gpio_data *ptr, uin8_t bank, uin32_t data);
void uk_gpio_setDirection(uk_gpio_data *ptr, uin8_t bank, uin32_t direction);
uin32_t uk_gpio_getDirection(uk_gpio_data *ptr, uin8_t bank);
void uk_gpio_setOutputEnable(uk_gpio_data *ptr, uin8_t bank, uin32_t opEnable);
uin32_t uk_gpio_getOutputEnable(uk_gpio_data *ptr, uin8_t bank);
void uk_gpio_getbankPin(uin8_t pinNumber, uin8_t *bankNumber, uin8_t *pinNumberInbank);

/* Pin APIs */
uin32_t uk_gpio_readPin(uk_gpio_data *ptr, uin32_t pin);
void uk_gpio_writePin(uk_gpio_data *ptr, uin32_t pin, uin32_t data);
void uk_gpio_setDirectionPin(uk_gpio_data *ptr, uin32_t pin, uin32_t direction);
uin32_t uk_gpio_getDirectionPin(uk_gpio_data *ptr, uin32_t pin);
void uk_gpio_setOutputEnablePin(uk_gpio_data *ptr, uin32_t pin, uin32_t opEnable);
uin32_t uk_gpio_getOutputEnablePin(uk_gpio_data *ptr, uin32_t pin);


#ifdef __cplusplus
}
#endif

#endif /* __UK_COMMON_DRIVER_INTERFACE_H__ */