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
typedef void (*uk_gpio_handler) (void *callback_ref, uint32_t bank, uint32_t status);


/**
 * The GPIO driver instance data. The user is required to allocate a
 * variable of this type for the GPIO device in the system. A pointer
 * to a variable of this type is then passed to the driver API functions.
 */
typedef struct {
	uin16_t deviceId;          /**< Unique ID of device */
    uint32_t baseAddr;          /**< Register base address */
	uint32_t ready;			   /**< Device is initialized and ready */
	uint32_t platform;	       /**< Platform data */
	uint32_t maxPinNum;	       /**< Max pins in the GPIO device */
	uin8_t  maxbanks;	       /**< Max banks in a GPIO device */
    uk_gpio_handler handler;   /**< Status handlers for all banks */
    void *callback_ref;        /**< Callback ref for bank handlers */
} uk_gpio_data;

/* Bank API driver callback types */

/* Driver callback type to read the data register of the specified GPIO bank.*/
typedef int (*uk_gpio_read_t)(uk_gpio_data *ptr, uin8_t bank);

/* Driver callback type to write to the data register of the specified GPIO bank */
typedef void (*uk_gpio_write_t)(uk_gpio_data *ptr, uin8_t bank, uint32_t data);

/* Driver callback type to set the direction of the pins of the specified GPIO Bank */
typedef void (*uk_gpio_setDirection_t)(uk_gpio_data *ptr,
                uin8_t bank, uint32_t direction);

/* Driver callback type to get the direction of the pins of the specified GPIO Bank */
typedef uint32_t (*uk_gpio_getDirection_t)(uk_gpio_data *ptr, uin8_t bank);

/* Driver callback type to set the output enable of the pins of the specified GPIO Bank.*/
typedef void (*uk_gpio_setOutputEnable_t)(uk_gpio_data *ptr, uin8_t bank, uint32_t opEnable);

/* Driver callback type to get the output enable of the pins of the specified GPIO Bank. */
typedef uint32_t (*uk_gpio_getOutputEnable_t)(uk_gpio_data *ptr, uin8_t bank);

/** Driver callback type to get the bank and pin number in the bank, for the given pin number 
 * in the GPIO device. */
typedef void (*uk_gpio_getbankPin_t)(uin8_t pinNumber, uin8_t *bankNumber, uin8_t *pinNumberInbank);

/* Driver callback type for device initialization */
typedef uint32_t (*uk_gpio_initialize_t)(uk_gpio_data *ptr, uint32_t effectiveAddr);

typedef struct 
{
    uk_gpio_read_t              read_data_register;
    uk_gpio_write_t             write_data_register;
    uk_gpio_setDirection_t      set_direction;
    uk_gpio_getDirection_t      get_direction;
    uk_gpio_setOutputEnable_t   set_output_enable;
    uk_gpio_getOutputEnable_t   get_output_enable;
    uk_gpio_getbankPin_t        get_bank_pin;
    uk_gpio_initialize_t        gpio_init;
} uk_gpio_bank_ops;

/* Pin APIs driver callback types */

/* Driver callback type to read the data of the specified pin.*/
typedef uint32_t (*uk_gpio_readPin_t)(uk_gpio_data *ptr, uint32_t pin);

/* Driver callback type to write to the data register of the specified pin */
typedef void (*uk_gpio_writePin_t)(uk_gpio_data *ptr, uint32_t pin, uint32_t data);

/* Driver callback type to set the direction of the pins of the specified pin */
typedef void (*uk_gpio_setDirectionPin_t)(uk_gpio_data *ptr, uint32_t pin, uint32_t direction);

/* Driver callback type to get the direction of the pins of the specified pin */
typedef uint32_t (*uk_gpio_getDirectionPin_t)(uk_gpio_data *ptr, uint32_t pin);

/* Driver callback type to set the output enable of the pins of the specified pin.*/
typedef void (*uk_gpio_setOutputEnablePin_t)(uk_gpio_data *ptr, uint32_t pin, uint32_t opEnable);

/* Driver callback type to get the output enable of the pins of the specified pin. */
typedef uint32_t (*uk_gpio_getOutputEnablePin_t)(uk_gpio_data *ptr, uint32_t pin);

typedef struct
{
    uk_gpio_readPin_t               read_data_pin;
    uk_gpio_writePin_t              write_data_pin;
    uk_gpio_setDirectionPin_t       set_direction_pin;
    uk_gpio_getDirectionPin_t       get_direction_pin;
    uk_gpio_setOutputEnablePin_t    set_output_enable_pin;
    uk_gpio_getOutputEnablePin_t    get_output_enable_pin;
} uk_gpio_pin_ops;

struct uk_gpio
{
    /* GPIO driver data information */
    uk_gpio_data        gpio_data;
    /* GPIO bank APIs */
    uk_gpio_bank_ops    bank_ops;
    /* GPIO pin APIs */
    uk_gpio_pin_ops     pin_ops;
    /* Entry for list of block devices */
    UK_TAILQ_ENTRY(struct uk_gpio) _list;
};

/************************** Function Prototypes ******************************/

/* Registration function */
uint32_t uk_gpio_drv_register(struct uk_gpio *dev);

/* Deregister device from the GPIO driver */
void uk_gpio_unregister(struct uk_gpio *dev);

/* Initialization function */
uint32_t uk_gpio_initialize(struct uk_gpio *dev, uint32_t effectiveAddr);

/* Bank APIs */
uint32_t uk_gpio_read(struct uk_gpio *dev, uin8_t bank);
void uk_gpio_write(struct uk_gpio *dev, uin8_t bank, uint32_t data);
void uk_gpio_setDirection(struct uk_gpio *dev, uin8_t bank, uint32_t direction);
uint32_t uk_gpio_getDirection(struct uk_gpio *dev, uin8_t bank);
void uk_gpio_setOutputEnable(struct uk_gpio *dev, uin8_t bank, uint32_t opEnable);
uint32_t uk_gpio_getOutputEnable(struct uk_gpio *dev, uin8_t bank);
void uk_gpio_getbankPin(struct uk_gpio *dev, uin8_t pinNumber, uin8_t *bankNumber, uin8_t *pinNumberInbank);

/* Pin APIs */
uint32_t uk_gpio_readPin(struct uk_gpio *dev, uint32_t pin);
void uk_gpio_writePin(struct uk_gpio *dev, uint32_t pin, uint32_t data);
void uk_gpio_setDirectionPin(struct uk_gpio *dev, uint32_t pin, uint32_t direction);
uint32_t uk_gpio_getDirectionPin(struct uk_gpio *dev, uint32_t pin);
void uk_gpio_setOutputEnablePin(struct uk_gpio *dev, uint32_t pin, uint32_t opEnable);
uint32_t uk_gpio_getOutputEnablePin(struct uk_gpio *dev, uint32_t pin);


#ifdef __cplusplus
}
#endif

#endif /* __UK_COMMON_DRIVER_INTERFACE_H__ */