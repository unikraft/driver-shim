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

#include <uk/gpio_interface.h>
#include <uk/alloc.h>
#include <uk/assert.h>
#include <stdio.h>
#include <inttypes.h>

#ifdef __cplusplus
extern "C" {
#endif

/* List of registered devices */
UK_TAILQ_HEAD(uk_gpio_list, struct uk_gpio);

struct uk_gpio_list uk_gpio_list =
UK_TAILQ_HEAD_INITIALIZER(uk_gpio_list);

/* Number of registered devices */
static uint16_t gpiodev_count;

/* Private function to check if a device was registered with the driver */
static int8_t check_gpio_device_registered(uint16_t id);

/* Register device with the GPIO driver */
uint32_t uk_gpio_drv_register(struct uk_gpio *dev)
{
    UK_ASSERT(dev);

    /* assert configuration */
    UK_ASSERT(dev->gpio_data);
    UK_ASSERT(dev->bank_ops);
    UK_ASSERT(dev->pin_ops);


    /* Add device to device queue */
    UK_TAILQ_INSERT_TAIL(&uk_gpio_list, dev, _list);
    uk_pr_info("Registered gpio device: %u\n", dev->gpio_config.deviceId);

    return gpiodev_count++;
}

/* Initialization function */
uint32_t uk_gpio_initialize(struct uk_gpio *dev, uint32_t effectiveAddr)
{   
    UK_ASSERT(dev);
    UK_ASSERT(dev->gpio_data);
    UK_ASSERT(dev->bank_ops);
    UK_ASSERT(dev->bank_ops.gpio_init);
    UK_ASSERT(effectiveAddr != (uint32_t)0U);
    UK_ASSERT(check_gpio_device_registered(dev->gpio_data.deviceId) == 1);
    
    return dev->bank_ops.gpio_init(dev->gpio_data, effectiveAddr);
    
}

/* GPIO read function */
uint32_t uk_gpio_read(uk_gpio *ptr, uin8_t bank)
{
    UK_ASSERT(dev);
    UK_ASSERT(dev->gpio_data);
    UK_ASSERT(dev->bank_ops);
    UK_ASSERT(dev->bank_ops.read_data_register);
    UK_ASSERT(check_gpio_device_registered(dev->gpio_data.deviceId) == 1);
    
    return dev->bank_ops.read_data_register(dev->gpio_data, bank);
}

void uk_gpio_write(uk_gpio *ptr, uin8_t bank, uint32_t data)
{
    UK_ASSERT(dev);
    UK_ASSERT(dev->gpio_data);
    UK_ASSERT(dev->bank_ops);
    UK_ASSERT(dev->bank_ops.write_data_register);
    UK_ASSERT(check_gpio_device_registered(dev->gpio_data.deviceId) == 1);
    
    dev->bank_ops.write_data_register(dev->gpio_data, bank);
}

void uk_gpio_setDirection(uk_gpio *ptr, uin8_t bank, uint32_t direction)
{
    UK_ASSERT(dev);
    UK_ASSERT(dev->gpio_data);
    UK_ASSERT(dev->bank_ops);
    UK_ASSERT(dev->bank_ops.set_direction);
    UK_ASSERT(check_gpio_device_registered(dev->gpio_data.deviceId) == 1);
    
    dev->bank_ops.set_direction(dev->gpio_data, bank, direction);
}

uint32_t uk_gpio_getDirection(uk_gpio *ptr, uin8_t bank)
{
    UK_ASSERT(dev);
    UK_ASSERT(dev->gpio_data);
    UK_ASSERT(dev->bank_ops);
    UK_ASSERT(dev->bank_ops.get_direction);
    UK_ASSERT(check_gpio_device_registered(dev->gpio_data.deviceId) == 1);
    
    return dev->bank_ops.get_direction(dev->gpio_data, bank);
}

void uk_gpio_setOutputEnable(uk_gpio *ptr, uin8_t bank, uint32_t opEnable)
{
    UK_ASSERT(dev);
    UK_ASSERT(dev->gpio_data);
    UK_ASSERT(dev->bank_ops);
    UK_ASSERT(dev->bank_ops.set_output_enable);
    UK_ASSERT(check_gpio_device_registered(dev->gpio_data.deviceId) == 1);
    
    dev->bank_ops.set_output_enable(dev->gpio_data, bank, opEnable);
}

uint32_t uk_gpio_getOutputEnable(uk_gpio *ptr, uin8_t bank)
{
    UK_ASSERT(dev);
    UK_ASSERT(dev->gpio_data);
    UK_ASSERT(dev->bank_ops);
    UK_ASSERT(dev->bank_ops.get_output_enable);
    UK_ASSERT(check_gpio_device_registered(dev->gpio_data.deviceId) == 1);
    
    return dev->bank_ops.get_output_enable(dev->gpio_data, bank);
}

void uk_gpio_getbankPin(struct uk_gpio *dev, uin8_t pinNumber,
                            uin8_t *bankNumber, uin8_t *pinNumberInbank)
{
    UK_ASSERT(dev);
    UK_ASSERT(dev->gpio_data);
    UK_ASSERT(dev->bank_ops);
    UK_ASSERT(dev->bank_ops.get_bank_pin);
    UK_ASSERT(bankNumber);
    UK_ASSERT(pinNumberInbank);
    UK_ASSERT(check_gpio_device_registered(dev->gpio_data.deviceId) == 1);
    
    dev->bank_ops.get_bank_pin(pinNumber, bankNumber, pinNumberInbank);
}

/* Deregister device from the GPIO driver */
void uk_gpio_unregister(struct uk_gpio *dev)
{
    uint16_t id = dev->gpio_config.deviceId;

    dev->callback_ref = NULL;

    /* Remove device from queue */
    UK_TAILQ_REMOVE(&uk_gpio_list, dev, _list);
    gpiodev_count--;

    uk_pr_info("Unregistered gpio device: %u\n",id);
}

/* Pin APIs */
uint32_t uk_gpio_readPin(struct uk_gpio *dev, uint32_t pin)
{
    UK_ASSERT(dev);
    UK_ASSERT(dev->gpio_data);
    UK_ASSERT(dev->pin_ops);
    UK_ASSERT(dev->pin_ops.read_data_pin);
    UK_ASSERT(check_gpio_device_registered(dev->gpio_data.deviceId) == 1);
    
    return dev->pin_ops.read_data_pin(dev->gpio_data, pin);
}

void uk_gpio_writePin(struct uk_gpio *dev, uint32_t pin, uint32_t data)
{
    UK_ASSERT(dev);
    UK_ASSERT(dev->gpio_data);
    UK_ASSERT(dev->pin_ops);
    UK_ASSERT(dev->pin_ops.write_data_pin);
    UK_ASSERT(check_gpio_device_registered(dev->gpio_data.deviceId) == 1);
    
    dev->pin_ops.write_data_pin(dev->gpio_data, pin);
}

void uk_gpio_setDirectionPin(struct uk_gpio *dev, uint32_t pin, uint32_t direction)
{
    UK_ASSERT(dev);
    UK_ASSERT(dev->gpio_data);
    UK_ASSERT(dev->pin_ops);
    UK_ASSERT(dev->pin_ops.set_direction_pin);
    UK_ASSERT(check_gpio_device_registered(dev->gpio_data.deviceId) == 1);
    
    dev->pin_ops.set_direction_pin(dev->gpio_data, pin, direction);
}

uint32_t uk_gpio_getDirectionPin(struct uk_gpio *dev, uint32_t pin)
{
    UK_ASSERT(dev);
    UK_ASSERT(dev->gpio_data);
    UK_ASSERT(dev->pin_ops);
    UK_ASSERT(dev->pin_ops.get_direction_pin);
    UK_ASSERT(check_gpio_device_registered(dev->gpio_data.deviceId) == 1);
    
    return dev->pin_ops.get_direction_pin(dev->gpio_data, pin);
}

void uk_gpio_setOutputEnablePin(struct uk_gpio *dev, uint32_t pin, uint32_t opEnable)
{
    UK_ASSERT(dev);
    UK_ASSERT(dev->gpio_data);
    UK_ASSERT(dev->pin_ops);
    UK_ASSERT(dev->pin_ops.set_output_enable_pin);
    UK_ASSERT(check_gpio_device_registered(dev->gpio_data.deviceId) == 1);
    
    dev->pin_ops.set_output_enable_pin(dev->gpio_data, pin, opEnable);
}

uint32_t uk_gpio_getOutputEnablePin(struct uk_gpio *dev, uint32_t pin)
{
    UK_ASSERT(dev);
    UK_ASSERT(dev->gpio_data);
    UK_ASSERT(dev->pin_ops);
    UK_ASSERT(dev->pin_ops.get_output_enable_pin);
    UK_ASSERT(check_gpio_device_registered(dev->gpio_data.deviceId) == 1);
    
    return dev->pin_ops.get_output_enable_pin(dev->gpio_data, pin);
}

static int8_t check_gpio_device_registered(uint16_t id)
{
    /* search the device in the device queue */
    UK_TAILQ_FOREACH(dev, &uk_gpio_list, _list) {
        if (dev->gpio_data.deviceId == id)
            return 1;
    }

    uk_pr_info("GPIO device is not registered\n");
    return 0;
}


#ifdef __cplusplus
}
#endif

#endif /* __UK_COMMON_DRIVER_INTERFACE_H__ */