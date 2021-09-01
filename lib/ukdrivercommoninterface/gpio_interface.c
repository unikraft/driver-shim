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

#include <gpio_interface.h>

#ifdef __cplusplus
extern "C" {
#endif


/* Initialization function */
s32 uk_gpio_initialize(uk_gpio_data *ptr, uk_gpio_config *configPtr,
			   uin32_t effectiveAddr)
{
    XGpioPs xilInstancePtr;
    xilInstancePtr.GpioConfig = ptr.gpio_config;
    xilInstancePtr.IsReady = ptr.ready;
    xilInstancePtr.Handler = ptr.handler;
    xilInstancePtr.CallBackRef = ptr.callback_ref;
    xilInstancePtr.Platform = ptr.platform;
    xilInstancePtr.MaxPinNum = ptr.maxPinNum;
    xilInstancePtr.MaxBanks = ptr.maxbanks;

    XGpioPs_Config xilConfigPtr;
    xilConfigPtr.deviceId = configPtr.DeviceId;
    xilConfigPtr.baseAddr = configPtr.BaseAddr;

    return s32 XGpioPs_CfgInitialize(&xilInstancePtr, &xilConfigPtr, effectiveAddr);
}

uin32_t uk_gpio_read(uk_gpio_data *ptr, uin8_t bank)
{
    XGpioPs xilInstancePtr;
    xilInstancePtr.GpioConfig = ptr.gpio_config;
    xilInstancePtr.IsReady = ptr.ready;
    xilInstancePtr.Handler = ptr.handler;
    xilInstancePtr.CallBackRef = ptr.callback_ref;
    xilInstancePtr.Platform = ptr.platform;
    xilInstancePtr.MaxPinNum = ptr.maxPinNum;
    xilInstancePtr.MaxBanks = ptr.maxbanks;

    return XGpioPs_Read(&xilInstancePtr, bank);
}


#ifdef __cplusplus
}
#endif

#endif /* __UK_COMMON_DRIVER_INTERFACE_H__ */