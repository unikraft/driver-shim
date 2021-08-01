/* SPDX-License-Identifier: ISC */
/*
 * Authors: Sharan Santhanam <sharan.santhanam@neclab.eu>
 *
 * Copyright (c) 2020, NEC Laboratries Europe GmbH
 *
 * Permission to use, copy, modify, and/or distribute this software
 * for any purpose with or without fee is hereby granted, provided
 * that the above copyright notice and this permission notice appear
 * in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
 * WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE
 * AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR
 * CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS
 * OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */
#include <uk/config.h>
#include <uk/arch/types.h>
#include <uk/essentials.h>
#include <uk/assert.h>
#ifdef CONFIG_ZYNQMP_LIBOFW
#include <libfdt_env.h>
#include <fdt.h>
#include <ofw/ofw_bus.h>
#include <ofw/fdt.h>
#endif /* CONFIG_ZYNQMP_LIBOFW */
#include <errno.h>
#include <zynqmp/config.h>
#include <rtc/rtc.h>


#define RTC_SET_TIME_WR_OFF		0x00
#define RTC_SET_TIME_WR_RSTVAL   0x00000000U
#define RTC_SET_TIME_WR_VAL_SHIFT   0U
#define RTC_SET_TIME_WR_VAL_WIDTH   32U
#define RTC_SET_TIME_WR_VAL_MASK    0xffffffffU
#define RTC_SET_TIME_WR_VAL_DEFVAL  0x0U

#define RTC_SET_TIME_RD_OFF		0x01
#define RTC_SET_TIME_RD_RSTVAL   0x00000000U
#define RTC_SET_TIME_RD_VAL_SHIFT   0U
#define RTC_SET_TIME_RD_VAL_WIDTH   32U
#define RTC_SET_TIME_RD_VAL_MASK    0xffffffffU
#define RTC_SET_TIME_RD_VAL_DEFVAL  0x0U

#define RTC_CALIB_WR_OFF		0x02
#define RTC_CALIB_WR_RSTVAL   0x00000000U
#define RTC_CALIB_WR_FRACTN_EN_SHIFT   20U
#define RTC_CALIB_WR_FRACTN_EN_WIDTH   1U
#define RTC_CALIB_WR_FRACTN_EN_MASK    0x00100000U
#define RTC_CALIB_WR_FRACTN_EN_DEFVAL  0x0U
#define RTC_CALIB_WR_FRACTN_DATA_SHIFT   16U
#define RTC_CALIB_WR_FRACTN_DATA_WIDTH   4U
#define RTC_CALIB_WR_FRACTN_DATA_MASK    0x000f0000U
#define RTC_CALIB_WR_FRACTN_DATA_DEFVAL  0x0U
#define RTC_CALIB_WR_MAX_TCK_SHIFT   0U
#define RTC_CALIB_WR_MAX_TCK_WIDTH   16U
#define RTC_CALIB_WR_MAX_TCK_MASK    0x0000ffffU
#define RTC_CALIB_WR_MAX_TCK_DEFVAL  0x0U

#define RTC_CALIB_RD_OFF		0x03
#define RTC_CALIB_RD_RSTVAL   0x00000000U
#define RTC_CALIB_RD_FRACTN_EN_SHIFT   20U
#define RTC_CALIB_RD_FRACTN_EN_WIDTH   1U
#define RTC_CALIB_RD_FRACTN_EN_MASK    0x00100000U
#define RTC_CALIB_RD_FRACTN_EN_DEFVAL  0x0U
#define RTC_CALIB_RD_FRACTN_DATA_SHIFT   16U
#define RTC_CALIB_RD_FRACTN_DATA_WIDTH   4U
#define RTC_CALIB_RD_FRACTN_DATA_MASK    0x000f0000U
#define RTC_CALIB_RD_FRACTN_DATA_DEFVAL  0x0U
#define RTC_CALIB_RD_MAX_TCK_SHIFT   0U
#define RTC_CALIB_RD_MAX_TCK_WIDTH   16U
#define RTC_CALIB_RD_MAX_TCK_MASK    0x0000ffffU
#define RTC_CALIB_RD_MAX_TCK_DEFVAL  0x0U

#define RTC_CUR_TIME_OFF		0x04
#define RTC_CUR_TIME_RSTVAL   0x00000000U
#define RTC_CUR_TIME_VAL_SHIFT   0U
#define RTC_CUR_TIME_VAL_WIDTH   32U
#define RTC_CUR_TIME_VAL_MASK    0xffffffffU
#define RTC_CUR_TIME_VAL_DEFVAL  0x0U

#define RTC_CUR_TCK_OFF			0x05
#define RTC_CUR_TCK_RSTVAL   0x00000000U
#define RTC_CUR_TCK_VAL_SHIFT   0U
#define RTC_CUR_TCK_VAL_WIDTH   16U
#define RTC_CUR_TCK_VAL_MASK    0x0000ffffU
#define RTC_CUR_TCK_VAL_DEFVAL  0x0U

#define RTC_ALRM_OFF			0x06
#define RTC_ALRM_RSTVAL   0x00000000U
#define RTC_ALRM_VAL_SHIFT   0U
#define RTC_ALRM_VAL_WIDTH   32U
#define RTC_ALRM_VAL_MASK    0xffffffffU
#define RTC_ALRM_VAL_DEFVAL  0x0U

#define RTC_INT_STS_OFF			0x08
#define RTC_INT_STS_RSTVAL   0x00000000U
#define RTC_INT_STS_ALRM_SHIFT   1U
#define RTC_INT_STS_ALRM_WIDTH   1U
#define RTC_INT_STS_ALRM_MASK    0x00000002U
#define RTC_INT_STS_ALRM_DEFVAL  0x0U
#define RTC_INT_STS_SECS_SHIFT   0U
#define RTC_INT_STS_SECS_WIDTH   1U
#define RTC_INT_STS_SECS_MASK    0x00000001U
#define RTC_INT_STS_SECS_DEFVAL  0x0U

#define RTC_INT_MSK_OFF			0x09
#define RTC_INT_MSK_RSTVAL   0x00000003U
#define RTC_INT_MSK_ALRM_SHIFT   1U
#define RTC_INT_MSK_ALRM_WIDTH   1U
#define RTC_INT_MSK_ALRM_MASK    0x00000002U
#define RTC_INT_MSK_ALRM_DEFVAL  0x1U
#define RTC_INT_MSK_SECS_SHIFT   0U
#define RTC_INT_MSK_SECS_WIDTH   1U
#define RTC_INT_MSK_SECS_MASK    0x00000001U
#define RTC_INT_MSK_SECS_DEFVAL  0x1U

#define RTC_INT_EN_OFF			0x0a
#define RTC_INT_EN_RSTVAL   0x00000000U
#define RTC_INT_EN_ALRM_SHIFT   1U
#define RTC_INT_EN_ALRM_WIDTH   1U
#define RTC_INT_EN_ALRM_MASK    0x00000002U
#define RTC_INT_EN_ALRM_DEFVAL  0x0U
#define RTC_INT_EN_SECS_SHIFT   0U
#define RTC_INT_EN_SECS_WIDTH   1U
#define RTC_INT_EN_SECS_MASK    0x00000001U
#define RTC_INT_EN_SECS_DEFVAL  0x0U

#define RTC_INT_DIS_OFF			0x0b
#define RTC_INT_DIS_RSTVAL   0x00000000U
#define RTC_INT_DIS_ALRM_SHIFT   1U
#define RTC_INT_DIS_ALRM_WIDTH   1U
#define RTC_INT_DIS_ALRM_MASK    0x00000002U
#define RTC_INT_DIS_ALRM_DEFVAL  0x0U
#define RTC_INT_DIS_SECS_SHIFT   0U
#define RTC_INT_DIS_SECS_WIDTH   1U
#define RTC_INT_DIS_SECS_MASK    0x00000001U
#define RTC_INT_DIS_SECS_DEFVAL  0x0U


#define RTC_ADD_ERR_OFF			0x0c
#define RTC_ADD_ERR_RSTVAL   0x00000000U
#define RTC_ADD_ERR_STS_SHIFT   0U
#define RTC_ADD_ERR_STS_WIDTH   1U
#define RTC_ADD_ERR_STS_MASK    0x00000001U
#define RTC_ADD_ERR_STS_DEFVAL  0x0U


#define RTC_ADD_ERR_INT_MSK_OFF		0x0d
#define RTC_ADD_ERR_INT_MSK_RSTVAL   0x00000001U
#define RTC_ADD_ERR_INT_MSK_SHIFT   0U
#define RTC_ADD_ERR_INT_MSK_WIDTH   1U
#define RTC_ADD_ERR_INT_MSK_MASK    0x00000001U
#define RTC_ADD_ERR_INT_MSK_DEFVAL  0x1U

#define RTC_ADD_ERR_INT_EN_OFF		0x0e
#define RTC_ADD_ERR_INT_EN_RSTVAL   0x00000000U
#define RTC_ADD_ERR_INT_EN_MSK_SHIFT   0U
#define RTC_ADD_ERR_INT_EN_MSK_WIDTH   1U
#define RTC_ADD_ERR_INT_EN_MSK_MASK    0x00000001U
#define RTC_ADD_ERR_INT_EN_MSK_DEFVAL  0x0U

#define RTC_ADD_ERR_INT_DIS_OFF		0x0f
#define RTC_ADD_ERR_INT_DIS_RSTVAL   0x00000000U
#define RTC_ADD_ERR_INT_DIS_MSK_SHIFT   0U
#define RTC_ADD_ERR_INT_DIS_MSK_WIDTH   1U
#define RTC_ADD_ERR_INT_DIS_MSK_MASK    0x00000001U
#define RTC_ADD_ERR_INT_DIS_MSK_DEFVAL  0x0U


#define RTC_CTL_OFF			0x10
#define RTC_CTL_RSTVAL   0x01000000U
#define RTC_CTL_BATTERY_EN_SHIFT   31U
#define RTC_CTL_BATTERY_EN_WIDTH   1U
#define RTC_CTL_BATTERY_EN_MASK    0x80000000U
#define RTC_CTL_BATTERY_EN_DEFVAL  0x0U
#define RTC_CTL_OSC_SHIFT   24U
#define RTC_CTL_OSC_WIDTH   4U
#define RTC_CTL_OSC_MASK    0x0f000000U
#define RTC_CTL_OSC_DEFVAL  0x1U
#define RTC_CTL_SLVERR_EN_SHIFT   0U
#define RTC_CTL_SLVERR_EN_WIDTH   1U
#define RTC_CTL_SLVERR_EN_MASK    0x00000001U
#define RTC_CTL_SLVERR_EN_DEFVAL  0x0U

#define RTC_CALIB_DEF			0x198233
#define RTCPSU_CRYSTAL_OSC_EN      ((uint32_t)1 << RTC_CTL_OSC_SHIFT)


/**
 * We assume there is a rtc device.
 * TODO:
 * Need a way to support multiple devices.
 */
static struct rtc_device rtc_dev;

struct ofw_bus_data rtc_dtc_data = {
	.compatible="xlnx-zynmp.rtc",
	.device_data = &rtc_dev,
};

static int _rtc_time_raw_get(struct rtc_device *rdev, uint64_t *currtime)
{
	uint32_t status;
	uint64_t time;

	status = ioreg_read32(((uk_reg32_t)rdev->addr) + RTC_INT_STS_OFF);
	if (status & RTC_INT_STS_SECS_MASK) {
		time  = ioreg_read32(((uk_reg32_t)rdev->addr) +
				     RTC_CUR_TIME_OFF);
	} else {
		time  = ioreg_read32(((uk_reg32_t)rdev->addr) +
				     RTC_SET_TIME_RD_OFF);
	}
	*currtime = time;
	return 0;
}

uint64_t ukplat_rtc_get_boottick(void)
{
	return _libzynqmpplat_cfg.rtc_bootticks;
}

int ukplat_rtc_time_get(struct rtc_device *dev, struct timespec *timeval)
{
	uint64_t raw_time;
	int rc;

	UK_ASSERT(dev && timeval);

	rc = _rtc_time_raw_get(dev, &raw_time);

	/**
	 * TODO:
	 *	Function to convert the raw time to the timespec val.
	 */

	return -EINVAL;
}

int ukplat_rtc_time_set(struct rtc_device *dev, struct timespec *timeval)
{
	UK_ASSERT(dev && timeval);

	return -EINVAL;
}

int ukplat_rtc_alarm_get(struct rtc_device *dev, struct timespec *timeval)
{
	UK_ASSERT(dev && timeval);

	return -EINVAL;
}

int ukplat_rtc_alarm_set(struct rtc_device *dev, struct timespec *timeval)
{
	UK_ASSERT(dev && timeval);

	return -EINVAL;
}

int ukplat_rtc_irq_enable(struct rtc_device *dev, int enabled)
{
	UK_ASSERT(dev);

	return -EINVAL;
}

static void _rtc_dev_init(uint32_t calibration_info)
{
	int rc;
	uint32_t ctrl_reg;

	ioreg_write32(((uk_reg32_t)rtc_dev.addr) + RTC_CALIB_WR_OFF,
		      calibration_info);

	/*  Set the Oscillator crystal and Battery switch enable
	 *  in control register.
	 */
	ctrl_reg = ioreg_read32(((uk_reg32_t)rtc_dev.addr)
					+ RTC_CTL_OFF);
	ioreg_write32(((uk_reg32_t)rtc_dev.addr) + RTC_CTL_OFF,
			 (ctrl_reg | (uint32_t)RTCPSU_CRYSTAL_OSC_EN |
			 (uint32_t)RTC_CTL_BATTERY_EN_MASK));

	/* Clear the Interrupt Status and Disable the interrupts. */
	ioreg_write32(((uk_reg32_t)rtc_dev.addr) +
		      RTC_INT_STS_OFF, ((uint32_t)RTC_INT_STS_ALRM_MASK |
		      (uint32_t)RTC_INT_STS_SECS_MASK));

	ioreg_write32(((uk_reg32_t)rtc_dev.addr) + RTC_INT_DIS_OFF,
		      ((uint32_t)RTC_INT_DIS_ALRM_MASK |
		      (uint32_t)RTC_INT_DIS_SECS_MASK));
}

int _libplat_rtc_init(const void *dtb __maybe_unused)
{
	int offset, len, rc;
	uint64_t naddr, nsize;
	uint32_t calibration_info, *fdt_prop32;

#ifdef CONFIG_ZYNQMP_LIBOFW
	UK_ASSERT(dtb);

	/* find the compatible device node from the device tree */
	offset = fdt_node_offset_by_compatible(dtb, -1,
					       rtc_dtc_data.compatible);
	if (offset < 0)
		goto error_exit;

	/* Read the register property of the rtc device */
	rc = fdt_get_address(dtb, offset, 0, &naddr, &nsize);
	if (rc < 0)
		goto error_exit;

	rtc_dev.addr = (const void *) naddr;
	rtc_dev.size = nsize;

	/* Initialize the rtc device */
	fdt_prop32 = fdt_getprop(dtb, offset, "calibration", &rc);
	if (!fdt_prop32)
		calibration_info = RTC_CALIB_DEF;
	else
		calibration_info = fdt32_to_cpu(*fdt_prop32);

	/* Read the interrupt device id */
	_rtc_dev_init(calibration_info);

	/* Read the rtc device */
	_rtc_time_raw_get(&rtc_dev, &_libzynqmpplat_cfg.rtc_bootticks);

	/* Set the clock ticks */
#else
	goto error_exit;
#endif /* CONFIG_ZYNQMP_LIBOFW */

	return 0;

error_exit:
	_libzynqmpplat_cfg.rtc_bootticks = 0;
	return -1;
}
