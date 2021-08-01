#ifndef __PLAT_DRV_RTC_H
#define __PLAT_DRV_RTC_H

#include <time.h>

struct rtc_device {
	// Base address of the device.
	const void *addr;
	uint64_t   size;
	// Identifier of the device.
	int id;
};

uint64_t ukplat_rtc_get_boottick(void);
int ukplat_rtc_time_get(struct rtc_device *dev, struct timespec *timeval);
int ukplat_rtc_time_set(struct rtc_device *dev, struct timespec *timeval);
int ukplat_rtc_alarm_get(struct rtc_device *dev, struct timespec *timeval);
int ukplat_rtc_alarm_set(struct rtc_device *dev, struct timespec *timeval);
int ukplat_rtc_irq_enable(struct rtc_device *dev, int enabled);
#endif /* __PLAT_DRV_RTC_H */
