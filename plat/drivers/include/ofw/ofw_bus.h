#ifndef PLAT_DRV_OFW_BUS_H
#define PLAT_DRV_OFW_BUS_H

struct ofw_bus_data {
	const char *compatible;
	uintptr_t   device_data;
};

#endif /* PLAT_DRV_OFW_BUS_H */
