#include <stdio.h>

/* Import user configuration: */
#ifdef __Unikraft__
#include <uk/config.h>
#endif /* __Unikraft__ */

#include "blinky.h"

#define DIRECTION_OUTPUT 1
#define OUTPUT_ENABLED   1
#define PIN_LED_OUTPUT   10 /* to check correct pin number */

/* callbackfunctions for the GPIO driver */
void uk_gpio_handler_func(void *callback_ref, uint32_t bank, uint32_t status);
void uk_gpio_callback_func(void);

/* bank functions */
static uk_gpio_bank_ops bank_ops;

int uk_bank_read(uk_gpio_data *ptr, uin8_t bank);
void uk_bank_write(uk_gpio_data *ptr, uin8_t bank, uin32_t data);
void uk_bank_setDirection(uk_gpio_data *ptr, uin8_t bank, uin32_t direction);
uint32_t uk_bank_getDirection(uk_gpio_data *ptr, uin8_t bank);
void uk_bank_setOutputEnable(uk_gpio_data *ptr, uin8_t bank, uin32_t opEnable);
uint32_t uk_bank_getOutputEnable(uk_gpio_data *ptr, uin8_t bank);
void uk_bank_getbankPin(uin8_t pinNumber, uin8_t *bankNumber, uin8_t *pinNumberInbank);
uint32_t uk_bank_initialize(uk_gpio_data *ptr, uin32_t effectiveAddr);

/* pin funcitons */
static uk_gpio_pin_ops pin_ops;

uint32_t uk_read_pin(uk_gpio_data *ptr, uin32_t pin);
void uk_write_pin(uk_gpio_data *ptr, uin32_t pin, uin32_t data);
void uk_set_pin_direction(uk_gpio_data *ptr, uin32_t pin, uin32_t direction);
uint32_t uk_get_pin_direction(uk_gpio_data *ptr, uin32_t pin);
void uk_set_pin_output_enable(uk_gpio_data *ptr, uin32_t pin, uin32_t opEnable);
uint32_t uk_get_pin_output_enable(uk_gpio_data *ptr, uin32_t pin);

/* config data for the GPIO driver */
static uk_gpio_data gpio_data;
static struct uk_gpio dev;

/* GPIO driver instance data for Xilinx zynqmp platform */
static XGpioPs_Config *configPtr = NULL;
static XGpioPs *instancePtr = NULL;
static struct uk_alloc a;

void uk_gpio_handler_func(void *callback_ref, uint32_t bank, uint32_t status)
{
	uk_pr_info("GPIO handler bank %d status %d\n", bank, status);
}

uint32_t uk_bank_initialize(uk_gpio_data *ptr, uin32_t effectiveAddr)
{
	if (configPtr == NULL)
	{
		configPtr = uk_calloc(&a, 1, sizeof(XGpioPs_Config));
	}
	UK_ASSERT(configPtr);
	configPtr.DeviceId = ptr->gpio_data.deviceId;
	configPtr.BaseAddr = ptr->gpio_data.baseAddr;
	
	if (instancePtr == NULL)
	{
		instancePtr = uk_calloc(&a, 1, sizeof(XGpioPs));
	}
	UK_ASSERT(instancePtr);
	instancePtr.GpioConfig = configPtr;
	instancePtr.IsReady = ptr->gpio_data.ready;
	instancePtr.Handler = ptr->gpio_data.uk_gpio_handler;
	instancePtr.CallBackRef = ptr->gpio_data.callback_ref;
	instancePtr.Platform = ptr->gpio_data.platform;
	instancePtr.MaxPinNum = ptr->gpio_data.maxPinNum;
	instancePtr.MaxBanks = ptr->gpio_data.maxbanks;

	/* call the platform function to initialize the driver */
	return XGpioPs_CfgInitialize(&instancePtr, &configPtr, effectiveAddr);
}

uint32_t uk_read_pin(uk_gpio_data *ptr, uin32_t pin)
{
	if (instancePtr)
	{
		return XGpioPs_ReadPin(&instancePtr, pin);			
	}
}

void uk_write_pin(uk_gpio_data *ptr, uin32_t pin, uin32_t data)
{
	if (instancePtr)
	{
		XGpioPs_WritePin(&instancePtr, pin, data);			
	}
}

void uk_set_pin_direction(uk_gpio_data *ptr, uin32_t pin, uin32_t direction)
{
	if (instancePtr)
	{
		XGpioPs_SetDirectionPin(&instancePtr, pin, direction);			
	}
}

void uk_set_pin_output_enable(uk_gpio_data *ptr, uin32_t pin, uin32_t opEnable)
{
	if (instancePtr)
	{
		XGpioPs_SetOutputEnablePin(&instancePtr, pin, opEnable);			
	}
}

static void init_gpio_driver(void)
{
	gpio_data.deviceId = XPAR_PSU_GPIO_0_DEVICE_ID;
	gpio_data.baseAddr = XPAR_PSU_GPIO_0_BASEADDR;
	gpio_data.ready = XIL_COMPONENT_IS_STARTED; /* defined in xil_types.h */
	gpio_data.platform = XPLAT_ZYNQ_ULTRA_MP; /* defined in platform_info.h */
	gpio_data.maxPinNum = XGPIOPS_BANK_MAX_PINS;
	gpio_data.maxbanks = XGPIOPS_MAX_BANKS_ZYNQMP;
	gpio_data.uk_gpio_handler = &uk_gpio_handler_func;
	gpio_data.callback_ref = &uk_gpio_callback_func;

	bank_ops.read_data_register = &uk_bank_read;
	bank_ops.write_data_register = &uk_bank_write;
	bank_ops.set_direction = &uk_bank_setDirection;
	bank_ops.get_direction = &uk_bank_getDirection;
	bank_ops.set_output_enable = &uk_bank_setOutputEnable;
	bank_ops.get_output_enable = &uk_bank_getOutputEnable;
	bank_ops.get_bank_pin = &uk_bank_getbankPin;
	bank_ops.gpio_init = &uk_bank_initialize;

	pin_ops.read_data_pin = &uk_read_pin;
	pin_ops.write_data_pin = &uk_write_pin;
	pin_ops.set_direction_pin = &uk_set_pin_direction;
	pin_ops.get_direction_pin = &uk_get_pin_direction;
	pin_ops.set_output_enable_pin = &uk_set_pin_output_enable;
	pin_ops.get_output_enable_pin = &;uk_get_pin_output_enable;

	dev.gpio_data = &gpio_data;
	dev.bank_ops = &bank_ops;
	dev.pin_ops = &pin_ops;
	/* Register and initialize the driver */
	uk_gpio_drv_register(&dev);
	UK_ASSERT(uk_gpio_initialize(&dev, XPAR_PSU_GPIO_0_BASEADDR) == 0);
}

static void setLED(uint32_t pin_no)
{
	uk_set_pin_direction(&instancePtr, pin_no, DIRECTION_OUTPUT);
	uk_set_pin_output_enable(&instancePtr, pin_no, OUTPUT_ENABLED);
	uk_write_pin(&instancePtr, pin_no);
}

static void toggleLED(uint32_t pin_no)
{
	uint32_t pin_state;
	pin_state = uk_read_pin(&instancePtr, pin_no);
	uk_write_pin(&instancePtr, pin_no, !pin_state);

}

int main(int argc, char *argv[])
{
	init_gpio_driver();
	setLED(PIN_LED_OUTPUT);
	sleep(10);
	toggleLED(PIN_LED_OUTPUT);
}
