/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file px4-stm32f4ve_spi.c
 *
 * Board-specific SPI functions.
 */

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <px4_config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>
#include <unistd.h>

#include <nuttx/spi/spi.h>
#include <arch/board/board.h>

#include <up_arch.h>
#include <chip.h>
#include <stm32.h>
#include "board_config.h"
#include <systemlib/err.h>

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: stm32_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the PX4FMU board.
 *
 ************************************************************************************/

#ifdef CONFIG_STM32_SPI1
static void stm32_spi1_initialize(void)
{
	stm32_configgpio(GPIO_SPI1_CS_PC2);
	stm32_configgpio(GPIO_SPI1_CS_PD7);
	stm32_configgpio(GPIO_SPI1_CS_PC13);
	stm32_configgpio(GPIO_SPI1_CS_PC1);
}
#endif // CONFIG_STM32_SPI1

#ifdef CONFIG_STM32_SPI3
static void stm32_spi3_initialize(void)
{
	stm32_configgpio(GPIO_SPI3_CS_PB0);
}
#endif //CONFIG_STM32_SPI3

__EXPORT void stm32_spiinitialize(void)
{
#ifdef CONFIG_STM32_SPI1
	stm32_spi1_initialize();
#endif

#ifdef CONFIG_STM32_SPI2
	
#endif

//~ #ifdef CONFIG_STM32_SPI4
	//~ stm32_spi4_initialize();
//~ #endif
}

#ifdef CONFIG_STM32_SPI1
__EXPORT void stm32_spi1select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
	switch (devid) {
		
		/* FMUv2 SPI1 chip selects Assignments */
		//~ #define GPIO_SPI1_CS_MPU				GPIO_SPI_CS_GPIO_SPI1_CS_PC2  /* MPU9250 */
		//~ #define GPIO_SPI1_CS_GYRO				GPIO_SPI1_CS_PC13  /* L3GD20HTR */
		//~ #define GPIO_SPI1_CS_BARO				GPIO_SPI_CS_PD7  /* BMP280 */
		//~ #define GPIO_SPI1_CS_LPS331AP			GPIO_SPI_CS_PC1  /* LPS331AP Baro */

	case PX4_SPIDEV_ICM:

	/* Intended fallthrough */

	case PX4_SPIDEV_ICM_20602:

	/* Intended fallthrough */

	case PX4_SPIDEV_ICM_20608:

		/* Making sure the other peripherals are not selected */
		stm32_gpiowrite(GPIO_SPI1_CS_MPU, 1);
		stm32_gpiowrite(GPIO_SPI1_CS_GYRO, 1);
		stm32_gpiowrite(GPIO_SPI1_CS_BARO, 1);
		stm32_gpiowrite(GPIO_SPI1_CS_LPS331AP, 1);
		break;

	case PX4_SPIDEV_BARO:
		/* Making sure the other peripherals are not selected */
		stm32_gpiowrite(GPIO_SPI1_CS_MPU, 1);
		stm32_gpiowrite(GPIO_SPI1_CS_GYRO, 1);
		stm32_gpiowrite(GPIO_SPI1_CS_BARO, !selected);
		stm32_gpiowrite(GPIO_SPI1_CS_LPS331AP, 1);
		break;

	case PX4_SPIDEV_GYRO:
		/* Making sure the other peripherals are not selected */
		stm32_gpiowrite(GPIO_SPI1_CS_MPU, 1);
		stm32_gpiowrite(GPIO_SPI1_CS_GYRO, !selected);
		stm32_gpiowrite(GPIO_SPI1_CS_BARO, 1);
		stm32_gpiowrite(GPIO_SPI1_CS_LPS331AP, 1);
		break;

	case PX4_SPIDEV_MPU:
		/* Making sure the other peripherals are not selected */
		stm32_gpiowrite(GPIO_SPI1_CS_MPU, !selected);
		stm32_gpiowrite(GPIO_SPI1_CS_GYRO, 1);
		stm32_gpiowrite(GPIO_SPI1_CS_BARO, 1);
		stm32_gpiowrite(GPIO_SPI1_CS_LPS331AP, 1);
		break;

	case PX4_SPIDEV_LPS331AP:
		/* Making sure the other peripherals are not selected */
		stm32_gpiowrite(GPIO_SPI1_CS_MPU, 1);
		stm32_gpiowrite(GPIO_SPI1_CS_GYRO, 1);
		stm32_gpiowrite(GPIO_SPI1_CS_BARO, 1);
		stm32_gpiowrite(GPIO_SPI1_CS_LPS331AP, !selected);
		break;

	default:
		break;
	}
}

__EXPORT uint8_t stm32_spi1status(FAR struct spi_dev_s *dev, uint32_t devid)
{
	return SPI_STATUS_PRESENT;
}
#endif // CONFIG_STM32_SPI1

#ifdef CONFIG_STM32_SPI3
__EXPORT void stm32_spi3select(FAR struct spi_dev_s *dev, uint32_t devid, bool selected)
{
	/* there can only be one device on this bus, so always select it */
	stm32_gpiowrite(GPIO_SPI3_CS_PB0, !selected);
}

__EXPORT uint8_t stm32_spi3status(FAR struct spi_dev_s *dev, uint32_t devid)
{
	/* FRAM is always present */
	return SPI_STATUS_PRESENT;
}
#endif


__EXPORT void board_spi_reset(int ms)
{
	//~ stm32_configgpio(GPIO_SPI1_CS_PC2);
	//~ stm32_configgpio(GPIO_SPI1_CS_PD7);
	//~ stm32_configgpio(GPIO_SPI1_CS_PC13);
	//~ stm32_configgpio(GPIO_SPI1_CS_PC1);
	
	/* disable SPI bus */
	stm32_configgpio(_PIN_OFF(GPIO_SPI1_CS_PC2));
	stm32_configgpio(_PIN_OFF(GPIO_SPI1_CS_PC13));
	stm32_configgpio(_PIN_OFF(GPIO_SPI1_CS_PC1));
	stm32_configgpio(_PIN_OFF(GPIO_SPI1_CS_PD7));

	stm32_gpiowrite(_PIN_OFF(GPIO_SPI1_CS_PC2), 0);
	stm32_gpiowrite(_PIN_OFF(GPIO_SPI1_CS_PC13), 0);
	stm32_gpiowrite(_PIN_OFF(GPIO_SPI1_CS_PC1), 0);
	stm32_gpiowrite(_PIN_OFF(GPIO_SPI1_CS_PD7), 0);

	stm32_configgpio(_PIN_OFF(GPIO_SPI1_SCK));
	stm32_configgpio(_PIN_OFF(GPIO_SPI1_MISO));
	stm32_configgpio(_PIN_OFF(GPIO_SPI1_MOSI));

	stm32_gpiowrite(_PIN_OFF(GPIO_SPI1_SCK), 0);
	stm32_gpiowrite(_PIN_OFF(GPIO_SPI1_MISO), 0);
	stm32_gpiowrite(_PIN_OFF(GPIO_SPI1_MOSI), 0);

	//~ stm32_configgpio(_PIN_OFF(GPIO_SPI1_EXTI_DRDY_PB0));
	//~ stm32_configgpio(_PIN_OFF(GPIO_SPI1_EXTI_DRDY_PB1));
	//~ stm32_configgpio(_PIN_OFF(GPIO_SPI1_EXTI_DRDY_PB4));
	//~ stm32_configgpio(_PIN_OFF(GPIO_SPI1_EXTI_DRDY_PD15));

	//~ stm32_gpiowrite(_PIN_OFF(GPIO_SPI1_EXTI_DRDY_PB0), 0);
	//~ stm32_gpiowrite(_PIN_OFF(GPIO_SPI1_EXTI_DRDY_PB1), 0);
	//~ stm32_gpiowrite(_PIN_OFF(GPIO_SPI1_EXTI_DRDY_PB4), 0);
	//~ stm32_gpiowrite(_PIN_OFF(GPIO_SPI1_EXTI_DRDY_PD15), 0);

	/* set the sensor rail off */
	//~ stm32_configgpio(GPIO_VDD_3V3_SENSORS_EN);
	//~ stm32_gpiowrite(GPIO_VDD_3V3_SENSORS_EN, 0);

	/* wait for the sensor rail to reach GND */
	usleep(ms * 1000);
	warnx("reset done, %d ms", ms);

	/* re-enable power */

	/* switch the sensor rail back on */
	//~ stm32_gpiowrite(GPIO_VDD_3V3_SENSORS_EN, 1);

	/* wait a bit before starting SPI, different times didn't influence results */
	usleep(100);

	/* reconfigure the SPI pins */
	stm32_configgpio(GPIO_SPI1_SCK);
	stm32_configgpio(GPIO_SPI1_MISO);
	stm32_configgpio(GPIO_SPI1_MOSI);

	stm32_spi1_initialize();
}

//~ __EXPORT bool px4_spi_bus_external(int bus)
//~ {
	//~ if (HW_VER_FMUV3 == board_get_hw_version()) {
		//~ /* all FMUv3 2.1 spi buses are internal */
		//~ return false;

	//~ } else {
		//~ if (bus == PX4_SPI_BUS_EXT) {
			//~ return true;
		//~ }
	//~ }

	//~ return false;
//~ }
