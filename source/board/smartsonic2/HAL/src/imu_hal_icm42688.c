/*
<!-- BEGIN INVN LICENSE -->
TDK InvenSense 5 Clause License

Copyright (c) 2025, Invensense, Inc.

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

4. This software, with or without modification, must only be used with a
TDK InvenSense sensor.

5. Any software provided in binary form under this license, whether embedded
in source code or provided as a compiled library or application, must not
be reverse engineered, decompiled, modified and/or disassembled.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
<!-- END INVN LICENSE -->
*/

#include <ioport.h>
#include <pio.h>
#include <pio_handler.h>
#include <conf_board.h>

#include "chirp_smartsonic.h"

#include "spi_hal.h"
#include "time.h"
#include "imu_hal.h"
#include "io_expander.h"

#include "Invn\Drivers\Icm426xx\Icm426xxDefs.h"
#include "Invn\Drivers\Icm426xx\Icm426xxDriver_HL.h"
#include "Invn\Drivers\Icm426xx\Icm426xxTransport.h"
#include "Invn\Drivers\Icm426xx\Icm426xxExtFunc.h"

/* Define output data in us rate for ICM */
#define IMU_SAMPLE_RATE_TYP_200HZ_IN_US (5000) /* 5 ms <=> 200Hz */
#define IMU_SAMPLE_RATE_MAX_200HZ_IN_US (5250) /* +- 5% margin */
#define IMU_SAMPLE_RATE_MIN_200HZ_IN_US (4750) /* +- 5% margin */

extern uint64_t imu_isr_timestamp;

/* Icm426xx driver object */
static struct inv_icm426xx inv_icm426xx_dev;
/* Timestamp of previous IMU data sampling */
static uint64_t prev_timestamp    = 0;
static uint32_t imu_sampling_time = IMU_SAMPLE_RATE_TYP_200HZ_IN_US;

extern void imu_raw_ag_received_callback(uint64_t data_timestamp, int16_t *acc_data, int16_t *gyro_data);
extern void imu_data_ready_isr_callback(void);

static void evb_int_pins_int_handler(uint32_t int_id, uint32_t int_mask)
{
	if ((int_id & EVB_INT_PORT_ID) && (int_mask & EVB_INT_MASKS))
		imu_data_ready_isr_callback();
}

/****************************************************************************/
/*! Low-level serial interface function implementation for SPI
 */
/****************************************************************************/
static int imu_io_hal_read_reg(struct inv_icm426xx_serif *serif, uint8_t reg, uint8_t *buf, uint32_t len)
{
	int rc;
	(void)serif;
	spi_hal_cs_on(SPI_EVB_CSB);
	rc = spi_hal_read_register(SPI_BUS_IDX_EVB, reg, buf, len);
	spi_hal_cs_off(SPI_EVB_CSB);

	return rc;
}

static int imu_io_hal_write_reg(struct inv_icm426xx_serif *serif, uint8_t reg, const uint8_t *buf, uint32_t len)
{
	(void)serif;
	int rc = 0;

	spi_hal_cs_on(SPI_EVB_CSB);
	for (uint8_t reg_ofs = 0; reg_ofs < len; reg_ofs++)
		rc += spi_hal_write_register(SPI_BUS_IDX_EVB, reg + reg_ofs, &buf[reg_ofs], 1);
	spi_hal_cs_off(SPI_EVB_CSB);

	return rc;
}

/****************************************************************************/
/*! IMU Data callback
 */
/****************************************************************************/

static void imu_raw_data_received_callback(inv_icm426xx_sensor_event_t *event)
{
	uint64_t estimated_timestamp;
	uint32_t delta;

	if ((event->sensor_mask & (1 << INV_ICM426XX_SENSOR_ACCEL)) &&
	    (event->sensor_mask & (1 << INV_ICM426XX_SENSOR_GYRO))) {
		/* Reconstruct the timestamp if we have multiple elements in the FIFO */
		if ((imu_isr_timestamp - prev_timestamp) > IMU_SAMPLE_RATE_MAX_200HZ_IN_US) {
			estimated_timestamp = prev_timestamp + imu_sampling_time;
		} else {
			estimated_timestamp = imu_isr_timestamp;
			delta               = imu_isr_timestamp - prev_timestamp;
			if ((delta > IMU_SAMPLE_RATE_MIN_200HZ_IN_US) && (delta < IMU_SAMPLE_RATE_MAX_200HZ_IN_US))
				imu_sampling_time = delta;
		}
		prev_timestamp = estimated_timestamp;

		/* notify gyr, acc, timestamp_fsync and fsync_event to upper layer */
		imu_raw_ag_received_callback(estimated_timestamp, &event->accel[0], &event->gyro[0]);
	}
}

/****************************************************************************/
/*! Public API
 */
/****************************************************************************/

int8_t imu_hal_init(void)
{
	struct inv_icm426xx_serif icm426xx_serif;
	uint8_t who_am_i = 0;
	int8_t rc;

	printf("Booting up ICM426xx...\r\n");

	/* Init EVB SPI pins */
	spi_hal_init(SPI_BUS_IDX_EVB);
	/* Connect EVB SPI pins to DB Connector */
	io_exp_set_pins_level(PCA9535_EXPB_I2C_ADDRESS, IO_EXP_P1X_PORT, IO_EXPB_MSB_EVB_BUS_CTRL_SW, 0x00);

	/* Configure INT pins as input */
	pio_configure(EVB_INT_PORT, PIO_INPUT, EVB_INT_MASKS, EVB_INT_PINS_CONFIG);
	pio_handler_set(EVB_INT_PORT, EVB_INT_PORT_ID, EVB_INT_MASKS, EVB_INT_PINS_CONFIG, evb_int_pins_int_handler);
	pio_handler_set_priority(EVB_INT_PORT, EVB_INT_PINS_IRQ, INT_PRIO_EVB_INT_PINS);

	/* Initialize serial interface between MCU and Icm426xx */
	icm426xx_serif.serif_type = ICM426XX_UI_SPI4;
	icm426xx_serif.max_read   = 32768; /* 1024*32 */
	icm426xx_serif.max_write  = 32768; /* 1024*32 */
	icm426xx_serif.context    = 0;     /*no need */
	icm426xx_serif.read_reg   = imu_io_hal_read_reg;
	icm426xx_serif.write_reg  = imu_io_hal_write_reg;

	rc = inv_icm426xx_init(&inv_icm426xx_dev, &icm426xx_serif, imu_raw_data_received_callback);
	if (rc != INV_ERROR_SUCCESS) {
		printf("inv_icm426xx_init() failed\r\n");
		return rc;
	}

	/* Check WHOAMI */
	printf("Reading ICM426xx WHOAMI...\r\n");
	rc = inv_icm426xx_get_who_am_i(&inv_icm426xx_dev, &who_am_i);
	if (rc != INV_ERROR_SUCCESS) {
		printf("inv_icm426xx_get_who_am_i() failed\r\n");
		return rc;
	}

	if (who_am_i != ICM42688_WHOAMI) {
		printf("Unexpected WHOAMI value %d. Aborting setup\r\n", who_am_i);
		return INV_ERROR;
	} else {
		printf("ICM426xx WHOAMI value: 0x%x\r\n", who_am_i);
	}

	/* Configure sensors */
	rc |= inv_icm426xx_set_accel_fsr(&inv_icm426xx_dev, ICM426XX_ACCEL_CONFIG0_FS_SEL_16g);
	rc |= inv_icm426xx_set_gyro_fsr(&inv_icm426xx_dev, ICM426XX_GYRO_CONFIG0_FS_SEL_2000dps);

	rc |= inv_icm426xx_set_accel_frequency(&inv_icm426xx_dev, ICM426XX_ACCEL_CONFIG0_ODR_200_HZ);
	rc |= inv_icm426xx_set_gyro_frequency(&inv_icm426xx_dev, ICM426XX_GYRO_CONFIG0_ODR_200_HZ);

	return rc;
}

int8_t imu_hal_enable(void)
{
	/* Enable INT1 & INT2 interrupts to get IMU data */
	pio_enable_interrupt(EVB_INT_PORT, EVB_INT_MASKS);

	int8_t rc  = inv_icm426xx_enable_accel_low_noise_mode(&inv_icm426xx_dev);
	rc        |= inv_icm426xx_enable_gyro_low_noise_mode(&inv_icm426xx_dev);

	prev_timestamp = time_get_in_us();

	return rc;
}

int8_t imu_hal_disable(void)
{
	/* Disable INT1 & INT2 interrupts to get IMU data */
	pio_disable_interrupt(EVB_INT_PORT, EVB_INT_MASKS);

	int8_t rc  = inv_icm426xx_disable_accel(&inv_icm426xx_dev);
	rc        |= inv_icm426xx_disable_gyro(&inv_icm426xx_dev);

	return rc;
}

void imu_hal_start_data_read(void)
{
	/*
	 * Extract packets from FIFO. Callback defined at init time (i.e. imu_raw_data_received_callback)
	 * will be called for each valid packet extracted from FIFO.
	 */
	inv_icm426xx_get_data_from_fifo(&inv_icm426xx_dev);
}

/****************************************************************************/
/*! Sleep & Get time implementation for ICM426xx
 */
/****************************************************************************/
extern void inv_icm426xx_sleep_us(uint32_t us)
{
	time_delay_us(us);
}

uint64_t inv_icm426xx_get_time_us(void)
{
	return time_get_in_us();
}