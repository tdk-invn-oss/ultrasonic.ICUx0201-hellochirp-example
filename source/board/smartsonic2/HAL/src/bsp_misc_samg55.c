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

#include <asf.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <math.h>

#include "conf_board.h"
#include "app_config.h"

#include "bsp.h"
#include "chirp_smartsonic.h"
#include <invn/soniclib/soniclib.h>
#include <invn/soniclib/chirp_bsp.h>

#include "pio_handler.h"
#include "pdc.h"
#include "ioport.h"
#include "time.h"

#if defined(USE_USER_BUTTON) && (USE_USER_BUTTON == 1)
#include "button.h"
#endif
#include "i2c_hal.h"
#if defined(INCLUDE_SHASTA_SUPPORT)
#include "spi_hal.h"
#endif
#include "io_expander.h"

#define I2C_TIMEOUT_COUNT (10000)  // loop counter to detect timeout in I2C handler

#ifdef USE_IMU
extern void imu_data_ready_isr_callback(void);
#endif
#ifdef USE_BOARD_SYNC_MEAS
extern void board_sync_meas_ready_hal_isr_callback(void);
#endif
extern void icux_int_pins_handler(uint32_t status);
extern void chx01_int_pins_handler(uint32_t status);
extern ioport_port_mask_t get_icu_int_pins_mask(void);

/**
 * \brief Parallel IO Controller A interrupt handler.
 * Redefined PIOA interrupt handler for NVIC interrupt table.
 *
 * Important note: on Smartsonic board, due to the level shifter on the INT line,
 *    the ISR must be as short as possible to ensure the INT level is set back to
 *    0 logical level before the CHx01 is ready for a new measurement. The maximum
 *    time for ISR is 8 us.
 */
void PIOA_Handler(void)
{
	/* Read the ISR and IMR registers to know which interrupt(s) is(are) pending */
	uint32_t int_status = pio_get_interrupt_status(PIOA);
	uint32_t status     = int_status & pio_get_interrupt_mask(PIOA);
#if defined(INCLUDE_SHASTA_SUPPORT)
	/* For ICU interrupts, don't get real pio int status but
	 * get the pins mask of connected sensors
	 */
	uint32_t int_icu_status = int_status & get_icu_int_pins_mask();
#endif

#if 0
#ifdef USE_BOARD_SYNC_MEAS
	if (status & PIN_EXT_SyncMeasINT_MASK)
		board_sync_meas_ready_hal_isr_callback();
#endif
#endif

#if defined(INCLUDE_SHASTA_SUPPORT)
	if (int_icu_status & ICUx_INT_MASKS)
		icux_int_pins_handler(int_icu_status);
#endif
	if (status & CHx01_INT_MASKS)
		chx01_int_pins_handler(status);
	if (status & IO_EXP_INT_MASK)
		io_exp_int_pin_handler();
#if defined(USE_USER_BUTTON) && (USE_USER_BUTTON == 1)
	if (status & USER_BUTTON_MASK)
		button_isr_callback();
#endif
}

void FLEXCOM0_Handler(void)
{
	i2c_hal_interrupt_handler(I2C_BUS_IDX_CHx01);
}
void FLEXCOM2_Handler(void)
{
	i2c_hal_interrupt_handler(I2C_BUS_IDX_CONTROL);
}

void FLEXCOM5_Handler(void)
{
#if defined(INCLUDE_SHASTA_SUPPORT)
	spi_hal_interrupt_handler(SPI_BUS_IDX_ICU);
#endif
}

void RTT_Handler(void)
{
	rtt_get_status(RTT);
}

/* Interrupt handler for TC0/Channel0 peripheral */
void TC0_Handler(void)
{
	/* This handles the case of counter overflow on TC_CHANNEL_LSEPOCH and clears the status register */
	time_get_in_us();
}

/* Interrupt handler for TC0/Channel1 peripheral */
void TC1_Handler(void)
{
	uint32_t status   = tc_get_status(TC0, TC_CHANNEL_US);
	uint32_t int_mask = tc_get_interrupt_mask(TC0, TC_CHANNEL_US);

	if (status & (int_mask & TC_IMR_CPCS)) {
		periodic_timer_handler();
	}
}

void switch_board_leds(enum led_color color, bool on_off)
{
	uint8_t leds_pins_mask      = 0x00; /* default : leds on */
	uint8_t set_clear_pins_mask = 0x00;

	if (color & BOARD_LED_RED)
		leds_pins_mask |= IO_EXPB_MSB_LED_RED;
	if (color & BOARD_LED_GREEN)
		leds_pins_mask |= IO_EXPB_MSB_LED_GREEN;
	if (!on_off)
		set_clear_pins_mask = leds_pins_mask;
	io_exp_set_pins_level(PCA9535_EXPB_I2C_ADDRESS, IO_EXP_P1X_PORT, leds_pins_mask, set_clear_pins_mask);
}

void switch_user_led(bool on_off)
{
	switch_board_leds(BOARD_LED_GREEN, on_off);
}

void external_rtc_power_en(bool en)
{
	uint8_t set_clear_pins_mask;

	if (en) {
		set_clear_pins_mask = (IO_EXP_SET_LEVEL << IO_EXP_PIN_P1_POS);
	} else {
		set_clear_pins_mask = (IO_EXP_CLEAR_LEVEL << IO_EXP_PIN_P1_POS);
	}

	io_exp_set_pins_level(PCA9535_EXPA_I2C_ADDRESS, IO_EXP_P1X_PORT, IO_EXPA_MSB_OSC_CTL, set_clear_pins_mask);
}