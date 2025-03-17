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

#include <stdio.h>
#include <stdint.h>

#include <asf.h>
#include "matrix.h"
#include "rtt.h"
#include "sleep.h"
#include "twi.h"
#include "ioport.h"
#include "delay.h"
#include "pio_handler.h"

#include <invn/soniclib/soniclib.h>  // Chirp SonicLib API definitions
#include <invn/soniclib/chirp_bsp.h>

#include "board.h"  // board header in Atmel ASF
#include "conf_board.h"
#include "chirp_smartsonic.h"  // header with board-specific defines
#include "bsp.h"
#include "spi_hal.h"
#include "i2c_hal.h"
#include "console.h"
#include "time.h"
#include "io_expander.h"

#define SPI_BUFFER_SIZE 1040

#if defined(INCLUDE_SHASTA_SUPPORT)
#define NB_ICU_INT1_PIN (sizeof(icu_int1_pins) / sizeof(uint32_t))
#define NB_ICU_INT_PINS (NB_ICU_INT1_PIN + 1) /* 4 x INT1 + INT2 */
#else
#define NB_CHX01_INT_PIN  (sizeof(chx01_int_pins) / sizeof(uint32_t))
#define CHX01_INT_DIR_OUT (0)
#define CHX01_INT_DIR_IN  (1)
#endif

#define NB_SENSOR_LEDS (8)

#if defined(INCLUDE_SHASTA_SUPPORT)
const uint32_t icu_cs_pins[]              = {SPI_ICU_CSB0, SPI_ICU_CSB1, SPI_ICU_CSB2, SPI_ICU_CSB3};
static const uint32_t icu_int1_pins[]     = {ICUx_INT1_0, ICUx_INT1_1, ICUx_INT1_2, ICUx_INT1_3};
static const uint32_t icu_int1_pins_irq[] = {ICUx_INT1_0_MASK, ICUx_INT1_1_MASK, ICUx_INT1_2_MASK, ICUx_INT1_3_MASK};
static const uint32_t icu_int_pins_irq[]  = {ICUx_INT1_0_MASK, ICUx_INT1_1_MASK, ICUx_INT1_2_MASK, ICUx_INT1_3_MASK,
                                             ICUx_INT2_MASK};
void icux_user_int_handler(ch_group_t *sensor_group_ptr, uint8_t sensor_idx);
#endif
#if defined(INCLUDE_WHITNEY_SUPPORT)
static const uint32_t chx01_int_pins[]     = {CHx01_INT0, CHx01_INT1, CHx01_INT2, CHx01_INT3};
static const uint32_t chx01_int_pins_irq[] = {CHx01_INT0_MASK, CHx01_INT1_MASK, CHx01_INT2_MASK, CHx01_INT3_MASK};
static const uint8_t chx01_prog_pins[]     = {IO_EXPA_MSB_CHX01_PROG0, IO_EXPA_MSB_CHX01_PROG1, IO_EXPA_MSB_CHX01_PROG2,
                                              IO_EXPA_MSB_CHX01_PROG3};
static const uint8_t chx01_i2c_addrs[]     = CHX01_I2C_ADDRS;
static const uint8_t chx01_i2c_buses[]     = CHX01_I2C_BUSES;
#endif
/* Workaround on schematic (US-1014) : leds not aligned with sensors */
static const uint8_t sensors_leds_pins[] = {IO_EXPA_LSB_LED2, IO_EXPA_LSB_LED1, IO_EXPA_LSB_LED4, IO_EXPA_LSB_LED3,
                                            IO_EXPA_LSB_LED6, IO_EXPA_LSB_LED5, IO_EXPA_MSB_LED8, IO_EXPA_LSB_LED7};

/* Callback function pointers */
static ch_timer_callback_t periodic_timer_callback_ptr = NULL;

static uint16_t ultrasound_timer_period_in_tick = 0xFFFF;
static uint16_t ultrasound_prev_period_end_in_tick;

/* Counter used to decimate call to ultrasound timer callback from TC0 ISR in case decimation
   factor is != 1 */
static uint8_t decimation_counter = 0;

/* Used in case the timer resolution and range (16 bits HW counter) overflows */
static uint8_t decimation_factor;

static ch_group_t *sensor_group_ptr;

extern inline void icux_int_pins_handler(uint32_t status);
extern inline void chx01_int_pins_handler(uint32_t status);
extern ioport_port_mask_t get_icu_int_pins_mask(void);
static int get_lowest_connected_dev_num(void);

static inline ioport_port_mask_t group_to_pins_mask(ch_group_t *grp_ptr, const uint32_t *pins_list)
{
	ioport_port_mask_t mask = 0;

	for (uint8_t dev_num = 0; dev_num < ch_get_num_ports(grp_ptr); dev_num++) {
		ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);

		if (ch_sensor_is_connected(dev_ptr))
			mask |= ioport_pin_to_mask(pins_list[dev_num]);
	}
	return mask;
}

static uint32_t get_int_pin(ch_dev_t *dev_ptr)
{
#if defined(INCLUDE_SHASTA_SUPPORT)
	const uint32_t *int_pins = &icu_int1_pins[0];
#else
	const uint32_t *int_pins = &chx01_int_pins[0];
#endif
	return int_pins[ch_get_dev_num(dev_ptr)];
}

/**
 * @brief      Sets the chx01x interrupts dir.
 *
 * @param[in]  n_out_in  The n_out_in = 0 (Out) / 1 (In)
 */
static inline void set_chx01_int_dir(uint8_t n_out_in)
{
	/* DIR = 0 : B to A => sensor to MCU => CHx01_INTx = IN
	 * DIR = 1 : A to B => MCU to sensor => CHx01_INTx = OUT */
	/* TODO : what is done not aligned with schematic/datasheet */
	ioport_set_pin_level(CHx01_INT_DIR, n_out_in ? IOPORT_PIN_LEVEL_LOW : IOPORT_PIN_LEVEL_HIGH);
}

static void sensors_pin_init(void)
{
	pmc_enable_periph_clk(ICUx_INT_PORT_ID);

	ioport_set_port_dir(ICUx_INT_PINS_PORT, ICUx_INT_MASKS, IOPORT_DIR_INPUT);

	/* Enable pull-ups on the INT pins */
	pio_pull_up(ICUx_INT_PORT, ICUx_INT_PINS, 1);
	pio_pull_down(ICUx_INT_PORT, ICUx_INT_PINS, 0);

	/* Configure INT pins as input */
	pio_configure(ICUx_INT_PORT, PIO_INPUT, ICUx_INT_MASKS, PIO_DEFAULT);
#if defined(INCLUDE_WHITNEY_SUPPORT)
	pio_configure(CHx01_INT_PORT, PIO_INPUT, CHx01_INT_MASKS, PIO_DEFAULT);
	ioport_set_pin_dir(CHx01_INT_DIR, IOPORT_DIR_OUTPUT);
	set_chx01_int_dir(CHX01_INT_DIR_IN);
#endif

	/* Configure MUTCLK as output with high level (not used by ICU sensors as clk) */
	pio_configure(ICUx_MUTCLK_PORT, PIO_OUTPUT_1, ICUx_MUTCLK_MASK, PIO_OUTPUT_1);

	/* Initialize sensors INT pins interrupts */
	pio_handler_set(ICUx_INT_PORT, ICUx_INT_PORT_ID, ICUx_INT_MASKS, ICUx_INT_PIN_CONFIG, NULL);
#if defined(INCLUDE_WHITNEY_SUPPORT)
	pio_handler_set(CHx01_INT_PORT, CHx01_INT_PORT_ID, CHx01_INT_MASKS, CHx01_INT_PIN_CONFIG, NULL);
#endif
	pio_handler_set_priority(ICUx_INT_PORT, ICUx_INT_PIN_IRQ, INT_PRIO_ICUx_INT_PINS);

	/* Disable all ICU and CHx interrupts */
	pio_disable_interrupt(ICUx_INT_PORT, ICUx_INT_MASKS);
#if defined(INCLUDE_WHITNEY_SUPPORT)
	pio_disable_interrupt(CHx01_INT_PORT, CHx01_INT_MASKS);
#endif
}

static int i2c_init(void)
{
	int rc  = i2c_hal_init(I2C_BUS_IDX_CONTROL);
	rc     |= i2c_hal_init(I2C_BUS_IDX_CHx01);

	return rc;
}

void i2c_hal_read_nb_done_cbk(uint8_t bus_index)
{
	if (I2C_BUS_IDX_CHx01 == bus_index)
		/* Notify sensor driver that this transaction is complete */
		ch_io_notify(sensor_group_ptr, 0);
}

#if defined(INCLUDE_SHASTA_SUPPORT)
static void spi_init(void)
{
	spi_hal_init(SPI_BUS_IDX_ICU);
}

void spi_hal_read_nb_done_cbk(uint8_t bus_index)
{
	if (SPI_BUS_IDX_ICU == bus_index)
		/* Notify sensor driver that this transaction is complete */
		ch_io_notify(sensor_group_ptr, 0);
}
#endif

/* Configure initial state of pins on IO Expander */
static void io_exp_config(void)
{
	io_exp_configure_pins(PCA9535_EXPA_I2C_ADDRESS, IO_EXP_P0X_PORT, IO_EXP_PIN_ALL, IO_EXPA_LSB_PINS_DIR, 0x00,
	                      IO_EXPA_LSB_PINS_DFT_VAL);
	io_exp_configure_pins(PCA9535_EXPA_I2C_ADDRESS, IO_EXP_P1X_PORT, IO_EXP_PIN_ALL, IO_EXPA_MSB_PINS_DIR, 0x00,
	                      IO_EXPA_MSB_PINS_DFT_VAL);
	io_exp_configure_pins(PCA9535_EXPB_I2C_ADDRESS, IO_EXP_P0X_PORT, IO_EXP_PIN_ALL, IO_EXPB_LSB_PINS_DIR, 0x00,
	                      IO_EXPB_LSB_PINS_DFT_VAL);
	io_exp_configure_pins(PCA9535_EXPB_I2C_ADDRESS, IO_EXP_P1X_PORT, IO_EXP_PIN_ALL, IO_EXPB_MSB_PINS_DIR, 0x00,
	                      IO_EXPB_MSB_PINS_DFT_VAL);
}

static void timers_init(void)
{
	/* Configure the PMC to enable the TC module and channels */
	sysclk_enable_peripheral_clock(ID_TC0);
	sysclk_enable_peripheral_clock(ID_TC1);
	sysclk_enable_peripheral_clock(ID_TC2);
	/* Create on PCK3 a 499985 Hz clock from the PLLA clock. */
	pmc_disable_pck(PMC_PCK_3);
	pmc_switch_pck_to_pllack(PMC_PCK_3, PMC_PCK_PRES(240 - 1));
	pmc_enable_pck(PMC_PCK_3);

	/* Reset all TC0 counters */
	TC0->TC_BCR = TC_BCR_SYNC;

	/* Enable TC0 - Channel 0 interrupt */
	NVIC_DisableIRQ(TC0_IRQn);
	NVIC_ClearPendingIRQ(TC0_IRQn);
	NVIC_SetPriority(TC0_IRQn, INT_PRIO_TIMER_EPOCH);
	NVIC_EnableIRQ(TC0_IRQn);

	/* Enable TC0 - Channel 1 interrupt */
	NVIC_DisableIRQ(TC1_IRQn);
	NVIC_ClearPendingIRQ(TC1_IRQn);
	NVIC_SetPriority(TC1_IRQn, INT_PRIO_TIMER_US);
	NVIC_EnableIRQ(TC1_IRQn);

	/* Create the lsepoch timer running on PCK3 and start it immediately */
	tc_init(TC0, TC_CHANNEL_LSEPOCH, TC_CMR_TCCLKS_TIMER_CLOCK5 | TC_CMR_WAVE | TC_CMR_WAVSEL_UP);
	tc_enable_interrupt(TC0, TC_CHANNEL_LSEPOCH, TC_IER_COVFS);
	tc_start(TC0, TC_CHANNEL_LSEPOCH);

	/* Create the ultrasound periodic timer. */
	tc_init(TC0, TC_CHANNEL_US, TC_CMR_TCCLKS_TIMER_CLOCK5 | TC_CMR_WAVE | TC_CMR_WAVSEL_UP);

	/* Create the ultrasound delay timer. : TIMER_CLOCK3 = MCK/32 */
	tc_init(TC0, TC_CHANNEL_DELAY, TC_CMR_TCCLKS_TIMER_CLOCK3 | TC_CMR_WAVE | TC_CMR_WAVSEL_UP);
}

static void misc_pin_init(void)
{
	/* Debug pins */
	/* configure system pins as PIO */
	uint32_t sys_io  = matrix_get_system_io();
	sys_io          |= (DBG0_SYSIO_MASK | DBG1_SYSIO_MASK);
	matrix_set_system_io(sys_io);
	/* Configure PIO */
	ioport_set_pin_dir(DBG0, IOPORT_DIR_OUTPUT);
	ioport_set_pin_dir(DBG1, IOPORT_DIR_OUTPUT);
	ioport_set_pin_level(DBG0, IOPORT_PIN_LEVEL_LOW);
	ioport_set_pin_level(DBG1, IOPORT_PIN_LEVEL_LOW);

	/* TODO */
	// pio_handler_set(ICUx_INT_PORT, PIN_EXT_INTERRUPT_ID, PIN_EXT_SyncMeasINT_MASK,
	// 	PIN_SyncMeas_INTERRUPT_ATTR, NULL);
	// pio_enable_interrupt(PIN_EXT_INTERRUPT_PIO, PIN_EXT_SyncMeasINT_MASK);
	// pio_disable_interrupt(ICUx_INT_PORT, PIN_EXT_SyncMeasINT_MASK);
}

static void evb_connector_pins_init(void)
{
	// pio_handler_set(ICUx_INT_PORT, ICUx_INT_PORT_ID, PIN_EXT_MotionINT_MASK,
	// 	ICUx_INT_PIN_CONFIG, NULL);
	/* Disable Motion interrupt before IMU initialization */
	// pio_disable_interrupt(ICUx_INT_PORT, PIN_EXT_MotionINT_MASK);
}

static void sensor_leds_indicate_alive(void)
{
	uint8_t leds_pins_mask_lsb = 0x00;
	uint8_t leds_pins_mask_msb = 0x00;

	for (uint8_t led_idx = 0; led_idx < NB_SENSOR_LEDS; led_idx++) {
		/* warning : this test is to command IO_EXPA_MSB_LED8 which is not on LSB port of IO EXPA */
		if (led_idx == 6)
			leds_pins_mask_msb |= sensors_leds_pins[led_idx];
		else
			leds_pins_mask_lsb |= sensors_leds_pins[led_idx];
	}
	/* All Leds on */
	io_exp_set_pins_level(PCA9535_EXPA_I2C_ADDRESS, IO_EXP_P0X_PORT, leds_pins_mask_lsb, leds_pins_mask_lsb);
	io_exp_set_pins_level(PCA9535_EXPA_I2C_ADDRESS, IO_EXP_P1X_PORT, leds_pins_mask_msb, leds_pins_mask_msb);
	/* light up for 1s */
	delay_s(1);
	/* All Leds off */
	io_exp_set_pins_level(PCA9535_EXPA_I2C_ADDRESS, IO_EXP_P0X_PORT, leds_pins_mask_lsb, 0x00);
	io_exp_set_pins_level(PCA9535_EXPA_I2C_ADDRESS, IO_EXP_P1X_PORT, leds_pins_mask_msb, 0x00);
}

static uint32_t get_period_in_tick(uint32_t interval_us)
{
	uint64_t timer_period_in_tick = (uint64_t)ULTRASOUND_TIMER_FREQUENCY * interval_us / 1000000;

	/* If the ODR is too slow to be handled then program a faster interrupt and decimate it */
	if (timer_period_in_tick > UINT16_MAX)
		decimation_factor = timer_period_in_tick / UINT16_MAX + 1;
	else
		decimation_factor = 1;

	/* Calculate the final tick in case a decimation is needed */
	return (uint32_t)(timer_period_in_tick / decimation_factor);
}

static void program_next_period(void)
{
	uint32_t time                      = ultrasound_prev_period_end_in_tick + ultrasound_timer_period_in_tick;
	ultrasound_prev_period_end_in_tick = time;
	tc_write_rc(TC0, TC_CHANNEL_US, (uint16_t)(time & 0xFFFF));
}

inline void icux_int_pins_handler(uint32_t status)
{
#if defined(INCLUDE_SHASTA_SUPPORT)
	for (uint8_t sensor_idx = 0; sensor_idx < NB_ICU_INT_PINS; sensor_idx++) {
		if (status & icu_int_pins_irq[sensor_idx]) {
			/* Int happened */
			if (sensor_idx == 4) {
				/* INT2 pin => same pin for all sensors */
				printf("INT detected : Error INT2 pin not managed\r\n");
				break;
			}
#ifdef USE_MINIMAL_INT_HANDLER
			uint8_t group_status = ch_minimal_int_handler(sensor_group_ptr, sensor_idx);
			if (!group_status) {
				icux_user_int_handler(sensor_group_ptr, sensor_idx);
			}
#else
			ch_interrupt(sensor_group_ptr, sensor_idx);
#endif
		}
	}
#else
	(void)status;
#endif
}

inline void chx01_int_pins_handler(uint32_t status)
{
#if defined(INCLUDE_WHITNEY_SUPPORT)
	for (uint8_t sensor_idx = 0; sensor_idx < NB_CHX01_INT_PIN; sensor_idx++) {
		if (status & chx01_int_pins_irq[sensor_idx]) {
			/* Int happened */
			ch_interrupt(sensor_group_ptr, sensor_idx);
		}
	}
#else
	(void)status;
#endif
}

#if defined(INCLUDE_SHASTA_SUPPORT)
/* Public function to return the pin mask of ICU INT pins for connected sensors */
ioport_port_mask_t get_icu_int_pins_mask(void)
{
	return group_to_pins_mask(sensor_group_ptr, icu_int1_pins);
}
#endif

void bsp_init(ch_group_t *grp_ptr)
{
	/* Make local copy of group pointer */
	sensor_group_ptr = grp_ptr;

	/* Initialize the SAM system. */
	sysclk_init();
	WDT->WDT_MR = WDT_MR_WDDIS;
	ioport_init();
	pmc_enable_periph_clk(ID_PIOA);
	pmc_enable_periph_clk(ID_PIOB);

	/* Enable printf ASAP */
	console_init();

	i2c_init();
#if defined(INCLUDE_SHASTA_SUPPORT)
	spi_init();
#endif
	timers_init();
	time_init();

	io_exp_init();
	/* Configure pins on IO expanders to be able to init sensors
	 * All board leds are connected on IO Exp too */
	io_exp_config();

	sensors_pin_init();
	misc_pin_init();
	evb_connector_pins_init();

	/* Blink sensor leds at end of init to indicate end of boot */
	sensor_leds_indicate_alive();
}

uint8_t periodic_timer_init(uint16_t interval_ms, ch_timer_callback_t callback_func_ptr)
{
	/* Save callback function */
	periodic_timer_callback_ptr = callback_func_ptr;

	/* Convert the ODR in ms to ticks */
	ultrasound_timer_period_in_tick = get_period_in_tick(interval_ms * 1000);

	return 0;
}

void periodic_timer_change_period(uint32_t new_interval_us)
{
	uint16_t prev_expiration = ultrasound_prev_period_end_in_tick - ultrasound_timer_period_in_tick;

	ultrasound_timer_period_in_tick    = get_period_in_tick(new_interval_us);
	ultrasound_prev_period_end_in_tick = prev_expiration;
	program_next_period();
}

void periodic_timer_irq_enable(void)
{
	/* Clear any pending CPCS before enabling it */
	tc_get_status(TC0, TC_CHANNEL_US);
	tc_enable_interrupt(TC0, TC_CHANNEL_US, TC_IER_CPCS);
}

void periodic_timer_irq_disable(void)
{
	tc_disable_interrupt(TC0, TC_CHANNEL_US, TC_IDR_CPCS);
}

uint8_t periodic_timer_start(void)
{
	decimation_counter = 0;
	/* The timer start done at the very end is resetting the counter */
	ultrasound_prev_period_end_in_tick = 0;
	program_next_period();

	/* Start the HW counter (this resets the counter */
	tc_start(TC0, TC_CHANNEL_US);

	return 0;
}

uint8_t periodic_timer_stop(void)
{
	tc_stop(TC0, TC_CHANNEL_US);
	return 0;
}

void periodic_timer_handler(void)
{
	ch_timer_callback_t func_ptr = periodic_timer_callback_ptr;

	decimation_counter++;
	program_next_period();
	if (decimation_counter >= decimation_factor) {
		decimation_counter = 0;
		if (func_ptr != NULL) {
			(*func_ptr)();  // call application timer callback routine
		}
	}
}

void proc_sleep(void)
{
	/* use sleep mode defined in chirp_smartsonic.h */
	pmc_sleep(PROC_SLEEP_MODE);
}

void bsp_led_on(uint8_t led_num)
{
#if defined(INCLUDE_WHITNEY_SUPPORT)
	/* trick because we handle only 4 sensors at a time (4 whitney or 4 shasta) */
	led_num += 4;
#endif
	/* IO_EXPA_LSB_LEDx = 0 */
	uint8_t port = (led_num == 6) ? IO_EXP_P1X_PORT : IO_EXP_P0X_PORT; /* all leds are on EXPA LSB except LED8 on MSB */
	io_exp_set_pins_level(PCA9535_EXPA_I2C_ADDRESS, port, sensors_leds_pins[led_num], sensors_leds_pins[led_num]);
}

void bsp_led_off(uint8_t led_num)
{
#if defined(INCLUDE_WHITNEY_SUPPORT)
	/* trick because we handle only 4 sensors at a time (4 whitney or 4 shasta) */
	led_num += 4;
#endif
	/* IO_EXPA_LSB_LEDx = 1 */
	uint8_t port = (led_num == 6) ? IO_EXP_P1X_PORT : IO_EXP_P0X_PORT; /* all leds are on EXPA LSB except LED8 on MSB */
	io_exp_set_pins_level(PCA9535_EXPA_I2C_ADDRESS, port, sensors_leds_pins[led_num], 0x00);
}

void bsp_led_toggle(uint8_t led_num)
{
#if defined(INCLUDE_WHITNEY_SUPPORT)
	/* trick because we handle only 4 sensors at a time (4 whitney or 4 shasta) */
	led_num += 4;
#endif
	uint8_t port = (led_num == 6) ? IO_EXP_P1X_PORT : IO_EXP_P0X_PORT; /* all leds are on EXPA LSB except LED8 on MSB */
	io_exp_toggle_pins_level(PCA9535_EXPA_I2C_ADDRESS, port, sensors_leds_pins[led_num]);
}

void chbsp_set_int1_dir_out(ch_dev_t *dev_ptr)
{
#if defined(INCLUDE_WHITNEY_SUPPORT)
	set_chx01_int_dir(CHX01_INT_DIR_OUT);
#endif
	ioport_set_pin_dir(get_int_pin(dev_ptr), IOPORT_DIR_OUTPUT);
}

void chbsp_set_int1_dir_in(ch_dev_t *dev_ptr)
{
#if defined(INCLUDE_WHITNEY_SUPPORT)
	set_chx01_int_dir(CHX01_INT_DIR_IN);
#endif
	ioport_set_pin_dir(get_int_pin(dev_ptr), IOPORT_DIR_INPUT);
}

void chbsp_int1_clear(ch_dev_t *dev_ptr)
{
	ioport_set_pin_level(get_int_pin(dev_ptr), IOPORT_PIN_LEVEL_LOW);
}

void chbsp_int1_set(ch_dev_t *dev_ptr)
{
	ioport_set_pin_level(get_int_pin(dev_ptr), IOPORT_PIN_LEVEL_HIGH);
}

void chbsp_int1_interrupt_enable(ch_dev_t *dev_ptr)
{
	if (ch_sensor_is_connected(dev_ptr)) {
		uint8_t dev_num = ch_get_dev_num(dev_ptr);
		/* Warning : enable interrupt AND set pin as input */
#if defined(INCLUDE_WHITNEY_SUPPORT)
		set_chx01_int_dir(CHX01_INT_DIR_IN);
#endif
		ioport_set_pin_dir(get_int_pin(dev_ptr), IOPORT_DIR_INPUT);
#if defined(INCLUDE_WHITNEY_SUPPORT)
		pio_enable_interrupt(CHx01_INT_PORT, chx01_int_pins_irq[dev_num]);
#else
		/* Warning :
		 * pio_get_interrupt_status() clear all interrupts from port
		 * In hw trigger mode we enable interrupts just after triggering sensor so we shouldn't miss another interrupt
		 * In freerun we shall enable interrupts at init and don't change pins after so we shouln't miss interrupts
		 */
		if (ch_get_mode(dev_ptr) != CH_MODE_FREERUN)
			pio_get_interrupt_status(ICUx_INT_PORT);
		pio_enable_interrupt(ICUx_INT_PORT, icu_int1_pins_irq[dev_num]);
#endif
	}
}

void chbsp_int1_interrupt_disable(ch_dev_t *dev_ptr)
{
	if (ch_sensor_is_connected(dev_ptr)) {
		uint8_t dev_num = ch_get_dev_num(dev_ptr);
#if defined(INCLUDE_WHITNEY_SUPPORT)
		pio_disable_interrupt(CHx01_INT_PORT, chx01_int_pins_irq[dev_num]);
#else
		pio_disable_interrupt(ICUx_INT_PORT, icu_int1_pins_irq[dev_num]);
#endif
	}
}

static int get_lowest_connected_dev_num()
{
	for (int dev_num = 0; dev_num < sensor_group_ptr->num_ports; dev_num++) {
		ch_dev_t *dev_ptr = ch_get_dev_ptr(sensor_group_ptr, dev_num);
		if (ch_sensor_is_connected(dev_ptr)) {
			return dev_num;
		}
	}
	return -1;  // invalid
}

void chbsp_set_int2_dir_out(ch_dev_t *dev_ptr)
{
	// only control int2 for lowest connected device number since int2 is shared
	int dev_num = get_lowest_connected_dev_num();
	if (dev_ptr == NULL || ((int)ch_get_dev_num(dev_ptr)) == dev_num) {
		ioport_set_pin_dir(ICUx_INT2, IOPORT_DIR_OUTPUT);
	}
}

void chbsp_set_int2_dir_in(ch_dev_t *dev_ptr)
{
	// only control int2 for lowest connected device number since int2 is shared
	int dev_num = get_lowest_connected_dev_num();
	if (dev_ptr == NULL || ((int)ch_get_dev_num(dev_ptr)) == dev_num) {
		ioport_set_pin_dir(ICUx_INT2, IOPORT_DIR_INPUT);
	}
}

void chbsp_int2_clear(ch_dev_t *dev_ptr)
{
	// only control int2 for lowest connected device number since int2 is shared
	int dev_num = get_lowest_connected_dev_num();
	if (dev_ptr == NULL || ((int)ch_get_dev_num(dev_ptr)) == dev_num) {
		ioport_set_pin_level(ICUx_INT2, IOPORT_PIN_LEVEL_LOW);
	}
}

void chbsp_int2_set(ch_dev_t *dev_ptr)
{
	// only control int2 for lowest connected device number since int2 is shared
	int dev_num = get_lowest_connected_dev_num();
	if (dev_ptr == NULL || ((int)ch_get_dev_num(dev_ptr)) == dev_num) {
		ioport_set_pin_level(ICUx_INT2, IOPORT_PIN_LEVEL_HIGH);
	}
}

void chbsp_int2_interrupt_enable(ch_dev_t __attribute__((unused)) * dev_ptr)
{
	/* Warning : enable interrupt AND set pin as input */
	ioport_set_pin_dir(ICUx_INT2_MASK, IOPORT_DIR_INPUT);
	pio_enable_interrupt(ICUx_INT_PORT, ICUx_INT2_MASK);
}

void chbsp_int2_interrupt_disable(ch_dev_t __attribute__((unused)) * dev_ptr)
{
	pio_disable_interrupt(ICUx_INT_PORT, ICUx_INT2_MASK);
}

void chbsp_group_set_int1_dir_out(ch_group_t *grp_ptr)
{
#if defined(INCLUDE_WHITNEY_SUPPORT)
	ioport_port_mask_t mask_chx01 = group_to_pins_mask(grp_ptr, chx01_int_pins);

	set_chx01_int_dir(CHX01_INT_DIR_OUT);
	ioport_set_port_dir(CHx01_INT_PINS_PORT, mask_chx01, IOPORT_DIR_OUTPUT);
#else
	ioport_port_mask_t mask_icu = group_to_pins_mask(grp_ptr, icu_int1_pins);

	ioport_set_port_dir(ICUx_INT_PINS_PORT, mask_icu, IOPORT_DIR_OUTPUT);
#endif
}

void chbsp_group_set_int1_dir_in(ch_group_t *grp_ptr)
{
#if defined(INCLUDE_WHITNEY_SUPPORT)
	ioport_port_mask_t mask_chx01 = group_to_pins_mask(grp_ptr, chx01_int_pins);

	set_chx01_int_dir(CHX01_INT_DIR_OUT);
	ioport_set_port_dir(CHx01_INT_PINS_PORT, mask_chx01, IOPORT_DIR_INPUT);
#else
	ioport_port_mask_t mask_icu = group_to_pins_mask(grp_ptr, icu_int1_pins);

	ioport_set_port_dir(ICUx_INT_PINS_PORT, mask_icu, IOPORT_DIR_INPUT);
#endif
}

void chbsp_group_int1_clear(ch_group_t *grp_ptr)
{
#if defined(INCLUDE_WHITNEY_SUPPORT)
	ioport_port_mask_t mask_chx01 = group_to_pins_mask(grp_ptr, chx01_int_pins);

	ioport_set_port_level(CHx01_INT_PINS_PORT, mask_chx01, IOPORT_DIR_INPUT);
#else
	ioport_port_mask_t mask_icu = group_to_pins_mask(grp_ptr, icu_int1_pins);

	ioport_set_port_level(ICUx_INT_PINS_PORT, mask_icu, IOPORT_PIN_LEVEL_LOW);
#endif
}

void chbsp_group_int1_set(ch_group_t *grp_ptr)
{
#if defined(INCLUDE_WHITNEY_SUPPORT)
	ioport_port_mask_t mask_chx01 = group_to_pins_mask(grp_ptr, chx01_int_pins);

	ioport_set_port_level(CHx01_INT_PINS_PORT, mask_chx01, IOPORT_PIN_LEVEL_HIGH);
#else
	ioport_port_mask_t mask_icu = group_to_pins_mask(grp_ptr, icu_int1_pins);

	ioport_set_port_level(ICUx_INT_PINS_PORT, mask_icu, IOPORT_PIN_LEVEL_HIGH);
#endif
}

void chbsp_group_int1_interrupt_enable(ch_group_t *grp_ptr)
{
	for (uint8_t dev_num = 0; dev_num < ch_get_num_ports(grp_ptr); dev_num++)
		chbsp_int1_interrupt_enable(ch_get_dev_ptr(grp_ptr, dev_num));
}

void chbsp_group_int1_interrupt_disable(ch_group_t *grp_ptr)
{
	for (uint8_t dev_num = 0; dev_num < ch_get_num_ports(grp_ptr); dev_num++)
		chbsp_int1_interrupt_disable(ch_get_dev_ptr(grp_ptr, dev_num));
}

void chbsp_group_set_int2_dir_in(ch_group_t __attribute__((unused)) * grp_ptr)
{
	/* On Smartsonic V2 board, only one INT2 pin */
	chbsp_set_int2_dir_in(NULL);
}

void chbsp_group_set_int2_dir_out(ch_group_t __attribute__((unused)) * grp_ptr)
{
	/* On Smartsonic V2 board, only one INT2 pin */
	chbsp_set_int2_dir_out(NULL);
}

void chbsp_group_int2_clear(ch_group_t __attribute__((unused)) * grp_ptr)
{
	/* On Smartsonic V2 board, only one INT2 pin */
	chbsp_int2_clear(NULL);
}

void chbsp_group_int2_set(ch_group_t __attribute__((unused)) * grp_ptr)
{
	/* On Smartsonic V2 board, only one INT2 pin */
	chbsp_int2_set(NULL);
}

void chbsp_group_int2_interrupt_enable(ch_group_t __attribute__((unused)) * grp_ptr)
{
	chbsp_int2_interrupt_enable(NULL);
}

void chbsp_group_int2_interrupt_disable(ch_group_t __attribute__((unused)) * grp_ptr)
{
	chbsp_int2_interrupt_disable(NULL);
}

#if defined(INCLUDE_SHASTA_SUPPORT)

/* Disable or Enable the INT pin interrupt of others sensor on same bus */
static void switch_sensors_int(ch_group_t *group_ptr, bool interrupt_enable)
{
	uint32_t mask_connected_int_pins = 0;
	/* This loop depends on board schematic
		We consider all sensors are :
	  - on the same SPI bus
	  - on the same INT port
	*/
	for (uint8_t dev_num = 0; dev_num < group_ptr->num_ports; dev_num++) {
		ch_dev_t *dev_ptr = ch_get_dev_ptr(group_ptr, dev_num);
		if (dev_ptr->sensor_connected)
			mask_connected_int_pins |= icu_int1_pins_irq[dev_num];
	}
	if (mask_connected_int_pins) {
		cpu_irq_enter_critical();
		if (interrupt_enable) {
			pio_enable_interrupt(ICUx_INT_PORT, mask_connected_int_pins);
		} else {
			pio_disable_interrupt(ICUx_INT_PORT, mask_connected_int_pins);
		}
		cpu_irq_leave_critical();
	}
}

void chbsp_spi_cs_on(ch_dev_t *dev_ptr)
{
	/* Disable interrupt of other sensors on bus to avoid deadlock on SPI bus mutex */
	switch_sensors_int(sensor_group_ptr, false);
	spi_hal_cs_on(icu_cs_pins[ch_get_dev_num(dev_ptr)]);
}

void chbsp_spi_cs_off(ch_dev_t *dev_ptr)
{
	spi_hal_cs_off(icu_cs_pins[ch_get_dev_num(dev_ptr)]);
	/* Re-enable interrupt of all sensors*/
	switch_sensors_int(sensor_group_ptr, true);
}

int chbsp_spi_write(ch_dev_t __attribute__((unused)) * dev_ptr, const uint8_t *data, uint16_t num_bytes)
{
	return spi_hal_write(SPI_BUS_IDX_ICU, data, num_bytes);
}

int chbsp_spi_read(ch_dev_t __attribute__((unused)) * dev_ptr, uint8_t *data, uint16_t num_bytes)
{
	return spi_hal_read(SPI_BUS_IDX_ICU, data, num_bytes);
}

int chbsp_spi_mem_read_nb(ch_dev_t __attribute__((unused)) * dev_ptr, uint16_t mem_addr, uint8_t *data,
                          uint16_t num_bytes)
{
	(void)mem_addr; /* unused */
	return spi_hal_read_nb(SPI_BUS_IDX_ICU, data, num_bytes);
}

#endif /* defined(INCLUDE_SHASTA_SUPPORT) */

void chbsp_reset_assert(void)
{
	/* CHX01_RST_N = 0 */
	io_exp_set_pins_level(PCA9535_EXPA_I2C_ADDRESS, IO_EXP_P1X_PORT, IO_EXPA_MSB_CHX01_RST, IO_EXPA_MSB_CHX01_RST);
}

void chbsp_reset_release(void)
{
	/* CHX01_RST_N = 1 */
	io_exp_set_pins_level(PCA9535_EXPA_I2C_ADDRESS, IO_EXP_P1X_PORT, IO_EXPA_MSB_CHX01_RST, 0x00);
}

void chbsp_program_enable(ch_dev_t *dev_ptr)
{
#if defined(INCLUDE_WHITNEY_SUPPORT)
	/* IO_EXPA_MSB_CHX01_PROGx = 1 */
	uint8_t dev_num = ch_get_dev_num(dev_ptr);
	io_exp_set_pins_level(PCA9535_EXPA_I2C_ADDRESS, IO_EXP_P1X_PORT, chx01_prog_pins[dev_num],
	                      chx01_prog_pins[dev_num]);
#else
	(void)dev_ptr;
#endif
}

void chbsp_program_disable(ch_dev_t *dev_ptr)
{
#if defined(INCLUDE_WHITNEY_SUPPORT)
	/* IO_EXPA_MSB_CHX01_PROGx = 0 */
	uint8_t dev_num = ch_get_dev_num(dev_ptr);
	io_exp_set_pins_level(PCA9535_EXPA_I2C_ADDRESS, IO_EXP_P1X_PORT, chx01_prog_pins[dev_num], 0x00);
#else
	(void)dev_ptr;
#endif
}

int chbsp_i2c_init(void)
{
	/* TODO : already done in board init */
	return 0;
}

void chbsp_i2c_reset(ch_dev_t *dev_ptr)
{
	(void)dev_ptr;
	// TODO necessary to reset after non blocking transaction ??
	i2c_hal_init(I2C_BUS_IDX_CHx01);
}

uint8_t chbsp_i2c_get_info(ch_group_t __attribute__((unused)) * grp_ptr, uint8_t io_index, ch_i2c_info_t *info_ptr)
{
	if (io_index >= CHIRP_MAX_NUM_SENSORS)
		return 1;

#if defined(INCLUDE_WHITNEY_SUPPORT)
	info_ptr->address   = chx01_i2c_addrs[io_index];
	info_ptr->bus_num   = chx01_i2c_buses[io_index];
	info_ptr->drv_flags = 0; /* no special I2C handling by SonicLib driver is needed */
#else
	(void)info_ptr;
#endif
	return 0;
}

int chbsp_i2c_write(ch_dev_t *dev_ptr, const uint8_t *data, uint16_t num_bytes)
{
	return i2c_hal_write(I2C_BUS_IDX_CHx01, ch_get_i2c_address(dev_ptr), data, num_bytes);
}

int chbsp_i2c_read(ch_dev_t *dev_ptr, uint8_t *data, uint16_t num_bytes)
{
	return i2c_hal_read(I2C_BUS_IDX_CHx01, ch_get_i2c_address(dev_ptr), data, num_bytes);
}

int chbsp_i2c_read_nb(ch_dev_t *dev_ptr, uint8_t *data, uint16_t num_bytes)
{
	return i2c_hal_read_nb(I2C_BUS_IDX_CHx01, ch_get_i2c_address(dev_ptr), data, num_bytes);
}

int chbsp_i2c_mem_write(ch_dev_t *dev_ptr, uint16_t mem_addr, uint8_t *data, uint16_t num_bytes)
{
	return i2c_hal_write_register(I2C_BUS_IDX_CHx01, ch_get_i2c_address(dev_ptr), mem_addr, data, num_bytes);
}

int chbsp_i2c_mem_write_nb(ch_dev_t __attribute__((unused)) * dev_ptr, uint16_t __attribute__((unused)) mem_addr,
                           uint8_t __attribute__((unused)) * data, uint16_t __attribute__((unused)) num_bytes)
{
	// XXX not implemented
	return 1;
}

int chbsp_i2c_mem_read(ch_dev_t *dev_ptr, uint16_t mem_addr, uint8_t *data, uint16_t num_bytes)
{
	return i2c_hal_read_register(I2C_BUS_IDX_CHx01, ch_get_i2c_address(dev_ptr), mem_addr, data, num_bytes);
}

int chbsp_i2c_mem_read_nb(ch_dev_t *dev_ptr, uint16_t mem_addr, uint8_t *data, uint16_t num_bytes)
{
	return i2c_hal_read_register_nb(I2C_BUS_IDX_CHx01, ch_get_i2c_address(dev_ptr), mem_addr, data, num_bytes);
}

void chbsp_delay_us(uint32_t us)
{
	time_delay_us(us);
}

void chbsp_delay_ms(uint32_t ms)
{
	time_delay_ms(ms);
}

uint32_t chbsp_timestamp_ms(void)
{
	uint32_t time = time_get_in_us();
	return (time / 1000);
}

void chbsp_debug_on(uint8_t dbg_pin_num)
{
	ioport_set_pin_level((dbg_pin_num == 0) ? DBG0 : DBG1, IOPORT_PIN_LEVEL_HIGH);
}

void chbsp_debug_off(uint8_t dbg_pin_num)
{
	ioport_set_pin_level((dbg_pin_num == 0) ? DBG0 : DBG1, IOPORT_PIN_LEVEL_LOW);
}

void chbsp_debug_toggle(uint8_t dbg_pin_num)
{
	ioport_toggle_pin_level((dbg_pin_num == 0) ? DBG0 : DBG1);
}

void chbsp_print_str(const char *str)
{
	printf(str);
}
