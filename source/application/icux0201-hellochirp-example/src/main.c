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

/***********************************************************************
 *
 * These values are used to initialize the
 * Hello Shasta! - an example application for ultrasonic sensing
 *
 * This project is designed to be your first introduction to using
 * the SonicLib API to control ultrasonic sensors in an embedded C
 * application.
 *
 * It configures connected ICU sensors, sets up a measurement
 * timer, and triggers the sensors each time the timer expires.
 * On completion of each measurement, it reads out the sensor data and
 * prints it over the console serial port.
 *
 * The settings used to configure the sensors are defined in
 * the app_config.h header file.
 *
 ***********************************************************************/

#define CH_LOG_MODULE_NAME "ICU_HELLO"
#include <invn/soniclib/ch_log.h>

#include <invn/soniclib/soniclib.h>  // Chirp SonicLib sensor API definitions
#include <invn/soniclib/sensor_fw/icu_gpt/icu_gpt.h>
#include <invn/soniclib/ch_rangefinder_types.h>
#include <invn/soniclib/chirp_bsp.h>  // board support package function definitions
#include <invn/soniclib/ch_extra_display_utils.h>

#include "bsp.h"
#include "chirp_board_config.h"  // required header with basic device counts etc.
#include "app_config.h"
#include "app_version.h"

//#define USE_TASKFLAGS_LOCK  // uncomment to block interrupts during taskflags check
#ifdef USE_TASKFLAGS_LOCK
#include "samg55.h"
#endif

#if (OUTPUT_AMP_LOG || OUTPUT_IQ_LOG)
#include "time.h"
#endif

/* Bit flags used in main loop to check for completion of sensor I/O.  */
#define DATA_READY_FLAG (1 << 0)
#define READ_DONE_FLAG  (1 << 1)
#define TIMER_FLAG      (1 << 2)

/* Number of LEDs to blink */
#define NUM_LEDS (3)

/* Measurement number to use in this example */
#define MEAS_NUM (CH_DEFAULT_MEAS_NUM)

/* Check build option dependencies */
#if ((OUTPUT_AMP_DATA_CSV + OUTPUT_AMP_LOG + OUTPUT_IQ_DATA + OUTPUT_IQ_LOG) > 1)
#error Cannot specify multiple output options at the same time (OUTPUT_AMP_xxx or OUTPUT_IQ_xxx).
#endif

#if (OUTPUT_AMP_DATA_CSV || OUTPUT_AMP_LOG)
#define READ_AMPLITUDE_DATA 1
#else
#define READ_AMPLITUDE_DATA 0
#endif

#ifndef READ_IQ_DATA
#if (OUTPUT_IQ_DATA || OUTPUT_IQ_LOG || OUTPUT_IQ_DATA_SUMMARY)
#define READ_IQ_DATA 1
#else
#define READ_IQ_DATA 0
#endif
#endif

/* chirp_data_t - Structure to hold measurement data for one sensor
 *   This structure is used to hold the data from one measurement cycle from
 *   a sensor.  The data values include the measured range, the ultrasonic
 *   signal amplitude, the number of valid samples  in the measurement, and
 *   (optionally) the full amplitude data or raw I/Q data
 *   from the measurement.
 *
 *  The format of this data structure is specific to this example application, so
 *  you may change it as desired.
 *
 *  A "chirp_data[]" array of these structures, one for each possible sensor,
 *  is declared in the main.c file.  The sensor's device number is
 *  used to index the array.
 */
typedef struct {
	uint8_t rx_sensor_num;  // receiving sensor number
	uint8_t tx_sensor_num;  // transmitting sensor number
	uint32_t range;         // from ch_get_range()
	uint16_t amplitude;     // from ch_get_amplitude()
	uint16_t num_samples;   // from ch_get_num_samples()
#if READ_AMPLITUDE_DATA
	uint16_t amp_data[APP_DATA_MAX_SAMPLES];
	// from ch_get_amplitude_data()
#endif
#if READ_IQ_DATA
	ch_iq_sample_t iq_data[APP_DATA_MAX_SAMPLES];
	// from ch_get_iq_data()
#endif
} chirp_data_t;

#ifdef USE_RANGEFINDING
/* Detection level settings
 *   This structure is passed to the icu_gpt_algo_configure() function to set the target
 *   detection thresholds.  Each threshold entry consists of a starting sample
 *   number and a threshold amplitude level.  The values are defined in app_config.h.
 */
ch_thresholds_t chirp_detect_thresholds      = {.threshold = {
                                                   {CHIRP_THRESH_0_START, CHIRP_THRESH_0_LEVEL}, /* threshold 0 */
                                                   {CHIRP_THRESH_1_START, CHIRP_THRESH_1_LEVEL}, /* threshold 1 */
                                                   {CHIRP_THRESH_2_START, CHIRP_THRESH_2_LEVEL}, /* threshold 2 */
                                                   {CHIRP_THRESH_3_START, CHIRP_THRESH_3_LEVEL}, /* threshold 3 */
                                                   {CHIRP_THRESH_4_START, CHIRP_THRESH_4_LEVEL}, /* threshold 4 */
                                                   {CHIRP_THRESH_5_START, CHIRP_THRESH_5_LEVEL}, /* threshold 5 */
                                                   {CHIRP_THRESH_6_START, CHIRP_THRESH_6_LEVEL}, /* threshold 6 */
                                                   {CHIRP_THRESH_7_START, CHIRP_THRESH_7_LEVEL}, /* threshold 7 */
                                           }};
ch_thresholds_t *chirp_detect_thresholds_ptr = &chirp_detect_thresholds;
#else
ch_thresholds_t *chirp_detect_thresholds_ptr = NULL;
#endif

#if IMPORT_MEASUREMENT
/* Imported measurement definition from measurement_config.c */
extern measurement_queue_t measurement_config_queue;
extern InvnAlgoRangeFinderConfig measurement_config_cfg;
#else
/* Measurement configuration struct - starting/default values */
static ch_meas_config_t meas_config = {
		.odr         = CHIRP_SENSOR_ODR,
		.meas_period = 0,
};

#ifdef USE_RANGEFINDING
static icu_gpt_algo_config_t algo_config = {
		.num_ranges              = CHIRP_MAX_TARGETS,
		.ringdown_cancel_samples = CHIRP_RINGDOWN_FILTER_SAMPLES,
		.static_filter_samples   = 0,            // may be changed later using ch_set_static_range()
		.iq_output_format        = CH_OUTPUT_IQ, /* return (Q, I) */
		.filter_update_interval  = 0,            /* update filter every sample */
};
#endif  //USE_RANGEFINDING
#endif  //IMPORT_MEASUREMENT

#ifdef USE_RANGEFINDING
InvnAlgoRangeFinderConfig gpt_algo_instance[CHIRP_MAX_NUM_SENSORS];
#endif  //USE_RANGEFINDING

/* Array of structs to hold measurement data, one for each possible device */
chirp_data_t chirp_data[CHIRP_MAX_NUM_SENSORS];

/* Array of ch_dev_t device descriptors, one for each possible device */
static ch_dev_t chirp_devices[CHIRP_MAX_NUM_SENSORS];

/* Descriptor structure for group of sensors */
ch_group_t chirp_group;

/* Task flag word
 *   This variable contains the TIMER_FLAG, DATA_READY_FLAG and READ_DONE_FLAG
 *   bit flags that are set in interrupt callback routines.  The flags are
 *   checked in the main() loop and, if set, will cause an appropriate function to
 *   be called to process sensor data.
 */
volatile uint32_t taskflags = 0;

/* Device tracking variables
 *   These are bit-field variables which contain a separate bit assigned to
 *   each (possible) sensor, indexed by the device number.  The active_devices
 *   variable contains the bit pattern describing which ports have active
 *   sensors connected.  The data_ready_devices variable is set bit-by-bit
 *   as sensors interrupt, indicating they have completed a measurement
 *   cycle.  The two variables are compared to determine when all active
 *   devices have interrupted.
 */
static uint32_t active_devices;
static uint32_t data_ready_devices;

/* Number of connected sensors */
static uint8_t num_connected_sensors = 0;

/* Number of sensors that use external triggering to start measurement */
static uint8_t num_triggered_sensors = 0;

#if (READ_DATA_NONBLOCKING && (READ_AMPLITUDE_DATA || READ_IQ_DATA))
/* Count of non-blocking data reads queued */
static uint16_t num_io_queued = 0;
#endif

#if (OUTPUT_AMP_LOG || OUTPUT_IQ_LOG)
/* Data log support - only needed if using formal logging interface */
#define ODR_TO_DECIMATION(odr) ((odr == CH_ODR_FREQ_DIV_32) ? 4 : ((odr == CH_ODR_FREQ_DIV_16) ? 2 : 1))  // default = 1

static uint8_t log_id             = 0;
static const uint16_t interval_ms = (CHIRP_FIRST_SENSOR_MODE == CH_MODE_CONTINUOUS_RX) ? 0 : MEASUREMENT_INTERVAL_MS;
ch_log_cfg_t log_config           = {
				  .interval_ms       = 0,
				  .output_type       = (OUTPUT_AMP_LOG == 1) ? CH_OUTPUT_AMP : CH_OUTPUT_IQ,
				  .decimation_factor = ODR_TO_DECIMATION(CHIRP_SENSOR_ODR),
				  .start_sample      = 0,
};

#endif  //  (OUTPUT_AMP_LOG || OUTPUT_IQ_LOG)

#if defined(CHIRP_MEAS_OPTIMIZE) && defined(USE_RANGEFINDING)
extern uint8_t icu_init_init(ch_dev_t *dev_ptr, fw_info_t **fw_info);
#endif

/* Forward declarations */
static uint8_t configure_sensors(ch_group_t *grp_ptr, uint8_t num_ports, uint32_t *active_devices_ptr,
                                 uint8_t *num_connected_ptr, uint8_t *num_triggered_ptr);
static uint8_t handle_data_ready(ch_group_t *grp_ptr);
static void sensor_int_callback(ch_group_t *grp_ptr, uint8_t dev_num, ch_interrupt_type_t int_type);
static void periodic_timer_callback(void);

#ifdef USE_RANGEFINDING
static void display_algo_output(ch_dev_t *dev_ptr, uint8_t dev_num);
#endif

#if (READ_AMPLITUDE_DATA || READ_IQ_DATA)
static uint8_t read_measurement_data(ch_dev_t *dev_ptr);
#endif
#if (OUTPUT_AMP_DATA_CSV || OUTPUT_IQ_DATA)
static void display_sensor_data(chirp_data_t *sensor_data);
#endif
#if (OUTPUT_AMP_LOG || OUTPUT_IQ_LOG)
static void log_sensor_data(chirp_data_t *sensor_data);
#endif
#if (READ_DATA_NONBLOCKING && (READ_AMPLITUDE_DATA || READ_IQ_DATA))
static void io_complete_callback(ch_group_t *grp_ptr);
static uint8_t handle_read_done(ch_group_t *grp_ptr);
#endif

/* main() - entry point and main loop
 *
 * This function contains the initialization sequence for the application
 * on the board, including system hardware initialization, sensor discovery
 * and configuration, callback routine registration, and timer setup.  After
 * the initialization sequence completes, this routine enters an infinite
 * loop that will run for the remainder of the application execution.
 */

int main(void)
{
	ch_group_t *grp_ptr;  // pointer to group descriptor
	ch_dev_t *dev_ptr;    // pointer to individual device descriptor
	uint8_t chirp_error = 0;
	uint8_t num_ports;
	uint8_t dev_num;
#if (OUTPUT_AMP_LOG || OUTPUT_IQ_LOG)
	log_config.interval_ms = interval_ms;
#endif

	/* Initialize board hardware functions
	 *   This call to the board support package (BSP) performs all necessary
	 *   hardware initialization for the application to run on this board.
	 *   This includes setting up memory regions, initializing clocks and
	 *   peripherals (including SPI and serial port), and any processor-specific
	 *   startup sequences.
	 *
	 *   The bsp_init() function also initializes fields within the
	 *   sensor group descriptor, including number of supported sensors and
	 *   the RTC clock calibration pulse length.
	 */
	grp_ptr = &chirp_group;
	bsp_init(grp_ptr);

	CH_LOG_APP("Hello Chirp! - SonicLib Example Application for ICU Sensors");
	CH_LOG_APP("    Compile time:  %s %s", __DATE__, __TIME__);
	CH_LOG_APP("    Version: %u.%u.%u-%s", APP_VERSION_MAJOR, APP_VERSION_MINOR, APP_VERSION_REV, APP_VERSION_SUFFIX);
	CH_LOG_APP("    SonicLib version: %u.%u.%u%s", SONICLIB_VER_MAJOR, SONICLIB_VER_MINOR, SONICLIB_VER_REV,
	           SONICLIB_VER_SUFFIX);

	ch_group_init(&chirp_group, CHIRP_MAX_NUM_SENSORS, CHIRP_NUM_BUSES, CHIRP_RTC_CAL_PULSE_MS);

	/* Get the number of (possible) sensor devices on the board
	 *   Set by the BSP during bsp_init()
	 */
	num_ports = ch_get_num_ports(grp_ptr);

	/* Initialize sensor descriptors.
	 *   This loop initializes each (possible) sensor's ch_dev_t descriptor,
	 *   although we don't yet know if a sensor is actually connected.
	 *
	 *   The call to ch_init() specifies the sensor descriptor, the sensor group
	 *   it will be added to, the device number within the group, and the sensor
	 *   firmware initialization routine that will be used.
	 */
	CH_LOG_APP("Initializing sensor(s)... ");

	for (dev_num = 0; dev_num < num_ports; dev_num++) {
		dev_ptr = &(chirp_devices[dev_num]);  // init struct in array
											  /* Init device descriptor
		 *   Note that this assumes all sensors will use the same sensor
		 *   firmware as specified by CHIRP_SENSOR_FW_INIT_FUNC in app_config.h.
		 */
#ifdef USE_RANGEFINDING
/* ICU GPT - general purpose transceiver */
#define CHIRP_SENSOR_FW_INIT_FUNC (icu_gpt_init)
#else
/* ICU INIT - no rangefinding; increased max IQ samples */
#define CHIRP_SENSOR_FW_INIT_FUNC (icu_init_init)
#endif

		chirp_error |= ch_init(dev_ptr, grp_ptr, dev_num, CHIRP_SENSOR_FW_INIT_FUNC);
#if CHIRP_MEAS_OPTIMIZE
		chirp_error |= ch_set_init_firmware(dev_ptr, icu_init_init);
#endif
	}

	/* Start all sensors.
	 *   The ch_group_start() function will search each port (that was
	 *   initialized above) for a sensor. If it finds one, it programs it (with
	 *   the firmware specified above during ch_init()) and waits for it to
	 *   perform a self-calibration step.  Then, once it has found all the
	 *   sensors, ch_group_start() completes a timing reference calibration by
	 *   applying a pulse of known length to the sensor's INT line.
	 */
	if (chirp_error == 0) {
		CH_LOG_APP_START("starting group... ");
		chirp_error = ch_group_start(grp_ptr);
	}

	if (chirp_error == 0) {
		CH_LOG_APP_MSG("OK");
	} else {
		CH_LOG_APP_MSG("FAILED: %d", chirp_error);
	}
	CH_LOG_APP_END();

#if IS_CH_LOG_USED
	/* Get and display the initialization results for each connected sensor.
	 *   This loop checks each device number in the sensor group to determine
	 *   if a sensor is actually connected.  If so, it makes a series of
	 *   function calls to get different operating values, including the
	 *   operating frequency, clock calibration values, and firmware version.
	 */
	ch_extra_display_init_info(grp_ptr);
#endif

	/* Register callback function to be called when Chirp sensor interrupts */
	ch_io_int_callback_set(grp_ptr, sensor_int_callback);

#if (READ_DATA_NONBLOCKING && (READ_AMPLITUDE_DATA || READ_IQ_DATA))
	/* Register callback function called when non-blocking data readout completes */
	ch_io_complete_callback_set(grp_ptr, io_complete_callback);
#endif

	/* Configure each sensor with its operating parameters */
	chirp_error =
			configure_sensors(grp_ptr, num_ports, &active_devices, &num_connected_sensors, &num_triggered_sensors);

	/* Initialize periodic timer if needed for triggering.
	 *   Initialize a timer that will interrupt every time it expires after
	 *   the specified measurement interval.  periodic_timer_init() also
	 *   registers a callback function that will be called from the timer
	 *   handler when the interrupt occurs.  The callback function will be
	 *   used to trigger a measurement cycle on the group of sensors.
	 */
	if (num_triggered_sensors != 0) {
		CH_LOG_APP_START("Initializing sample timer for %u ms interval... ", MEASUREMENT_INTERVAL_MS);

		periodic_timer_init(MEASUREMENT_INTERVAL_MS, periodic_timer_callback);

		/* Enable interrupt and start timer to trigger sensor sampling */
		periodic_timer_irq_enable();
		periodic_timer_start();
		CH_LOG_APP_MSG("OK");
		CH_LOG_APP_END();
	}

	if (!chirp_error) {
		CH_LOG_APP("Starting measurements...");
	}

#if (OUTPUT_AMP_LOG || OUTPUT_IQ_LOG)
	/* Initialize data log, if used. Do this last because it will output column headers. */
	log_id = ch_log_init(grp_ptr, CH_LOG_FMT_REDSWALLOW, &log_config);
	if (log_id == 0) {
		CH_LOG_ERR("ERROR initializing log output.");
	}
#endif

	/***************************************/
	/**  Begin Infinite Measurement Loop  **/
	/***************************************/

	while (1) {
		/*
		 * Put processor in light sleep mode if there are no pending tasks, but
		 * never turn off the main clock, so that interrupts can still wake
		 * the processor.
		 */

		/* *** WARNING - Race Condition ***
		 * The following test of the taskflags is used to determine if the processor
		 * should be put in sleep mode.  However, because the task flags are updated
		 * at interrupt level, it is possible for them to change before the sleep
		 * state is entered.  This can result in a "missed" measurement cycle or
		 * similar problems.
		 *
		 * It is recommended to lock interrupts at this point (at least blocking the
		 * sensor interrupt, periodic timer, and non-blocking I/O) before testing
		 * taskflags, and then restore the interrupt enable state after the test or
		 * as part of the sleep function.
		 *
		 * Because such solutions are hardware-specific, they are not included in this
		 * simple example.  However, you should add this protection or otherwise handle
		 * this timing possibility if adapting this code for your actual application.
		 */
#ifdef USETASKFLAGS_LOCK
		__disable_irq();
#endif
		if (taskflags == 0) {
			proc_sleep();  // put processor in low-power sleep mode
						   /* We only continue here after an interrupt wakes the processor */
		}
#ifdef USE_TASKFLAGS_LOCK
		__enable_irq();
#endif

		if (taskflags & TIMER_FLAG) {
			taskflags &= ~TIMER_FLAG;

			if (CHIRP_TRIGGER_TYPE == CH_TRIGGER_TYPE_SW) {
				for (dev_num = 0; dev_num < num_ports; dev_num++) {
					dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);
					if (ch_sensor_is_connected(dev_ptr))
						ch_trigger_soft(dev_ptr);
				}
			}
		}

		/* Check for sensor data-ready interrupt(s) */
		if (taskflags & DATA_READY_FLAG) {

			/* All sensors have interrupted - handle sensor data */
			taskflags &= ~DATA_READY_FLAG;  // clear flag
			handle_data_ready(grp_ptr);     // read and display measurement

#if ((READ_AMPLITUDE_DATA || READ_IQ_DATA) && READ_DATA_NONBLOCKING)
			/* Start any pending non-blocking data reads */
			if (num_io_queued != 0) {
				ch_io_start_nb(grp_ptr);
				num_io_queued = 0;
			}
#endif
		}  // end if (taskflags & DATA_READY_FLAG)

#if ((READ_AMPLITUDE_DATA || READ_IQ_DATA) && READ_DATA_NONBLOCKING)
		/* Check for non-blocking data read complete */
		if (taskflags & READ_DONE_FLAG) {

			/* All non-blocking data readouts have completed */
			taskflags &= ~READ_DONE_FLAG;  // clear flag
			handle_read_done(grp_ptr);     // display measurement data
		}
#endif
	}
	// *** End while(1) infinite measurement loop ***

}  // end main()

#if (CHIRP_IMPROVE_PITCH_CATCH_SENSITIVITY == 1)
static uint32_t compute_common_fop(ch_group_t *grp_ptr, uint8_t num_ports)
{
	ch_dev_t *dev_ptr;
	uint32_t sensors_mean_fop    = 0;
	uint32_t sensors_fop         = 0;
	uint8_t nb_connected_sensors = 0;

	for (uint8_t dev_num = 0; dev_num < num_ports; dev_num++) {
		dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);

		if (ch_sensor_is_connected(dev_ptr)) {
			sensors_fop       = ch_get_frequency(dev_ptr);
			sensors_mean_fop += sensors_fop;
			nb_connected_sensors++;
		}
	}
	if (nb_connected_sensors > 1) {
		sensors_mean_fop /= nb_connected_sensors;
		CH_LOG_APP("Mean frequency of connected sensors = %lu Hz", sensors_mean_fop);
	} else {
		/* No possible pitch-catch */
		sensors_mean_fop = 0;
	}

	return sensors_mean_fop;
}
#endif /* (CHIRP_IMPROVE_PITCH_CATCH_SENSITIVITY == 1) */

/*
 * configure_sensors() - apply configuration settings
 *
 * This function performs the detailed configuration of the sensor, including
 * definition of the measurement and selecting various optional features.
 * Values defined in the app_config.h file are used here as parameters to
 * individual SonicLib API functions.
 *
 * If IMPORT_MEASUREMENT is defined as non-zero, this function will import a defined
 * measurement instead.
 */
uint8_t configure_sensors(ch_group_t *grp_ptr,           // pointer to sensor group
                          uint8_t num_ports,             // number of possible sensors on board
                          uint32_t *active_devices_ptr,  // pointer to active devices tracking var
                          uint8_t *num_connected_ptr,    // pointer to count of connected devices
                          uint8_t *num_triggered_ptr)    // pointer to count of triggered devices
{
	ch_dev_t *dev_ptr;
	ch_mode_t mode;
	uint8_t dev_num;
	uint8_t chirp_error;
	uint8_t first_connected_num = 0;
#if IS_CH_LOG_USED
	char const *mode_string = "";
#endif

	chirp_error         = 0;
	*num_connected_ptr  = 0;
	*active_devices_ptr = 0;

#if (CHIRP_IMPROVE_PITCH_CATCH_SENSITIVITY == 1)
	uint32_t sensors_mean_fop = 0;

	sensors_mean_fop = compute_common_fop(grp_ptr, num_ports);
#endif  // (CHIRP_IMPROVE_PITCH_CATCH_SENSITIVITY == 1)

	/* Set configuration values for each connected sensor */
	for (dev_num = 0; dev_num < num_ports; dev_num++) {
		dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);  // get this device's ch_dev_t addr

		if (ch_sensor_is_connected(dev_ptr)) {
			CH_LOG_APP("Configuring Device %u...", dev_num);

			(*num_connected_ptr)++;                   // count one more connected sensor
			(*active_devices_ptr) |= (1 << dev_num);  // add to active device bit mask

			if (*num_connected_ptr == 1) {  // if this is the first sensor
				mode                = CHIRP_FIRST_SENSOR_MODE;
				first_connected_num = dev_num;
			} else {
				mode = CHIRP_OTHER_SENSOR_MODE;
			}

#if IS_CH_LOG_USED
			switch (mode) {
			case CH_MODE_FREERUN:
				mode_string = "CH_MODE_FREERUN";
				break;
			case CH_MODE_TRIGGERED_TX_RX:
				mode_string = "CH_MODE_TRIGGERED_TX_RX";
				break;
			case CH_MODE_TRIGGERED_RX_ONLY:
				mode_string = "CH_MODE_TRIGGERED_RX_ONLY";
				break;
			case CH_MODE_IDLE:
				mode_string = "CH_MODE_IDLE";
				break;
			case CH_MODE_CONTINUOUS_RX:
				mode_string = "CH_MODE_CONTINUOUS_RX";
				break;
			default:
				chirp_error = 1;  // bad mode specified in app_config.h
				CH_LOG_ERR("ERROR - bad sensor mode %d", mode);
				break;
			}
#endif

			/* Initialize application structure(s) to hold measurement data */
			if (!chirp_error) {
				chirp_data[dev_num].rx_sensor_num = dev_num;  // this will be the receiving sensor
				if (mode == CH_MODE_TRIGGERED_RX_ONLY) {
					chirp_data[dev_num].tx_sensor_num = first_connected_num;  // first sensor will transmit
				} else {
					chirp_data[dev_num].tx_sensor_num = dev_num;  // this sensor will transmit
				}
			}

#if (CHIRP_IMPROVE_PITCH_CATCH_SENSITIVITY == 1)
			if (!chirp_error && (sensors_mean_fop != 0)) {
				CH_LOG_APP_START("  Setting common fop... ");
				/* Change of fop shall be done before optimizing measurement */
				chirp_error = ch_set_frequency(dev_ptr, sensors_mean_fop);
				CH_LOG_APP_MSG("%s", chirp_error ? "ERROR" : "OK");
				CH_LOG_APP_END();
			}
#endif  // (CHIRP_IMPROVE_PITCH_CATCH_SENSITIVITY == 1)

#ifdef USE_RANGEFINDING
			/* Initialize algorithm on sensor */
			if (!chirp_error) {
				chirp_error = icu_gpt_algo_init(dev_ptr, &gpt_algo_instance[dev_num]);
			}
#endif

#if IMPORT_MEASUREMENT  // If importing a whole pre-defined measurement
			(void)num_triggered_ptr;
			if (!chirp_error && (CHIRP_MEAS_OPTIMIZE == 0)) {
				CH_LOG_APP_START("  Importing measurement: ");
				chirp_error = ch_meas_import(dev_ptr, &measurement_config_queue, &measurement_config_cfg);
			} else {
				CH_LOG_APP_START("  Importing & optimizing measurement: ");
				chirp_error = ch_meas_optimize(dev_ptr, &measurement_config_queue, &measurement_config_cfg);
			}
			CH_LOG_APP_MSG("%s", chirp_error ? "ERROR" : "OK");
			CH_LOG_APP_END();

#else  // If defining a measurement using SonicLib functions

			/* Initialize measurement */
			if (!chirp_error) {
				CH_LOG_APP_START("  Initializing measurement %d... ", MEAS_NUM);
				chirp_error  = ch_meas_init(dev_ptr, MEAS_NUM, &meas_config, NULL);
#ifdef USE_RANGEFINDING
				chirp_error |= icu_gpt_algo_configure(dev_ptr, MEAS_NUM, &algo_config, chirp_detect_thresholds_ptr);
#endif
				if (!chirp_error && mode == CH_MODE_CONTINUOUS_RX) {
					// initialize the other meas config since both are used in continuous rx
					chirp_error  = ch_meas_init(dev_ptr, (MEAS_NUM + 1) % 2, &meas_config, NULL);
#ifdef USE_RANGEFINDING
					chirp_error |= icu_gpt_algo_configure(dev_ptr, (MEAS_NUM + 1) % 2, &algo_config,
					                                      chirp_detect_thresholds_ptr);
#endif
				}
				CH_LOG_APP_MSG("%s", chirp_error ? "ERROR" : "OK");
				CH_LOG_APP_END();
			}

			/* Add transmit segment unless rx-only */
			if (!chirp_error) {
				if (mode != CH_MODE_TRIGGERED_RX_ONLY && mode != CH_MODE_CONTINUOUS_RX) {
					CH_LOG_APP_START("  Adding tx segment... ");
					chirp_error =
							ch_meas_add_segment_tx(dev_ptr, MEAS_NUM, CHIRP_TX_SEG_CYCLES, CHIRP_TX_SEG_PULSE_WIDTH,
					                               CHIRP_TX_SEG_PHASE, CHIRP_TX_SEG_INT_EN);
				} else {
					CH_LOG_APP_START("  Adding count (delay) segment to match Tx sensor... ");
					chirp_error = ch_meas_add_segment_count(dev_ptr, MEAS_NUM, CHIRP_TX_SEG_CYCLES, 0);
				}
				CH_LOG_APP_MSG("%s", chirp_error ? "ERROR" : "OK");
				CH_LOG_APP_END();
			}

			/* Add first receive segment for very early part of measurement */
			if (!chirp_error) {
				CH_LOG_APP_START("  Adding rx segment 0... ");
				chirp_error =
						ch_meas_add_segment_rx(dev_ptr, MEAS_NUM, CHIRP_RX_SEG_0_SAMPLES, CHIRP_RX_SEG_0_GAIN_REDUCE,
				                               CHIRP_RX_SEG_0_ATTEN, CHIRP_RX_SEG_0_INT_EN);
				CH_LOG_APP_MSG("%s", chirp_error ? "ERROR" : "OK");
				CH_LOG_APP_END();
			}

			/* Add second receive segment for remainder of measurement */
			if (!chirp_error) {
				CH_LOG_APP_START("  Adding rx segment 1... ");
				chirp_error =
						ch_meas_add_segment_rx(dev_ptr, MEAS_NUM, CHIRP_RX_SEG_1_SAMPLES, CHIRP_RX_SEG_1_GAIN_REDUCE,
				                               CHIRP_RX_SEG_1_ATTEN, CHIRP_RX_SEG_1_INT_EN);
				CH_LOG_APP_MSG("%s", chirp_error ? "ERROR" : "OK");
				CH_LOG_APP_END();
			}

			/* Write complete meas config (measurement queue) to device */
			if (!chirp_error) {
				CH_LOG_APP_START("  Writing meas queue... ");
				chirp_error = ch_meas_write_config(dev_ptr);
				CH_LOG_APP_MSG("%s", chirp_error ? "ERROR" : "OK");
				CH_LOG_APP_END();
			}

			/* Optimize measurement segments, if specified */
			if (!chirp_error && CHIRP_MEAS_OPTIMIZE) {
				CH_LOG_APP_START("  Optimizing measurement segments... ");
				chirp_error = ch_meas_optimize(dev_ptr, NULL, NULL);
				CH_LOG_APP_MSG("%s", chirp_error ? "ERROR" : "OK");
				CH_LOG_APP_END();
			}

#ifdef USE_RANGEFINDING
			if (!chirp_error) {
				/* Write algo config to sensor */
				chirp_error = ch_set_algo_config(dev_ptr, &gpt_algo_instance[dev_num]);
			}
			/* Set sensing range based on sensor model - this can change the total
			 * number of rx samples set during ch_meas_add_segment_rx() above.
			 */
			if (!chirp_error) {
				uint16_t max_range   = 0;
				uint16_t part_number = ch_get_part_number(dev_ptr);

				if (part_number == ICU30201_PART_NUMBER) {
					max_range = CHIRP_ICU30201_MAX_RANGE_MM;
				} else if (part_number == ICU10201_PART_NUMBER) {
					max_range = CHIRP_ICU10201_MAX_RANGE_MM;
				} else {
					max_range = CHIRP_ICU20201_MAX_RANGE_MM;
				}
				CH_LOG_APP_START("  Setting max range to %u mm... ", max_range);
				chirp_error = ch_meas_set_max_range(dev_ptr, MEAS_NUM, max_range);
				CH_LOG_APP_MSG("%s", chirp_error ? "ERROR" : "OK");
				CH_LOG_APP_END();
			}
#endif
			/* Set static target rejection range, if specified */
			if (!chirp_error && (CHIRP_STATIC_REJECT_SAMPLES != 0)) {
				CH_LOG_APP_START("  Setting static target rejection (%u samples)... ", CHIRP_STATIC_REJECT_SAMPLES);
				chirp_error = icu_gpt_set_static_filter(dev_ptr, MEAS_NUM, CHIRP_STATIC_REJECT_SAMPLES);
				CH_LOG_APP_MSG("%s", chirp_error ? "ERROR" : "OK");
				CH_LOG_APP_END();
			}

			/* Set rx holdoff, if specified */
			if (!chirp_error && (CHIRP_RX_HOLDOFF_SAMPLES != 0)) {
				CH_LOG_APP_START("  Setting rx holdoff (%u samples)... ", CHIRP_RX_HOLDOFF_SAMPLES);
				chirp_error = icu_gpt_set_rx_holdoff(dev_ptr, MEAS_NUM, CHIRP_RX_HOLDOFF_SAMPLES);
				CH_LOG_APP_MSG("%s", chirp_error ? "ERROR" : "OK");
				CH_LOG_APP_END();
			}

			/* Enable target interrupt filtering, if specified */
			if (!chirp_error && CHIRP_TARGET_INT_FILTER) {
				CH_LOG_APP_START("  Enabling target interrupt filtering... ");
				chirp_error = ch_set_target_interrupt(dev_ptr, CH_TGT_INT_FILTER_ANY);
				CH_LOG_APP_MSG("%s", chirp_error ? "ERROR" : "OK");
				CH_LOG_APP_END();
			}
#endif  // IMPORT_MEASUREMENT

			if (!chirp_error && (mode == CH_MODE_CONTINUOUS_RX)) {
				// pulsed interrupt mode required in continuous modes
				ch_set_interrupt_mode(dev_ptr, CH_INTERRUPT_MODE_PULSE);
			}

			/* Enable amplitude output from sensor if selected */
			if (!chirp_error && READ_AMPLITUDE_DATA) {
				CH_LOG_APP_START("  Enabling amplitude output... ");
				chirp_error = ch_meas_set_iq_output(dev_ptr, MEAS_NUM, CH_OUTPUT_AMP);
				if (!chirp_error) {
					// continuous mode uses both meas nums
					chirp_error = ch_meas_set_iq_output(dev_ptr, (MEAS_NUM + 1) % 2, CH_OUTPUT_AMP);
				}
				CH_LOG_APP_MSG("%s", chirp_error ? "ERROR" : "OK");
				CH_LOG_APP_END();
			}

			/* Enable software triggering if selected */
			if (!chirp_error && (CHIRP_TRIGGER_TYPE == CH_TRIGGER_TYPE_SW)) {

				CH_LOG_APP_START("  Enabling soft triggering... ");
				ch_set_trigger_type(dev_ptr, CH_TRIGGER_TYPE_SW);
				CH_LOG_APP_MSG("OK");
				CH_LOG_APP_END();
			}

			if (!chirp_error) {
				/* If sensor will be free-running, set internal sample interval */
				if (mode == CH_MODE_FREERUN) {
					ch_set_freerun_interval(dev_ptr, MEASUREMENT_INTERVAL_MS);
				} else if (mode == CH_MODE_CONTINUOUS_RX) {
					// do nothing
				} else {
					(*num_triggered_ptr)++;  // count one more triggered sensor
				}

				/* Set interrupt line(s) as input and enable, if not also used for triggering */
				if ((mode == CH_MODE_FREERUN) || (CHIRP_TRIGGER_TYPE == CH_TRIGGER_TYPE_SW) ||
				    (CHIRP_SENSOR_INT_PIN != CHIRP_SENSOR_TRIG_PIN)) {

					CH_LOG_APP_START("  Setting INT line as input and enabling... ");
					chdrv_int_group_set_dir_in(grp_ptr);        // set INT line as input
					chdrv_int_group_interrupt_enable(grp_ptr);  // enable interrupt
					CH_LOG_APP_MSG("OK");
					CH_LOG_APP_END();
				}
			}

#ifdef USE_RANGEFINDING
			if (!chirp_error) {
				/* Init algo with new config */
				chirp_error = ch_init_algo(dev_ptr);
			}
#endif

			if (!chirp_error) {
				/* Set sensor mode - must come last, because sensing will be enabled and may begin */
				CH_LOG_APP_START("  Enabling in %s mode... ", mode_string);
				chirp_error = ch_set_mode(dev_ptr, mode);
				CH_LOG_APP_MSG("%s", chirp_error ? "ERROR" : "OK");
				CH_LOG_APP_END();
			}

			/* Read back and display config settings */
			if (!chirp_error) {
				CH_LOG_APP("Device %u: Configuration OK", dev_num);
#if IS_CH_LOG_USED
				ch_extra_display_config_info(dev_ptr);
				icu_gpt_display_algo_thresholds(dev_ptr);
#endif
			} else {
				CH_LOG_APP("Device %u: ERROR during configuration", dev_num);
			}
		} /*  end if ch_sensor_is_connected() */

	} /*  end for dev_num < num_ports */

	/* Enable receive sensor rx pre-triggering for group, if specified */
	ch_set_rx_pretrigger(grp_ptr, CHIRP_RX_PRETRIGGER_ENABLE);

	return chirp_error;
}

/*
 * handle_data_ready() - get and display data from a measurement
 *
 * This routine is called from the main() loop after all sensors have
 * interrupted. It shows how to read the sensor data once a measurement is
 * complete.  This routine always reads out the range and amplitude, and
 * optionally will read out the amplitude data or raw I/Q for all samples
 * in the measurement.
 *
 * See the comments in app_config.h for information about the amplitude data
 * and I/Q readout build options.
 */
static uint8_t handle_data_ready(ch_group_t *grp_ptr)
{
	uint8_t dev_num;
	uint16_t num_samples = 0;
	uint8_t ret_val      = 0;

	/* Read and display data from each connected sensor
	 *   This loop will write the sensor data to this application's "chirp_data"
	 *   array.  Each sensor has a separate chirp_data_t structure in that
	 *   array, so the device number is used as an index.
	 *
	 *   Multiple detected targets will be displayed if DISPLAY_MULTI_TARGET is non-zero.
	 *   Otherwise, only the closest detected target will be displayed.
	 */

	for (dev_num = 0; dev_num < ch_get_num_ports(grp_ptr); dev_num++) {
		ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);

		if (ch_sensor_is_connected(dev_ptr)) {
			CH_LOG_APP_START("Dev %u:  ", dev_num);

#ifdef USE_RANGEFINDING
			display_algo_output(dev_ptr, dev_num);
#endif
			CH_LOG_APP_END();

			/* Store number of active samples in this measurement */
			num_samples                     = ch_meas_get_num_samples(dev_ptr, 0);
			chirp_data[dev_num].num_samples = num_samples;

#if (READ_AMPLITUDE_DATA || READ_IQ_DATA)
			/* Optionally read all samples in measurement from sensor */
			read_measurement_data(dev_ptr);

			if (!READ_DATA_NONBLOCKING) {
				/* Data read is already complete - log or display sample data */
#if (OUTPUT_AMP_LOG || OUTPUT_IQ_LOG)
				log_sensor_data(&chirp_data[dev_num]);
#elif (OUTPUT_AMP_DATA_CSV || OUTPUT_IQ_DATA)
				display_sensor_data(&chirp_data[dev_num]);
#else
				CH_LOG_APP_MSG("read %u IQ samples", chirp_data[dev_num].num_samples);
#endif
			}
#endif     // (READ_AMPLITUDE_DATA || READ_IQ_DATA)
		}  // end if (ch_sensor_is_connected(dev_ptr))
	}      // end for (dev_num < ch_get_num_ports(grp_ptr))

	return ret_val;
}

#if (READ_AMPLITUDE_DATA || READ_IQ_DATA)
uint8_t read_measurement_data(ch_dev_t *dev_ptr)
{
	uint16_t start_sample = 0;
	uint8_t error         = 0;
	uint16_t num_samples  = ch_meas_get_num_samples(dev_ptr, 0);
	uint8_t dev_num       = ch_get_dev_num(dev_ptr);

#if !(READ_DATA_NONBLOCKING)
	/* Reading sensor data in normal, blocking mode */

#if READ_AMPLITUDE_DATA
	error = ch_get_amplitude_data(dev_ptr, chirp_data[dev_num].amp_data, start_sample, num_samples, CH_IO_MODE_BLOCK);
#elif READ_IQ_DATA
	error = ch_get_iq_data(dev_ptr, chirp_data[dev_num].iq_data, start_sample, num_samples, CH_IO_MODE_BLOCK);
#endif
	if (error) {
		CH_LOG_ERR("  ERROR reading %u samples", num_samples);
	}

#else
	/* Reading sensor data in non-blocking mode - queue a read operation */
	CH_LOG_APP_START("     queuing %u samples...", num_samples);

#if READ_AMPLITUDE_DATA
	error = ch_get_amplitude_data(dev_ptr, chirp_data[dev_num].amp_data, start_sample, num_samples,
	                              CH_IO_MODE_NONBLOCK);
#elif READ_IQ_DATA
	error = ch_get_iq_data(dev_ptr, chirp_data[dev_num].iq_data, start_sample, num_samples, CH_IO_MODE_NONBLOCK);
#endif

	if (!error) {
		num_io_queued++;  // record a pending non-blocking read
		CH_LOG_APP_MSG("OK");
	} else {
		CH_LOG_APP_MSG("**ERROR**");
	}
	CH_LOG_APP_END();
#endif  // !(READ_DATA_NONBLOCKING)

	return error;
}
#endif  // (READ_AMPLITUDE_DATA || READ_IQ_DATA)

/*
 * periodic_timer_callback() - periodic timer callback routine
 *
 * This function is called by the periodic timer interrupt when the timer
 * expires.  It sets a flag to that will be checked in the main() loop.
 *
 * If the sensor is operating in hardware-triggered mode, this function calls
 * ch_group_trigger() during each execution.
 *
 * This function is registered by the call to periodic_timer_init()
 * in main().
 */
static void periodic_timer_callback(void)
{
	/* Set timer flag - it will be checked and cleared in main() loop */
	taskflags |= TIMER_FLAG;

	/* If using normal h/w triggering, trigger all sensors now
	 *   If software triggering selected, it will be done at task level in
	 *   main() loop based on TIMER_FLAG.
	 */
	if ((CHIRP_TRIGGER_TYPE == CH_TRIGGER_TYPE_HW) && (num_triggered_sensors > 0)) {
		ch_group_trigger(&chirp_group);
	}
}

/*
 * sensor_int_callback() - sensor interrupt callback routine
 *
 * This function is called by the board support package's interrupt handler for
 * the sensor's INT line every time that the sensor interrupts.  The device
 * number parameter, dev_num, is used to identify the interrupting device
 * within the sensor group.  (Generally the device number is same as the port
 * number used in the BSP to manage I/O pins, etc.)
 *
 * Each time this function is called, a bit is set in the data_ready_devices
 * variable to identify the interrupting device.  When all active sensors have
 * interrupted (found by comparing with the active_devices variable), the
 * DATA_READY_FLAG is set.  That flag will be detected in the main() loop.
 *
 * This callback function is registered by the call to ch_io_int_callback_set()
 * in main().
 */
static void sensor_int_callback(ch_group_t *grp_ptr, uint8_t dev_num, ch_interrupt_type_t int_type)
{
	ch_dev_t *dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);

#ifdef USE_RANGEFINDING
	if (int_type != CH_INTERRUPT_TYPE_DATA_RDY) {
		/* Check soniclib.h CH_INTERRUPT_TYPE_* for values details */
		CH_LOG_ERR("Sensor %u : Bad interrupt type : %04X", dev_num, int_type);
		return;
	}
#else
	// allow ALGO_ERROR flag because ASIC FW has no ALGO and we won't read out algo data
	if ((int_type != CH_INTERRUPT_TYPE_DATA_RDY) && (int_type != CH_INTERRUPT_TYPE_ALGO_ERROR)) {
		/* Check soniclib.h CH_INTERRUPT_TYPE_* for values details */
		CH_LOG_ERR("Sensor %u : Bad interrupt type : %04X", dev_num, int_type);
		return;
	}
#endif
	bsp_led_toggle(dev_num);  // toggle led to show sensor active

	data_ready_devices |= (1 << dev_num);  // add to data-ready bit mask

	if (data_ready_devices == active_devices) {
		/* All active sensors have interrupted after performing a measurement */
		data_ready_devices = 0;

		/* Set data-ready flag - it will be checked in main() loop */
		taskflags |= DATA_READY_FLAG;

		/* Disable interrupt for shared int/trig pin if in h/w triggered mode
		*   It will automatically be re-enabled by the next ch_group_trigger()
		*/
		if ((ch_get_mode(dev_ptr) != CH_MODE_FREERUN) && (ch_get_trigger_type(dev_ptr) == CH_TRIGGER_TYPE_HW) &&
		    (CHIRP_SENSOR_INT_PIN == CHIRP_SENSOR_TRIG_PIN)) {

			chdrv_int_group_interrupt_disable(grp_ptr);
		} else {
			chdrv_int_set_dir_in(dev_ptr);  // set INT line as input and enable int
			chdrv_int_group_interrupt_enable(grp_ptr);
		}
	}
}

#if (READ_DATA_NONBLOCKING && (READ_AMPLITUDE_DATA || READ_IQ_DATA))
/*
 * handle_read_done() - handle sensor data from a non-blocking read
 *
 * This function is called from the main() loop when a queued non-blocking
 * readout of the sample data has completed for all sensors.  The data will
 * have been placed in this application's "chirp_data" array, in the
 * chirp_data_t structure for each sensor, indexed by the device number.
 *
 * If the OUTPUT_AMP_DATA_CSV or OUTPUT_IQ_DATA build symbol
 * is defined, this function will call display_sensor_data() to display
 * the sample values.
 */
static uint8_t handle_read_done(ch_group_t *grp_ptr)
{
	int dev_num;
	ch_dev_t *dev_ptr;

	for (dev_num = 0; dev_num < ch_get_num_ports(grp_ptr); dev_num++) {
		dev_ptr = ch_get_dev_ptr(grp_ptr, dev_num);

		if (ch_sensor_is_connected(dev_ptr)) {

			chirp_data[dev_num].num_samples = ch_meas_get_num_samples(dev_ptr, 0);

			CH_LOG_APP_MSG("  Read %u samples from device %u: ", chirp_data[dev_num].num_samples, dev_num);

#if (OUTPUT_AMP_DATA_CSV || OUTPUT_IQ_DATA)
			display_sensor_data(&chirp_data[dev_num]);
#elif (OUTPUT_AMP_LOG || OUTPUT_IQ_LOG)
			log_sensor_data(&chirp_data[dev_num]);
#endif
		}
	}

	return 0;
}

/*
 * io_complete_callback() - non-blocking I/O complete callback routine
 *
 * This function is called by SonicLib's SPI DMA handling function when all
 * outstanding non-blocking data readouts have completed.  It simply sets a flag
 * that will be detected and handled in the main() loop.
 *
 * This callback function is registered by the call to
 * ch_io_complete_callback_set() in main().
 *
 *  Note: This callback is only used if READ_DATA_NONBLOCKING is defined to
 *  select non-blocking sample data readout in this application.
 */
static void io_complete_callback(ch_group_t __attribute__((unused)) * grp_ptr)
{
	taskflags |= READ_DONE_FLAG;
}
#endif  // (READ_DATA_NONBLOCKING  && (READ_AMPLITUDE_DATA || READ_IQ_DATA))

#ifdef USE_RANGEFINDING
static void display_algo_output(ch_dev_t *dev_ptr, uint8_t dev_num)
{
	ch_range_t range_type;
	uint32_t range;
	uint16_t amplitude;
	uint8_t num_targets;

	/*  For sensors in transmit/receive mode, report one-way echo
	 *   range.  For sensors in receive-only mode, report direct
	 *   one-way range from transmitting sensor.
	 */
	if (ch_get_mode(dev_ptr) == CH_MODE_TRIGGERED_RX_ONLY || ch_get_mode(dev_ptr) == CH_MODE_CONTINUOUS_RX) {
		range_type = CH_RANGE_DIRECT;
	} else {
		range_type = CH_RANGE_ECHO_ONE_WAY;
	}

#if CHIRP_DETECT_TARGET_IN_RINGDOWN == 1
	CH_LOG_APP_MSG("Tgt in ringdown : %u ", icu_gpt_algo_is_target_in_ringdown(dev_ptr));
#endif

	/* Display detected targets (if any) */
	num_targets = icu_gpt_algo_get_num_targets(dev_ptr);
	if (num_targets == 0) {
		CH_LOG_APP_MSG("        no target found  ");
		return;
	}

#if !defined(DISPLAY_MULTI_TARGET) || (DISPLAY_MULTI_TARGET == 0)
	num_targets = 1;
#endif  // DISPLAY_MULTI_TARGET

	for (uint8_t target_num = 0; target_num < num_targets; target_num++) {
		range = icu_gpt_algo_get_target_range(dev_ptr, target_num, range_type);
		if (range == CH_NO_TARGET) {
			CH_LOG_APP_MSG("        no target found  ");
			continue;
		}

		amplitude = icu_gpt_algo_get_target_amplitude(dev_ptr, target_num);

		if (target_num == 0) {
			/* store first target values */
			chirp_data[dev_num].range     = range;
			chirp_data[dev_num].amplitude = amplitude;
		}

		CH_LOG_APP_MSG("Tgt %d: %0.2f mm ", target_num, (float)range / 32.0f);

		if (DISPLAY_SAMPLE_NUM) {
			uint16_t num_mm = (range / 32);

			/* ch_mm_to_samples() assumes num_mm is one-way (1/2 round-trip time-of-flight) */
			if (range_type != CH_RANGE_ECHO_ONE_WAY) {
				num_mm /= 2;  // mm distance was entire time-of-flight, so convert
			}
			CH_LOG_APP_MSG("(sample %u) ", ch_meas_mm_to_samples(dev_ptr, ch_meas_get_last_num(dev_ptr), num_mm));
		}

		if (DISPLAY_AMP_VALUE) {
			CH_LOG_APP_MSG("amp=%5u ", amplitude);
		}
		CH_LOG_APP_MSG("  ");
	}
}
#endif /* USE_RANGEFINDING */

#if (OUTPUT_AMP_DATA_CSV || OUTPUT_IQ_DATA)
/*
 * display_sensor_data() - display full sample data from a measurement
 *
 * This function displays the complete set of sample values from a measurement.  Either
 * the amplitude or raw I/Q values may be output, based on the definitions of
 * OUTPUT_AMP_DATA_CSV and OUTPUT_IQ_DATA.
 */
static void display_sensor_data(chirp_data_t *sensor_data)
{
	uint16_t num_samples = sensor_data->num_samples;
#if OUTPUT_IQ_DATA
	ch_iq_sample_t *iq_ptr = (ch_iq_sample_t *)&sensor_data->iq_data[0];
#endif

	/* Output amplitude or I/Q values for each sample in measurement */
	if (num_samples > APP_DATA_MAX_SAMPLES) {  // don't overflow app data buffer
		num_samples = APP_DATA_MAX_SAMPLES;
	}
#if OUTPUT_AMP_DATA_CSV
	CH_LOG_APP_START("AMP_DATA: ");
#endif
	for (uint16_t sample_num = 0; sample_num < num_samples; sample_num++) {
		/* Use selected format for this sample */
#if OUTPUT_AMP_DATA_CSV
		if (sample_num != 0) {
			CH_LOG_APP_MSG(", ");
		}
		CH_LOG_APP_MSG("%u", sensor_data->amp_data[sample_num]);  // continue line with comma and new amp value
#elif OUTPUT_IQ_DATA
		CH_LOG_APP("%d,%d", iq_ptr->q, iq_ptr->i);  // Q, I on new line
		iq_ptr++;
#endif
	}
#if OUTPUT_AMP_DATA_CSV
	CH_LOG_APP_END();
#endif
}
#endif  // (OUTPUT_AMP_DATA_CSV || OUTPUT_IQ_DATA)

#if (OUTPUT_AMP_LOG || OUTPUT_IQ_LOG)
/*
 * log_sensor_data() - log full sample data from a measurement
 *
 * This function logs the complete set of sample values from a measurement.  Either
 * the amplitude or raw I/Q values may be included in the log file, based on the
 * definitions of OUTPUT_AMP_LOG and OUTPUT_IQ_LOG.
 *
 * Each measurement generates one line of output, formatted to match the
 * Red Swallow logging layout.  The column headers will have already been output
 * when measurements begin.
 */
static void log_sensor_data(chirp_data_t *sensor_data)
{
	/* Set up log data struct */
	ch_log_data_t log_data = {
		.tx_sensor_id = sensor_data->tx_sensor_num,
		.rx_sensor_id = sensor_data->rx_sensor_num,
		.range        = sensor_data->range,
		.amplitude    = sensor_data->amplitude,
		.annotation   = 0,
#if OUTPUT_IQ_LOG
		.output_type            = CH_OUTPUT_IQ,
		.raw_data.iq_sample_ptr = &sensor_data->iq_data[0],
#elif OUTPUT_AMP_LOG
		.output_type           = CH_OUTPUT_AMP,
		.raw_data.mag_data_ptr = &sensor_data->amp_data[0],
#endif
		.start_sample = 0,
		.num_samples  = sensor_data->num_samples,
	};

	/* Log data */
	ch_log_append(log_id, CH_LOG_FMT_REDSWALLOW, time_get_in_us(), &log_data);
}
#endif  // (OUTPUT_AMP_LOG || OUTPUT_IQ_LOG)
