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

/*! \file app_config.h */
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <invn/soniclib/soniclib.h>

/*========================= Sensor Firmware Selection ===========================*/

/* Select sensor firmware to use
 *   The sensor firmware type is specified during the call to ch_init(), by
 *   giving the name (address) of the firmware initialization function that will
 *   be called.  The CHIRP_SENSOR_FW_INIT_FUNC symbol is used to specify the
 *   init routine for the sensor firmware to be used.
 *
 *   If you only need raw sensor data and need more than the standard number of
 *   supported IQ samples (around 340), you can use the icu_init firmware. Follow
 *   the instructions below to switch to the init firmware.
 *
 *      * Comment #define USE_RANGEFINDING
 *      * Change CHIRP_SENSOR_ODR to desired value
 *      * Change CHIRP_RX_SEG_1_SAMPLES to the desired maximum number of
 *        receive samples - 1 (max 679)
 *      * Set OUTPUT_IQ_DATA_SUMMARY to 1 and disable the others
 *      * If using IMPORT_MEASUREMENT, ensure your measurement queue has no more
 *        than 680 RX samples total
 */
#define USE_RANGEFINDING

/*=========================== Sensor Configuration ===========================*/

/* Operating mode for the sensor(s)
 *	 These two values define the mode in which the sensor(s) will operate.
 *
 *	 - CH_MODE_FREERUN - sensor is triggered from internal RTC
 *	 - CH_MODE_TRIGGERED_TX_RX - sensor is hardware triggered. Each measurement
 *	   does a transmit followed by a receive
 *	 - CH_MODE_TRIGGERED_RX_ONLY - sensor is hardware triggered. Each
 *	   measurement does a receive only
 *	 - CH_MODE_CONTINUOUS_RX - receiver runs continuously (100% duty cycle)
 *
 *	 The CHIRP_FIRST_SENSOR_MODE value specifies the mode for the first sensor
 *	 (lowest numbered) that is present.  If only one sensor is attached, this
 *	 value must be one of CH_MODE_TRIGGERED_TX_RX, CH_MODE_FREERUN,
 *	 or CH_MODE_CONTINUOUS_RX.
 *
 *	 The CHIRP_OTHER_SENSOR_MODE value specifies the mode for all other sensors
 *	 that are present.
 *
 *	 For typical Pitch-Catch operation using two or more sensors, set
 *	 CHIRP_FIRST_SENSOR_MODE to CH_MODE_TRIGGERED_TX_RX and set
 *	 CHIRP_OTHER_SENSOR_MODE to CH_MODE_TRIGGERED_RX_ONLY.
 *
 *	 In CONTINUOUS_RX mode, it is suggested to set CHIRP_RINGDOWN_CANCEL_SAMPLES
 *	 to 0.
 *
 *	 Do not mix the TRIGGERED modes with the FREERUN or CONTINUOUS modes.
 */
#define CHIRP_FIRST_SENSOR_MODE (CH_MODE_TRIGGERED_TX_RX)
#define CHIRP_OTHER_SENSOR_MODE (CH_MODE_TRIGGERED_RX_ONLY)

/* Trigger type for the sensor(s)
 *
 * To trigger a sensor by generating a pulse on the INT line (hardware), set the
 * sensor trigger type CHIRP_TRIGGER_TYPE to CH_TRIGGER_TYPE_HW.
 * To trigger a sensor using software (SPI) interface, set CHIRP_TRIGGER_TYPE
 * to CH_TRIGGER_TYPE_SW.
 *
 * Software triggering is not recommended for multi-sensor pitch-catch operation
 * because there is a time lag between triggering the sensors.
 */
#define CHIRP_TRIGGER_TYPE (CH_TRIGGER_TYPE_HW)

/* Maximum detection range for the sensor
 * This value will determine how long the sensor "listens" for an ultrasound
 * signal.  Note that the maximum possible range will vary depending on sensor
 * model and sensor firmware type.  If the value specified here is greater than
 * the maximum possible range, the maximum possible range will be used.
 * These values are only used when USE_RANGEFINDING is defined.
 */
#define CHIRP_ICU10201_MAX_RANGE_MM (750)  /* maximum range, in mm for ICU10201 */
#define CHIRP_ICU20201_MAX_RANGE_MM (4000) /* maximum range, in mm for ICU20201 */
#define CHIRP_ICU30201_MAX_RANGE_MM (7000) /* maximum range, in mm for ICU30201 */

// Static target rejection sample range, in samples (0=disabled)
/*
 * This value specifies if static target rejection (STR) will be used.  If
 * CHIRP_STATIC_REJECT_SAMPLES is non-zero, STR will be enabled and will apply
 * to the specified number of samples at the beginning of a measurement.
 * The sensor will only report targets that are non-stationary.
 *
 * To enable STR filtering for the entire measurement, set this value
 * to ICU_MAX_NUM_SAMPLES.
 *
 * This feature is often combined with target interrupt filtering to reduce the
 * number of data ready interrupts, for power conservation.
 */
#define CHIRP_STATIC_REJECT_SAMPLES (0)

/* Target interrupt filter
 * This setting controls whether the sensor will generate an interrupt after
 * every measurement, or only if a target is detected.  By default, a data
 * ready interrupt is generated after every measurement.
 *
 * Set this value to non-zero to enable target interrupt filtering.  The sensor
 * will only generate a data ready interrupt if a target was detected during
 * the measurement.
 *
 * This feature is often combined with static target rejection to reduce the
 * number of data ready interrupts, for power conservation.
 */
#define CHIRP_TARGET_INT_FILTER (0)

/* Rx holdoff
 * This value specifies a certain number of samples at the start of each
 * measurement that will be ignored during target detection.  Although data
 * from these samples will be measured (and reported if amplitude or I/Q
 * output is enabled), no target will be reported within this range.  This
 * may be useful for ignoring objects close to the sensor.
 *
 * For ICU sensors, the Rx Holdoff setting will modify the detection thresholds
 * for the specified range of samples.
 *
 * Set CHIRP_RX_HOLDOFF_SAMPLES to non-zero to specify a number of samples
 * to ignore, or zero to disable.
 */
#define CHIRP_RX_HOLDOFF_SAMPLES (0)

/* Receive sensor pre-triggering
 * This value specifies if receive (rx) sensor pre-triggering will be used.
 * This setting only applies if more than one sensor is used, and one or more
 * sensor is operating in CH_MODE_TRIGGERED_RX_ONLY.

 * Receive pre-triggering improves performance in pitch-catch operation at
 * short distances, by triggering the receive-only sensor(s) slightly before
 * the transmitting sensor.  However, this setting will reduce maximum range of
 * rx-only sensors approximately 200mm, relative to the
 * CHIRP_MAX_RANGE_MM setting, above.
 *
 * Set RX_PRETRIGGER_ENABLE to non-zero to enable rx pretriggering, or zero
 * to disable.
 */
#define CHIRP_RX_PRETRIGGER_ENABLE (1)

/* Force sensor Fop
 * If set, this value force the same fop for all sensors in pitch-catch.
 * so they'll have a better sensitivity to hear each other.
 */
#define CHIRP_IMPROVE_PITCH_CATCH_SENSITIVITY (0)

/* Maximum number of targets
 * The specifies the maximum number of separate target objects for which the sensor will
 * report range and amplitude values.
 */
#define CHIRP_MAX_TARGETS (4) /* Maximum number of targets to detect */

/* Target detection thresholds
 * These definitions set the sensor's target detection thresholds.  These thresholds specify
 * how large a signal must be received, and at what point in the measurement, for the sensor
 * to indicate that a target was detected and calculate range, etc.
 *
 * Each threshold consists of a starting sample number within the measurement and the
 * corresponding amplitude value that must be reached.  A threshold extends until the
 * starting sample number of the next threshold, if any.
 *
 * These values are used to initialize the chirp_detect_thresholds structure defined
 * in main.c, which is passed to the icu_gpt_algo_configure() function.
 */
#define CHIRP_THRESH_0_START (0) /* threshold 0 - must start at sample zero */
#define CHIRP_THRESH_0_LEVEL (2500)

#define CHIRP_THRESH_1_START (40) /* threshold 1 */
#define CHIRP_THRESH_1_LEVEL (800)

#define CHIRP_THRESH_2_START (70) /* threshold 2 */
#define CHIRP_THRESH_2_LEVEL (400)

#define CHIRP_THRESH_3_START (100) /* threshold 3 */
#define CHIRP_THRESH_3_LEVEL (200)

#define CHIRP_THRESH_4_START (160) /* threshold 4 */
#define CHIRP_THRESH_4_LEVEL (140)

#define CHIRP_THRESH_5_START (200) /* threshold 5 */
#define CHIRP_THRESH_5_LEVEL (95)

#define CHIRP_THRESH_6_START (240) /* threshold 6 */
#define CHIRP_THRESH_6_LEVEL (50)

#define CHIRP_THRESH_7_START (0) /* threshold 7 (not used) */
#define CHIRP_THRESH_7_LEVEL (0)

/*============================ Measurement Settings ============================*/

/* Importing a measurement
 * This definition controls whether the example application will use a measurement
 * definition imported from a file (for example, one exported from the ICU-x0201
 * EVK development kit) instead of using the settings defined below.
 *
 * If you set IMPORT_MEASUREMENT to non-zero, the application will use the measurement
 * defined in the example measurement_config.c file.
 */
#define IMPORT_MEASUREMENT (0) /* if non-zero, measurement will be imported */

/* Configuration of measurement segments
 * The following symbols define the individual transmit and receive segments within
 * the measurement.
 */
#define CHIRP_TX_SEG_CYCLES      (640) /* Transmit segment - length (cycles) */
#define CHIRP_TX_SEG_PULSE_WIDTH (3)   /* width of each cycle pulse */
#define CHIRP_TX_SEG_PHASE       (8)   /* tx phase */
#define CHIRP_TX_SEG_INT_EN      (0)   /* no interrupt when done */

#define CHIRP_RX_SEG_0_SAMPLES     (1)  /* Receive segment 0 - single sample */
#define CHIRP_RX_SEG_0_GAIN_REDUCE (24) /* reduce gain */
#define CHIRP_RX_SEG_0_ATTEN       (1)  /* use attenuation */
#define CHIRP_RX_SEG_0_INT_EN      (0)  /* no interrupt when done */

#define CHIRP_RX_SEG_1_SAMPLES     (295) /* Receive segment 1 - remainder of measurement */
#define CHIRP_RX_SEG_1_GAIN_REDUCE (0)   /* no gain reduction */
#define CHIRP_RX_SEG_1_ATTEN       (0)   /* no attenuation */
#define CHIRP_RX_SEG_1_INT_EN      (1)   /* generate interrupt when done (if eligible) */

/* Ringdown filtering
 * The ICU sensor performs "ringdown cancellation" filtering to remove artifacts from
 * the received signal in the first samples in a measurement.  This symbols specifies
 * how many samples at the beginning of the measurement will be subject to
 * ringdown filtering.  You may need to increase this value if you increase the
 * data rate for the sensor (CHIRP_SENSOR_ODR, below), because more samples will be
 * taken during the ringdown phase.
 */
#define CHIRP_RINGDOWN_FILTER_SAMPLES (20) /* number of samples to filter */

#define CHIRP_DETECT_TARGET_IN_RINGDOWN (0) /* if non-zero, print if a target is detected near sensor */

/* Output data rate
 * This symbol specifies the output data rate for the sensor, i.e. how often
 * a sample is taken within each measurement.
 */
#define CHIRP_SENSOR_ODR (CH_ODR_FREQ_DIV_8)

/* The following symbol enables optimization of the measurement segments to
 * improve performance at close distances.  Additional segments will be inserted
 * into the sequence that was defined above or imported from a file.
 *
 * Set to non-zero to enable measurement optimization.
 */
#define CHIRP_MEAS_OPTIMIZE (0) /* if non-zero, meas segments will be optimized */

/*============================ Application Timing ============================*/

/* Define how often the application will get a new sample from the sensor(s)
 *   This macro defines the sensor measurement interval, in milliseconds.
 *
 *   For sensors in triggered mode (CH_MODE_TRIGGERED_TX_RX or
 *   CH_MODE_TRIGGERED_RX_ONLY), the application will use a periodic timer to
 *   trigger a sensor measurement each time this period elapses.
 *
 *   For sensors in free-running mode (CH_MODE_FREERUN), the application will
 *   set this period as the sensor's internal sample interval. Additionally, in
 *   free-running mode, a sample-to-sample random "time hop" will be added to
 *   the sample period. This one method of coexistence mitigation when combined
 *   with certain sensor firmware. To disable the time hopping behavior, you
 *   should call ch_freerun_time_hop_disable() from your application.
 *
 *   For sensors in continuous RX mode, this field is ignored.
 */
#define MEASUREMENT_INTERVAL_MS (100)

/*==================  Application Storage for Sensor Data ====================*/

/* Define how many samples per measurement are expected by this application
 *   The following symbol is used to allocate array storage in the "chirp_data_t"
 *   structure defined in main.c.  That structure contains arrays for
 *   individual data values (I/Q or amplitude) that describe the raw samples
 *   within an ultrasound measurement.
 * 
 * IMPORTANT: APP_DATA_MAX_SAMPLES must be configured to be greater than the
 * expected number of IQ samples, or buffer overflow will result, which causes
 * undefined behavior.
 */
#ifdef USE_RANGEFINDING
#define APP_DATA_MAX_SAMPLES (ICU_MAX_NUM_SAMPLES)
#else
#include <invn/soniclib/sensor_fw/icu_init/icu_init.h>

#define APP_DATA_MAX_SAMPLES (ICU_INIT_MAX_SAMPLES)
#endif

/*================  Build Options for Sample Data Handling ==================*/

/* The following build options control if and how the full set of values for
 * all internal samples within an ultrasound measurement will be read and
 * displayed.  This data is separate from the standard range and simple target
 * amplitude values that are normally output.
 *
 * The measurement data can be read and displayed either as net amplitude values
 * or as individual I and Q components.
 *
 * Note that reading the full data set is not required for most basic
 * sensing applications - the reported range value, possibly combined with the
 * simple target amplitude value, is typically all that is required.  However,
 * the full set of sample values may be read and analyzed for more advanced
 * sensing or data capture needs.
 *
 * In CONTINUOUS_RX mode, the amount of incoming data can be too much for the logging
 * to keep up with. If you see issues, such as "bad interrupt type", then you should
 * reduce the CHIRP_SENSOR_ODR and/or increase the USART_CONSOLE_BAUDRATE to 3000000,
 * found in conf_board.h.
 *
 * Set OUTPUT_AMP_DATA_CSV to 1 (non-zero) to enable output of the amplitude data via
 * the serial port, as a series of comma separated values all on one line.  This allows
 * easy cut/paste into a spreadsheet program to generate an "A-Scan" chart.  This
 * kind of summary graph can be very helpful to understand what the sensor is observing.
 *
 * Set OUTPUT_AMP_LOG to 1 (non-zero) to enable output of the sample amplitude data in
 * Red Swallow log file format.  This is the same output format used by the ICU-x0201
 * Evaluation Platform tools.
 *
 * Set OUTPUT_IQ_DATA to 1 (non-zero) to enable output of the I/Q pair values.
 * Each I/Q sample appears as a comma separated pair of (Q,I) values on its own line.
 * Having the separate I and Q values allows more complex analysis of signal phase, etc.
 *
 * Set OUTPUT_IQ_LOG to 1 (non-zero) to enable output of the sample I/Q data in
 * Red Swallow log file format.  This is the same output format used by the ICU-x0201
 * Evaluation Platform tools.
 *
 * Set OUTPUT_IQ_DATA_SUMMARY to 1 (non-zero) to enable output of a summary of the
 * IQ data that was read out. This is useful if you only want to see how many samples
 * were read.
 *
 * OUTPUT_AMP_DATA_CSV, OUTPUT_AMP_LOG, OUTPUT_IQ_DATA, OUTPUT_IQ_LOG, and
 * OUTPUT_IQ_DATA_SUMMARY cannot be used together.
 */

#define OUTPUT_AMP_DATA_CSV    (0) /* if non-zero, output amplitude data on one line */
#define OUTPUT_AMP_LOG         (0) /* if non-zero, output amplitude data in Red Swallow log format */
#define OUTPUT_IQ_DATA         (0) /* if non-zero, output I/Q data on multiple lines */
#define OUTPUT_IQ_LOG          (0) /* if non-zero, output I/Q data in Red Swallow log format */
#define OUTPUT_IQ_DATA_SUMMARY (0) /* if non-zero, output only a summary of IQ data read (e.g. number of samples) */

/* Blocking vs. non-blocking I/O
 * By default, this application will read sample data in blocking mode
 * (i.e. READ_DATA_NONBLOCKING is zero by default).  The amplitude or I/Q data will
 * be read from the device and placed in the corresponding data array field in the
 * application's chirp_data structure.  Because the data is read in blocking mode, the
 * calls to ch_get_amplitude_data() and ch_get_iq_data() will not return until the data
 * has actually been copied from the device.
 *
 * However, if READ_IQ_NONBLOCKING is non-zero, the sample data will be read in non-blocking
 * mode. The ch_get_amplitude_data() and ch_get_iq_data() calls will return immediately,
 * and a separate callback function will be called to notify the application when the read
 * operation is complete.
 */
#define READ_DATA_NONBLOCKING (0) /* if non-zero, use non-blocking mode */

/*========================  Application Display Options ==========================*/

/* These symbols control what values are displayed for each measurement. */
#define DISPLAY_AMP_VALUE  (1) /* if non-zero, show amplitude for detected target(s) */
#define DISPLAY_SAMPLE_NUM (1) /* if non-zero, show sample number for detected target(s) */

/* if non-zero, multiple detected targets displayed
   if zero, only closest target is displayed */
#define DISPLAY_MULTI_TARGET (0)

#endif /* APP_CONFIG_H */
