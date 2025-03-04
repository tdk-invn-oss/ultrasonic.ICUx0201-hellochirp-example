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

/**
 * \file chirp_board_config.h
 *
 * This file defines required symbols used to build an application with the Chirp SonicLib
 * API and driver.  These symbols are used for static array allocations and counters in SonicLib
 * (and often applications), and are based on the number of specific resources on the target board.
 *
 * Two symbols must be defined:
 *  CHIRP_MAX_NUM_SENSORS - the number of possible sensor devices (i.e. the number of sensor ports)
 *  CHIRP_NUM_BUSES - the number of I2C/SPI buses on the board that are used for those sensor ports
 *
 * This file must be in the C pre-processor include path when the application is built with SonicLib
 * and this board support package.
 */

#ifndef CHIRP_BOARD_CONFIG_H
#define CHIRP_BOARD_CONFIG_H

/* Settings for the Chirp SmartSonic board */
#define CHIRP_MAX_NUM_SENSORS 4  // maximum possible number of sensor devices
#define CHIRP_NUM_BUSES       1  // number of I2C/SPI buses used by sensors

/* Following defines only used if application uses INCLUDE_SHASTA_SUPPORT */
#define CHIRP_SENSOR_INT_PIN  1
#define CHIRP_SENSOR_TRIG_PIN 1

#ifndef CHIRP_RTC_CAL_PULSE_MS
#define CHIRP_RTC_CAL_PULSE_MS 100 /* Default length of RTC calibration pulse */
#endif

/* Following defines only used if application uses INCLUDE_WHITNEY_SUPPORT */

/* On SAMG55, the I2C speed has an SCL low level limit to 384 KHz,
 * mean the SCL 400 kKhz expected speed value wthe error to improve its real value
 * See twi.c:twi_set_speed() for more details
 */
#define CHIRP_I2C_SPEED_HZ (400e3 + 384e3) / 2

/* Deactivate use of debug I2C interface */
#define USE_STD_I2C_FOR_IQ (1)

#endif /* CHIRP_BOARD_CONFIG_H */