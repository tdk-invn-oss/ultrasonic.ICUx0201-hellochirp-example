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

#ifndef CHIRP_SMARTSONIC_H
#define CHIRP_SMARTSONIC_H

#include <stdbool.h>

#include <invn/soniclib/soniclib.h>
#include "chirp_board_config.h"
#include "app_config.h"

/* I2C Address assignments for each possible device */
#define CHX01_I2C_ADDRS \
	{ \
		0x2D, 0x2C, 0x2B, 0x2A \
	}
#define CHX01_I2C_BUSES \
	{ \
		0, 0, 0, 0 \
	} /* all sensors on same bus */

/* Processor sleep mode */
#define PROC_SLEEP_MODE SAM_PM_SMODE_SLEEP_WFI /* wait for interrupt */

/* TC channel used for the ultrasound timer and lsepoch of the system */
#define TC_CHANNEL_LSEPOCH (0)
#define TC_CHANNEL_US      (1)
#define TC_CHANNEL_DELAY   (2)

/* Define the HW frequency of the TC used for the ultrasound periodic timer */
#define ULTRASOUND_TIMER_FREQUENCY (499985)            /* = 32768 * 3662 / 240 */
#define DELAY_TIMER_FREQUENCY      (32768 * 3662 / 32) /* EXT XC * PLLA_MUL / TIMER3_DIV = 32k * 3662 / 32 */

/* Interrupts priority (Hi > Lo) */
#define INT_PRIO_UART_COMM       (0)
#define INT_PRIO_I2C_BUS         (1)
#define INT_PRIO_SPI_BUS         (1)
#define INT_PRIO_ICUx_INT_PINS   (2)
#define INT_PRIO_EVB_INT_PINS    (2)
#define INT_PRIO_IO_EXP_INT_PINS (2)
#define INT_PRIO_TIMER_US        (3)
#define INT_PRIO_TIMER_EPOCH     (4)
#define INT_PRIO_ADC             (5)

extern ch_group_t chirp_group;

#endif /* CHIRP_SMARTSONIC_H */
