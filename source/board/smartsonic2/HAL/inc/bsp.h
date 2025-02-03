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

/** \file bsp.h */
#ifndef BSP_H
#define BSP_H

#include <stdint.h>
#include <stdbool.h>
#include <invn/soniclib/soniclib.h>
#include "samg55.h"

enum led_color {
	BOARD_LED_GREEN = 0x01,
	BOARD_LED_RED   = 0x02,
	BOARD_LED_ALL   = 0x03,
};

void bsp_init(ch_group_t *grp_ptr);

uint8_t periodic_timer_init(uint16_t interval_ms, ch_timer_callback_t callback_func_ptr);
void periodic_timer_handler(void);
uint8_t periodic_timer_start(void);
uint8_t periodic_timer_stop(void);
void periodic_timer_change_period(uint32_t new_interval_us);
void periodic_timer_irq_disable(void);
void periodic_timer_irq_enable(void);

void proc_sleep(void);

void bsp_led_on(uint8_t led_num);
void bsp_led_off(uint8_t led_num);
void bsp_led_toggle(uint8_t led_num);
void indicate_alive(void);
void switch_user_led(bool on_off);
void switch_board_leds(enum led_color color, bool on_off);

void external_rtc_power_en(bool en);

#endif /* BSP_H */
