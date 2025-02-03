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

#include <stdbool.h>
#include <asf.h>
#include "time_hal.h"
#include "delay.h"
#include "tc.h"
#include "chirp_smartsonic.h"

#define DELAY_TIMER_TIME_TO_TICK(time_us) ((uint64_t)DELAY_TIMER_FREQUENCY * time_us / 1000000)

#define DELAY_TIMER_OVERHEAD_IN_TICK DELAY_TIMER_TIME_TO_TICK(15)

static void delay_us_timer(uint32_t us)
{
	uint32_t status;
	uint32_t delay_tick = DELAY_TIMER_TIME_TO_TICK(us);
	uint16_t delay_tick_w;

	/* remove computing delay offset to have an accurate delay */
	delay_tick -= DELAY_TIMER_OVERHEAD_IN_TICK;
	/* If delay in tick > 65535, make multiple loop */
	while (delay_tick > 0) {
		tc_get_status(TC0, TC_CHANNEL_DELAY); /* reset flags */
		delay_tick_w = (delay_tick > 0xFFFF) ? 0xFFFF : delay_tick;
		tc_write_rc(TC0, TC_CHANNEL_DELAY, delay_tick_w);
		tc_start(TC0, TC_CHANNEL_DELAY);
		do {
			status = tc_get_status(TC0, TC_CHANNEL_DELAY);
		} while (!(status & TC_SR_CPCS));
		tc_stop(TC0, TC_CHANNEL_DELAY);
		delay_tick -= delay_tick_w;
	}
}

int8_t time_hal_init(void)
{
	/* TC0 init'd in chbsp_chirp_samg55.c / timers_init() */
	return 0;
}

uint64_t time_hal_get_in_ticks(void)
{
	static uint64_t lsepoch_overflow_counter_in_tick = 0;

	cpu_irq_enter_critical();
	/* Check if a counter overflow occurs since last call and ensure no ovf occurs during the read */
	if (TC0->TC_CHANNEL[TC_CHANNEL_LSEPOCH].TC_SR & TC_SR_COVFS)
		lsepoch_overflow_counter_in_tick += 65536;

	uint16_t timer_counter = TC0->TC_CHANNEL[TC_CHANNEL_LSEPOCH].TC_CV;

	if (TC0->TC_CHANNEL[TC_CHANNEL_LSEPOCH].TC_SR & TC_SR_COVFS) {
		lsepoch_overflow_counter_in_tick += 65536;
		/* Overflow occurred during the reading */
		timer_counter = 0;
	}

	/* Convert to us */
	uint64_t curr_ticks = (lsepoch_overflow_counter_in_tick + timer_counter);
	cpu_irq_leave_critical();

	return (curr_ticks);
}

uint64_t time_hal_get_in_us(void)
{
	uint64_t curr_ticks = time_hal_get_in_ticks();

	/* Convert to us */
	return (curr_ticks * 1000000) / ULTRASOUND_TIMER_FREQUENCY;
}

void time_hal_delay_us(uint32_t us)
{
	if (us > 10)
		delay_us_timer(us);
	else
		delay_us(us - 5);
}
