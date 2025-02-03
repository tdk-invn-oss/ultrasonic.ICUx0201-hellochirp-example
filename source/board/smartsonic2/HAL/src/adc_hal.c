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

#include <adc2.h>
#include <delay.h>

#include "conf_board.h"
#include "chirp_smartsonic.h"
#include "adc_hal.h"

#define ADC_CONV_TIME_US (30)

static const enum adc_interrupt_source channels_int_src[] = {
		ADC_INTERRUPT_EOC_0, ADC_INTERRUPT_EOC_1, ADC_INTERRUPT_EOC_2, ADC_INTERRUPT_EOC_3,
		ADC_INTERRUPT_EOC_4, ADC_INTERRUPT_EOC_5, ADC_INTERRUPT_EOC_6, ADC_INTERRUPT_EOC_7};

static volatile bool stop_continuous_conv   = true;
static adc_hal_interrupt_cbk_t app_callback = NULL;

static void end_of_conversion_cbk(uint8_t channel)
{
	uint16_t adc_value = adc_channel_get_value(ADC, channel);
	if (app_callback != NULL)
		app_callback(channel, adc_value);
	if (!stop_continuous_conv)
		adc_start_software_conversion(ADC);
}

inline static void end_of_conversion_cbk_chan0(void)
{
	end_of_conversion_cbk(0);
}

inline static void end_of_conversion_cbk_chan1(void)
{
	end_of_conversion_cbk(1);
}

inline static void end_of_conversion_cbk_chan2(void)
{
	end_of_conversion_cbk(2);
}

inline static void end_of_conversion_cbk_chan3(void)
{
	end_of_conversion_cbk(3);
}

inline static void end_of_conversion_cbk_chan4(void)
{
	end_of_conversion_cbk(4);
}

inline static void end_of_conversion_cbk_chan5(void)
{
	end_of_conversion_cbk(5);
}

inline static void end_of_conversion_cbk_chan6(void)
{
	end_of_conversion_cbk(6);
}

inline static void end_of_conversion_cbk_chan7(void)
{
	end_of_conversion_cbk(7);
}

static const adc_callback_t channels_int_cbk[] = {
		end_of_conversion_cbk_chan0, end_of_conversion_cbk_chan1, end_of_conversion_cbk_chan2,
		end_of_conversion_cbk_chan3, end_of_conversion_cbk_chan4, end_of_conversion_cbk_chan5,
		end_of_conversion_cbk_chan6, end_of_conversion_cbk_chan7,
};

int adc_hal_init(void)
{
	int rc;
	struct adc_config adc_cfg;

	stop_continuous_conv = true;
	adc_enable();
	adc_select_clock_source_mck(ADC);
	adc_get_config_defaults(&adc_cfg);
	rc = adc_init(ADC, &adc_cfg);

	if (!rc) {
		adc_set_trigger(ADC, ADC_TRIG_SW);
	} else {
		printf("ERROR : during ADC HAL init : %u\r\n", rc);
	}
	return rc;
}

int adc_hal_enable(uint32_t channel)
{
	adc_channel_enable(ADC, channel);

	return 0;
}

int adc_hal_disable(uint32_t channel)
{
	adc_channel_disable(ADC, channel);
	adc_disable_interrupt(ADC, channels_int_src[channel]);

	return 0;
}

void adc_hal_stop(uint32_t channel)
{
	(void)channel;
	stop_continuous_conv = true;
}

uint32_t adc_hal_read(uint32_t channel)
{
	uint32_t result;

	adc_start_software_conversion(ADC);
	// while (adc_get_interrupt_status(ADC) & (1 << channel));
	delay_us(ADC_CONV_TIME_US); /* wait for ADC conversion time */
	result = adc_channel_get_value(ADC, channel);

	return result;
}

void adc_hal_read_continous(uint32_t channel)
{
	stop_continuous_conv = false;
	adc_set_callback(ADC, ADC_INTERRUPT_DATA_READY, channels_int_cbk[channel], INT_PRIO_ADC);
	adc_start_software_conversion(ADC);
}

void adc_hal_set_int_cbk(adc_hal_interrupt_cbk_t interrupt_callback)
{
	app_callback = interrupt_callback;
}