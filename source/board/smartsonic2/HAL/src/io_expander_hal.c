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
#include "conf_board.h"
#include "i2c_hal.h"
#include "io_expander_hal.h"
#include "chirp_smartsonic.h"

int8_t io_exp_hal_init(void)
{
	pmc_enable_periph_clk(ICUx_INT_PORT_ID);

	/* Enable pull-ups on the INT pins */
	pio_pull_up(IO_EXP_INT_PORT, IO_EXP_INT_PINS, 1);
	pio_pull_down(IO_EXP_INT_PORT, IO_EXP_INT_PINS, 0);

	/* Configure PIOs as input pins. */
	pio_configure(IO_EXP_INT_PORT, PIO_INPUT, IO_EXP_INT_MASK, IO_EXP_INT_PIN_CONFIG);
	pio_handler_set(IO_EXP_INT_PORT, IO_EXP_INT_PORT_ID, IO_EXP_INT_MASK, IO_EXP_INT_PIN_CONFIG, NULL);
	pio_handler_set_priority(IO_EXP_INT_PORT, IO_EXP_INT_PIN_IRQ, INT_PRIO_IO_EXP_INT_PINS);

	/* I2C already initialized in I2C HAL */
	return 0;
}

int8_t io_exp_hal_read_register(uint8_t i2c_addr, uint8_t reg, uint8_t *rbuffer, uint32_t rlen)
{
	return i2c_hal_read_register(I2C_BUS_IDX_CONTROL, i2c_addr, reg, rbuffer, rlen);
}

int8_t io_exp_hal_write_register(uint8_t i2c_addr, uint8_t reg, const uint8_t *wbuffer, uint32_t wlen)
{
	return i2c_hal_write_register(I2C_BUS_IDX_CONTROL, i2c_addr, reg, wbuffer, wlen);
}
