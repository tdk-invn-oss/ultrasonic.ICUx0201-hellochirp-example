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
#include "io_expander.h"
#include "io_expander_hal.h"

#define PCA9535_BASE_REG_INPUT_PORT         0x00
#define PCA9535_BASE_REG_OUTPUT_PORT        0x02
#define PCA9535_BASE_REG_POL_INVERT_PORT    0x04
#define PCA9535_BASE_REG_CONFIGURATION_PORT 0x06

static int8_t read_modify_write_single_reg(uint8_t i2c_addr, uint8_t reg_addr, uint8_t pin_mask, uint8_t new_reg_val)
{
	int8_t status;
	uint8_t reg_val;

	status = io_exp_hal_read_register(i2c_addr, reg_addr, &reg_val, 1);
	if (status != 0)
		return status;

	/* unselect pin which do not have to change */
	reg_val &= ~pin_mask;
	/* set new value on selected pins */
	reg_val |= (new_reg_val & pin_mask);

	status = io_exp_hal_write_register(i2c_addr, reg_addr, &reg_val, 1);

	return status;
}

static int8_t toggle_single_reg(uint8_t i2c_addr, uint8_t reg_addr, uint8_t pin_mask)
{
	int8_t status;
	uint8_t reg_val;

	status = io_exp_hal_read_register(i2c_addr, reg_addr, &reg_val, 1);
	if (status != 0)
		return status;

	/* toogle selected pins */
	reg_val ^= pin_mask;

	status = io_exp_hal_write_register(i2c_addr, reg_addr, &reg_val, 1);

	return status;
}

int8_t io_exp_init(void)
{
	return io_exp_hal_init();
}

int8_t io_exp_configure_pins(uint8_t i2c_addr, uint8_t port, uint8_t pins_mask, uint8_t pins_direction,
                             uint8_t input_pins_inversion, uint8_t output_pins_default_values)
{
	int8_t status;

	status = io_exp_set_pins_level(i2c_addr, port, pins_mask, output_pins_default_values);
	if (status == 0)
		status = read_modify_write_single_reg(i2c_addr, PCA9535_BASE_REG_POL_INVERT_PORT + port, pins_mask,
		                                      input_pins_inversion);
	if (status == 0)
		status = read_modify_write_single_reg(i2c_addr, PCA9535_BASE_REG_CONFIGURATION_PORT + port, pins_mask,
		                                      pins_direction);

	if (status != 0)
		printf("Pin config error : %d", status);

	return status;
}

int8_t io_exp_set_pins_level(uint8_t i2c_addr, uint8_t port, uint8_t pins_bit_mask, uint8_t pins_level_mask)
{
	return read_modify_write_single_reg(i2c_addr, PCA9535_BASE_REG_OUTPUT_PORT + port, pins_bit_mask, pins_level_mask);
}

int8_t io_exp_toggle_pins_level(uint8_t i2c_addr, uint8_t port, uint8_t pins_bit_mask)
{
	return toggle_single_reg(i2c_addr, PCA9535_BASE_REG_OUTPUT_PORT + port, pins_bit_mask);
}

int8_t io_exp_get_pins_level(uint8_t i2c_addr, uint8_t port, uint8_t *pins_level)
{
	return io_exp_hal_read_register(i2c_addr, PCA9535_BASE_REG_INPUT_PORT + port, pins_level, 1);
}

void io_exp_int_pin_handler(void)
{
	/* TODO handle interrupts */
}