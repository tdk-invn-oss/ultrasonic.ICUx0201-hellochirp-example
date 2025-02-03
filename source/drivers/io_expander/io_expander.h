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

#ifndef _IO_EXP_H_
#define _IO_EXP_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * Define pins addressable by externals gpios
 *
 * \{
 */
#define IO_EXP_PIN_P0_POS 0
#define IO_EXP_PIN_P1_POS 1
#define IO_EXP_PIN_P2_POS 2
#define IO_EXP_PIN_P3_POS 3
#define IO_EXP_PIN_P4_POS 4
#define IO_EXP_PIN_P5_POS 5
#define IO_EXP_PIN_P6_POS 6
#define IO_EXP_PIN_P7_POS 7

#define IO_EXP_PIN_P0 (uint8_t)(1 << IO_EXP_PIN_P0_POS)
#define IO_EXP_PIN_P1 (uint8_t)(1 << IO_EXP_PIN_P1_POS)
#define IO_EXP_PIN_P2 (uint8_t)(1 << IO_EXP_PIN_P2_POS)
#define IO_EXP_PIN_P3 (uint8_t)(1 << IO_EXP_PIN_P3_POS)
#define IO_EXP_PIN_P4 (uint8_t)(1 << IO_EXP_PIN_P4_POS)
#define IO_EXP_PIN_P5 (uint8_t)(1 << IO_EXP_PIN_P5_POS)
#define IO_EXP_PIN_P6 (uint8_t)(1 << IO_EXP_PIN_P6_POS)
#define IO_EXP_PIN_P7 (uint8_t)(1 << IO_EXP_PIN_P7_POS)

#define IO_EXP_PIN_ALL \
	IO_EXP_PIN_P0 | IO_EXP_PIN_P1 | IO_EXP_PIN_P2 | IO_EXP_PIN_P3 | IO_EXP_PIN_P4 | IO_EXP_PIN_P5 | IO_EXP_PIN_P6 | \
			IO_EXP_PIN_P7
/** \} */

#define IO_EXP_SET_LEVEL   1
#define IO_EXP_CLEAR_LEVEL 0

#define IO_EXP_IN_DIR            1
#define IO_EXP_OUT_DIR           0
#define IO_EXP_INV_IN_PIN        1
#define IO_EXP_DO_NOT_INV_IN_PIN 0

#define IO_EXP_P0X_PORT (0)
#define IO_EXP_P1X_PORT (1)

/**
 * @brief      Initialize External GPIO driver
 * @return     0 if no error, negative value in other cases
 */
int8_t io_exp_init(void);

/**
 * @brief      Configure external GPIO driver pins
 *
 * @param[in]  i2c_addr                    I2C address of IO Exp device
 * @param[in]  port                        Lower (0) or Higher (1) port to configure
 * @param[in]  pins_mask                   Pins to initialize
 * @param[in]  pins_direction              pins direction bitmask of driver (1= in, 0 = out)
 * @param[in]  input_pins_inversion        The input pins inversion bitmask (1= inverted, 0 = not
 * inverted)
 * @param[in]  output_pins_default_values  The output pins default values (for output pins)
 *
 * @return     0 if no error, negative value in other cases
 */
int8_t io_exp_configure_pins(uint8_t i2c_addr, uint8_t port, uint8_t pins_mask, uint8_t pins_direction,
                             uint8_t input_pins_inversion, uint8_t output_pins_default_values);

/*!
 * \brief Read the current pin level
 *
 * @param[in]  i2c_addr                    I2C address of IO Exp device
 * @param[in]  port                        Lower (0) or Higher (1) port to read
 * \param[in] *pins_level Pointer to store the pins level
 * \return 0 if no error, negative value in other cases
 */
int8_t io_exp_get_pins_level(uint8_t i2c_addr, uint8_t port, uint8_t *pins_level);

/**
 * @brief      Set outputs to pin levels values
 *
 * @param[in]  i2c_addr                    I2C address of IO Exp device
 * @param[in]  port                        Lower (0) or Higher (1) port to set
 * @param[in]  pins_bit_mask    Bitmask of outputs to set/clear
 * @param[in]  pins_level_mask  Levels to set on pins
 * @return 0 in case of success, negative values otherwise
 */
int8_t io_exp_set_pins_level(uint8_t i2c_addr, uint8_t port, uint8_t pins_bit_mask, uint8_t pins_level_mask);

/**
 * @brief      Toggle level on selected pins
 * @param[in]  i2c_addr       I2C address of IO Exp device
 * @param[in]  port           Lower (0) or Higher (1) port to set
 * @param[in]  pins_bit_mask  Bitmask of outputs to set/clear
 * @return 0 in case of success, negative values otherwise
 */
int8_t io_exp_toggle_pins_level(uint8_t i2c_addr, uint8_t port, uint8_t pins_bit_mask);

/**
 * @brief      Handler to call when an interrupt happend on IO Expander INT pin
 */
void io_exp_int_pin_handler(void);

#ifdef __cplusplus
}
#endif

#endif /* _IO_EXP_H_ */
