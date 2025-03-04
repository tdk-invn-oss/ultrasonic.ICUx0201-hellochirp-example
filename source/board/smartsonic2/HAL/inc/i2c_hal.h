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

#ifndef _I2C_HAL_H_
#define _I2C_HAL_H_

#include <stdint.h>

#define I2C_BUS_IDX_CONTROL (0)
#define I2C_BUS_IDX_CHx01   (1)

int i2c_hal_init(uint8_t bus_index);
int i2c_hal_write(uint8_t bus_index, uint8_t i2c_addr, const uint8_t *data, uint16_t num_bytes);
int i2c_hal_read(uint8_t bus_index, uint8_t i2c_addr, uint8_t *data, uint16_t num_bytes);
int i2c_hal_write_register(uint8_t bus_index, uint8_t i2c_addr, uint8_t register_addr, const uint8_t *data,
                           uint16_t num_bytes);
int i2c_hal_read_register(uint8_t bus_index, uint8_t i2c_addr, uint8_t register_addr, uint8_t *data,
                          uint16_t num_bytes);
int i2c_hal_read_register_nb(uint8_t bus_index, uint8_t i2c_addr, uint8_t register_addr, uint8_t *data,
                             uint16_t num_bytes);
int i2c_hal_write_wregister(uint8_t bus_index, uint8_t i2c_addr, uint16_t register_addr, const uint8_t *data,
                            uint16_t num_bytes);
int i2c_hal_read_wregister(uint8_t bus_index, uint8_t i2c_addr, uint16_t register_addr, uint8_t *data,
                           uint16_t num_bytes);
int i2c_hal_read_nb(uint8_t bus_index, uint8_t i2c_addr, uint8_t *data, uint16_t num_bytes);
void i2c_hal_interrupt_handler(uint8_t bus_index);
void i2c_hal_read_nb_done_cbk(uint8_t bus_index);

#endif /* _I2C_HAL_H_ */