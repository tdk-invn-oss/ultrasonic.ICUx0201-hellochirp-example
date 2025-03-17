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

#ifndef IO_EXP_HAL_H
#define IO_EXP_HAL_H

#include <stdint.h>

/*!
 * \brief Initialize the host's I2C hardware and the I/O pins.
 * \return 0 in case of success, negative values otherwise
 * \note Customer has to implement this function.
 */
int8_t io_exp_hal_init(void);

/*!
 * \brief Read on the specified serial interface a buffer of rlen bytes.
 * \return 0 on success. On error, any negative number.
 * \param[in] i2c_addr  I2C address
 * \param[in] reg  Register to read.
 * \param[in] rbuffer Pointer to receive register value.
 * \param[in] rlen Size of the register to read.
 * \note Customer has to implement this function.
 */
int8_t io_exp_hal_read_register(uint8_t i2c_addr, uint8_t reg, uint8_t *rbuffer, uint32_t rlen);

/*!
 * \brief Writes on the specified serial interface a buffer of wlen bytes.
 * \return 0 on success. On error, any negative number.
 * \param[in] i2c_addr  I2C address
 * \param[in] reg  Register to write.
 * \param[in] wbuffer Pointer to value to send.
 * \param[in] wlen Size of the message to send.
 * \note Customer has to implement this function.
 */
int8_t io_exp_hal_write_register(uint8_t i2c_addr, uint8_t reg, const uint8_t *wbuffer, uint32_t wlen);

#endif /* IO_EXP_HAL_H */
