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
#include <ioport.h>
#include "conf_board.h"
#include "stdio_serial.h"
#include "console_hal.h"

/* Use the EDBG console for debug */
void console_hal_init(void)
{
	/* Configure pins for USART */
	ioport_set_port_mode(USART_CONSOLE_PORT, USART_CONSOLE_PINS, USART_CONSOLE_PINS_FLAGS);
	ioport_disable_port(USART_CONSOLE_PORT, USART_CONSOLE_PINS);
	ioport_set_pin_mode(USART_CONSOLE_TXD, USART_CONSOLE_PINS_FLAGS);
	ioport_disable_pin(USART_CONSOLE_TXD);
	ioport_set_pin_mode(USART_CONSOLE_RXD, USART_CONSOLE_PINS_FLAGS);
	ioport_disable_pin(USART_CONSOLE_RXD);

	/* Configure console UART. */
	const usart_serial_options_t uart_serial_options = {
			.baudrate   = USART_CONSOLE_BAUDRATE,
			.charlength = USART_CONSOLE_CHAR_LENGTH,
			.paritytype = USART_CONSOLE_PARITY,
			.stopbits   = USART_CONSOLE_STOP_BITS,
	};
	sysclk_enable_peripheral_clock(USART_CONSOLE_FLEXCOM_ID);
	usart_serial_init(USART_CONSOLE, (usart_serial_options_t *)&uart_serial_options);

	stdio_serial_init(USART_CONSOLE, &uart_serial_options);
	usart_enable_rx(USART_CONSOLE);
	usart_enable_tx(USART_CONSOLE);
	/* no IRQ on console UART */
}