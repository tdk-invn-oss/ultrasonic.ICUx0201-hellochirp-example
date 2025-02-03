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
#include <delay.h>
#include <string.h>

#include "uart.h"
#include "uart_hal.h"
#include "time.h"
#include "redswallow.h"
#include "redswallow_hal.h"

/* This file defines an UART backend implementation for the protocol for the Atmel MCU */

/* Private singleton */
static struct UartManager {
	uint8_t *new_bytes_buffer;
	volatile uint32_t new_bytes_buffer_idx;
	size_t new_bytes_buffer_size;

	volatile bool tx_done;
} uart_mngr = {0};

static uint8_t uart_rx_buffer;
/* Debug purpose */
static uint32_t uart_rx_cnt = 0;
static uint32_t uart_tx_cnt = 0;

void uart_hal_rx_received_callback(void)
{
	uart_dma_getc(&uart_rx_buffer, 1);
	uart_rx_cnt++;
	if (uart_mngr.new_bytes_buffer_idx < uart_mngr.new_bytes_buffer_size) {
		uart_mngr.new_bytes_buffer[uart_mngr.new_bytes_buffer_idx] = uart_rx_buffer;
		uart_mngr.new_bytes_buffer_idx++;
	} else {
		redswallow_log_error("PROTOCOL_ERROR uart_mngr.new_bytes_buffer full");
	}
	redswallow_new_char_callback();
}

void uart_hal_tx_sent_callback(void)
{
	uart_tx_cnt++;
	uart_mngr.tx_done = true;
}

/* Public functions */
void redswallow_hal_init(uint8_t *new_bytes_buffer, size_t new_bytes_buffer_size)
{
	memset(&uart_mngr, 0, sizeof(uart_mngr));

	uart_mngr.new_bytes_buffer      = new_bytes_buffer;
	uart_mngr.new_bytes_buffer_size = new_bytes_buffer_size;

	uart_init(USE_RTS_CTS, &uart_rx_buffer);

	uart_mngr.tx_done = true;
}

bool redswallow_hal_is_connected_to_host(void)
{
	return uart_is_connected();
}

void redswallow_hal_new_input_bytes(void)
{
	cpu_irq_enter_critical();
	redswallow_new_input_bytes(uart_mngr.new_bytes_buffer, uart_mngr.new_bytes_buffer_idx);
	uart_mngr.new_bytes_buffer_idx = 0;
	cpu_irq_leave_critical();
}

bool redswallow_hal_send(const uint8_t *data, size_t len)
{
	bool rc = true;
	uint64_t t_start_send;
	uint64_t time_out_us;

	/* Wait UART ready to send */
	if (!uart_is_connected())
		return false;

	t_start_send = time_get_in_us();
	do {
		/* Check send time-out */
		time_out_us  = time_get_in_us();
		time_out_us -= t_start_send;
		if (time_out_us > (uint64_t)REDSWALLOW_HAL_SEND_TIME_OUT_US) {
			uart_mngr.tx_done = true;
			rc                = false;
		}
	} while (!uart_mngr.tx_done);

	if (rc) {
		/* send data */
		uart_mngr.tx_done = false;
		uart_dma_puts(data, len);
	}

	return rc;
}

void redswallow_hal_delay_us(uint32_t delay_us)
{
	delay_us(delay_us);
}

void redswallow_hal_packet_lock(void)
{
}

void redswallow_hal_packet_unlock(void)
{
}
