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

#include <usb_protocol_cdc.h>
#include <udi_cdc.h>
#include <udc.h>
#include <udd.h>

#include "conf_usb.h"

#include "uart_hal.h"
#include "chirp_smartsonic.h"

static bool autorize_cdc_transfert;
static volatile bool is_usb_connected = false;
static volatile bool usb_data_sent    = false;

int8_t uart_hal_init(uint8_t rts_cts, uint8_t *uart_rx_buffer)
{
	(void)rts_cts;
	(void)uart_rx_buffer;
	autorize_cdc_transfert = false;
	is_usb_connected       = false;
	udc_start();

	return 0;
}

bool uart_hal_is_connected(void)
{
	return is_usb_connected;
}

void uart_hal_dma_getc(uint8_t *data, uint32_t len)
{
	uint32_t data_read_idx = 0;
	while (len--)
		data[data_read_idx++] = udi_cdc_getc();
}

void uart_hal_dma_puts(const uint8_t *data, uint32_t len)
{
	if (!autorize_cdc_transfert)
		return;

	udi_cdc_write_buf(data, len);
	usb_data_sent = false;
	while (!usb_data_sent)
		;
}

void usb_cdc_cbk_suspend_action(void)
{
	is_usb_connected = false;
}

void usb_cdc_cbk_resume_action(void)
{
	is_usb_connected = true;
}

bool usb_cdc_cbk_enable(__attribute__((unused)) uint8_t port)
{
	autorize_cdc_transfert = true;
	return true;
}
void usb_cdc_cbk_disable(__attribute__((unused)) uint8_t port)
{
	autorize_cdc_transfert = false;
}

void usb_cdc_cbk_rx_notify(uint8_t port)
{
	(void)port;
	while (udi_cdc_is_rx_ready())
		uart_hal_rx_received_callback();
}

void usb_cdc_cbk_tx_empty_notify(uint8_t port)
{
	(void)port;
	uart_hal_tx_sent_callback();
	usb_data_sent = true;
}

void usb_cdc_cbk_set_config(uint8_t port, usb_cdc_line_coding_t *cfg)
{
	(void)port;
	(void)cfg;
}

void usb_cdc_cbk_set_dtr(uint8_t port, bool b_enable)
{
	(void)port;
	(void)b_enable;
}

void usb_cdc_cbk_set_rts(uint8_t port, bool b_enable)
{
	(void)port;
	(void)b_enable;
}