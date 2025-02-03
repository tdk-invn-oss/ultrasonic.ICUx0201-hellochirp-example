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
#include "ioport.h"
#include "pdc.h"
#include "flexcom.h"
#include "twi.h"

#include "conf_board.h"
#include "chirp_smartsonic.h"
#include "i2c_hal.h"

#define NB_I2C_BUS        (2)
#define I2C_TIMEOUT_COUNT (10000) /* loop counter to detect timeout in I2C handler */

#define WAIT_END_RX(twi) \
	do { \
		uint32_t timeout_count = 0; \
		while ((twi->TWI_SR & TWI_SR_RXRDY) == 0) { \
			if (++timeout_count >= I2C_TIMEOUT_COUNT) { \
				break; \
			} \
		} \
	} while (0)

/* Structure to track non-blocking I2C transaction data */
typedef struct {
	uint8_t *buf_ptr;   /* pointer to data buffer */
	uint16_t num_bytes; /* number of bytes to transfer */
} non_blocking_transaction_t;

struct i2c_handler {
	Twi *p_i2c;
	non_blocking_transaction_t non_block_transact;
};

struct i2c_handler i2c_bus_hn[NB_I2C_BUS] = {
		[I2C_BUS_IDX_CONTROL] =
				{
						.p_i2c = I2C_CONTROL,
						.non_block_transact =
								{
										.buf_ptr   = NULL,
										.num_bytes = 0,
								},
				},
		[I2C_BUS_IDX_CHx01] =
				{
						.p_i2c = I2C_DB,
						.non_block_transact =
								{
										.buf_ptr   = NULL,
										.num_bytes = 0,
								},
				},
};

static void i2c_non_blocking_transact_int_handler(uint8_t bus_index)
{
	uint8_t *buf_ptr;
	uint32_t num_bytes;
	uint32_t twi_status;
	Twi *twi_ptr;
	Pdc *pdc_ptr;

	twi_ptr = i2c_bus_hn[bus_index].p_i2c;

	twi_status  = twi_get_interrupt_status(twi_ptr);
	twi_status &= twi_get_interrupt_mask(twi_ptr);

	if (twi_status & TWI_SR_ENDRX) {
		pdc_ptr   = twi_get_pdc_base(twi_ptr);
		buf_ptr   = i2c_bus_hn[bus_index].non_block_transact.buf_ptr;
		num_bytes = i2c_bus_hn[bus_index].non_block_transact.num_bytes;

		/* Disable the RX PDC transfer requests */
		pdc_disable_transfer(pdc_ptr, PERIPH_PTCR_RXTDIS);
		/* Disable TWI interrupts */
		twi_disable_interrupt(twi_ptr, TWI_SR_ENDRX);

		/* Wait for next-to-last byte to be read */
		WAIT_END_RX(twi_ptr);
		/* Set stop command */
		twi_ptr->TWI_CR        = TWI_CR_STOP;
		buf_ptr[num_bytes - 2] = twi_ptr->TWI_RHR;

		/* Wait for last byte to be read */
		WAIT_END_RX(twi_ptr);
		buf_ptr[num_bytes - 1] = twi_ptr->TWI_RHR;

		/* Wait for transfer to complete */
		WAIT_END_RX(twi_ptr);
		i2c_bus_hn[bus_index].non_block_transact.num_bytes = 0;

		/* notify app that transaction is done */
		i2c_hal_read_nb_done_cbk(bus_index);
	}
}

int i2c_hal_init(uint8_t bus_index)
{
	twi_options_t opt = {0};

	if (I2C_BUS_IDX_CONTROL == bus_index) {
		pio_pull_down(I2C_CONTROL_PORT, I2C_CONTROL_PINS_MASK, 0);
		pio_pull_up(I2C_CONTROL_PORT, I2C_CONTROL_PINS_MASK, 0);

		ioport_set_pin_mode(I2C_CONTROL_SCL, I2C_CONTROL_PINS_FLAGS);
		ioport_disable_pin(I2C_CONTROL_SCL);
		ioport_set_pin_mode(I2C_CONTROL_SDA, I2C_CONTROL_PINS_FLAGS);
		ioport_disable_pin(I2C_CONTROL_SDA);

		flexcom_enable(I2C_CONTROL_FLEXCOM);
		flexcom_set_opmode(I2C_CONTROL_FLEXCOM, FLEXCOM_TWI);

		opt.master_clk = sysclk_get_peripheral_hz();
		opt.speed      = I2C_CONTROL_CLK;
		if (twi_master_init(I2C_CONTROL, &opt) != TWI_SUCCESS)
			return 1;

		NVIC_DisableIRQ(I2C_CONTROL_FLEXCOM_IRQ);
		NVIC_ClearPendingIRQ(I2C_CONTROL_FLEXCOM_IRQ);
		NVIC_SetPriority(I2C_CONTROL_FLEXCOM_IRQ, INT_PRIO_I2C_BUS);
		NVIC_EnableIRQ(I2C_CONTROL_FLEXCOM_IRQ);
	} else {
		pio_pull_down(I2C_DB_PORT, I2C_DB_PINS_MASK, 0);
		pio_pull_up(I2C_DB_PORT, I2C_DB_PINS_MASK, 0);

		ioport_set_pin_mode(I2C_DB_SCL, I2C_DB_PINS_FLAGS);
		ioport_disable_pin(I2C_DB_SCL);
		ioport_set_pin_mode(I2C_DB_SDA, I2C_DB_PINS_FLAGS);
		ioport_disable_pin(I2C_DB_SDA);

		flexcom_enable(I2C_DB_FLEXCOM);
		flexcom_set_opmode(I2C_DB_FLEXCOM, FLEXCOM_TWI);

		opt.master_clk = sysclk_get_peripheral_hz();
		opt.speed      = I2C_DB_CLK;
		if (twi_master_init(I2C_DB, &opt) != TWI_SUCCESS)
			return 1;

		NVIC_DisableIRQ(I2C_DB_FLEXCOM_IRQ);
		NVIC_ClearPendingIRQ(I2C_DB_FLEXCOM_IRQ);
		NVIC_SetPriority(I2C_DB_FLEXCOM_IRQ, INT_PRIO_I2C_BUS);
		NVIC_EnableIRQ(I2C_DB_FLEXCOM_IRQ);
	}

	return 0;
}

int i2c_hal_write(uint8_t bus_index, uint8_t i2c_addr, const uint8_t *data, uint16_t num_bytes)
{
	twi_packet_t packet_write;

	packet_write.chip        = i2c_addr;
	packet_write.addr_length = 0;
	packet_write.buffer      = (uint8_t *)data;
	packet_write.length      = num_bytes;

	return twi_master_write(i2c_bus_hn[bus_index].p_i2c, &packet_write);
}

int i2c_hal_read(uint8_t bus_index, uint8_t i2c_addr, uint8_t *data, uint16_t num_bytes)
{
	twi_packet_t packet_read = {
			.chip        = i2c_addr,
			.addr_length = 0,
			.buffer      = data,
			.length      = num_bytes,
	};

	return twi_master_read(i2c_bus_hn[bus_index].p_i2c, &packet_read);
}

int i2c_hal_read_nb(uint8_t bus_index, uint8_t i2c_addr, uint8_t *data, uint16_t num_bytes)
{
	Twi *twi_ptr;
	Pdc *pdc_ptr;
	int error = 0;

	twi_packet_t packet_read = {
			.chip        = i2c_addr,
			.addr_length = 0,
			/* buffer and length fields not used */
	};
	pdc_packet_t pdc_packet = {
			.ul_addr = (uint32_t)data,
			.ul_size = (num_bytes - 2),
	};

	if (bus_index >= NB_I2C_BUS || data == NULL)
		return 1;
	if (i2c_bus_hn[bus_index].non_block_transact.num_bytes > 0)
		/* Transaction already started */
		return 2;
	twi_ptr = i2c_bus_hn[bus_index].p_i2c;
	pdc_ptr = twi_get_pdc_base(twi_ptr);

	/* Save buffer pointer and transfer length - it will be needed when reading final 2 bytes
	 * in DMA interrupt handler
	 */
	i2c_bus_hn[bus_index].non_block_transact.buf_ptr   = data;
	i2c_bus_hn[bus_index].non_block_transact.num_bytes = num_bytes;

	pdc_disable_transfer(pdc_ptr, PERIPH_PTCR_TXTDIS | PERIPH_PTCR_RXTDIS);
	pdc_rx_init(pdc_ptr, &pdc_packet, NULL);

	/* Set read mode, slave address, and internal address length */
	twi_ptr->TWI_MMR = 0;
	twi_ptr->TWI_MMR = TWI_MMR_MREAD | TWI_MMR_DADR(packet_read.chip) |
	                   ((packet_read.addr_length << TWI_MMR_IADRSZ_Pos) & TWI_MMR_IADRSZ_Msk);

	/* No internal memory (register) address within remote device */
	twi_ptr->TWI_IADR = 0;
	/* Enable the RX PDC transfer requests */
	pdc_enable_transfer(pdc_ptr, PERIPH_PTCR_RXTEN);
	/* Start the transfer */
	twi_ptr->TWI_CR = TWI_CR_START;
	/* Enable end-of-receive interrupt */
	twi_enable_interrupt(twi_ptr, TWI_IER_ENDRX);

	return error;
}

int i2c_hal_write_register(uint8_t bus_index, uint8_t i2c_addr, uint8_t register_addr, const uint8_t *data,
                           uint16_t num_bytes)
{
	twi_packet_t packet_write;

	packet_write.chip        = i2c_addr;
	packet_write.addr[0]     = register_addr;
	packet_write.addr_length = 1;
	packet_write.buffer      = (uint8_t *)data;
	packet_write.length      = num_bytes;

	return twi_master_write(i2c_bus_hn[bus_index].p_i2c, &packet_write);
}

int i2c_hal_read_register(uint8_t bus_index, uint8_t i2c_addr, uint8_t register_addr, uint8_t *data, uint16_t num_bytes)
{
	twi_packet_t packet_read;

	packet_read.chip        = i2c_addr;
	packet_read.addr[0]     = register_addr;
	packet_read.addr_length = 1;
	packet_read.buffer      = data;
	packet_read.length      = num_bytes;

	if (twi_master_read(i2c_bus_hn[bus_index].p_i2c, &packet_read) == TWI_SUCCESS)
		return TWI_SUCCESS;

	return TWI_BUSY;
}

int i2c_hal_read_register_nb(uint8_t bus_index, uint8_t i2c_addr, uint8_t register_addr, uint8_t *data,
                             uint16_t num_bytes)
{
	Twi *twi_ptr;
	Pdc *pdc_ptr;

	twi_packet_t packet_read = {
			.chip        = i2c_addr,
			.addr[0]     = register_addr, /* internal mem address */
			.addr_length = 1,
			/* buffer and length fields not used */
	};
	pdc_packet_t pdc_packet = {
			.ul_addr = (uint32_t)data,
			.ul_size = (num_bytes - 2),
	};

	if (bus_index >= NB_I2C_BUS || data == NULL)
		return 1;
	if (i2c_bus_hn[bus_index].non_block_transact.num_bytes > 0)
		/* Transaction already started */
		return 2;

	twi_ptr = i2c_bus_hn[bus_index].p_i2c;
	pdc_ptr = twi_get_pdc_base(twi_ptr);

	/* Save buffer pointer and transfer length - it will be needed when reading final 2 bytes
	 * in DMA interrupt handler
	 */
	i2c_bus_hn[bus_index].non_block_transact.buf_ptr   = data;
	i2c_bus_hn[bus_index].non_block_transact.num_bytes = num_bytes;

	pdc_disable_transfer(pdc_ptr, PERIPH_PTCR_TXTDIS | PERIPH_PTCR_RXTDIS);
	pdc_rx_init(pdc_ptr, &pdc_packet, NULL);

	/* Set read mode, slave address, and internal address length */
	twi_ptr->TWI_MMR = 0;
	twi_ptr->TWI_MMR = TWI_MMR_MREAD | TWI_MMR_DADR(packet_read.chip) |
	                   ((packet_read.addr_length << TWI_MMR_IADRSZ_Pos) & TWI_MMR_IADRSZ_Msk);
	/* Set internal memory (register) address within remote device */
	twi_ptr->TWI_IADR = 0;
	twi_ptr->TWI_IADR = twi_mk_addr(packet_read.addr, packet_read.addr_length);
	/* Enable the RX PDC transfer requests */
	pdc_enable_transfer(pdc_ptr, PERIPH_PTCR_RXTEN);
	/* Start the transfer */
	twi_ptr->TWI_CR = TWI_CR_START;
	/* Enable end-of-receive interrupt */
	twi_enable_interrupt(twi_ptr, TWI_IER_ENDRX);

	return 0;
}

int i2c_hal_write_wregister(uint8_t bus_index, uint8_t i2c_addr, uint16_t register_addr, const uint8_t *data,
                            uint16_t num_bytes)
{
	twi_packet_t packet_write;

	packet_write.chip        = i2c_addr;
	packet_write.addr[0]     = register_addr & 0x00FF;
	packet_write.addr[1]     = (register_addr & 0xFF00) >> 8;
	packet_write.addr_length = 2;
	packet_write.buffer      = (uint8_t *)data;
	packet_write.length      = num_bytes;

	return twi_master_write(i2c_bus_hn[bus_index].p_i2c, &packet_write);
}

int i2c_hal_read_wregister(uint8_t bus_index, uint8_t i2c_addr, uint16_t register_addr, uint8_t *data,
                           uint16_t num_bytes)
{
	twi_packet_t packet_read;

	packet_read.chip        = i2c_addr;
	packet_read.addr[0]     = register_addr & 0x00FF;
	packet_read.addr[1]     = (register_addr & 0xFF00) >> 8;
	packet_read.addr_length = 2;
	packet_read.buffer      = data;
	packet_read.length      = num_bytes;

	if (twi_master_read(i2c_bus_hn[bus_index].p_i2c, &packet_read) == TWI_SUCCESS)
		return TWI_SUCCESS;

	return TWI_BUSY;
}

void i2c_hal_interrupt_handler(uint8_t bus_index)
{
	if (bus_index >= NB_I2C_BUS)
		return;

	if (i2c_bus_hn[bus_index].non_block_transact.num_bytes > 0)
		i2c_non_blocking_transact_int_handler(bus_index);
}
