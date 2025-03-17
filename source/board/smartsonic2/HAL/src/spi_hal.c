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
#include "ioport.h"
#include "pdc.h"
#include "flexcom.h"
#include "spi.h"
#include "matrix.h"

#include "conf_board.h"
#include "chirp_smartsonic.h"
#include "spi_hal.h"

#define SPI_TX_BUFFER_SIZE 2720

/* Rx buffer size = tx max transfer size */
#if defined(INCLUDE_SHASTA_SUPPORT)
#include <invn/soniclib/details/icu.h>
#define SPI_RX_BUFFER_SIZE ICU_FW_SIZE
#else
#define SPI_RX_BUFFER_SIZE SPI_TX_BUFFER_SIZE
#endif

/* SPI chip select asserted level - active low */
#define CS_ASSERTED_LEVEL   IOPORT_PIN_LEVEL_LOW
#define CS_DEASSERTED_LEVEL IOPORT_PIN_LEVEL_HIGH

#define READ_BIT_MASK  0x80
#define WRITE_BIT_MASK 0x7F

#define NB_SPI_BUS   (2)
#define NB_MAX_SLAVE (4)

struct cs_pins {
	uint32_t pin[NB_MAX_SLAVE];
};

/* Structure to track non-blocking transaction data */
typedef struct {
	uint8_t *buf_ptr;   /* pointer to data buffer */
	uint16_t num_bytes; /* number of bytes to transfer */
} non_blocking_transaction_t;

struct spi_handler {
	Pdc *p_dma;
	Spi *p_spi;
	volatile uint32_t cs_pin_locking; /* track cs pin using the bus */
	IRQn_Type irq_num;
	uint8_t tx_buffer[SPI_TX_BUFFER_SIZE];
	uint8_t rx_buffer[SPI_RX_BUFFER_SIZE];
	non_blocking_transaction_t non_block_transact;
	volatile bool in_use; /* track if bus is being used */
};

struct spi_handler spi_bus_hn[NB_SPI_BUS] = {
		[SPI_BUS_IDX_ICU] =
				{
						.p_dma     = NULL,
						.p_spi     = SPI_ICU,
						.irq_num   = SPI_ICU_FLEXCOM_IRQ,
						.tx_buffer = {0},
						.rx_buffer = {0},
						.non_block_transact =
								{
										.buf_ptr   = NULL,
										.num_bytes = 0,
								},
						.in_use         = false,
						.cs_pin_locking = 0,
				},
		[SPI_BUS_IDX_EVB] =
				{
						.p_dma     = NULL,
						.p_spi     = SPI_EVB,
						.irq_num   = SPI_EVB_FLEXCOM_IRQ,
						.tx_buffer = {0},
						.rx_buffer = {0},
						.non_block_transact =
								{
										.buf_ptr   = NULL,
										.num_bytes = 0,
								},
						.in_use         = false,
						.cs_pin_locking = 0,
				},
};

/* Indexes of Slaves CS pins per SPI bus */
static const struct cs_pins spi_bus_cs[NB_SPI_BUS] = {
		[SPI_BUS_IDX_ICU] = {.pin = {SPI_ICU_CSB0, SPI_ICU_CSB1, SPI_ICU_CSB2, SPI_ICU_CSB3}},
		[SPI_BUS_IDX_EVB] =
				{
						.pin =
								{
										SPI_EVB_CSB,
								},
				},
};

static uint8_t find_bus(uint32_t cs_pin)
{
	uint8_t bus_found_index = NB_SPI_BUS;

	for (uint8_t bus_index = SPI_BUS_IDX_ICU; bus_index < NB_SPI_BUS; bus_index++) {
		for (uint8_t cs_pin_index = 0; cs_pin_index < NB_MAX_SLAVE; cs_pin_index++) {
			if (cs_pin == spi_bus_cs[bus_index].pin[cs_pin_index]) {
				bus_found_index = bus_index;
				break;
			}
		}
		if (bus_found_index != NB_SPI_BUS)
			break;
	}
	return bus_found_index;
}

static void lock_bus(uint32_t cs_pin)
{
	uint8_t bus_found_index = find_bus(cs_pin);

	if (bus_found_index != NB_SPI_BUS) {
		while (spi_bus_hn[bus_found_index].in_use) {
			/* wait here until bus not busy */
		};
		cpu_irq_enter_critical();
		spi_bus_hn[bus_found_index].in_use         = true;
		spi_bus_hn[bus_found_index].cs_pin_locking = cs_pin;
		cpu_irq_leave_critical();
	}
}

static void unlock_bus(uint32_t cs_pin)
{
	uint8_t bus_found_index = find_bus(cs_pin);

	if (bus_found_index != NB_SPI_BUS) {
		if (!(spi_bus_hn[bus_found_index].in_use && spi_bus_hn[bus_found_index].cs_pin_locking == cs_pin)) {
			printf("error unlocking bus %u by %lu\r\n", bus_found_index, cs_pin);
		} else {
			cpu_irq_enter_critical();
			spi_bus_hn[bus_found_index].cs_pin_locking = 0;
			spi_bus_hn[bus_found_index].in_use         = false;
			cpu_irq_leave_critical();
		}
	}
}

static inline void non_blocking_transact_int_handler(uint8_t bus_index)
{
	spi_bus_hn[bus_index].non_block_transact.num_bytes = 0;

	/* notify app that transaction is done */
	spi_hal_read_nb_done_cbk(bus_index);
}

void spi_hal_init(uint8_t bus_index)
{
	if (SPI_BUS_IDX_ICU == bus_index) {
		/* SPI Sensors ICU */
		ioport_set_pin_mode(SPI_ICU_MOSI, SPI_ICU_PINS_FLAGS);
		ioport_set_pin_mode(SPI_ICU_MISO, SPI_ICU_PINS_FLAGS);
		ioport_set_pin_mode(SPI_ICU_SCLK, SPI_ICU_PINS_FLAGS);
		ioport_disable_pin(SPI_ICU_MOSI);
		ioport_disable_pin(SPI_ICU_MISO);
		ioport_disable_pin(SPI_ICU_SCLK);

		/* configure system pin as PIO */
		uint32_t sys_io  = matrix_get_system_io();
		sys_io          |= SPI_ICU_CSB0_SYSIO_MASK;
		matrix_set_system_io(sys_io);

		ioport_set_pin_dir(SPI_ICU_CSB0, IOPORT_DIR_OUTPUT);
		ioport_set_pin_dir(SPI_ICU_CSB1, IOPORT_DIR_OUTPUT);
		ioport_set_pin_dir(SPI_ICU_CSB2, IOPORT_DIR_OUTPUT);
		ioport_set_pin_dir(SPI_ICU_CSB3, IOPORT_DIR_OUTPUT);
		ioport_set_pin_level(SPI_ICU_CSB0, IOPORT_PIN_LEVEL_HIGH);
		ioport_set_pin_level(SPI_ICU_CSB1, IOPORT_PIN_LEVEL_HIGH);
		ioport_set_pin_level(SPI_ICU_CSB2, IOPORT_PIN_LEVEL_HIGH);
		ioport_set_pin_level(SPI_ICU_CSB3, IOPORT_PIN_LEVEL_HIGH);

		/* Get pointer to SPI master PDC register base */
		spi_bus_hn[SPI_BUS_IDX_ICU].p_dma = spi_get_pdc_base(SPI_ICU);

		/* Enable the peripheral and set SPI mode. */
		flexcom_enable(SPI_ICU_FLEXCOM);
		flexcom_set_opmode(SPI_ICU_FLEXCOM, FLEXCOM_SPI);

		/* Configure SPI device */
		spi_disable(SPI_ICU);
		spi_reset(SPI_ICU);
		spi_set_master_mode(SPI_ICU);
		spi_disable_mode_fault_detect(SPI_ICU);
		/* CPOL = 1 (SCLK inactive high) CPHA = 1 (data valid on clock trailing edge) / NCHPA = 0 */
		spi_set_clock_polarity(SPI_ICU, 0, SPI_ICU_CFG_CPOL);
		spi_set_clock_phase(SPI_ICU, 0, SPI_ICU_CFG_CPHA);
		spi_set_bits_per_transfer(SPI_ICU, 0, SPI_ICU_CFG_BITS_PER_TRANSFER);
		spi_set_baudrate_div(SPI_ICU, 0, (sysclk_get_peripheral_hz() / SPI_ICU_CFG_SPEED));
		spi_set_transfer_delay(SPI_ICU, 0, SPI_ICU_CFG_DELAY_BS, SPI_ICU_CFG_DELAY_BCT);
		spi_enable(SPI_ICU);
		pdc_disable_transfer(spi_bus_hn[SPI_BUS_IDX_ICU].p_dma, PERIPH_PTCR_RXTDIS | PERIPH_PTCR_TXTDIS);

		NVIC_ClearPendingIRQ(SPI_ICU_FLEXCOM_IRQ);
		NVIC_SetPriority(SPI_ICU_FLEXCOM_IRQ, INT_PRIO_SPI_BUS);
		NVIC_EnableIRQ(SPI_ICU_FLEXCOM_IRQ);
	} else {
		/* SPI EVB */
		ioport_set_pin_mode(SPI_EVB_MOSI, SPI_EVB_PINS_FLAGS);
		ioport_set_pin_mode(SPI_EVB_MISO, SPI_EVB_PINS_FLAGS);
		ioport_set_pin_mode(SPI_EVB_SCLK, SPI_EVB_PINS_FLAGS);
		ioport_disable_pin(SPI_EVB_MOSI);
		ioport_disable_pin(SPI_EVB_MISO);
		ioport_disable_pin(SPI_EVB_SCLK);

		ioport_set_pin_dir(SPI_EVB_CSB, IOPORT_DIR_OUTPUT);
		ioport_set_pin_level(SPI_EVB_CSB, IOPORT_PIN_LEVEL_HIGH);

		/* Get pointer to SPI master PDC register base */
		spi_bus_hn[SPI_BUS_IDX_EVB].p_dma = spi_get_pdc_base(SPI_EVB);

		/* Enable the peripheral and set SPI mode. */
		flexcom_enable(SPI_EVB_FLEXCOM);
		flexcom_set_opmode(SPI_EVB_FLEXCOM, FLEXCOM_SPI);

		/* Configure SPI device */
		spi_disable(SPI_EVB);
		spi_reset(SPI_EVB);
		spi_set_master_mode(SPI_EVB);
		spi_disable_mode_fault_detect(SPI_EVB);
		/* CPOL = 1 (SCLK inactive high) CPHA = 1 (data valid on clock trailing edge) / NCHPA = 0 */
		spi_set_clock_polarity(SPI_EVB, 0, SPI_EVB_CFG_CPOL);
		spi_set_clock_phase(SPI_EVB, 0, SPI_EVB_CFG_CPHA);
		spi_set_bits_per_transfer(SPI_EVB, 0, SPI_EVB_CFG_BITS_PER_TRANSFER);
		spi_set_baudrate_div(SPI_EVB, 0, (sysclk_get_peripheral_hz() / SPI_EVB_CFG_SPEED));
		spi_set_transfer_delay(SPI_EVB, 0, SPI_EVB_CFG_DELAY_BS, SPI_EVB_CFG_DELAY_BCT);
		spi_enable(SPI_EVB);
		pdc_disable_transfer(spi_bus_hn[SPI_BUS_IDX_EVB].p_dma, PERIPH_PTCR_RXTDIS | PERIPH_PTCR_TXTDIS);
	}
}

void spi_hal_cs_on(uint32_t cs_pin)
{
	lock_bus(cs_pin);
	ioport_set_pin_level(cs_pin, CS_ASSERTED_LEVEL);
}

void spi_hal_cs_off(uint32_t cs_pin)
{
	ioport_set_pin_level(cs_pin, CS_DEASSERTED_LEVEL);
	unlock_bus(cs_pin);
}

int spi_hal_write(uint8_t bus_index, const uint8_t *data, uint16_t num_bytes)
{
	pdc_packet_t dma_packet;

	/* check nb sent bytes because we will receive the same number of bytes */
	if (num_bytes > SPI_RX_BUFFER_SIZE)
		return -1;
	if (bus_index >= NB_SPI_BUS)
		return -2;

	struct spi_handler *bus_hn = &spi_bus_hn[bus_index];

	/* Desactivate Irq during buffer write*/
	cpu_irq_enter_critical();

	dma_packet.ul_addr = (uint32_t)&bus_hn->rx_buffer[0];
	dma_packet.ul_size = num_bytes;
	pdc_rx_init(bus_hn->p_dma, &dma_packet, NULL);

	dma_packet.ul_addr = (uint32_t)&data[0];
	dma_packet.ul_size = num_bytes;
	pdc_tx_init(bus_hn->p_dma, &dma_packet, NULL);

	/* Enable the RX and TX PDC transfer requests */
	pdc_enable_transfer(bus_hn->p_dma, PERIPH_PTCR_RXTEN | PERIPH_PTCR_TXTEN);

	/* Re activate Irq */
	cpu_irq_leave_critical();

	/* Waiting transfer done*/
	while ((spi_read_status(bus_hn->p_spi) & SPI_SR_TXEMPTY) == 0)
		;

	/* Disable the RX and TX PDC transfer requests */
	pdc_disable_transfer(bus_hn->p_dma, PERIPH_PTCR_RXTDIS | PERIPH_PTCR_TXTDIS);

	return 0;
}

int spi_hal_read(uint8_t bus_index, uint8_t *data, uint16_t num_bytes)
{
	pdc_packet_t pdc_spi_packet;

	if (num_bytes > SPI_RX_BUFFER_SIZE || num_bytes > SPI_TX_BUFFER_SIZE)
		return -1;
	if (bus_index >= NB_SPI_BUS)
		return -2;

	struct spi_handler *bus_hn = &spi_bus_hn[bus_index];

	/* Desactivate Irq during buffer write*/
	cpu_irq_enter_critical();

	pdc_spi_packet.ul_addr = (uint32_t)&data[0];
	pdc_spi_packet.ul_size = num_bytes;
	pdc_rx_init(bus_hn->p_dma, &pdc_spi_packet, NULL);

	memset(&bus_hn->tx_buffer[0], 0x00, sizeof(uint8_t) * num_bytes);
	pdc_spi_packet.ul_addr = (uint32_t)&bus_hn->tx_buffer[0];
	pdc_spi_packet.ul_size = num_bytes;
	pdc_tx_init(bus_hn->p_dma, &pdc_spi_packet, NULL);

	/* Re activate Irq */
	cpu_irq_leave_critical();

	/* Enable the RX and TX PDC transfer requests */
	pdc_enable_transfer(bus_hn->p_dma, PERIPH_PTCR_RXTEN | PERIPH_PTCR_TXTEN);
	/* Waiting transfer done*/
	while ((spi_read_status(bus_hn->p_spi) & SPI_SR_ENDRX) == 0)
		;
	/* Disable the RX and TX PDC transfer requests */
	pdc_disable_transfer(bus_hn->p_dma, PERIPH_PTCR_RXTDIS | PERIPH_PTCR_TXTDIS);

	return 0;
}

int spi_hal_read_nb(uint8_t bus_index, uint8_t *data, uint16_t num_bytes)
{
	pdc_packet_t pdc_spi_packet;
	struct spi_handler *bus_hn = &spi_bus_hn[bus_index];

	if (num_bytes > SPI_TX_BUFFER_SIZE)
		return -1;
	if (bus_index >= NB_SPI_BUS)
		return -2;

	/* Desactivate Irq during buffer write*/
	cpu_irq_enter_critical();

	pdc_spi_packet.ul_addr = (uint32_t)&data[0];
	pdc_spi_packet.ul_size = num_bytes;
	pdc_rx_init(bus_hn->p_dma, &pdc_spi_packet, NULL);

	memset(&bus_hn->tx_buffer[0], 0x00, sizeof(uint8_t) * num_bytes);
	pdc_spi_packet.ul_addr = (uint32_t)&bus_hn->tx_buffer[0];
	pdc_spi_packet.ul_size = num_bytes;
	pdc_tx_init(bus_hn->p_dma, &pdc_spi_packet, NULL);

	/* Re activate Irq */
	cpu_irq_leave_critical();

	bus_hn->non_block_transact.buf_ptr   = data;
	bus_hn->non_block_transact.num_bytes = num_bytes;

	spi_enable_interrupt(bus_hn->p_spi, SPI_IDR_RXBUFF);
	/* Enable the RX and TX PDC transfer requests */
	pdc_enable_transfer(bus_hn->p_dma, PERIPH_PTCR_RXTEN | PERIPH_PTCR_TXTEN);

	return 0;
}

int spi_hal_write_register(uint8_t bus_index, uint16_t register_addr, const uint8_t *data, uint16_t num_bytes)
{
	pdc_packet_t dma_packet;
	struct spi_handler *bus_hn = &spi_bus_hn[bus_index];

	if ((num_bytes + 1) > SPI_TX_BUFFER_SIZE)
		return -1;
	if (bus_index >= NB_SPI_BUS)
		return -2;

	/* Desactivate Irq during buffer write*/
	cpu_irq_enter_critical();

	dma_packet.ul_addr = (uint32_t)&bus_hn->rx_buffer[0];
	dma_packet.ul_size = num_bytes + 1;
	pdc_rx_init(bus_hn->p_dma, &dma_packet, NULL);

	bus_hn->tx_buffer[0] = (uint8_t)(register_addr & WRITE_BIT_MASK);
	memcpy(&bus_hn->tx_buffer[1], (uint8_t *)data, num_bytes);

	dma_packet.ul_addr = (uint32_t)&bus_hn->tx_buffer[0];
	dma_packet.ul_size = num_bytes + 1;
	pdc_tx_init(bus_hn->p_dma, &dma_packet, NULL);

	/* Enable the RX and TX PDC transfer requests */
	pdc_enable_transfer(bus_hn->p_dma, PERIPH_PTCR_RXTEN | PERIPH_PTCR_TXTEN);

	/* Re activate Irq */
	cpu_irq_leave_critical();

	/* Waiting transfer done*/
	while ((spi_read_status(bus_hn->p_spi) & SPI_SR_TXEMPTY) == 0)
		;

	/* Disable the RX and TX PDC transfer requests */
	pdc_disable_transfer(bus_hn->p_dma, PERIPH_PTCR_RXTDIS | PERIPH_PTCR_TXTDIS);

	return 0;
}

int spi_hal_read_register(uint8_t bus_index, uint16_t register_addr, uint8_t *data, uint16_t num_bytes)
{
	pdc_packet_t dma_packet;
	struct spi_handler *bus_hn = &spi_bus_hn[bus_index];

	if ((num_bytes + 1) > SPI_TX_BUFFER_SIZE)
		return -1;
	if (bus_index >= NB_SPI_BUS)
		return -2;

	/* Desactivate Irq during buffer write*/
	cpu_irq_enter_critical();

	dma_packet.ul_addr = (uint32_t)&bus_hn->rx_buffer[0];
	dma_packet.ul_size = num_bytes + 1;
	pdc_rx_init(bus_hn->p_dma, &dma_packet, NULL);

	bus_hn->tx_buffer[0] = (uint8_t)(register_addr | READ_BIT_MASK);
	memset(&bus_hn->tx_buffer[1], 0x00, num_bytes);

	dma_packet.ul_addr = (uint32_t)&bus_hn->tx_buffer[0];
	dma_packet.ul_size = num_bytes + 1;
	pdc_tx_init(bus_hn->p_dma, &dma_packet, NULL);

	/* Enable the RX and TX PDC transfer requests */
	pdc_enable_transfer(bus_hn->p_dma, PERIPH_PTCR_RXTEN | PERIPH_PTCR_TXTEN);

	/* Re activate Irq */
	cpu_irq_leave_critical();

	/* Waiting transfer done*/
	while ((spi_read_status(bus_hn->p_spi) & SPI_SR_ENDRX) == 0)
		;

	/* Disable the RX and TX PDC transfer requests */
	pdc_disable_transfer(bus_hn->p_dma, PERIPH_PTCR_RXTDIS | PERIPH_PTCR_TXTDIS);

	memcpy(data, &bus_hn->rx_buffer[1], num_bytes);

	return 0;
}

void spi_hal_interrupt_handler(uint8_t bus_index)
{
	if (bus_index >= NB_SPI_BUS)
		return;

	NVIC_ClearPendingIRQ(spi_bus_hn[bus_index].irq_num);

	if (spi_bus_hn[bus_index].p_spi->SPI_SR & SPI_SR_RXBUFF) {
		/* Disable the RX and TX PDC transfer requests */
		pdc_disable_transfer(spi_bus_hn[bus_index].p_dma, PERIPH_PTCR_RXTDIS | PERIPH_PTCR_TXTDIS);
		/* Disable SPI IRQ */
		spi_disable_interrupt(spi_bus_hn[bus_index].p_spi, SPI_IDR_RXBUFF);

		if (spi_bus_hn[bus_index].non_block_transact.num_bytes > 0) {
			non_blocking_transact_int_handler(bus_index);
		}
	}
}