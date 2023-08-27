//
// Created by ilia.motornyi on 13-Dec-18.
//

#ifndef __NRF24L01_IMPL_ZEPHYR__
#define __NRF24L01_IMPL_ZEPHYR__

#include <nrf24.h>

/* External NRF24L01 device struct. */
extern struct nrf24L01_device _local_nrf24l01;

/* Struct initialization function. */
int nrf24l01_init_device(struct nrf24L01_device *nrf24l01);

#ifdef USE_HAL_DRIVER
// extern SPI_HandleTypeDef hspi1;

static inline void nRF24_CE_L()
{
	gpio_pin_set(_local_nrf24l01.dev_gpio_ce, _local_nrf24l01.flags_ce, 0);
}

static inline void nRF24_CE_H()
{
	gpio_pin_set(_local_nrf24l01.dev_gpio_ce, _local_nrf24l01.flags_ce, 1);
}

static inline void nRF24_CSN_L()
{
	gpio_pin_set(_local_nrf24l01.dev_gpio_csn, _local_nrf24l01.flags_csn, 1);
}

static inline void nRF24_CSN_H()
{
	gpio_pin_set(_local_nrf24l01.dev_gpio_csn, _local_nrf24l01.flags_csn, 0);
}

static inline uint8_t nRF24_LL_RW(uint8_t data)
{
	// Wait until TX buffer is empty
	uint8_t result_buf;
	uint8_t *result_ptr;
	struct spi_buf tx_buf, rx_buf;

	tx_buf.buf = &data;
	tx_buf.len = sizeof(data);

	rx_buf.buf = &result_buf;
	rx_buf.len = sizeof(result_buf);

	struct spi_buf_set tx_buf_set = {.buffers = &tx_buf, .count = 1};
	struct spi_buf_set rx_buf_set = {.buffers = &rx_buf, .count = 1};

	if(spi_transceive(_local_nrf24l01.bus_nrf->bus, _local_nrf24l01.bus_nrf->config, &tx_buf_set, &rx_buf_set)) {
		LOG_WRN("SPI TX failed.")
		return 1
	}

	result_ptr = (uint8_t *)rx_buf_set.buffers->buf;
	return result_ptr[0];
}

static inline void Delay_ms(uint32_t ms) { HAL_Delay(ms); }

#else

#error LL or HAL support must be enabled

#endif /* USE_HAL_DRIVER */

#endif /*__NRF24L01_IMPL_ZEPHYR__ */
