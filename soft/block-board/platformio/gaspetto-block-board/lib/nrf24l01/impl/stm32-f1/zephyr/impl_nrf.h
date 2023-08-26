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
	uint8_t result;
	if (HAL_SPI_TransmitReceive(&hspi1, &data, &result, 1, 2000) != HAL_OK)
	{
		Error_Handler();
	};
	return result;
}

static inline void Delay_ms(uint32_t ms) { HAL_Delay(ms); }

#else

#error LL or HAL support must be enabled

#endif /* USE_HAL_DRIVER */

#endif /*__NRF24L01_IMPL_ZEPHYR__ */
