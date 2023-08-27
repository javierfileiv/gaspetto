#include <nrf24.h>
#include "impl_nrf.h"

int nRF24_Init_Device(const struct nrf24L01_device *nrf24l01)
{
	if (!nrf24l01)
	{
		LOG_ERR("No device struct as input");
		return -1;
	}

	_local_nrf24l01 = nrf24l01;

	if (!_local_nrf24l01->dev_gpio_ce && _local_nrf24l01->dev_gpio_csn)
	{
		LOG_ERR("No CE nor CSN gpio defined");
		return -1;
	}

	if (!_local_nrf24l01->bus_nrf)
	{
		LOG_ERR("No SPI bus defined");
		return -1;
	}

	if (!spi_is_ready(_local_nrf24l01->bus_nrf))
	{
		LOG_ERR("SPI bus not ready");
		return -1;
	}

	int ret = gpio_pin_configure(_local_nrf24l01->dev_gpio_ce, _local_nrf24l01->pin_ce, _local_nrf24l01->flags_ce);
	if (ret < 0)
	{
		return -1;
	}

	return 0;
}