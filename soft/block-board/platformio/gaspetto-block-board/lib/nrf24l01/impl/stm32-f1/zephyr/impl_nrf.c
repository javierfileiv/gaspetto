#include <nrf24.h>

struct nrf24L01_device _local_nrf24l01;
int nrf24l01_init_device(struct nrf24L01_device *nrf24l01) {

	if (!nrf24l01)
		return -1;

	if(nrf24l01->dev_gpio_ce)
	_local_nrf24l01 = *nrf24l01;


	int ret = gpio_pin_configure(_local_nrf24l01.dev_gpio_ce, _local_nrf24l01.pin_ce, _local_nrf24l01.flags_ce);
	if (ret < 0) {
		return;
	}

	return 0;
}