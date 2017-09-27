/*
 * leds-att1272.c -  LED Driver
 *
 * Copyright (C) 2011 Rockchips
 * deng dalong <ddl@rock-chips.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Datasheet: http://www.rohm.com/products/databook/driver/pdf/att1272gu-e.pdf
 *
 */
#ifndef _LEDS_LEDS-SN3106B_H_
#define _LEDS_LEDS-SN3106B_H_

struct sn3106b_led_platform_data {
	const char		*name;
	int	en_gpio;
	int flen_gpio;
};

#endif /* _LEDS_LEDS-SN3106B_H_ */
