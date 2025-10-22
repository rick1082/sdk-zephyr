/*
 * Copyright (c) 2018-2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <soc.h>
#include <zephyr/device.h>

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>

#include "hal/debug.h"

/* Clock setup timeouts are unlikely, below values are experimental */
#define LFCLOCK_TIMEOUT_MS 500
#define HFCLOCK_TIMEOUT_MS 2

static uint16_t const sca_ppm_lut[] = {500, 250, 150, 100, 75, 50, 30, 20};

int lll_clock_init(void)
{
	return 0;
}

int lll_clock_deinit(void)
{
	return 0;
}

int lll_clock_wait(void)
{
	return 0;
}

int lll_hfclock_on(void)
{
	return 0;
}

int lll_hfclock_on_wait(void)
{
	return 0;
}

int lll_hfclock_off(void)
{
	return 0;
}

uint8_t lll_clock_sca_local_get(void)
{
	return 0;
}

uint32_t lll_clock_ppm_local_get(void)
{
	return sca_ppm_lut[0];
}

uint32_t lll_clock_ppm_get(uint8_t sca)
{
	return sca_ppm_lut[sca];
}
