/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/gpio.h>
#include <nrfx_i2s.h>
#include <nrfx_clock.h>
#include <stdint.h>
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

#define I2S_NL DT_NODELABEL(i2s20)
PINCTRL_DT_DEFINE(I2S_NL);
static nrfx_i2s_t i2s_inst = NRFX_I2S_INSTANCE(20);
static nrfx_i2s_config_t cfg = {
	/* Pins are configured by pinctrl. */
	.skip_gpio_cfg = true,
	.skip_psel_cfg = true,
	.irq_priority = DT_IRQ(I2S_NL, priority),
	.mode = NRF_I2S_MODE_MASTER,
	.format = NRF_I2S_FORMAT_I2S,
	.alignment = NRF_I2S_ALIGN_LEFT,
	.ratio = NRF_I2S_RATIO_64X,
	.sample_width = NRF_I2S_SWIDTH_16BIT,
	.channels = NRF_I2S_CHANNELS_STEREO,
	.mck_setup = NRF_I2S_MCK_32MDIV10,
};

#include <math.h>
#define I2S_SAMPLES_NUM 48
static uint16_t i2s_tx_buf_a[I2S_SAMPLES_NUM * 2]; // 2 channels, 16 bits each
static uint16_t i2s_tx_buf_b[I2S_SAMPLES_NUM * 2]; // 2 channels, 16 bits each
static uint16_t i2s_rx_buf_a[I2S_SAMPLES_NUM * 2]; // 2 channels, 16 bits each
static uint16_t i2s_rx_buf_b[I2S_SAMPLES_NUM * 2]; // 2 channels, 16 bits each


void audio_i2s_set_next_buf(const uint8_t *tx_buf, uint32_t *rx_buf)
{
	const nrfx_i2s_buffers_t i2s_buf = {.p_rx_buffer = rx_buf,
					    .p_tx_buffer = (uint32_t *)tx_buf,
					    .buffer_size = I2S_SAMPLES_NUM};

	nrfx_err_t ret;

	ret = nrfx_i2s_next_buffers_set(&i2s_inst, &i2s_buf);
	if (ret != NRFX_SUCCESS) {
        printf("Failed to set next buffers: %x\n", ret);
    }   
}

static void i2s_comp_handler(nrfx_i2s_buffers_t const *released_bufs, uint32_t status)
{
	if (status == NRFX_I2S_STATUS_NEXT_BUFFERS_NEEDED) {
        gpio_pin_toggle_dt(&led);
        if ((uint16_t *)released_bufs->p_tx_buffer == i2s_tx_buf_a) {
            //printf("TX buffer A released\n");
            //generate_sine_wave_segment(i2s_tx_buf_a, I2S_SAMPLES_NUM*2, 1000, 48000);
            audio_i2s_set_next_buf((const uint8_t *)i2s_tx_buf_a, (uint32_t *)i2s_rx_buf_a);
        } else if ((uint16_t *)released_bufs->p_tx_buffer == i2s_tx_buf_b) {
            //printf("TX buffer B released\n");
            //generate_sine_wave_segment(i2s_tx_buf_b, I2S_SAMPLES_NUM*2, 1000, 48000);
            audio_i2s_set_next_buf((const uint8_t *)i2s_tx_buf_b, (uint32_t *)i2s_rx_buf_b);
        }
	}
}


void audio_i2s_start(const uint8_t *tx_buf, uint32_t *rx_buf)
{
	const nrfx_i2s_buffers_t i2s_buf = {.p_rx_buffer = rx_buf,
					    .p_tx_buffer = (uint32_t *)tx_buf,
					    .buffer_size = I2S_SAMPLES_NUM};

	int ret;

	/* Buffer size in 32-bit words */
	ret = nrfx_i2s_start(&i2s_inst, &i2s_buf, 0);
    if (ret != NRFX_SUCCESS) {
        printf("Failed to start I2S: %d\n", ret);
    }
}

void audio_i2s_init(void)
{
    int ret;

	ret = pinctrl_apply_state(PINCTRL_DT_DEV_CONFIG_GET(I2S_NL), PINCTRL_STATE_DEFAULT);
    if (ret != 0) {
        printf("Failed to apply pinctrl state: %d\n", ret);
        return;
    }

	IRQ_CONNECT(DT_IRQN(I2S_NL), DT_IRQ(I2S_NL, priority), nrfx_isr, nrfx_i2s_20_irq_handler, 0);
	irq_enable(DT_IRQN(I2S_NL));

	ret = nrfx_i2s_init(&i2s_inst, &cfg, i2s_comp_handler);
    if (ret != NRFX_SUCCESS) {
        printf("Failed to initialize I2S: %x\n", ret);
        return;
    }
}

#include "nrf54l15.h"
int main(void)
{   
	printf("Hello World! %s\n", CONFIG_BOARD_TARGET);
    gpio_pin_configure_dt(&led, GPIO_OUTPUT);
    audio_i2s_init();

	audio_i2s_start((uint8_t *)i2s_tx_buf_a, (uint32_t *)i2s_rx_buf_a);
	audio_i2s_set_next_buf((const uint8_t *)i2s_tx_buf_b, (uint32_t *)i2s_rx_buf_b);

    while(1){
        k_sleep(K_MSEC(1000));
        printf("I2S is running...\n");
    }
	return 0;
}
