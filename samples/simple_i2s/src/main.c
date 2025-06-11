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
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <stdint.h>
#include <nrfx_i2s.h>
#include <nrfx_clock.h>
#include "nrf54l15.h"
#if defined(NRF54L15_XXAA)
#include <hal/nrf_clock.h>
#endif /* defined(NRF54L15_XXAA) */

#include <zephyr/drivers/i2c.h>
#define I2C_NODE DT_NODELABEL(tlv320)

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec rst = GPIO_DT_SPEC_GET(DT_ALIAS(led3), gpios);

#define I2S_NL DT_NODELABEL(i2s20)
PINCTRL_DT_DEFINE(I2S_NL);
static nrfx_i2s_t i2s_inst = NRFX_I2S_INSTANCE(20);
static nrfx_i2s_config_t cfg = {
	/* Pins are configured by pinctrl. */
	.skip_gpio_cfg = true,
	.skip_psel_cfg = true,
	.irq_priority = DT_IRQ(I2S_NL, priority),
	.mode = NRF_I2S_MODE_SLAVE,
	.format = NRF_I2S_FORMAT_I2S,
	.alignment = NRF_I2S_ALIGN_LEFT,
	.ratio = NRF_I2S_RATIO_64X,
	.sample_width = NRF_I2S_SWIDTH_16BIT,
	.channels = NRF_I2S_CHANNELS_STEREO,
	.mck_setup = NRF_I2S_MCK_32MDIV2,
};

#include <math.h>
#define I2S_SAMPLES_NUM 48
static uint16_t i2s_tx_buf_a[I2S_SAMPLES_NUM * 2]; // 2 channels, 16 bits each
static uint16_t i2s_tx_buf_b[I2S_SAMPLES_NUM * 2]; // 2 channels, 16 bits each
static uint16_t i2s_rx_buf_a[I2S_SAMPLES_NUM * 2]; // 2 channels, 16 bits each
static uint16_t i2s_rx_buf_b[I2S_SAMPLES_NUM * 2]; // 2 channels, 16 bits each
#include <zephyr/sys/ring_buffer.h>
RING_BUF_DECLARE(i2s_tx_ring_buf, I2S_SAMPLES_NUM * 2 * sizeof(uint16_t)*10); // 10 buffers of size I2S_SAMPLES_NUM * 2 * sizeof(uint16_t)
#define M_PI 3.1415926f


#define VOLUME 0.01f
#define TEST_FREQ 528
void sine_wave_segment(int16_t *buf, size_t size, float frequency, uint32_t sample_rate)
{
    static float phase = 0.0f;
    float phase_increment = (2.0f * M_PI * frequency) / sample_rate;
    for (size_t i = 0; i < size; i += 2) {
        buf[i] = (32767 * VOLUME * sinf(phase)); // Left channel
        buf[i + 1] = (32767 * VOLUME * sinf(phase)); // Right channel
        phase += phase_increment;
        if (phase >= 2.0f * M_PI) {
            phase -= 2.0f * M_PI;
        }
    }
}

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
        
		if ((uint16_t *)released_bufs->p_tx_buffer == i2s_tx_buf_a) {
			// printf("TX buffer A released\n");
            sine_wave_segment(i2s_tx_buf_a, I2S_SAMPLES_NUM*2, TEST_FREQ, 48000);
			audio_i2s_set_next_buf((const uint8_t *)i2s_tx_buf_a,
					       (uint32_t *)i2s_rx_buf_a);
		} else if ((uint16_t *)released_bufs->p_tx_buffer == i2s_tx_buf_b) {
			// printf("TX buffer B released\n");
			sine_wave_segment(i2s_tx_buf_b, I2S_SAMPLES_NUM*2, TEST_FREQ, 48000);
			audio_i2s_set_next_buf((const uint8_t *)i2s_tx_buf_b,
					       (uint32_t *)i2s_rx_buf_b);
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

	IRQ_CONNECT(DT_IRQN(I2S_NL), DT_IRQ(I2S_NL, priority), nrfx_isr, nrfx_i2s_20_irq_handler,
		    0);
	irq_enable(DT_IRQN(I2S_NL));

	ret = nrfx_i2s_init(&i2s_inst, &cfg, i2s_comp_handler);
	if (ret != NRFX_SUCCESS) {
		printf("Failed to initialize I2S: %x\n", ret);
		return;
	}
}

static int clocks_start(void)
{
	int err;
	int res;
	struct onoff_manager *clk_mgr;
	struct onoff_client clk_cli;

	clk_mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
	if (!clk_mgr) {
		printf("Unable to get the Clock manager\n");
		return -ENXIO;
	}

	sys_notify_init_spinwait(&clk_cli.notify);

	err = onoff_request(clk_mgr, &clk_cli);
	if (err < 0) {
		printf("Clock request failed: %d\n", err);
		return err;
	}

	do {
		err = sys_notify_fetch_result(&clk_cli.notify, &res);
		if (!err && res) {
			printf("Clock could not be started: %d\n", res);
			return res;
		}
	} while (err);

#if defined(NRF54L15_XXAA)
	/* MLTPAN-20 */
	nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_PLLSTART);
#endif /* defined(NRF54L15_XXAA) */

	printf("HF clock started\n");
	return 0;
}

void dac_i2c_write(const struct i2c_dt_spec *dev_i2c, uint8_t reg, uint8_t value)
{
	int ret;
	uint8_t config[2] = {reg, value};

	ret = i2c_write_dt(dev_i2c, config, sizeof(config));
	if (ret != 0) {
		printf("Failed to write to I2C device address %x at reg. %x\n", dev_i2c->addr, reg);
	} else {
		printf("I2C device address %x at reg. %x written successfully\n", dev_i2c->addr,
		       reg);
	}
}

void tlv320_setup(void)
{
	static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C_NODE);

	if (!device_is_ready(dev_i2c.bus)) {
		printf("I2C bus %s is not ready!\n", dev_i2c.bus->name);
		return;
	} else {
		printf("I2C bus %s is ready!\n", dev_i2c.bus->name);
	}

	dac_i2c_write(&dev_i2c, 0x00, 0x00); // Page 0 selected

	// (c) Initiate Software Reset (PLL is powered off as part of reset)
	//     Page 0 / Register 1 (0x01): Software Reset (D0=1 for reset)
	dac_i2c_write(&dev_i2c, 0x01, 0x01);
	k_sleep(K_MSEC(
		10)); // Wait for reset to complete and internal memories to initialize (min 1ms)

	// --- Step 2: Program Clock Settings ---
	// Target: MCLK = 16MHz, fS = 48KHz, I2S Master, 16-bit stereo.
	// CODEC_CLKIN = NDAC * MDAC * DOSR * DAC_fS
	// For 48KHz, a common CODEC_CLKIN is 256 * fS = 12.288 MHz.
	// However, using PLL example from datasheet (Table 6-28) for 16MHz MCLK and 48KHz fS:
	// PLL_CLKIN = 16 MHz, PLLP=1, PLLR=1, PLLJ=5, PLLD=3760
	// This results in PLL_CLK = 16 * (5.3760 * 1 / 1) = 86.016 MHz.
	// Then, NDAC=7, MDAC=2, DOSR=128 leads to CODEC_CLKIN = 7 * 2 * 128 * 48KHz = 86.016 MHz.

	// (a) Program PLL clock dividers P, J, D, and R
	//     Page 0 / Register 4 (0x04): Clock-Gen Muxing
	//     D3-D2 = 00 (PLL_CLKIN = MCLK), D1-D0 = 11 (CODEC_CLKIN = PLL_CLK)
	dac_i2c_write(
		&dev_i2c, 0x04,
		0x03 | (0b11 << 0)); // Set PLL_CLKIN source to MCLK, CODEC_CLKIN source to PLL_CLK

	//     Page 0 / Register 5 (0x05): PLL P and R Values
	//     D6-D4 = 001 (P=1), D3-D0 = 0001 (R=1)
	//     D7 (PLL Power Up) will be set later.
	dac_i2c_write(&dev_i2c, 0x05, (0b001 << 4) | (0b0001 << 0)); // P=1, R=1 (D7=0 initially)

	//     Page 0 / Register 6 (0x06): PLL J-Value
	//     D5-D0 = 000101 (J=5)
	dac_i2c_write(&dev_i2c, 0x06, 0x05); // J=5

	//     Page 0 / Register 7 (0x07): PLL D-Value MSB (D[13:8] of 3760 = 0x0E)
	dac_i2c_write(&dev_i2c, 0x07, 0x0E); // D[13:8] for D=3760

	//     Page 0 / Register 8 (0x08): PLL D-Value LSB (D[7:0] of 3760 = 0xB0)
	//     Note: Page 0 / Register 8 must be written immediately after Page 0 / Register 7 for
	//     D-value update.
	dac_i2c_write(&dev_i2c, 0x08, 0xB0); // D[7:0] for D=3760

	// (b) Power up PLL
	//     Page 0 / Register 5 (0x05): PLL P and R Values
	//     Set D7 = 1 to power up PLL, keeping P=1, R=1.
	dac_i2c_write(&dev_i2c, 0x05,
		      (1 << 7) | (0b001 << 4) | (0b0001 << 0)); // Power up PLL, P=1, R=1
	k_sleep(K_MSEC(15)); // Wait for PLL to stabilize (min 10ms)

	// (c) Program and power up NDAC
	//     Page 0 / Register 11 (0x0B): DAC NDAC_VAL
	//     D7=1 (power up), D6-D0 = 0000111 (NDAC=7)
	dac_i2c_write(&dev_i2c, 0x0B, 0x87); // NDAC powered up and set to 7

	// (d) Program and power up MDAC
	//     Page 0 / Register 12 (0x0C): DAC MDAC_VAL
	//     D7=1 (power up), D6-D0 = 0000010 (MDAC=2)
	dac_i2c_write(&dev_i2c, 0x0C, 0x82); // MDAC powered up and set to 2

	// (e) Program DOSR value
	//     Page 0 / Register 13 (0x0D): DAC DOSR_VAL MSB (D9-D8 = 00)
	dac_i2c_write(&dev_i2c, 0x0D, 0x00); // DOSR MSB (D9-D8) = 00

	//     Page 0 / Register 14 (0x0E): DAC DOSR_VAL LSB (D7-D0 = 10000000 for 128)
	//     Note: Page 0 / Register 14 must be written immediately after Page 0 / Register 13.
	dac_i2c_write(&dev_i2c, 0x0E, 0x80); // DOSR LSB (D7-D0) = 128 (Total DOSR = 128)

	// (f) Program I2S word length (16 bits) and Master mode (BCLK and WCLK are outputs)
	//     Page 0 / Register 27 (0x1B): Codec Interface Control 1
	//     D7-D6 = 00 (I2S mode)
	//     D5-D4 = 00 (16-bit word length)
	//     D3 = 1 (BCLK is output)
	//     D2 = 1 (WCLK is output)
	//     D1-D0 = 00 (Reserved)
	dac_i2c_write(&dev_i2c, 0x1B, 0x0C); // I2S, 16-bit, Master Mode (BCLK/WCLK output)

	//     Configure BCLK N-divider for 48KHz, 16-bit stereo I2S (BCLK = 2 * 16 * 48KHz = 1.536
	//     MHz) BDIV_CLKIN = DAC_MOD_CLK = CODEC_CLKIN / (NDAC * MDAC) = 86.016 MHz / (7 * 2)
	//     = 6.144 MHz N = BDIV_CLKIN / BCLK = 6.144 MHz / 1.536 MHz = 4 Page 0 / Register 30
	//     (0x1E): BCLK N_VAL D7=1 (power up), D6-D0 = 0000100 (N=4)
	dac_i2c_write(&dev_i2c, 0x1E, 0x84); // BCLK N-divider powered up and set to 4

	//     Select BDIV_CLKIN source as DAC_MOD_CLK
	//     Page 0 / Register 29 (0x1D): Codec Interface Control 2
	//     D1-D0 = 01 (BDIV_CLKIN = DAC_MOD_CLK)
	dac_i2c_write(&dev_i2c, 0x1D, (0b01 << 0)); // BDIV_CLKIN from DAC_MOD_CLK

	// (g) Program the processing block to be used
	//     For 48KHz high-performance, Filter A is recommended. PRB_P1 is Filter A, Stereo.
	//     Page 0 / Register 60 (0x3C): DAC Processing Block Selection
	//     D4-D0 = 00001 (PRB_P1)
	dac_i2c_write(&dev_i2c, 0x3C, 0x01); // Select Processing Block PRB_P1

	// (h) Miscellaneous page 0 controls
	//     DAC volume control through pin disabled (using register control)
	//     Page 0 / Register 116 (0x74): VOL/MICDET-Pin SAR ADC — Volume Control
	//     D7=0 (DAC volume control by register)
	dac_i2c_write(&dev_i2c, 0x74, 0x00);

	// --- Step 3: Program Analog Blocks ---
	// (a) Set register page to 1
	dac_i2c_write(&dev_i2c, 0x00, 0x01); // Page 1 selected

	// (b) Program common-mode voltage for headphone/lineout drivers
	//     Page 1 / Register 31 (0x1F): Headphone Drivers
	//     D4-D3 = 00 (Output common-mode voltage = 1.35V)
	//     Other bits (D7, D6, D5, D2, D1, D0) will be configured for power up later, or left as
	//     default/recommended. For now, set common mode and ensure power bits are off.
	dac_i2c_write(&dev_i2c, 0x1F, (0b00 << 3)); // Common-mode 1.35V, drivers off

	// (c) Program headphone-specific de-pop settings (if headphone driver is used)
	//     Page 1 / Register 33 (0x21): HP Output Drivers POP Removal Settings
	//     D6-D3 = 0111 (Driver power-on time = 304ms)
	//     D2-D1 = 11 (Driver ramp-up step time = 3.9ms)
	dac_i2c_write(&dev_i2c, 0x21, (0b0111 << 3) | (0b11 << 1)); // De-pop settings

	// (d) Program routing of DAC output to the output amplifier (headphone/lineout or speaker)
	//     Page 1 / Register 35 (0x23): DAC_L and DAC_R Output Mixer Routing
	//     D7-D6 = 01 (DAC_L routed to left-channel mixer amplifier)
	//     D5 = 0 (AIN1 not routed)
	//     D4 = 0 (AIN2 not routed)
	//     D3-D2 = 01 (DAC_R routed to right-channel mixer amplifier)
	//     D1 = 0 (AIN2 not routed to right mixer)
	//     D0 = 0 (HPL not routed to HPR)
	dac_i2c_write(&dev_i2c, 0x23, 0x44); // Route DAC_L to left mixer, DAC_R to right mixer

	// (e) Unmute and set gain of output drivers (Analog Volume)
	//     Page 1 / Register 36 (0x24): Left Analog Volume to HPL
	//     D7=1 (route to HPL), D6-D0 = 0000000 (0dB gain)
	dac_i2c_write(&dev_i2c, 0x24, 0x80); // Enable HPL analog volume, set = 0 dB

	//     Page 1 / Register 37 (0x25): Right Analog Volume to HPR
	//     D7=1 (route to HPR), D6-D0 = 0000000 (0dB gain)
	dac_i2c_write(&dev_i2c, 0x25, 0x80); // Enable HPR analog volume, set = 0 dB

	//     Page 1 / Register 40 (0x28): HPL Driver
	//     D6-D3 = 0000 (0dB gain), D2=1 (unmute)
	dac_i2c_write(&dev_i2c, 0x28, 0x06); // Unmute HPL, set gain = 0 dB (D1=1 for default)

	//     Page 1 / Register 41 (0x29): HPR Driver
	//     D6-D3 = 0000 (0dB gain), D2=1 (unmute)
	dac_i2c_write(&dev_i2c, 0x29, 0x06); // Unmute HPR, set gain = 0 dB (D1=1 for default)

	// (f) Power up output drivers
	//     Page 1 / Register 31 (0x1F): Headphone Drivers
	//     D7=1 (HPL powered up), D6=1 (HPR powered up), D4-D3 = 00 (1.35V common-mode)
	dac_i2c_write(&dev_i2c, 0x1F,
		      0xC0 | (0b00 << 3)); // HPL and HPR powered up, 1.35V common mode

	//     Power up Class-D driver (if used, otherwise skip)
	//     Page 1 / Register 32 (0x20): Class-D Speaker Amplifier
	//     D7=1 (Class-D powered up)
	// dac_i2c_write(&dev_i2c, 0x20, 0x80); // Power-up Class-D driver

	// --- Step 4: Apply waiting time for de-pop and soft-stepping ---
	// This delay ensures the analog stages are settled before DAC data starts.
	k_sleep(K_MSEC(300)); // Based on 304ms power-on time from de-pop settings (Reg 0x21)

	// --- Step 5: Power up DAC and set digital gain ---
	// (a) Set register page to 0
	dac_i2c_write(&dev_i2c, 0x00, 0x00); // Page 0 selected

	// (b) Power up DAC channels and set digital gain
	//     Page 0 / Register 63 (0x3F): DAC Data-Path Setup
	//     D7=1 (Left DAC ON), D6=1 (Right DAC ON)
	//     D5-D4=01 (Left data path = left data), D3-D2=01 (Right data path = right data)
	//     D1-D0=00 (Soft step enabled, one step per sample period)
	dac_i2c_write(&dev_i2c, 0x3F,
		      0xD4); // Powerup DAC left and right channels (soft step enabled)

	//     Page 0 / Register 65 (0x41): DAC Left Volume Control
	//     Set to 0dB (0x00)
	dac_i2c_write(&dev_i2c, 0x41, 0x00); // DAC Left gain = 0 dB

	//     Page 0 / Register 66 (0x42): DAC Right Volume Control
	//     Set to 0dB (0x00)
	dac_i2c_write(&dev_i2c, 0x42, 0x00); // DAC Right gain = 0 dB

	// (c) Unmute digital volume control
	//     Page 0 / Register 64 (0x40): DAC Volume Control
	//     D3=0 (Left DAC not muted), D2=0 (Right DAC not muted)
	//     D1-D0=00 (Independent volume control)
	dac_i2c_write(&dev_i2c, 0x40, 0x00); // Unmute DAC left and right channels

	dac_i2c_write(&dev_i2c, 0x00, 0x00);    // switch to Page 0
}

int main(void)
{
	static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C_NODE);

	printf("Hello I2S! %s\n", CONFIG_BOARD_TARGET);
	gpio_pin_configure_dt(&led, GPIO_OUTPUT);
	gpio_pin_configure_dt(&rst, GPIO_OUTPUT);
	clocks_start();

	gpio_pin_set_dt(&rst, 0); // Reset high
	k_sleep(K_MSEC(1000));    // Wait for reset to take effect
	gpio_pin_set_dt(&rst, 1); // Reset high
	tlv320_setup();

	audio_i2s_init();

	audio_i2s_start((uint8_t *)i2s_tx_buf_a, (uint32_t *)i2s_rx_buf_a);
	audio_i2s_set_next_buf((const uint8_t *)i2s_tx_buf_b, (uint32_t *)i2s_rx_buf_b);

	while (1) {
		printk("speed up, 48000\n");
        k_sleep(K_MSEC(1000)); // Main loop, can add more functionality here

		printk("speed up, 48375\n");
		//dac_i2c_write(&dev_i2c, 0x0D, (127 >> 8) & 0x03); // DOSR MSB = 0
		//dac_i2c_write(&dev_i2c, 0x0E, 127 & 0xFF);        // DOSR LSB = 127
		dac_i2c_write(&dev_i2c, 0x07, 0xFE); // D[13:8] for D=3760
		dac_i2c_write(&dev_i2c, 0x08, 0xBF); // D[7:0] for D=3760
			dac_i2c_write(&dev_i2c, 0x05,
		      (1 << 7) | (0b001 << 4) | (0b0001 << 0)); // Power up PLL, P=1, R=1
		k_sleep(K_MSEC(1000)); // Main loop, can add more functionality here

		printk("speed up, 48000\n");
		//dac_i2c_write(&dev_i2c, 0x0D, 0x00); // DOSR MSB (D9-D8) = 00
		//dac_i2c_write(&dev_i2c, 0x0E, 0x80); // DOSR LSB (D7-D0) = 128 (Total DOSR = 128)
		dac_i2c_write(&dev_i2c, 0x07, 0x0E); // D[13:8] for D=3760
		dac_i2c_write(&dev_i2c, 0x08, 0xB0); // D[7:0] for D=3760		
			dac_i2c_write(&dev_i2c, 0x05,
		      (1 << 7) | (0b001 << 4) | (0b0001 << 0)); // Power up PLL, P=1, R=1
		k_sleep(K_MSEC(1000)); // Main loop, can add more functionality here

		printk("speed down, 47880\n");
		//dac_i2c_write(&dev_i2c, 0x0D, (129 >> 8) & 0x03); // DOSR MSB = 0
		//dac_i2c_write(&dev_i2c, 0x0E, 129 & 0xFF);        // DOSR LSB = 129
		dac_i2c_write(&dev_i2c, 0x07, 0xFE); // D[13:8] for D=3760
		dac_i2c_write(&dev_i2c, 0x08, 0xA0); // D[7:0] for D=3760
			dac_i2c_write(&dev_i2c, 0x05,
		      (1 << 7) | (0b001 << 4) | (0b0001 << 0)); // Power up PLL, P=1, R=1
		k_sleep(K_MSEC(1000)); // Main loop, can add more functionality here

		//dac_i2c_write(&dev_i2c, 0x0D, 0x00); // DOSR MSB (D9-D8) = 00
		//dac_i2c_write(&dev_i2c, 0x0E, 0x80); // DOSR LSB (D7-D0) = 128 (Total DOSR = 128)
		dac_i2c_write(&dev_i2c, 0x07, 0xFE); // D[13:8] for D=3760
		dac_i2c_write(&dev_i2c, 0x08, 0xB0); // D[7:0] for D=3760
			dac_i2c_write(&dev_i2c, 0x05,
		      (1 << 7) | (0b001 << 4) | (0b0001 << 0)); // Power up PLL, P=1, R=1

	}
	return 0;
}
