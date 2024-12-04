/*
 * Copyright (c) 2023-2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <stdlib.h>

#include <sample_usbd.h>
#include "feedback.h"

#include <zephyr/cache.h>
#include <zephyr/device.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/usb/class/usbd_uac2.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/usb/class/usbd_hid.h>
//#include <zephyr/drivers/i2s.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(uac2_sample, LOG_LEVEL_INF);

#define HEADPHONES_OUT_TERMINAL_ID UAC2_ENTITY_ID(DT_NODELABEL(out_terminal))
#define MICROPHONE_IN_TERMINAL_ID UAC2_ENTITY_ID(DT_NODELABEL(in_terminal))

#define SAMPLES_PER_SOF     48
#define SAMPLE_FREQUENCY    (SAMPLES_PER_SOF * 1000)
#define SAMPLE_BIT_WIDTH    16
#define NUMBER_OF_CHANNELS  2
#define BYTES_PER_SAMPLE    DIV_ROUND_UP(SAMPLE_BIT_WIDTH, 8)
#define BYTES_PER_SLOT      (BYTES_PER_SAMPLE * NUMBER_OF_CHANNELS)
#define MIN_BLOCK_SIZE      ((SAMPLES_PER_SOF - 1) * BYTES_PER_SLOT)
#define BLOCK_SIZE          (SAMPLES_PER_SOF * BYTES_PER_SLOT)
#define MAX_BLOCK_SIZE      ((SAMPLES_PER_SOF + 1) * BYTES_PER_SLOT)

/* Absolute minimum is 5 TX buffers (1 actively consumed by I2S, 2nd queued as
 * next buffer, 3rd acquired by USB stack to receive data to, and 2 to handle
 * SOF/I2S offset errors), but add 2 additional buffers to prevent out of memory
 * errors when USB host decides to perform rapid terminal enable/disable cycles.
 */
#define I2S_BLOCKS          7
K_MEM_SLAB_DEFINE_STATIC(i2s_tx_slab, MAX_BLOCK_SIZE, I2S_BLOCKS, 4);
K_MEM_SLAB_DEFINE_STATIC(i2s_rx_slab, MAX_BLOCK_SIZE, I2S_BLOCKS, 4);

struct usb_i2s_ctx {
	const struct device *i2s_dev;
	bool headphones_enabled;
	bool microphone_enabled;
	bool i2s_started;
	bool rx_started;
	bool usb_data_received;
	/* Counter used to determine when to start I2S and then when to start
	 * sending RX packets to host. Overflows are not a problem because this
	 * variable is not necessary after both I2S and RX is started.
	 */
	uint8_t i2s_counter;
	struct feedback_ctx *fb;

	/* Leftover samples from I2S receive buffer, already compacted to mono,
	 * that were not sent to host. The buffer, if not NULL, is allocated
	 * from I2S RX slab.
	 */
	uint8_t *pending_mic_buf;
	uint8_t pending_mic_samples;

	/* Rolling bit buffers for tracking nominal + 1 and nominal - 1 samples
	 * sent. Bits are mutually exclusive, i.e.:
	 *   plus_ones | minus_ones = plus_ones ^ minus_ones
	 *
	 * Used to avoid overcompensation in feedback regulator. LSBs indicate
	 * latest write size.
	 */
	uint8_t plus_ones;
	uint8_t minus_ones;
};

static void uac2_terminal_update_cb(const struct device *dev, uint8_t terminal,
				    bool enabled, bool microframes,
				    void *user_data)
{
	struct usb_i2s_ctx *ctx = user_data;

	/* This sample is for Full-Speed only devices. */
	__ASSERT_NO_MSG(microframes == false);

	if (terminal == HEADPHONES_OUT_TERMINAL_ID) {
		ctx->headphones_enabled = enabled;
	} else if (terminal == MICROPHONE_IN_TERMINAL_ID) {
		ctx->microphone_enabled = enabled;
	}

}

static void *uac2_get_recv_buf(const struct device *dev, uint8_t terminal,
			       uint16_t size, void *user_data)
{
	ARG_UNUSED(dev);
	struct usb_i2s_ctx *ctx = user_data;
	void *buf = NULL;
	int ret;

	if (terminal == HEADPHONES_OUT_TERMINAL_ID) {
		__ASSERT_NO_MSG(size <= MAX_BLOCK_SIZE);

		if (!ctx->headphones_enabled) {
			LOG_ERR("Buffer request on disabled terminal");
			return NULL;
		}

		ret = k_mem_slab_alloc(&i2s_tx_slab, &buf, K_NO_WAIT);
		if (ret != 0) {
			buf = NULL;
		}
	}

	return buf;
}

static void uac2_data_recv_cb(const struct device *dev, uint8_t terminal,
			      void *buf, uint16_t size, void *user_data)
{
	struct usb_i2s_ctx *ctx = user_data;

	ctx->usb_data_received = true;

	if (!ctx->headphones_enabled && !ctx->microphone_enabled) {
		k_mem_slab_free(&i2s_tx_slab, buf);
		return;
	}
	size = 24;
	if (!size) {
		/* This code path is expected when host only records microphone
		 * data and is not streaming any audio to the headphones. Simply
		 * transmit as many zero-filled samples were last sent to allow
		 * the feedback regulator to work.
		 *
		 * When host is streaming audio, this can be a transient error.
		 * While the "feedback regulator delay" is likely to differ,
		 * it is still probably best to just zero-fill last sent number
		 * of samples. If we overcompensate as a result, the situation
		 * will stabilize after a while anyway.
		 *
		 * In either case, we have to keep I2S going and the only way
		 * we can control the SOF to I2S offset is by varying the number
		 * of samples sent.
		 */
		if (ctx->plus_ones & 1) {
			size = (SAMPLES_PER_SOF + 1) * BYTES_PER_SLOT;
		} else if (ctx->minus_ones & 1) {
			size = (SAMPLES_PER_SOF - 1) * BYTES_PER_SLOT;
		} else {
			size = SAMPLES_PER_SOF * BYTES_PER_SLOT;
		}
		memset(buf, 0, size);
		sys_cache_data_flush_range(buf, size);
	}

	LOG_DBG("Received %d data to input terminal %d", size, terminal);

	//ret = i2s_write(ctx->i2s_dev, buf, size);
	k_mem_slab_free(&i2s_tx_slab, buf);
}

static void uac2_buf_release_cb(const struct device *dev, uint8_t terminal,
				void *buf, void *user_data)
{
	if (terminal == MICROPHONE_IN_TERMINAL_ID) {
		k_mem_slab_free(&i2s_rx_slab, buf);
	}
}

static uint8_t __aligned(UDC_BUF_ALIGN) data_buffer[ROUND_UP(24, UDC_BUF_GRANULARITY)] = {0};
static void uac2_sof(const struct device *dev, void *user_data)
{
	int ret;
	ARG_UNUSED(dev);

	ret = usbd_uac2_send(dev, MICROPHONE_IN_TERMINAL_ID, data_buffer, 12);
	if (ret < 0) {
		LOG_ERR("Failed to send data to host, ret = %d", ret);
	}
	return ;
}

static struct uac2_ops usb_audio_ops = {
	.sof_cb = uac2_sof,
	.terminal_update_cb = uac2_terminal_update_cb,
	.get_recv_buf = uac2_get_recv_buf,
	.data_recv_cb = uac2_data_recv_cb,
	.buf_release_cb = uac2_buf_release_cb,
};

static void kb_iface_ready(const struct device *dev, const bool ready)
{
	LOG_INF("HID device %s interface is %s",
		dev->name, ready ? "ready" : "not ready");
}

static int kb_get_report(const struct device *dev,
			 const uint8_t type, const uint8_t id, const uint16_t len,
			 uint8_t *const buf)
{
	LOG_WRN("Get Report not implemented, Type %u ID %u", type, id);

	return 0;
}

static int kb_set_report(const struct device *dev,
			 const uint8_t type, const uint8_t id, const uint16_t len,
			 const uint8_t *const buf)
{
	return 0;
}

/* Idle duration is stored but not used to calculate idle reports. */
static void kb_set_idle(const struct device *dev,
			const uint8_t id, const uint32_t duration)
{
	LOG_INF("Set Idle %u to %u", id, duration);
}

static uint32_t kb_get_idle(const struct device *dev, const uint8_t id)
{
	return 0;
}

static void kb_set_protocol(const struct device *dev, const uint8_t proto)
{
	LOG_INF("Protocol changed to %s",
		proto == 0U ? "Boot Protocol" : "Report Protocol");
}

static void kb_output_report(const struct device *dev, const uint16_t len,
			     const uint8_t *const buf)
{
	LOG_HEXDUMP_DBG(buf, len, "o.r.");
	kb_set_report(dev, HID_REPORT_TYPE_OUTPUT, 0U, len, buf);
}

struct hid_device_ops kb_ops = {
	.iface_ready = kb_iface_ready,
	.get_report = kb_get_report,
	.set_report = kb_set_report,
	.set_idle = kb_set_idle,
	.get_idle = kb_get_idle,
	.set_protocol = kb_set_protocol,
	.output_report = kb_output_report,
};


static struct usb_i2s_ctx main_ctx;
const struct device *hid_dev;
static const uint8_t hid_report_desc[] = HID_KEYBOARD_REPORT_DESC();
int main(void)
{
	const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(uac2_headset));
	struct usbd_context *sample_usbd;
	int ret;

	main_ctx.fb = feedback_init();


	usbd_uac2_set_ops(dev, &usb_audio_ops, &main_ctx);

	hid_dev = DEVICE_DT_GET_ONE(zephyr_hid_device);
	if (!device_is_ready(hid_dev)) {
		LOG_ERR("HID Device is not ready");
		return -EIO;
	}

	ret = hid_device_register(hid_dev,
				  hid_report_desc, sizeof(hid_report_desc),
				  &kb_ops);
	if (ret != 0) {
		LOG_ERR("Failed to register HID Device, %d", ret);
		return ret;
	}


	sample_usbd = sample_usbd_init_device(NULL);
	if (sample_usbd == NULL) {
		return -ENODEV;
	}

	ret = usbd_enable(sample_usbd);
	if (ret) {
		return ret;
	}

	LOG_INF("usbd_enable");

	return 0;
}
