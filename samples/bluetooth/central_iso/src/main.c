/*
 * Copyright (c) 2021 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stddef.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/iso.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>
#include <zephyr/types.h>
#include <zephyr/zephyr.h>
LOG_MODULE_REGISTER(ble);

#define DEVICE_NAME_PEER_L     "Peripheral_L"
#define DEVICE_NAME_PEER_R     "Peripheral_R"
#define DEVICE_NAME_PEER_L_LEN (sizeof(DEVICE_NAME_PEER_L) - 1)
#define DEVICE_NAME_PEER_R_LEN (sizeof(DEVICE_NAME_PEER_R) - 1)
#define CIS_CONN_RETRY_TIMES   5
#define ISO_MAX_PAYLOAD 30
#define BT_LE_CONN_PARAM_MULTI BT_LE_CONN_PARAM(40, 40, 0, 400)
static void start_scan(void);

static struct bt_conn *gateway_conn_peer[CONFIG_BT_MAX_CONN];
static struct k_work_delayable iso_send_work;

static struct bt_iso_chan iso_chan[CONFIG_BT_ISO_MAX_CHAN];
static struct bt_iso_chan *iso_chan_p[CONFIG_BT_ISO_MAX_CHAN];
static struct k_work_delayable iso_cis_conn_work;

static uint32_t seq_num[2];
#define ISO_INTERVAL_US 5000//(5U * USEC_PER_MSEC) /* 10 ms */
NET_BUF_POOL_FIXED_DEFINE(tx_pool_l, 1, BT_ISO_SDU_BUF_SIZE(CONFIG_BT_ISO_TX_MTU), 8, NULL);
NET_BUF_POOL_FIXED_DEFINE(tx_pool_r, 1, BT_ISO_SDU_BUF_SIZE(CONFIG_BT_ISO_TX_MTU), 8, NULL);
struct worker_data {
	uint8_t channel;
	uint8_t retries;
} __aligned(4);

K_MSGQ_DEFINE(kwork_msgq, sizeof(struct worker_data), CONFIG_BT_ISO_MAX_CHAN, 4);

int ble_acl_gateway_conn_peer_get(uint8_t chan_number, struct bt_conn **p_conn)
{
	if (chan_number >= CONFIG_BT_MAX_CONN) {
		return -EINVAL;
	}
	*p_conn = gateway_conn_peer[chan_number];
	return 0;
}

int ble_acl_gateway_conn_peer_set(uint8_t chan_number, struct bt_conn **p_conn)
{
	if (chan_number >= CONFIG_BT_MAX_CONN) {
		return -EINVAL;
	}

	if (gateway_conn_peer[chan_number] != NULL) {
		if (*p_conn == NULL) {
			gateway_conn_peer[chan_number] = NULL;
		} else {
			LOG_WRN("Already have a connection for peer: %d", chan_number);
		}
		/* Ignore duplicates as several peripherals might be
		 * advertising at the same time
		 */
		return 0;
	}

	gateway_conn_peer[chan_number] = *p_conn;
	return 0;
}

bool ble_acl_gateway_all_links_connected(void)
{
	for (int i = 0; i < CONFIG_BT_MAX_CONN; i++) {
		if (gateway_conn_peer[i] == NULL) {
			return false;
		}
	}
	return true;
}

static void work_iso_cis_conn(struct k_work *work)
{
	int ret;
	struct bt_iso_connect_param connect_param;
	struct worker_data work_data;

	ret = k_msgq_get(&kwork_msgq, &work_data, K_NO_WAIT);
	if (ret) {
		LOG_ERR("k_msgq_get failed");
	}

	ret = ble_acl_gateway_conn_peer_get(work_data.channel, &connect_param.acl);
	if (ret) {
		LOG_ERR("Connection peer get error");
	}
	connect_param.iso_chan = iso_chan_p[work_data.channel];

	ret = bt_iso_chan_connect(&connect_param, 1);
	work_data.retries++;
	if (ret) {
		if (work_data.retries < CIS_CONN_RETRY_TIMES) {
			LOG_WRN("Got connect error from ch %d Retrying. code: %d count: %d",
				work_data.channel, ret, work_data.retries);
			ret = k_msgq_put(&kwork_msgq, &work_data, K_NO_WAIT);
			if (ret) {
				LOG_ERR("k_msgq_put failed");
			}
			/* Delay added to prevent controller overloading */
			k_work_reschedule(&iso_cis_conn_work, K_MSEC(500));
		} else {
			LOG_ERR("Could not connect ch %d after %d retries", work_data.channel,
				work_data.retries);
			bt_conn_disconnect(connect_param.acl, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
		}
	}
}
static uint8_t iso_chan_to_idx(struct bt_iso_chan *chan)
{
	if (chan == NULL) {
		LOG_ERR("chan is NULL");
	}

	for (uint8_t i = 0; i < CONFIG_BT_ISO_MAX_CHAN; i++) {
		if (chan == iso_chan_p[i]) {
			return i;
		}
	}

	LOG_ERR("No index found for this channel");
	CODE_UNREACHABLE;
	return UINT8_MAX;
}
/**
 * @brief Send ISO data on timeout
 *
 * This will send an increasing amount of ISO data, starting from 1 octet.
 *
 * First iteration : 0x00
 * Second iteration: 0x00 0x01
 * Third iteration : 0x00 0x01 0x02
 *
 * And so on, until it wraps around the configured ISO TX MTU (CONFIG_BT_ISO_TX_MTU)
 *
 * @param work Pointer to the work structure
 */
static void iso_timer_timeout(struct k_work *work)
{
	int ret;
	static uint8_t buf_data[ISO_MAX_PAYLOAD];
	static bool data_initialized;
	struct net_buf *buf;

	if (!data_initialized) {
		for (int i = 0; i < ARRAY_SIZE(buf_data); i++) {
			buf_data[i] = (uint8_t)i;
		}

		data_initialized = true;
	}

	if (iso_chan[0].state == BT_ISO_STATE_CONNECTED) {
		buf = net_buf_alloc(&tx_pool_l, K_FOREVER);
		net_buf_reserve(buf, BT_ISO_CHAN_SEND_RESERVE);

		net_buf_add_mem(buf, buf_data, sizeof(buf_data));

		ret = bt_iso_chan_send(&iso_chan[0], buf, seq_num[0]++, BT_ISO_TIMESTAMP_NONE);

		if (ret < 0) {
			printk("Failed to send ISO data (%d)", ret);
		}

	} 
	
	if (iso_chan[1].state == BT_ISO_STATE_CONNECTED) {
		buf = net_buf_alloc(&tx_pool_r, K_FOREVER);
		net_buf_reserve(buf, BT_ISO_CHAN_SEND_RESERVE);

		net_buf_add_mem(buf, buf_data, sizeof(buf_data));

		ret = bt_iso_chan_send(&iso_chan[1], buf, seq_num[1]++, BT_ISO_TIMESTAMP_NONE);

		if (ret < 0) {
			printk("Failed to send ISO data (%d)", ret);
		}

	} 


	k_work_schedule(&iso_send_work, K_USEC(ISO_INTERVAL_US));
}

static int device_found(uint8_t type, const uint8_t *data, uint8_t data_len,
			const bt_addr_le_t *addr)
{
	int ret;
	struct bt_conn *conn;

	if ((data_len == DEVICE_NAME_PEER_L_LEN) &&
	    (strncmp(DEVICE_NAME_PEER_L, data, DEVICE_NAME_PEER_L_LEN) == 0)) {

		ret = bt_le_scan_stop();
		if (ret) {
			LOG_INF("Stop scan failed: %d", ret);
		}

		ret = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN, BT_LE_CONN_PARAM_MULTI,
					&conn);
		if (ret) {
			LOG_INF("Could not init connection");
			return ret;
		}
		ret = ble_acl_gateway_conn_peer_set(0, &conn);
		if (ret) {
			LOG_ERR("Connection peer set error %d", ret);
		}
		return 0;
	} else if ((data_len == DEVICE_NAME_PEER_R_LEN) &&
		   (strncmp(DEVICE_NAME_PEER_R, data, DEVICE_NAME_PEER_R_LEN) == 0)) {
		ret = bt_le_scan_stop();
		if (ret) {
			LOG_ERR("Stop scan failed: %d", ret);
		}

		ret = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN, BT_LE_CONN_PARAM_MULTI,
					&conn);
		if (ret) {
			LOG_INF("Could not init connection");
			return ret;
		}
		ret = ble_acl_gateway_conn_peer_set(1, &conn);
		if (ret) {
			LOG_ERR("Connection peer set error %d", ret);
		}
		return 0;
	}

	return -ENOENT;
}

/** @brief  Parse BLE advertisement package.
 */
static void ad_parse(struct net_buf_simple *p_ad, const bt_addr_le_t *addr)
{
	while (p_ad->len > 1) {
		uint8_t len = net_buf_simple_pull_u8(p_ad);
		uint8_t type;

		/* Check for early termination */
		if (len == 0) {
			return;
		}

		if (len > p_ad->len) {
			LOG_ERR("AD malformed");
			return;
		}

		type = net_buf_simple_pull_u8(p_ad);

		if (device_found(type, p_ad->data, len - 1, addr) == 0) {
			return;
		}

		(void)net_buf_simple_pull(p_ad, len - 1);
	}
}

static void on_device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
			    struct net_buf_simple *p_ad)
{
	/* Direct advertising has no payload, so no need to parse */
	if (type == BT_GAP_ADV_TYPE_ADV_IND || type == BT_GAP_ADV_TYPE_SCAN_RSP ||
	    type == BT_GAP_ADV_TYPE_EXT_ADV) {
		/* Note: May lead to connection creation */
		ad_parse(p_ad, addr);
	}
}

static void start_scan(void)
{
	int err;

	/* This demo doesn't require active scan */
	err = bt_le_scan_start(BT_LE_SCAN_PASSIVE, on_device_found);
	if (err) {
		LOG_ERR("Scanning failed to start (err %d)", err);
		return;
	}

	LOG_INF("Scanning successfully started");
}

static void iso_connected(struct bt_iso_chan *chan)
{
	LOG_INF("ISO Channel %p connected", chan);
	uint8_t chan_num;
	chan_num = iso_chan_to_idx(chan);
	seq_num[chan_num] = 0U;
	LOG_INF("chan %d connected", chan_num);
	/* Start send timer */
	k_work_reschedule(&iso_send_work, K_MSEC(100));
}

static void iso_disconnected(struct bt_iso_chan *chan, uint8_t reason)
{
	LOG_INF("ISO Channel %p disconnected (reason 0x%02x)", (void *)chan, reason);
	k_work_cancel_delayable(&iso_send_work);
}

static struct bt_iso_chan_ops iso_ops = {
	.connected = iso_connected,
	.disconnected = iso_disconnected,
};

int ble_trans_iso_cis_connect(struct bt_conn *conn)
{
	int ret;
	struct bt_conn *conn_active;

	for (uint8_t i = 0; i < CONFIG_BT_MAX_CONN; i++) {
		ret = ble_acl_gateway_conn_peer_get(i, &conn_active);
		if (ret) {
			LOG_ERR("Connection peer get error");
		}
		if (conn == conn_active) {
			struct worker_data work_data;

			work_data.channel = i;
			work_data.retries = 0;

			ret = k_msgq_put(&kwork_msgq, &work_data, K_NO_WAIT);
			if (ret) {
				return ret;
			}

			k_work_schedule(&iso_cis_conn_work, K_MSEC(500 * i));
		}
	}

	return 0;
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];
	int ret;

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (err) {
		LOG_ERR("Failed to connect to %s (%u)", addr, err);
	}
	LOG_INF("Connected: %s", addr);


	ret = bt_conn_set_security(conn, BT_SECURITY_L2);
	if(ret) {
		LOG_ERR("set security failed %d", ret);
	}

}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	struct bt_conn *conn_active;

	for (int i = 0; i < CONFIG_BT_MAX_CONN; i++) {
		int ret;

		ret = ble_acl_gateway_conn_peer_get(i, &conn_active);
		if (ret) {
			LOG_ERR("Connection peer get error");
		}
		if (conn_active == conn) {
			bt_conn_unref(conn_active);
			conn_active = NULL;
			ret = ble_acl_gateway_conn_peer_set(i, &conn_active);
			if (ret) {
				LOG_ERR("Connection peer get error %d", ret);
			}
			LOG_DBG("Headset %d disconnected", i);
			break;
		}
	}

	start_scan();
}

static void mtu_exchange_handler(struct bt_conn *conn, uint8_t err,
				 struct bt_gatt_exchange_params *params)
{
	int ret;

	if (err) {
		LOG_ERR("MTU exchange failed, err = %d", err);
		ret = bt_conn_disconnect(conn, BT_HCI_ERR_LOCALHOST_TERM_CONN);
		if (ret) {
			LOG_ERR("Failed to disconnected, %d", ret);
		}
	} else {
		LOG_DBG("MTU exchange success");
		if (!ble_acl_gateway_all_links_connected()) {
			start_scan();
		} else {
			LOG_INF("All ACL links are connected");
			bt_le_scan_stop();
		}

		ret = ble_trans_iso_cis_connect(conn);
		if(ret) {
			LOG_ERR("Failed to connect to ISO CIS channel");
		}
	}
}

#include <zephyr/bluetooth/gatt.h>
static struct bt_gatt_exchange_params exchange_params;

int ble_acl_gateway_mtu_exchange(struct bt_conn *conn)
{
	exchange_params.func = mtu_exchange_handler;

	return bt_gatt_exchange_mtu(conn, &exchange_params);
}

static void security_changed_cb(struct bt_conn *conn, bt_security_t level, enum bt_security_err err)
{
	int ret;

	if (err) {
		LOG_ERR("Security failed: level %u err %d", level, err);
		ret = bt_conn_disconnect(conn, err);
		if (ret) {
			LOG_ERR("Failed to disconnect %d", ret);
		}
	} else {
		LOG_DBG("Security changed: level %u", level);
	}

	ret = ble_acl_gateway_mtu_exchange(conn);
	if (ret) {
		LOG_WRN("MTU exchange procedure failed = %d", ret);
	}
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
	.security_changed = security_changed_cb,
};

static struct bt_iso_chan_qos iso_cis_qos;

static struct bt_iso_chan_io_qos iso_tx_qos = {
	.sdu = ISO_MAX_PAYLOAD, /* bytes */
	.rtn = 1,
	.phy = BT_GAP_LE_PHY_2M,
	.path = NULL,
};
static struct bt_iso_cig_param cis_create_param = {
	.cis_channels = iso_chan_p,
	.num_cis = 2,
	.sca = BT_GAP_SCA_UNKNOWN,
	.packing = BT_ISO_PACKING_SEQUENTIAL,
	.framing = BT_ISO_FRAMING_UNFRAMED,
	.latency = 10,
	.interval = ISO_INTERVAL_US,
};

int ble_trans_iso_cig_create(void)
{
	int ret;

	struct bt_iso_cig *cig;

	ret = bt_iso_cig_create(&cis_create_param, &cig);
	if (ret) {
		LOG_ERR("Failed to create CIG (%d)\n", ret);
		return ret;
	}

	return 0;
}
void main(void)
{
	int err;

	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("Bluetooth init failed (err %d)", err);
		return;
	}

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	LOG_INF("Bluetooth initialized");

	for (int8_t i = 0; i < CONFIG_BT_ISO_MAX_CHAN; i++) {
		iso_chan_p[i] = &iso_chan[i];
	}

	for (int i = 0; i < CONFIG_BT_ISO_MAX_CHAN; i++) {
		iso_chan_p[i]->ops = &iso_ops;
		iso_chan_p[i]->qos = &iso_cis_qos;
	}
	iso_cis_qos.rx = NULL;
	iso_cis_qos.tx = &iso_tx_qos;
	err = ble_trans_iso_cig_create();
	if (err) {
		LOG_ERR("CIG create failed %d", err);
	}
	k_work_init_delayable(&iso_cis_conn_work, work_iso_cis_conn);
#if 0
	iso_chan.ops = &iso_ops;
	iso_chan.qos = &iso_qos;
#if defined(CONFIG_BT_SMP)
	iso_chan.required_sec_level = BT_SECURITY_L2,
#endif /* CONFIG_BT_SMP */

	channels[0] = &iso_chan;
	param.cis_channels = channels;
	param.num_cis = ARRAY_SIZE(channels);
	param.sca = BT_GAP_SCA_UNKNOWN;
	param.packing = 0;
	param.framing = 0;
	param.latency = 10;	      /* ms */
	param.interval = interval_us; /* us */

	err = bt_iso_cig_create(&param, &cig);

	if (err != 0) {
		LOG_ERR("Failed to create CIG (%d)", err);
		return;
	}
#endif
	start_scan();

	k_work_init_delayable(&iso_send_work, iso_timer_timeout);
}
