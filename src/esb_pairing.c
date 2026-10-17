/*
 * Pairing handshake and persistent storage of the pairing.
 *
 * Both boards enter pairing mode (button press). On a fixed channel and address
 * each one alternates between listening for a random time and sending a request.
 * Whoever hears a request answers inside the ESB ACK with its id and a random
 * key; the requester then confirms, and both store {peer id, key} and start the
 * link. The lower device id becomes the master.
 */
#include <zephyr/settings/settings.h>

#include "esb_internal.h"

LOG_MODULE_DECLARE(ESB);

#define PKT_PAIR_REQ 0xD1U
#define PKT_PAIR_RSP 0xD2U
#define PKT_PAIR_CONFIRM 0xD3U

#define PAIR_CHANNEL 2U
#define PAIR_SESSION_MS 30000
#define PAIR_TICK_MS 5
#define PAIR_LISTEN_MIN_MS 15
#define PAIR_LISTEN_SPAN_MS 40
#define PAIR_CONFIRM_WAIT_MS 300
#define PAIR_CONFIRM_TRIES 4

enum pair_phase { PH_LISTEN, PH_REQ_SENT, PH_WAIT_CONFIRM, PH_CONFIRM_SENT };

static const uint8_t pair_base[4] = {0x50, 0x41, 0x49, 0x52};
static const uint8_t pair_prefix = 0xC5;

static struct {
	enum pair_phase phase;
	int64_t phase_deadline;
	int64_t session_deadline;
	uint32_t my_key;
	uint32_t peer_id;
	uint32_t key;
	bool got_rsp;
	uint8_t confirm_tries;
} pair;

/* ---- Persistent storage ------------------------------------------------- */
static int settings_set_cb(const char *name, size_t len, settings_read_cb read_cb,
			   void *cb_arg)
{
	if (strcmp(name, "pair") == 0 && len == sizeof(stored)) {
		if (read_cb(cb_arg, &stored, sizeof(stored)) == sizeof(stored) &&
		    stored.key != 0 && stored.peer_id != 0) {
			stored_valid = true;
		}
	}
	return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(bridge, "bridge", NULL, settings_set_cb, NULL, NULL);

/* Load the stored pairing, if any, into `stored` / `stored_valid`. */
void pairing_init(void)
{
	int err = settings_subsys_init();

	if (!err) {
		err = settings_load();
	}
	if (err) {
		LOG_ERR("Settings init failed: %d (pairing will not persist)", err);
	}
}

static void save_pairing(void)
{
	int err = settings_save_one("bridge/pair", &stored, sizeof(stored));

	if (err) {
		LOG_ERR("Could not store pairing: %d", err);
	}
}

/* Forget the stored pairing and stop the link. */
void forget_pairing(void)
{
	stop_all_timers();
	(void)settings_delete("bridge/pair");
	memset(&stored, 0, sizeof(stored));
	stored_valid = false;
	set_link(false);
	esb_disable();
	state = ST_UNPAIRED;
	LOG_INF("Pairing erased");
}

/* ---- Handshake ---------------------------------------------------------- */
static void pair_listen(void)
{
	int err;

	pair.phase = PH_LISTEN;
	pair.got_rsp = false;
	pair.my_key = sys_rand32_get() | 1U;
	pair.phase_deadline = k_uptime_get() + PAIR_LISTEN_MIN_MS +
			      (sys_rand32_get() % PAIR_LISTEN_SPAN_MS);

	err = radio_setup(ESB_MODE_PRX, pair_base, pair_prefix, PAIR_CHANNEL, 0, 1000);
	if (err) {
		LOG_ERR("Pairing PRX setup failed: %d", err);
		return;
	}
	/* The answer to a pairing request rides in the ACK. */
	memset(&tx_payload, 0, sizeof(tx_payload));
	tx_payload.pipe = 0;
	tx_payload.length = 9;
	tx_payload.data[0] = PKT_PAIR_RSP;
	sys_put_le32(local_device_id(), &tx_payload.data[1]);
	sys_put_le32(pair.my_key, &tx_payload.data[5]);
	(void)esb_write_payload(&tx_payload);
	(void)esb_start_rx();
}

static int pair_send(uint8_t type)
{
	int err = radio_setup(ESB_MODE_PTX, pair_base, pair_prefix, PAIR_CHANNEL, 5, 600);

	if (err) {
		return err;
	}
	memset(&tx_payload, 0, sizeof(tx_payload));
	tx_payload.pipe = 0;
	tx_payload.length = 5;
	tx_payload.data[0] = type;
	sys_put_le32(local_device_id(), &tx_payload.data[1]);
	return esb_write_payload(&tx_payload);
}

static void pair_commit(void)
{
	stored.peer_id = pair.peer_id;
	stored.key = pair.key;
	stored_valid = true;
	save_pairing();
	LOG_INF("Paired with 0x%08x", stored.peer_id);
	(void)enter_link();
}

static void pair_exit(void)
{
	k_timer_stop(&pair_timer);
	if (stored_valid) {
		LOG_INF("Pairing timed out; resuming previous link");
		(void)enter_link();
	} else {
		esb_disable();
		state = ST_UNPAIRED;
		LOG_INF("Pairing timed out");
	}
}

void start_pairing(void)
{
	stop_all_timers();
	state = ST_PAIRING;
	pair.session_deadline = k_uptime_get() + PAIR_SESSION_MS;
	LOG_INF("Pairing mode: id=0x%08x (press the button on the other board)",
		local_device_id());
	pair_listen();
	k_timer_start(&pair_timer, K_MSEC(PAIR_TICK_MS), K_MSEC(PAIR_TICK_MS));
}

void pairing_radio_events(atomic_val_t ev)
{
	if (ev & EV_RX) {
		while (esb_read_rx_payload(&rx_payload) == 0) {
			uint8_t type = rx_payload.data[0];
			uint32_t id = rx_payload.length >= 5 ? sys_get_le32(&rx_payload.data[1]) : 0;

			if (id == 0 || id == local_device_id()) {
				continue;
			}
			if (type == PKT_PAIR_REQ && pair.phase == PH_LISTEN) {
				pair.peer_id = id;
				pair.key = pair.my_key;
				pair.phase = PH_WAIT_CONFIRM;
				pair.phase_deadline = k_uptime_get() + PAIR_CONFIRM_WAIT_MS;
			} else if (type == PKT_PAIR_REQ && pair.phase == PH_WAIT_CONFIRM &&
				   id == pair.peer_id) {
				/* Requester did not see our answer; give it again. */
				memset(&tx_payload, 0, sizeof(tx_payload));
				tx_payload.length = 9;
				tx_payload.data[0] = PKT_PAIR_RSP;
				sys_put_le32(local_device_id(), &tx_payload.data[1]);
				sys_put_le32(pair.key, &tx_payload.data[5]);
				esb_flush_tx();
				(void)esb_write_payload(&tx_payload);
			} else if (type == PKT_PAIR_RSP && pair.phase == PH_REQ_SENT &&
				   rx_payload.length == 9) {
				pair.peer_id = id;
				pair.key = sys_get_le32(&rx_payload.data[5]);
				pair.got_rsp = true;
			} else if (type == PKT_PAIR_CONFIRM && pair.phase == PH_WAIT_CONFIRM &&
				   id == pair.peer_id) {
				pair_commit();
				return;
			}
		}
	}
	if (ev & EV_TX_OK) {
		if (pair.phase == PH_REQ_SENT) {
			if (pair.got_rsp) {
				pair.phase = PH_CONFIRM_SENT;
				pair.confirm_tries = 0;
				if (pair_send(PKT_PAIR_CONFIRM)) {
					pair_commit();
				}
			} else {
				pair_listen();
			}
		} else if (pair.phase == PH_CONFIRM_SENT) {
			pair_commit();
		}
	}
	if (ev & EV_TX_FAIL) {
		esb_flush_tx();
		if (pair.phase == PH_REQ_SENT) {
			pair_listen();
		} else if (pair.phase == PH_CONFIRM_SENT) {
			if (++pair.confirm_tries < PAIR_CONFIRM_TRIES) {
				(void)pair_send(PKT_PAIR_CONFIRM);
			} else {
				LOG_WRN("Pairing confirmation not acknowledged; committing anyway");
				pair_commit();
			}
		}
	}
}

void pairing_tick(void)
{
	int64_t now = k_uptime_get();

	if (now >= pair.session_deadline) {
		pair_exit();
	} else if (pair.phase == PH_LISTEN && now >= pair.phase_deadline) {
		int err;

		pair.phase = PH_REQ_SENT;
		pair.got_rsp = false;
		err = pair_send(PKT_PAIR_REQ);
		if (err) {
			LOG_ERR("Pairing request failed: %d", err);
			pair_listen();
		}
	} else if (pair.phase == PH_WAIT_CONFIRM && now >= pair.phase_deadline) {
		pair_listen();
	}
}
