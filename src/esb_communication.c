/*
 * Wireless CAN bridge radio link (Nordic ESB): core.
 *
 * Overview
 * --------
 *  - Press the user button on both boards to pair them (LED blinks fast). The
 *    boards exchange device IDs and a random link key on a fixed pairing
 *    channel, store the result in non-volatile memory and start the link.
 *    Role is negotiated automatically: the board with the lower device ID is
 *    the master (PTX), the other one the slave (PRX). Hold the button for 5 s
 *    to forget the pairing.
 *  - Once linked, the pair runs a fixed 3 ms TDD cycle made of two 1.5 ms
 *    slots. Slot 1: the master transmits its CAN frames. Slot 2: the slave
 *    answers with its CAN frames inside the ESB ACK payload. A new RF channel
 *    is used every cycle, following a hop table that the master adapts to
 *    interference and sends to the slave in every packet.
 *  - Frames are delivered reliably and in order in both directions, and are
 *    timestamped so the receiving board can put them on its bus with the same
 *    spacing they had on the source bus.
 *
 * This file holds what everything else builds on: the event queue and timers
 * that drive the radio thread, the HF clock, the ESB setup and the public API.
 */
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <hal/nrf_clock.h>
#include <hal/nrf_power.h>
#include <nrf_erratas.h>

#include "esb_communication.h"
#include "esb_internal.h"

LOG_MODULE_REGISTER(ESB, LOG_LEVEL_DBG);

K_THREAD_STACK_DEFINE(esb_tx_thread_stack, ESB_TX_THREAD_STACK_SIZE);
struct k_thread esb_tx_thread_data;

/* State shared between the esb_*.c files. */
struct pairing_rec stored;
bool stored_valid;
volatile enum bridge_state state = ST_UNPAIRED;
volatile bool is_master;
struct link_params lp;
struct esb_stats esb_stats;
uint8_t chan_loss[POOL_COUNT];
uint16_t cycle;
uint8_t missed;
bool link_up;
volatile uint32_t rx_event_us;
struct esb_payload rx_payload;
struct esb_payload tx_payload;

static bool hf_clock_requested;
static struct onoff_client hf_clock_client;

/* ---- Events and timers -------------------------------------------------- */
K_SEM_DEFINE(esb_wakeup, 0, 1);
static atomic_t pending_events;

void esb_post_event(atomic_val_t ev)
{
	atomic_or(&pending_events, ev);
	k_sem_give(&esb_wakeup);
}

void esb_clear_events(atomic_val_t ev)
{
	atomic_and(&pending_events, ~ev);
}

static void cycle_timer_expiry(struct k_timer *t) { ARG_UNUSED(t); esb_post_event(EV_CYCLE); }
static void hop_timer_expiry(struct k_timer *t) { ARG_UNUSED(t); esb_post_event(EV_HOP); }
static void pair_timer_expiry(struct k_timer *t) { ARG_UNUSED(t); esb_post_event(EV_PAIR_TICK); }
static void ui_timer_expiry(struct k_timer *t) { ARG_UNUSED(t); esb_post_event(EV_UI_TICK); }

K_TIMER_DEFINE(cycle_timer, cycle_timer_expiry, NULL);
K_TIMER_DEFINE(hop_timer, hop_timer_expiry, NULL);
K_TIMER_DEFINE(pair_timer, pair_timer_expiry, NULL);
K_TIMER_DEFINE(ui_timer, ui_timer_expiry, NULL);

uint32_t local_device_id(void)
{
	return NRF_FICR->INFO.DEVICEID[0];
}

/* ---- Clock -------------------------------------------------------------- */
int hf_clock_start(void)
{
	int err;
	int res;
	struct onoff_manager *clk_mgr;

	if (hf_clock_requested) {
		return 0;
	}

	clk_mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
	if (!clk_mgr) {
		LOG_ERR("Unable to get the Clock manager");
		return -ENXIO;
	}

	sys_notify_init_spinwait(&hf_clock_client.notify);

	err = onoff_request(clk_mgr, &hf_clock_client);
	if (err < 0) {
		LOG_ERR("Clock request failed: %d", err);
		return err;
	}

	do {
		err = sys_notify_fetch_result(&hf_clock_client.notify, &res);
		if (!err && res) {
			LOG_ERR("Clock could not be started: %d", res);
			return res;
		}
	} while (err);
	hf_clock_requested = true;

#if NRF54L_ERRATA_20_PRESENT
	if (nrf54l_errata_20()) {
		nrf_power_task_trigger(NRF_POWER, NRF_POWER_TASK_CONSTLAT);
	}
#endif

	return 0;
}

/* ---- Radio -------------------------------------------------------------- */
static void event_handler(struct esb_evt const *event)
{
	switch (event->evt_id) {
	case ESB_EVENT_TX_SUCCESS:
		esb_post_event(EV_TX_OK);
		break;
	case ESB_EVENT_TX_FAILED:
		esb_post_event(EV_TX_FAIL);
		break;
	case ESB_EVENT_RX_RECEIVED:
		/* Timestamp the packet here, in the ISR, so the reference is as
		 * precise as possible. The slave also re-aligns its hop timer to
		 * it.
		 */
		rx_event_us = bridge_now_us();
		if (state == ST_LINK && !is_master) {
			k_timer_start(&hop_timer, K_USEC(HOP_AFTER_RX_US), K_USEC(CYCLE_US));
		}
		esb_post_event(EV_RX);
		break;
	}
}

int radio_setup(enum esb_mode mode, const uint8_t base[4], uint8_t prefix,
		uint8_t channel, uint16_t retx_count, uint16_t retx_delay)
{
	struct esb_config config = ESB_DEFAULT_CONFIG;
	int err;

	/* Cleanly leave RX before re-initialising; esb_init() on a running
	 * receiver leaves the radio PPI wiring in a state where the next
	 * transmission never completes.
	 */
	(void)esb_stop_rx();

	config.protocol = ESB_PROTOCOL_ESB_DPL;
	config.bitrate = ESB_BITRATE_2MBPS;
	config.mode = mode;
	config.retransmit_count = retx_count;
	config.retransmit_delay = retx_delay;
	config.selective_auto_ack = false;
	config.event_handler = event_handler;

	err = esb_init(&config);
	if (err) {
		return err;
	}
	err = esb_set_base_address_0(base);
	if (err) {
		return err;
	}
	err = esb_set_prefixes(&prefix, 1);
	if (err) {
		return err;
	}
	return esb_set_rf_channel(channel);
}

/* ---- Public API --------------------------------------------------------- */
int esb_initialize(void)
{
	int err = ui_init();

	if (err) {
		return err;
	}
	pairing_init();

	esb_clear_events(~0);
	k_timer_start(&ui_timer, K_MSEC(UI_TICK_MS), K_MSEC(UI_TICK_MS));

	LOG_INF("Device id 0x%08x", local_device_id());
	if (stored_valid) {
		LOG_INF("Stored pairing found: peer 0x%08x", stored.peer_id);
		return enter_link();
	}
	state = ST_UNPAIRED;
	LOG_INF("Not paired. Press the user button on both boards to pair.");
	if (IS_ENABLED(CONFIG_BRIDGE_AUTO_PAIR_ON_BOOT)) {
		start_pairing();
	}
	return 0;
}

int esb_tx_thread_init(void)
{
	k_tid_t tid = k_thread_create(&esb_tx_thread_data, esb_tx_thread_stack,
				      K_THREAD_STACK_SIZEOF(esb_tx_thread_stack),
				      esb_tx_thread, NULL, NULL, NULL,
				      ESB_TX_THREAD_PRIORITY, 0, K_NO_WAIT);

	if (!tid) {
		LOG_ERR("Error initializing esb_tx_thread");
		return -EFAULT;
	}
	return 0;
}

void esb_tx_thread(void *unused1, void *unused2, void *unused3)
{
	ARG_UNUSED(unused1);
	ARG_UNUSED(unused2);
	ARG_UNUSED(unused3);
	k_thread_name_set(k_current_get(), "esb_radio");

	while (true) {
		atomic_val_t ev;

		k_sem_take(&esb_wakeup, K_FOREVER);
		ev = atomic_set(&pending_events, 0);

		/* Radio results first, then timers. */
		if (ev & (EV_TX_OK | EV_TX_FAIL | EV_RX)) {
			if (state == ST_LINK) {
				link_radio_events(ev);
			} else if (state == ST_PAIRING) {
				pairing_radio_events(ev);
			}
		}
		if ((ev & EV_CYCLE) && state == ST_LINK && is_master) {
			master_cycle();
		}
		if ((ev & EV_HOP) && state == ST_LINK && !is_master) {
			slave_hop();
		}
		if ((ev & EV_PAIR_TICK) && state == ST_PAIRING) {
			pairing_tick();
		}
		if (ev & EV_UI_TICK) {
			ui_tick();
		}
	}
}
