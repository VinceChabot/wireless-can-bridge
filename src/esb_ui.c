/*
 * User interface: pairing button, status LED and periodic statistics.
 *
 *   short press      start pairing (LED blinks fast)
 *   hold for 5 s     forget the pairing
 *   LED solid        linked
 *   LED short flash  paired, link not up
 *   LED off          not paired
 */
#include <zephyr/drivers/gpio.h>

#include "can_communication.h"
#include "esb_internal.h"

LOG_MODULE_DECLARE(ESB);

#define FORGET_HOLD_MS 5000

#ifndef CONFIG_BRIDGE_SYNTHETIC_RATE
#define CONFIG_BRIDGE_SYNTHETIC_RATE 0   /* option disabled; keeps the code below compiling */
#endif

extern const struct device *const can_dev;

static const struct gpio_dt_spec pairing_button = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static const struct gpio_dt_spec link_led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

int ui_init(void)
{
	int err;

	if (!gpio_is_ready_dt(&pairing_button) || !gpio_is_ready_dt(&link_led)) {
		return -ENODEV;
	}
	err = gpio_pin_configure_dt(&pairing_button, GPIO_INPUT);
	if (err) {
		return err;
	}
	return gpio_pin_configure_dt(&link_led, GPIO_OUTPUT_INACTIVE);
}

static void print_stats(void)
{
	static uint32_t prev_tx, prev_rx, prev_can_rx, prev_can_tx;
	struct can_bridge_stats cs;

	esb_ctr_report();
	can_get_bridge_stats(can_dev, &cs);
	LOG_INF("%s link=%d cycles ok/lost=%u/%u | CAN rx=%u (dropped %u, overrun %u) "
		"-> radio acked=%u | radio rx=%u (dropped %u) -> CAN tx=%u "
		"(err %u, late >1ms %u >5ms %u >20ms %u, max %u us, can_send avg %u max %u us) | queues c2e=%u e2c=%u | gobacks=%u gaps=%u",
		is_master ? "master" : "slave", link_up, esb_stats.cycles_ok, esb_stats.cycles_lost,
		cs.rx_frames, cs.rx_dropped, cs.rx_overruns, esb_stats.tx_frames,
		esb_stats.rx_frames, esb_stats.rx_dropped, cs.tx_frames, cs.tx_errors, cs.tx_late_1ms,
		cs.tx_late_5ms, cs.tx_late_20ms, cs.tx_late_max_us,
		cs.tx_send_avg_us, cs.tx_send_max_us, k_msgq_num_used_get(&msgq_CAN_to_ESB),
		k_msgq_num_used_get(&msgq_ESB_to_CAN), esb_stats.gobacks, esb_stats.gaps);
	if (is_master) {
		char buf[HOP_COUNT * 9 + 1];
		int n = 0;

		for (int i = 0; i < HOP_COUNT; i++) {
			n += snprintk(&buf[n], sizeof(buf) - n, "%u:%u%% ", lp.hop[i],
				      chan_loss[POOL_IDX(lp.hop[i])] * 100U / 255U);
		}
		LOG_INF("master skipped=%u afh_swaps=%u table(ch:loss): %s",
			esb_stats.cycles_skipped, esb_stats.afh_swaps, buf);
	}
	LOG_INF("rate/s: CAN rx %u, radio acked %u, radio rx %u, CAN tx %u",
		(cs.rx_frames - prev_can_rx) * 1000U / STATS_PERIOD_MS,
		(esb_stats.tx_frames - prev_tx) * 1000U / STATS_PERIOD_MS,
		(esb_stats.rx_frames - prev_rx) * 1000U / STATS_PERIOD_MS,
		(cs.tx_frames - prev_can_tx) * 1000U / STATS_PERIOD_MS);
	prev_can_rx = cs.rx_frames;
	prev_tx = esb_stats.tx_frames;
	prev_rx = esb_stats.rx_frames;
	prev_can_tx = cs.tx_frames;
}

void ui_tick(void)
{
	static uint32_t held_ms;
	static bool forgotten;
	static uint32_t ticks;
	static uint32_t stats_ms;
	bool pressed = gpio_pin_get_dt(&pairing_button) > 0;
	bool led;

	ticks++;
	if (pressed) {
		held_ms += UI_TICK_MS;
		if (held_ms >= FORGET_HOLD_MS && !forgotten) {
			forgotten = true;
			forget_pairing();
		}
	} else {
		if (held_ms >= 2 * UI_TICK_MS && !forgotten) {
			start_pairing();
		}
		held_ms = 0;
		forgotten = false;
	}

	switch (state) {
	case ST_PAIRING:
		led = (ticks / 5) % 2;                     /* 5 Hz blink */
		break;
	case ST_LINK:
		led = link_up ? true : (ticks % 50) < 5;   /* solid, or short 1 Hz flash */
		break;
	default:
		led = false;
		break;
	}
	if (gpio_is_ready_dt(&link_led)) {
		gpio_pin_set_dt(&link_led, led ? 1 : 0);
	}

	if (IS_ENABLED(CONFIG_BRIDGE_SYNTHETIC_TRAFFIC) && state == ST_LINK && link_up) {
		/* Synthetic CAN traffic for bench-testing the link. */
		static uint8_t counter;
		static uint32_t credit;   /* frames x 1000 still owed */

		credit += CONFIG_BRIDGE_SYNTHETIC_RATE * UI_TICK_MS;
		while (credit >= 1000U) {
			struct bridge_frame bf = {.frame = {.id = is_master ? 0x101 : 0x102,
							    .dlc = 8},
						  .t_us = bridge_now_us()};

			credit -= 1000U;
			bf.frame.data[0] = counter++;
			(void)bridge_msgq_put_drop_oldest(&msgq_CAN_to_ESB, &bf);
		}
	}

	stats_ms += UI_TICK_MS;
	if (stats_ms >= STATS_PERIOD_MS) {
		stats_ms = 0;
		if (state == ST_LINK) {
			print_stats();
		}
	}
}
