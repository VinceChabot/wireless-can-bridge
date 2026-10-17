/*
 * Data link: send queue, reliable in-order delivery, TDD cycle and timestamps.
 *
 * Reliability
 *   Frames are numbered with a free-running 16-bit index. The master sends a packet
 *   every cycle and learns from the radio's own acknowledgement whether it arrived.
 *   The slave's packets ride in ESB ACK payloads, so it only finds out one cycle
 *   later, from the ack_idx the master echoes in its next packet. To keep the
 *   channel busy the slave sends optimistically and goes back to the confirmed
 *   index when the echo shows a payload was lost (go-back-N). The receiver accepts
 *   only the next expected index, which drops duplicates and out-of-order data.
 *
 * Timestamps
 *   Each frame carries its age (in AGE_UNIT_US) relative to the instant the packet
 *   was sent. The receiver works out when the frame was received on the source bus
 *   in its own clock and schedules it PLAYOUT_DELAY later, so the frames leave in
 *   the same relative timing they arrived with. Neither board's clock needs to be
 *   synchronised with the other.
 */
#include "esb_internal.h"

LOG_MODULE_DECLARE(ESB);

#define PLAYOUT_DELAY_US CONFIG_BRIDGE_PLAYOUT_DELAY_US

/* Outgoing frames: sq[] holds the frames [sq_head, sq_next) that the peer has not
 * confirmed yet.
 */
static struct bridge_frame sq[SQ_SIZE];
static uint16_t sq_head;          /* oldest frame not yet confirmed by the peer */
static uint16_t sq_next;          /* next frame to take from the CAN queue */
static uint16_t sq_send;          /* slave: next frame to put in an ACK payload */
static uint8_t last_sent_count;   /* master: frames in the packet on air */
static bool q_valid;              /* slave: an ACK payload is queued... */
static uint16_t q_end;            /* ...ending at this frame index */
static uint16_t last_tx_end;      /* slave: end index of the last transmitted payload */

/* Incoming frames. */
static uint16_t rx_expected;      /* next frame index expected from the peer */
static bool rx_synced;
static uint32_t last_release_us;  /* keeps the release times in order */
static bool release_valid;

/* Channel handling. */
static uint8_t cur_hop_slot;
static bool scanning;             /* slave: searching for the master */
static uint8_t scan_idx;

/* Master: when each recent packet was sent, to date the slave's frames. */
#define TXRING_SIZE 32U
struct tx_note {
	uint32_t t_ref;
	uint16_t len;
	uint16_t cycle;
	bool valid;
};
static struct tx_note txring[TXRING_SIZE];

#ifdef CONFIG_BRIDGE_COUNTER_CHECK
/* Bench check of a periodic test message: extended id (29-bit), 4-bit counter
 * in byte 1 (start bit 8, little-endian). Reports gaps in the counter.
 */
#define CTR_ID CONFIG_BRIDGE_COUNTER_ID
struct ctr_check {
	const char *where;
	bool init;
	uint8_t last;
	uint32_t seen, lost, repeats;
};
static struct ctr_check ctr_src = {.where = "CAN->radio"};
static struct ctr_check ctr_dst = {.where = "radio->CAN"};

static void ctr_feed(struct ctr_check *c, const struct can_frame *f)
{
	uint8_t cur;
	uint8_t gap;

	if (f->id != CTR_ID || !(f->flags & CAN_FRAME_IDE) || f->dlc < 2) {
		return;
	}
	cur = f->data[1] & 0x0F;
	if (c->init) {
		gap = (cur - c->last - 1) & 0x0F;
		if (cur == c->last) {
			c->repeats++;
			LOG_WRN("ctr %s: repeat of %u", c->where, cur);
		} else if (gap) {
			c->lost += gap;
			LOG_WRN("ctr %s: missing %u frame(s), %u -> %u", c->where, gap, c->last, cur);
		}
	}
	c->init = true;
	c->last = cur;
	c->seen++;
}

void esb_ctr_report(void)
{
	LOG_INF("ctr %s: seen=%u lost=%u repeats=%u | %s: seen=%u lost=%u repeats=%u",
		ctr_src.where, ctr_src.seen, ctr_src.lost, ctr_src.repeats,
		ctr_dst.where, ctr_dst.seen, ctr_dst.lost, ctr_dst.repeats);
}
#define CTR_FEED_SRC(f) ctr_feed(&ctr_src, f)
#define CTR_FEED_DST(f) ctr_feed(&ctr_dst, f)
#else
#define CTR_FEED_SRC(f) do { } while (0)
#define CTR_FEED_DST(f) do { } while (0)
#endif

static inline int16_t idx_diff(uint16_t a, uint16_t b)
{
	return (int16_t)(a - b);
}

/* Time from the moment a packet is handed to the radio to the moment the
 * receiver's RX event fires (end of the packet).
 */
static inline uint32_t air_us(uint16_t len)
{
	return RADIO_RAMP_US + (len + ESB_OVERHEAD_BYTES) * 4U;
}

void set_link(bool up)
{
	if (up == link_up) {
		return;
	}
	link_up = up;
	LOG_INF("Link %s (%s)", up ? "UP" : "DOWN", is_master ? "master" : "slave");
}

/* ---- Send queue and packets --------------------------------------------- */

/* Take frames from the CAN queue into the send queue. */
static void sq_fill(void)
{
	while ((uint16_t)(sq_next - sq_head) < SQ_SIZE) {
		struct bridge_frame bf;

		if (k_msgq_get(&msgq_CAN_to_ESB, &bf, K_NO_WAIT)) {
			break;
		}
		if (bf.frame.dlc > 8U) {
			LOG_WRN("Dropping CAN frame with invalid DLC");
			continue;
		}
		CTR_FEED_SRC(&bf.frame);
		sq[sq_next % SQ_SIZE] = bf;
		sq_next++;
	}
}

/* Build a packet carrying as many frames as fit, starting at index `first`.
 * `t_ref` is the instant the packet counts as "sent" for the frame ages.
 * Returns the number of frames in the packet.
 */
static uint8_t build_data_packet(struct esb_payload *p, uint16_t first, uint32_t t_ref)
{
	uint16_t avail = sq_next - first;
	uint16_t off = HDR_SIZE;
	uint8_t n = 0;

	memset(p, 0, sizeof(*p));
	p->pipe = 0;
	p->noack = false;
	while (n < avail) {
		const struct bridge_frame *f = &sq[(uint16_t)(first + n) % SQ_SIZE];
		uint8_t size = FRAME_OVERHEAD + f->frame.dlc;
		int32_t age = bridge_us_diff(t_ref, f->t_us) / AGE_UNIT_US;

		if (off + size > CONFIG_ESB_MAX_PAYLOAD_LENGTH) {
			break;
		}
		age = CLAMP(age, INT16_MIN, INT16_MAX);
		sys_put_le32(f->frame.id, &p->data[off]);
		p->data[off + 4] = (f->frame.dlc & 0x0F) | ((f->frame.flags & 0x0F) << 4);
		sys_put_le16((uint16_t)(int16_t)age, &p->data[off + 5]);
		memcpy(&p->data[off + FRAME_OVERHEAD], f->frame.data, f->frame.dlc);
		off += size;
		n++;
	}
	p->length = off;
	p->data[0] = PKT_DATA;
	p->data[1] = n;
	sys_put_le16(first, &p->data[2]);
	sys_put_le16(rx_expected, &p->data[4]);
	sys_put_le16(cycle, &p->data[6]);
	p->data[8] = 0;
	memcpy(&p->data[9], lp.hop, HOP_COUNT);
	return n;
}

/* Slave: the master tells us (ack_idx) how many of our frames it has accepted.
 * That echo lags one cycle behind, so payloads are sent optimistically and we
 * go back to the confirmed index if the echo shows a payload was lost.
 */
static void slave_process_ack(uint16_t ack_idx)
{
	uint16_t expect_min = last_tx_end;

	if (q_valid) {
		/* The payload queued last time left with this packet's ACK. */
		last_tx_end = q_end;
		q_valid = false;
	}

	if (idx_diff(ack_idx, sq_head) < 0 || idx_diff(sq_next, ack_idx) < 0) {
		/* Echo outside the frames we hold: the master restarted. Resend all. */
		sq_send = sq_head;
		last_tx_end = sq_head;
		esb_stats.gobacks++;
		return;
	}
	esb_stats.tx_frames += (uint16_t)(ack_idx - sq_head);
	sq_head = ack_idx;
	if (idx_diff(sq_send, ack_idx) < 0) {
		sq_send = ack_idx; /* an earlier, since-reset transmission got through */
	}
	if (idx_diff(ack_idx, expect_min) < 0) {
		/* A payload we sent never arrived: resend from the confirmed index. */
		sq_send = ack_idx;
		last_tx_end = ack_idx;
		esb_stats.gobacks++;
	}
}

/* Process one received data packet. Returns false when it was malformed. */
static bool handle_data_packet(const struct esb_payload *p)
{
	uint8_t count;
	uint16_t first, ack_idx, peer_cycle;
	uint16_t off = HDR_SIZE;
	uint32_t t_base;   /* this board's clock at the instant the sender calls "sent" */

	if (p->length < HDR_SIZE || p->data[0] != PKT_DATA) {
		return false;
	}
	count = p->data[1];
	first = sys_get_le16(&p->data[2]);
	ack_idx = sys_get_le16(&p->data[4]);
	peer_cycle = sys_get_le16(&p->data[6]);

	if (is_master) {
		/* The slave's reference is its RX event for master packet `peer_cycle`. */
		const struct tx_note *n = &txring[peer_cycle % TXRING_SIZE];

		if (n->valid && n->cycle == peer_cycle) {
			t_base = n->t_ref + air_us(n->len);
		} else {
			t_base = bridge_now_us() - 2 * CYCLE_US;
		}
	} else {
		/* The master's reference is the instant it sent this packet. */
		t_base = rx_event_us - air_us(p->length);
	}

	if (!rx_synced || abs(idx_diff(rx_expected, first)) > RESYNC_WINDOW) {
		rx_expected = first;
		rx_synced = true;
	}
	if (idx_diff(first, rx_expected) > 0) {
		esb_stats.gaps++; /* a frame is missing before this packet; wait for the resend */
	}

	for (uint8_t i = 0; i < count; i++) {
		struct bridge_frame bf = {0};
		uint8_t dlc;

		if (off + FRAME_OVERHEAD > p->length) {
			return false;
		}
		dlc = p->data[off + 4] & 0x0F;
		if (dlc > 8U || off + FRAME_OVERHEAD + dlc > p->length) {
			return false;
		}
		if ((uint16_t)(first + i) == rx_expected) {
			int16_t age = (int16_t)sys_get_le16(&p->data[off + 5]);
			uint32_t release = t_base - (int32_t)age * AGE_UNIT_US + PLAYOUT_DELAY_US;

			bf.frame.id = sys_get_le32(&p->data[off]);
			bf.frame.dlc = dlc;
			bf.frame.flags = p->data[off + 4] >> 4;
			memcpy(bf.frame.data, &p->data[off + FRAME_OVERHEAD], dlc);
			CTR_FEED_DST(&bf.frame);

			if (release_valid && bridge_us_diff(release, last_release_us) < 0) {
				release = last_release_us; /* never reorder */
			}
			last_release_us = release;
			release_valid = true;
			bf.t_us = release;

			rx_expected++;
			if (bridge_msgq_put_drop_oldest(&msgq_ESB_to_CAN, &bf)) {
				esb_stats.rx_dropped++;
				if (esb_stats.rx_dropped == 1U || (esb_stats.rx_dropped % 5000U) == 0U) {
					LOG_ERR("CAN transmit queue full, dropped %u old frames",
						esb_stats.rx_dropped);
				}
			}
			esb_stats.rx_frames++;
		}
		off += FRAME_OVERHEAD + dlc;
	}

	if (!is_master) {
		for (uint8_t i = 0; i < HOP_COUNT; i++) {
			if (p->data[9 + i] <= 80U) {
				lp.hop[i] = p->data[9 + i];
			}
		}
		cycle = peer_cycle;
		missed = 0;
		scanning = false;
		set_link(true);
		slave_process_ack(ack_idx);
	}
	return true;
}

/* Slave: queue the ACK payload that will answer the next master packet.
 *
 * Do not flush the ESB TX FIFO here: ESB keeps an ACK payload at the front of
 * its FIFO after sending it and removes it when the next new packet arrives.
 * A flush would make ESB drop the freshly queued payload instead.
 */
static void slave_prepare_ack(void)
{
	sq_fill();
	if ((uint16_t)(sq_next - sq_send) > 0U) {
		uint8_t n = build_data_packet(&tx_payload, sq_send, rx_event_us);
		if (esb_write_payload(&tx_payload)) {
			LOG_ERR("Failed to queue ACK payload");
			return;
		}
		q_end = sq_send + n;
		sq_send = q_end;
		q_valid = true;
	}
}

static void link_drain_rx(void)
{
	bool got = false;

	while (esb_read_rx_payload(&rx_payload) == 0) {
		if (!handle_data_packet(&rx_payload)) {
			LOG_WRN("Dropping malformed packet (len %u)", rx_payload.length);
			continue;
		}
		got = true;
	}
	if (got && !is_master) {
		slave_prepare_ack();
	}
}

/* ---- TDD cycle ---------------------------------------------------------- */

/* Master: start of a cycle. */
void master_cycle(void)
{
	uint32_t t_ref;

	cycle++;
	if ((cycle % AFH_PERIOD_CYCLES) == 0U) {
		afh_update();
	}
	if (!esb_is_idle()) {
		esb_stats.cycles_skipped++;
		return;
	}
	sq_fill();
	cur_hop_slot = cycle % HOP_COUNT;
	esb_set_rf_channel(lp.hop[cur_hop_slot]);
	t_ref = bridge_now_us();
	last_sent_count = build_data_packet(&tx_payload, sq_head, t_ref);
	txring[cycle % TXRING_SIZE].t_ref = t_ref;
	txring[cycle % TXRING_SIZE].len = tx_payload.length;
	txring[cycle % TXRING_SIZE].cycle = cycle;
	txring[cycle % TXRING_SIZE].valid = true;
	if (esb_write_payload(&tx_payload)) {
		esb_flush_tx();
		(void)esb_write_payload(&tx_payload);
	}
}

void link_radio_events(atomic_val_t ev)
{
	if (ev & EV_RX) {
		link_drain_rx();
	}
	if (!is_master) {
		return;
	}
	if (ev & EV_TX_OK) {
		esb_stats.tx_frames += last_sent_count;
		sq_head += last_sent_count;
		last_sent_count = 0;
		esb_stats.cycles_ok++;
		afh_record(lp.hop[cur_hop_slot], true);
		missed = 0;
		set_link(true);
	}
	if (ev & EV_TX_FAIL) {
		esb_flush_tx();
		last_sent_count = 0;
		esb_stats.cycles_lost++;
		afh_record(lp.hop[cur_hop_slot], false);
		if (++missed >= MISS_LIMIT) {
			set_link(false);
			missed = MISS_LIMIT;
		}
	}
}

/* Slave: move to the channel of the next cycle, or keep searching. */
void slave_hop(void)
{
	cycle++;
	esb_stop_rx();
	if (!scanning) {
		if (missed < MISS_LIMIT) {
			missed++;
		}
		if (missed >= MISS_LIMIT) {
			/* Lost the master. It may have moved channels, so sweep the
			 * whole pool, dwelling long enough for its hop sequence to
			 * come around (HOP_COUNT cycles = 77 ms).
			 */
			scanning = true;
			set_link(false);
			k_timer_start(&hop_timer, K_MSEC(SCAN_DWELL_MS), K_MSEC(SCAN_DWELL_MS));
		}
	}
	if (scanning) {
		scan_idx = (scan_idx + 1U) % POOL_COUNT;
		esb_set_rf_channel(POOL_CH(scan_idx));
	} else {
		esb_set_rf_channel(lp.hop[cycle % HOP_COUNT]);
	}
	esb_start_rx();
}

/* ---- Start / stop ------------------------------------------------------- */
void stop_all_timers(void)
{
	k_timer_stop(&cycle_timer);
	k_timer_stop(&hop_timer);
	k_timer_stop(&pair_timer);
	esb_clear_events(EV_CYCLE | EV_HOP | EV_PAIR_TICK | EV_TX_OK | EV_TX_FAIL | EV_RX);
}

int enter_link(void)
{
	int err;

	stop_all_timers();
	state = ST_LINK;
	is_master = local_device_id() < stored.peer_id;
	derive_link(stored.key, local_device_id(), stored.peer_id);
	afh_reset();

	sq_head = sq_next = sq_send = sys_rand32_get() & 0xFFFF;
	last_tx_end = sq_head;
	last_sent_count = 0;
	q_valid = false;
	rx_expected = 0;
	rx_synced = false;
	release_valid = false;
	memset(txring, 0, sizeof(txring));
	cycle = 0;
	missed = MISS_LIMIT;
	link_up = false;
	scanning = !is_master;
	scan_idx = 0;

	if (is_master) {
		err = radio_setup(ESB_MODE_PTX, lp.base, lp.prefix, lp.hop[0], 0,
				  LINK_RETRANSMIT_DELAY_US);
		if (!err) {
			k_timer_start(&cycle_timer, K_USEC(CYCLE_US), K_USEC(CYCLE_US));
		}
	} else {
		err = radio_setup(ESB_MODE_PRX, lp.base, lp.prefix, POOL_CH(0), 0,
				  LINK_RETRANSMIT_DELAY_US);
		if (!err) {
			err = esb_start_rx();
		}
		if (!err) {
			k_timer_start(&hop_timer, K_MSEC(SCAN_DWELL_MS), K_MSEC(SCAN_DWELL_MS));
		}
	}
	if (err) {
		LOG_ERR("Link radio setup failed: %d", err);
		return err;
	}
	LOG_INF("Link started: peer=0x%08x, role=%s", stored.peer_id,
		is_master ? "master" : "slave");
	return 0;
}
