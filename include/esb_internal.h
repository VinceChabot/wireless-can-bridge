/*
 * Internal definitions shared by the esb_*.c files. Not part of the public API
 * (see esb_communication.h for that).
 *
 *   esb_communication.c  core: events, timers, HF clock, radio setup, public API, radio thread
 *   esb_link.c           data link: send queue, go-back-N, TDD cycle, timestamps
 *   esb_afh.c            adaptive frequency hopping
 *   esb_pairing.c        pairing handshake and persistent storage
 *   esb_ui.c             button, LED and statistics
 */
#ifndef ESB_INTERNAL_H
#define ESB_INTERNAL_H

#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/random/random.h>
#include <stdlib.h>
#include <string.h>
#include <nrf.h>
#include <esb.h>

#include "bridge_time.h"
#include "message_queues.h"

/* ---- Tunables ----------------------------------------------------------- */
#define ESB_TX_THREAD_STACK_SIZE 2048
#define ESB_TX_THREAD_PRIORITY K_PRIO_COOP(1)

/* TDD cycle: one exchange (master packet + slave ACK payload) per cycle, on a new
 * channel each time. An exchange with two maximum-size packets lasts about 2.3 ms;
 * the slave must hop after its ACK has gone out (<= ~1.3 ms after it saw the packet)
 * and before the master's next packet (cycle - ~1.3 ms at the latest).
 */
#define CYCLE_US 3000U
#define HOP_AFTER_RX_US 1500U     /* slave hops this long after a received packet */
#define HOP_COUNT 16U             /* slots in the hop table */
#define POOL_COUNT 26U            /* candidate channels: 2403, 2406 ... 2478 MHz (3 MHz apart) */
#define POOL_CH(i) (3U + 3U * (i))
#define POOL_IDX(ch) (((ch) - 3U) / 3U)
#define SCAN_DWELL_MS 80U         /* slave: time spent on each channel while searching */
#define AFH_PERIOD_CYCLES 128U    /* master: how often the hop table is reviewed */
#define AFH_NEUTRAL_LOSS 64U      /* assumed loss (of 255) for channels with no data */
#define AFH_SWAP_MARGIN 40U       /* a channel must be this much worse to be replaced */
#define AFH_REPEAT_PENALTY 24     /* extra cost for each repeat of a channel in the table */
#define MISS_LIMIT 8U             /* missed cycles before the link is declared down */
#define LINK_RETRANSMIT_DELAY_US 2500U

#define UI_TICK_MS 20
#define STATS_PERIOD_MS 5000

/* ---- Wire format -------------------------------------------------------- */
#define PKT_DATA 0xC3U

/* Data packet: header, then frames packed by their real length.
 *   header: type(1) count(1) first_idx(2) ack_idx(2) cycle(2) rsvd(1) hop_table(16)
 *   frame : id(4) dlc|flags<<4(1) age(2, signed, AGE_UNIT_US) data(dlc)
 */
#define HDR_SIZE 25U
#define FRAME_OVERHEAD 7U
#define AGE_UNIT_US 16
#define ESB_OVERHEAD_BYTES 9U     /* preamble, address, packet control, CRC */
#define RADIO_RAMP_US 150U        /* TX start delay after the packet is written */
#define SQ_SIZE 64U               /* send queue, power of two */
#define RESYNC_WINDOW 1024        /* index jump that means "the peer restarted" */

BUILD_ASSERT(HDR_SIZE + FRAME_OVERHEAD + 8U <= CONFIG_ESB_MAX_PAYLOAD_LENGTH,
	     "ESB payload configuration is too small for a CAN frame");

/* ---- Events for the radio thread ---------------------------------------- */
#define EV_TX_OK BIT(0)
#define EV_TX_FAIL BIT(1)
#define EV_RX BIT(2)
#define EV_CYCLE BIT(3)   /* master: start of a TDD cycle */
#define EV_HOP BIT(4)     /* slave: time to move to the next channel */
#define EV_PAIR_TICK BIT(5)
#define EV_UI_TICK BIT(6)

/* ---- Shared state ------------------------------------------------------- */
enum bridge_state { ST_UNPAIRED, ST_PAIRING, ST_LINK };

struct pairing_rec {
	uint32_t peer_id;
	uint32_t key;
};

struct link_params {
	uint8_t base[4];
	uint8_t prefix;
	uint8_t hop[HOP_COUNT];
};

struct esb_stats {
	uint32_t tx_frames;        /* frames confirmed by the peer */
	uint32_t rx_frames;        /* frames received from the peer and queued for CAN */
	uint32_t rx_dropped;       /* ...of which old frames dropped because the queue was full */
	uint32_t cycles_ok;
	uint32_t cycles_lost;
	uint32_t cycles_skipped;
	uint32_t gobacks;
	uint32_t gaps;
	uint32_t afh_swaps;
};

extern struct pairing_rec stored;
extern bool stored_valid;
extern volatile enum bridge_state state;
extern volatile bool is_master;
extern struct link_params lp;
extern struct esb_stats esb_stats;
extern uint8_t chan_loss[POOL_COUNT];   /* master: smoothed loss per channel, 0..255 */
extern uint16_t cycle;                  /* TDD cycle counter (master's numbering) */
extern uint8_t missed;
extern bool link_up;
extern volatile uint32_t rx_event_us;   /* bridge_now_us() when the last packet was received */
extern struct esb_payload rx_payload;
extern struct esb_payload tx_payload;

extern struct k_timer cycle_timer;
extern struct k_timer hop_timer;
extern struct k_timer pair_timer;
extern struct k_timer ui_timer;

/* ---- esb_communication.c ------------------------------------------------ */
uint32_t local_device_id(void);
void esb_post_event(atomic_val_t ev);
void esb_clear_events(atomic_val_t ev);
int radio_setup(enum esb_mode mode, const uint8_t base[4], uint8_t prefix,
		uint8_t channel, uint16_t retx_count, uint16_t retx_delay);

/* ---- esb_link.c --------------------------------------------------------- */
void set_link(bool up);
int enter_link(void);
void stop_all_timers(void);
void link_radio_events(atomic_val_t ev);
void master_cycle(void);
void slave_hop(void);
#ifdef CONFIG_BRIDGE_COUNTER_CHECK
void esb_ctr_report(void);
#else
static inline void esb_ctr_report(void) { }
#endif

/* ---- esb_afh.c ---------------------------------------------------------- */
void derive_link(uint32_t key, uint32_t id_a, uint32_t id_b);
void afh_reset(void);
void afh_record(uint8_t ch, bool ok);
void afh_update(void);

/* ---- esb_pairing.c ------------------------------------------------------ */
void pairing_init(void);
void start_pairing(void);
void forget_pairing(void);
void pairing_radio_events(atomic_val_t ev);
void pairing_tick(void);

/* ---- esb_ui.c ----------------------------------------------------------- */
int ui_init(void);
void ui_tick(void);

#endif
