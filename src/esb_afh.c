/*
 * Link addressing and adaptive frequency hopping.
 *
 * The master scores every channel of a 26-channel pool by how often exchanges on
 * it fail, and every AFH_PERIOD_CYCLES replaces the worst slot of the 16-slot hop
 * table by a better channel. The table travels in every packet, so the slave
 * simply follows it.
 */
#include "esb_internal.h"

LOG_MODULE_DECLARE(ESB);

static uint32_t xorshift32(uint32_t *s)
{
	uint32_t x = *s;

	x ^= x << 13;
	x ^= x >> 17;
	x ^= x << 5;
	return *s = x;
}

/* Avoid ESB address bytes with poor bit transitions. */
static uint8_t fix_addr_byte(uint8_t b)
{
	return (b == 0x00 || b == 0xFF || b == 0x55 || b == 0xAA) ? (b ^ 0x37) : b;
}

/* Derive the private address and the initial hop table from the pairing key. */
void derive_link(uint32_t key, uint32_t id_a, uint32_t id_b)
{
	uint32_t s = key ^ id_a ^ id_b;
	uint8_t pool[POOL_COUNT];

	if (s == 0) {
		s = 0x9E3779B9;
	}
	for (int i = 0; i < 4; i++) {
		lp.base[i] = fix_addr_byte(xorshift32(&s) >> 24);
	}
	lp.prefix = fix_addr_byte(xorshift32(&s) >> 24);

	/* Initial hop table: the first HOP_COUNT channels of a shuffled pool. */
	for (uint8_t i = 0; i < POOL_COUNT; i++) {
		pool[i] = POOL_CH(i);
	}
	for (uint8_t i = POOL_COUNT - 1; i > 0; i--) {
		uint8_t j = xorshift32(&s) % (i + 1);
		uint8_t t = pool[i];

		pool[i] = pool[j];
		pool[j] = t;
	}
	memcpy(lp.hop, pool, HOP_COUNT);
}

/* Forget everything learned about the channels. */
void afh_reset(void)
{
	for (uint8_t i = 0; i < POOL_COUNT; i++) {
		chan_loss[i] = AFH_NEUTRAL_LOSS;
	}
}

/* Record the outcome of one exchange on channel `ch`. */
void afh_record(uint8_t ch, bool ok)
{
	uint8_t i = POOL_IDX(ch);
	int target = ok ? 0 : 255;

	chan_loss[i] = (uint8_t)(chan_loss[i] + (target - (int)chan_loss[i]) / 8);
}

/* Review the hop table: replace the slot with the costliest channel by the
 * cheapest candidate if that is clearly better. A channel may appear in several
 * slots, but each extra use adds a penalty so the sequence stays spread out.
 * Never touches the slots used in this and the next cycle, so the slave (which
 * learns the table from each packet) stays in step.
 */
void afh_update(void)
{
	uint8_t uses[POOL_COUNT] = {0};
	int worst_slot = -1;
	int best_pool = -1;
	int worst_cost = 0;
	int best_cost = 0;
	uint8_t cur = cycle % HOP_COUNT;
	uint8_t nxt = (cycle + 1U) % HOP_COUNT;

	for (uint8_t slot = 0; slot < HOP_COUNT; slot++) {
		uses[POOL_IDX(lp.hop[slot])]++;
	}
	for (uint8_t slot = 0; slot < HOP_COUNT; slot++) {
		uint8_t pi = POOL_IDX(lp.hop[slot]);
		int cost = chan_loss[pi] + AFH_REPEAT_PENALTY * (uses[pi] - 1);

		if (slot != cur && slot != nxt && (worst_slot < 0 || cost > worst_cost)) {
			worst_slot = slot;
			worst_cost = cost;
		}
	}
	for (uint8_t i = 0; i < POOL_COUNT; i++) {
		int cost;

		if (uses[i] == 0U) {
			/* Unused channels drift back to "unknown" so they get retried. */
			chan_loss[i] = (uint8_t)(chan_loss[i] +
				((int)AFH_NEUTRAL_LOSS - (int)chan_loss[i]) / 16);
		}
		if (worst_slot >= 0 && POOL_CH(i) == lp.hop[worst_slot]) {
			continue;
		}
		cost = chan_loss[i] + AFH_REPEAT_PENALTY * uses[i];
		if (best_pool < 0 || cost < best_cost) {
			best_pool = i;
			best_cost = cost;
		}
	}
	if (worst_slot >= 0 && best_pool >= 0 && worst_cost > best_cost + AFH_SWAP_MARGIN) {
		LOG_INF("AFH: slot %d ch %u (%u%% loss) -> ch %u (%u%%)", worst_slot,
			lp.hop[worst_slot], chan_loss[POOL_IDX(lp.hop[worst_slot])] * 100U / 255U,
			POOL_CH(best_pool), chan_loss[best_pool] * 100U / 255U);
		lp.hop[worst_slot] = POOL_CH(best_pool);
		esb_stats.afh_swaps++;
	}
}
