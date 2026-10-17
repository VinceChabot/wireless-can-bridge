#ifndef MESSAGE_QUEUES_H
#define MESSAGE_QUEUES_H

#include <zephyr/kernel.h>
#include <zephyr/drivers/can.h>

#define MSGQ_QUEUE_SIZE 512

/**
 * A CAN frame travelling through the bridge, with a timestamp (bridge_now_us()
 * clock of this board):
 *  - in msgq_CAN_to_ESB: when the frame was received from the CAN bus;
 *  - in msgq_ESB_to_CAN: when it should be sent on the CAN bus, which keeps the
 *    spacing the frames had on the other bus.
 */
struct bridge_frame {
	struct can_frame frame;
	uint32_t t_us;
};

// Extern declarations for message queues
extern struct k_msgq msgq_ESB_to_CAN;
extern struct k_msgq msgq_CAN_to_ESB;

/**
 * Put a frame in a queue. If the queue is full the oldest frame is dropped to
 * make room, so a stalled consumer never blocks fresh data.
 *
 * @retval true if an old frame had to be dropped
 */
static inline bool bridge_msgq_put_drop_oldest(struct k_msgq *q, const struct bridge_frame *f)
{
	if (k_msgq_put(q, f, K_NO_WAIT) == 0) {
		return false;
	}
	struct bridge_frame old;

	(void)k_msgq_get(q, &old, K_NO_WAIT);
	(void)k_msgq_put(q, f, K_NO_WAIT);
	return true;
}

#endif
