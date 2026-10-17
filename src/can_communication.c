#include <zephyr/drivers/can.h>
#include <zephyr/logging/log.h>
#include "can_communication.h"
#include "message_queues.h"
#include "bridge_time.h"

#define CAN_RX_THREAD_STACK_SIZE 512
#define CAN_RX_THREAD_PRIORITY 2
#define CAN_TX_THREAD_STACK_SIZE 512
#define CAN_TX_THREAD_PRIORITY 2
#define POLL_CAN_ERRORS_THREAD_STACK_SIZE 1024
#define POLL_CAN_ERRORS_THREAD_PRIORITY 2 

K_THREAD_STACK_DEFINE(can_rx_thread_stack, CAN_RX_THREAD_STACK_SIZE);
K_THREAD_STACK_DEFINE(can_tx_thread_stack, CAN_TX_THREAD_STACK_SIZE);
K_THREAD_STACK_DEFINE(poll_can_errors_stack, POLL_CAN_ERRORS_THREAD_STACK_SIZE);

struct k_thread can_rx_thread_data;
struct k_thread can_tx_thread_data;
struct k_thread poll_can_errors_thread_data;

struct k_work can_state_change_work;
enum can_state current_state;
struct can_bus_err_cnt current_err_cnt;
static uint32_t can_rx_queue_overflows;
static uint32_t can_rx_frames;
static uint32_t can_tx_frames;
static uint32_t can_tx_errors;
static uint32_t can_tx_late_1ms, can_tx_late_5ms, can_tx_late_20ms;
static uint32_t can_tx_late_max_us;
static uint64_t can_tx_send_us_total;
static uint32_t can_tx_send_us_max;

/* A frame is "late" if it leaves more than this after its scheduled time. */
#define CAN_TX_LATE_THRESHOLD_US 1000
/* Longest we ever hold a frame back, whatever its timestamp says. */
#define CAN_TX_MAX_HOLD_US 200000
/* How long to wait for a free controller buffer before giving the frame up. */
#define CAN_TX_BUFFER_TIMEOUT K_MSEC(5)
static bool first_can_rx_reported;

LOG_MODULE_REGISTER(CAN, LOG_LEVEL_DBG); //(module name, maximum log level)


int can_init_interface(const struct device * can_dev)
{
    int err;

	if (!device_is_ready(can_dev)) {
		LOG_ERR("CAN device %s not ready", can_dev->name);
		return EFAULT;
	}
	LOG_INF("CAN device %s ready", can_dev->name);

	//Set CAN mode
    err = can_set_mode(can_dev, CAN_MODE_NORMAL);
	if (err != 0) {
		LOG_ERR("Error setting CAN mode on device %s [%d]", can_dev->name, err);
		return err;
	}

	//Set CAN bitrate
	err = can_set_bitrate(can_dev, 500000);
		if (err != 0) {
		LOG_ERR("Error setting CAN bitrate on device %s [%d]", can_dev->name, err);
		return err;
	}

	//Start CAN controller
    err = can_start(can_dev);
	if (err != 0) {
		LOG_ERR("Error starting CAN controller on device %s [%d]", can_dev->name, err);
		return err;
	}

	//Set CAN state change callback
	can_set_state_change_callback(can_dev, can_state_change_callback, &can_state_change_work);

	//Initialize work for CAN state change
	k_work_init(&can_state_change_work, can_state_change_work_handler);

    return 0;
}


void can_init_rx_callback(const struct device * can_dev)
{
	//Create filters for CAN message reception
	const struct can_filter filter_std = {
		.flags = 0,
		.id = 0,     // Match any ID
		.mask = 0    // Ignore ID mask
	};

	const struct can_filter filter_extd = {
		.flags = CAN_FILTER_IDE,
		.id = 0,     // Match any ID
		.mask = 0    // Ignore ID mask
	};


	//Set CAN receive callback
	can_add_rx_filter(can_dev, rx_irq_callback, NULL, &filter_std);
	can_add_rx_filter(can_dev, rx_irq_callback, NULL, &filter_extd);

}


void tx_irq_callback(const struct device *dev, int error, void *arg)
{
	static uint32_t errors;

	ARG_UNUSED(dev);

	if (error != 0) {
		errors++;
		if (errors == 1U || (errors % 1000U) == 0U) {
			LOG_ERR("CAN transmit error %d (%u so far), sender: %s", error, errors,
				(char *)arg);
		}
	}
}

void rx_irq_callback(const struct device *dev, struct can_frame *frame, void *user_data)
{
	struct bridge_frame bf = {.frame = *frame, .t_us = bridge_now_us()};

	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);

	if (IS_ENABLED(CONFIG_BRIDGE_SYNTHETIC_ISOLATE)) {
		return; /* radio-only test: ignore the real bus */
	}
	if (!first_can_rx_reported) {
		LOG_INF("First CAN frame received on local bus: id=0x%x, dlc=%u",
			frame->id, frame->dlc);
		first_can_rx_reported = true;
	}

	/* If the radio cannot keep up, drop the oldest frame, not the newest. */
	if (bridge_msgq_put_drop_oldest(&msgq_CAN_to_ESB, &bf)) {
		can_rx_queue_overflows++;
		if (can_rx_queue_overflows == 1U || (can_rx_queue_overflows % 5000U) == 0U) {
			LOG_ERR("rx_irq_callback: radio queue full, dropped %u old frames",
				can_rx_queue_overflows);
		}
	}
	can_rx_frames++;
}


void can_get_bridge_stats(const struct device *can_dev, struct can_bridge_stats *out)
{
	out->rx_frames = can_rx_frames;
	out->rx_dropped = can_rx_queue_overflows;
	out->tx_frames = can_tx_frames;
	out->tx_errors = can_tx_errors;
	out->tx_late_1ms = can_tx_late_1ms;
	out->tx_late_5ms = can_tx_late_5ms;
	out->tx_late_20ms = can_tx_late_20ms;
	out->tx_late_max_us = can_tx_late_max_us;
	out->tx_send_avg_us = can_tx_frames ? (uint32_t)(can_tx_send_us_total / can_tx_frames) : 0;
	out->tx_send_max_us = can_tx_send_us_max;
#ifdef CONFIG_CAN_STATS
	out->rx_overruns = can_stats_get_rx_overruns(can_dev);
#else
	ARG_UNUSED(can_dev);
	out->rx_overruns = 0;
#endif
}


int can_tx_thread_init(const struct device * can_dev)
{
	k_tid_t can_tx_tid;

	can_tx_tid = k_thread_create(&can_tx_thread_data, can_tx_thread_stack,
				 K_THREAD_STACK_SIZEOF(can_tx_thread_stack),
				 can_tx_thread, (void *)can_dev, NULL, NULL,
				 CAN_TX_THREAD_PRIORITY, 0, K_NO_WAIT);

	if (!can_tx_tid) {
		LOG_ERR("Error initializing can_tx_thread ");
		return EFAULT;
	}

	return 0;
}


void can_tx_thread(void *arg1, void *unused2, void *unused3)
{
	k_thread_name_set(k_current_get(), "can_tx");

	const struct device *can_dev = (const struct device *)arg1;
	struct bridge_frame bf;
	bool first_esb_can_frame_reported = false;
	int ret;

	while (1) {
		int32_t wait_us;
		int32_t late_us;

		ret = k_msgq_get(&msgq_ESB_to_CAN, &bf, K_FOREVER);
		if (ret != 0) {
			continue;
		}
		if (!first_esb_can_frame_reported) {
			LOG_INF("First CAN frame received from ESB: id=0x%x, dlc=%u",
				bf.frame.id, bf.frame.dlc);
			first_esb_can_frame_reported = true;
		}

		/* Playout: hold the frame until its scheduled time, so it leaves
		 * with the same spacing it had on the source bus.
		 */
		wait_us = bridge_us_diff(bf.t_us, bridge_now_us());
		if (wait_us > 0) {
			k_usleep(MIN(wait_us, CAN_TX_MAX_HOLD_US));
		}
		late_us = bridge_us_diff(bridge_now_us(), bf.t_us);
		if (late_us > CAN_TX_LATE_THRESHOLD_US) {
			can_tx_late_1ms++;
			can_tx_late_5ms += late_us > 5000;
			can_tx_late_20ms += late_us > 20000;
			if ((uint32_t)late_us > can_tx_late_max_us) {
				can_tx_late_max_us = late_us;
			}
		}

		/* Never block for long: if the controller has no free buffer (no node
		 * acknowledging, bus off...) give the frame up instead of stalling.
		 */
		uint32_t t_send = bridge_now_us();

		if (IS_ENABLED(CONFIG_BRIDGE_SYNTHETIC_ISOLATE)) {
			can_tx_frames++;   /* radio-only test: count it, do not touch the bus */
			continue;
		}
		ret = can_send(can_dev, &bf.frame, CAN_TX_BUFFER_TIMEOUT, tx_irq_callback,
			       "can_tx_thread_msg");
		t_send = bridge_now_us() - t_send;
		can_tx_send_us_total += t_send;
		if (t_send > can_tx_send_us_max) {
			can_tx_send_us_max = t_send;
		}
		if (ret != 0) {
			can_tx_errors++;
			if (can_tx_errors == 1U || (can_tx_errors % 500U) == 0U) {
				LOG_ERR("Failed to send ESB frame on CAN: %d (%u so far)", ret,
					can_tx_errors);
			}
		} else {
			can_tx_frames++;
		}
	}
}


int poll_can_errors_thread_init(const struct device * can_dev)
{
	k_tid_t poll_can_errors_tid;

	poll_can_errors_tid = k_thread_create(&poll_can_errors_thread_data, poll_can_errors_stack,
					K_THREAD_STACK_SIZEOF(poll_can_errors_stack),
					poll_can_errors_thread, (void *)can_dev, NULL, NULL,
					POLL_CAN_ERRORS_THREAD_PRIORITY, 0,
					K_NO_WAIT);

	if (!poll_can_errors_tid) {
		LOG_ERR("Error initializing poll_can_errors_thread ");
		return EFAULT;
	}

	return 0;
}


void poll_can_errors_thread(void *arg1, void *unused2, void *unused3)
{
	k_thread_name_set(k_current_get(), "poll_can_errors");

	const struct device *can_dev = (const struct device *)arg1;

	struct can_bus_err_cnt err_cnt = {0, 0};
	struct can_bus_err_cnt err_cnt_prev = {0, 0};
	enum can_state state_prev = CAN_STATE_ERROR_ACTIVE;
	enum can_state state;
	int err;

	while (1) {
		err = can_get_state(can_dev, &state, &err_cnt);
		if (err != 0) {
			LOG_WRN("Failed to get CAN controller state: %d", err);
			k_sleep(K_MSEC(100));
			continue;
		}

		if (err_cnt.tx_err_cnt != err_cnt_prev.tx_err_cnt ||
		    err_cnt.rx_err_cnt != err_cnt_prev.rx_err_cnt ||
		    state_prev != state) {

			err_cnt_prev.tx_err_cnt = err_cnt.tx_err_cnt;
			err_cnt_prev.rx_err_cnt = err_cnt.rx_err_cnt;
			state_prev = state;

			LOG_DBG("rx errors = %d | tx errors = %d", err_cnt.rx_err_cnt, err_cnt.tx_err_cnt);

		} else {
			k_sleep(K_MSEC(100));
		}
	}
}


void can_state_change_callback(const struct device *dev, enum can_state state,
			   struct can_bus_err_cnt err_cnt, void *user_data)
{
	struct k_work *work = (struct k_work *)user_data;

	ARG_UNUSED(dev);

	current_state = state;
	current_err_cnt = err_cnt;

	k_work_submit(work);
}


void can_state_change_work_handler(struct k_work *work)
{
	LOG_DBG("CAN state changed to:  %s", can_state_to_str(current_state));
}


char *can_state_to_str(enum can_state state)
{
	switch (state) {
	case CAN_STATE_ERROR_ACTIVE:
		return "error - active state";
	case CAN_STATE_ERROR_WARNING:
		return "error - warning state";
	case CAN_STATE_ERROR_PASSIVE:
		return "error - passive state";
	case CAN_STATE_BUS_OFF:
		return "bus-off state";
	case CAN_STATE_STOPPED:
		return "stopped state";
	default:
		return "unknown";
	}
}


char *can_msg_to_str(struct can_frame frame)
{
	static char message_data_string[80];
	static char data_buffer[64];
	char temp[8]; // Temporary buffer for one data element

	snprintf(message_data_string, sizeof(message_data_string), "ID=0x%08X - ", UNALIGNED_GET((uint32_t *)&frame.id));

	strcpy(data_buffer, "");
	for (int i = 0; i < frame.dlc; i++) {
		snprintf(temp, sizeof(temp), "[0x%02X]", UNALIGNED_GET((uint8_t *)&frame.data[i]));
		strncat(data_buffer, temp, sizeof(data_buffer) - strlen(data_buffer) - 1);
	}

	// Concatenate data_buffer into final string
	strncat(message_data_string, data_buffer, sizeof(message_data_string) - strlen(message_data_string) - 1);

	return message_data_string;
}