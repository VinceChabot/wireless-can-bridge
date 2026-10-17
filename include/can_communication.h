#ifndef CAN_COMMUNICATION_H
#define CAN_COMMUNICATION_H

#include <zephyr/device.h>
#include <zephyr/drivers/can.h>


/**
 * @brief Initialise CAN interface
 *
 * @param can_dev Pointer to the device structure for the driver instance.
 * 
 * @retval 0 if successful.
 */
int can_init_interface(const struct device * can_dev);


/** 
 * @brief Function to add reception filters to a CAN driver instance.
 * Messages matching these filters will call the rx callback.
 *
 * @param dev Pointer to the device structure for the driver instance.
 */
void can_init_rx_callback(const struct device * can_dev);


/** 
 * @brief CAN transmit callback
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param error 
 * @param arg 
 */
void tx_irq_callback(const struct device *dev, int error, void *arg);


/** 
 * @brief CAN receive callback
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param frame Pointer to received CAN frame
 * @param user_data
 */
void rx_irq_callback(const struct device *dev, struct can_frame *frame, void *user_data);


/**
 * @brief Initialise can_tx_thread
 * 
 * @param can_dev Pointer to the device structure for the driver instance.
 * 
 * @retval 0 if successful.
 */
int can_tx_thread_init(const struct device * can_dev);


/**
 * @brief CAN transmit thread
 *
 * @param arg1 Pointer to the device structure for the driver instance.
 * @param unused2 unused
 * @param unused3 unused
 */
void can_tx_thread(void *arg1, void *unused2, void *unused3);


/**
 * @brief Initialise poll_can_errors_thread
 * 
 * @param can_dev Pointer to the device structure for the driver instance.
 * 
 * @retval 0 if successful.
 */
int poll_can_errors_thread_init(const struct device * can_dev);


/**
 * @brief poll can errors thread
 *
 * @param arg1 Pointer to the device structure for the driver instance.
 * @param arg2 unused
 * @param arg3 unused
 */
void poll_can_errors_thread(void *unused1, void *unused2, void *unused3);


/**
 * @brief Defines the CAN state change callback handler function signature
 *
 * @param dev       Pointer to the device structure for the driver instance.
 * @param state     State of the CAN controller.
 * @param err_cnt   CAN controller error counter values.
 * @param user_data User data provided the callback was set.
 */
void can_state_change_callback(const struct device *dev, enum can_state state,
			                struct can_bus_err_cnt err_cnt, void *user_data);


/**
 * @brief Work handler for the CAN state change
 *
 * @param work Pointer to the work
 */
void can_state_change_work_handler(struct k_work *work);


/**
 * @brief Returns corresponding string of can_state enum value
 *
 * @param state enum value
 * 
 * @retval string of can_state enum
 */
char *can_state_to_str(enum can_state state);


/**
 * @brief Returns string with message ID and data
 *
 * @param can_frame
 * 
 * @retval string with message ID and data
 */
char *can_msg_to_str(struct can_frame frame);



/** Frame counters for throughput/loss diagnostics. */
struct can_bridge_stats {
	uint32_t rx_frames;      /* received from the bus and queued for the radio */
	uint32_t rx_dropped;     /* old frames dropped because the radio queue was full */
	uint32_t tx_frames;      /* frames from the radio sent on the bus */
	uint32_t tx_errors;      /* frames that could not be sent (no free buffer, bus off...) */
	uint32_t tx_late_1ms;    /* frames sent more than 1 ms after their scheduled time */
	uint32_t tx_late_5ms;    /* ...more than 5 ms */
	uint32_t tx_late_20ms;   /* ...more than 20 ms */
	uint32_t tx_late_max_us; /* worst lateness seen */
	uint32_t tx_send_avg_us; /* average time can_send() takes (SPI transfers to the controller) */
	uint32_t tx_send_max_us; /* ...and the worst */
	uint32_t rx_overruns;    /* controller RX buffer overruns (frames lost in hardware) */
};

void can_get_bridge_stats(const struct device *can_dev, struct can_bridge_stats *out);

#endif