#include <stdio.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>

#include "message_queues.h"
#include "can_communication.h"
#include "esb_communication.h"

LOG_MODULE_REGISTER(Main, LOG_LEVEL_DBG); //(module name, maximum log level)

K_MSGQ_DEFINE(msgq_CAN_to_ESB, sizeof(struct bridge_frame), MSGQ_QUEUE_SIZE, 4);
K_MSGQ_DEFINE(msgq_ESB_to_CAN, sizeof(struct bridge_frame), MSGQ_QUEUE_SIZE, 4);

const struct device *const can_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));

int main(void)
{
	int err;

	LOG_DBG("Test log 7");

	//Initialize CAN
	int can_init_status = can_init_interface(can_dev);
	printk("CAN init result=%d, device ready=%d\n", can_init_status,
	       device_is_ready(can_dev));
	can_init_rx_callback(can_dev);

	//Initialize CAN threads
	can_tx_thread_init(can_dev);
	poll_can_errors_thread_init(can_dev);


	//Initialize Enhanced ShockBurst (ESB)
	err = hf_clock_start();
	if (err) {
		LOG_ERR("HF clock initialization failed: %d", err);
		return err;
	}
	err = esb_initialize();
	if (err) {
		LOG_ERR("ESB initialization failed: %d", err);
		return err;
	}

	//Initialise ESB threads
	err = esb_tx_thread_init();
	if (err) {
		LOG_ERR("ESB thread initialization failed: %d", err);
		return err;
	}

	LOG_INF("Finished initialization");

	return 0;
}
