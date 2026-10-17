#ifndef ESB_COMMUNICATION_H
#define ESB_COMMUNICATION_H

/**
 * @brief Starts the high-frequency clock (HFCLK)
 * 
 * @retval 0 if successful
 */
int hf_clock_start(void);


/**
 * @brief Initialize Enhanced Shockburst (ESB)
 * 
 * @retval 0 if successful
 */
int esb_initialize(void);


/**
 * @brief Initialise esb_tx_thread
 * 
 * @retval 0 if successful.
 */
int esb_tx_thread_init(void);


/**
 * @brief ESB transmit thread
 *
 * @param unused1 unused
 * @param unused2 unused
 * @param unused3 unused
 */
void esb_tx_thread(void *unused1, void *unused2, void *unused3);


#endif