#ifndef SDIO_H
#define SDIO_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Start the SDC module (1-bit mode).
 *
 */
void sdio_start(void);

/**
 * @brief Check the presence of the micro sd card.
 *
 * @return			The media state.
 * @retval FALSE	media not inserted.
 * @retval TRUE		media inserted.
 */
uint8_t sdio_is_present(void);

/**
 * @brief   Performs the initialization procedure on the inserted card.
 *
 * @return 		The operation status.
 * @retval 0  	operation succeeded.
 * @retval 1   	operation failed.
 *
 */
uint8_t sdio_connect(void);

/**
 * @brief   Brings the driver in a state safe for card removal.
 *
 * @return 		The operation status.
 * @retval 0  	operation succeeded.
 * @retval 1   	operation failed.
 *
 */
uint8_t sdio_disconnect(void);


#ifdef __cplusplus
}
#endif

#endif
