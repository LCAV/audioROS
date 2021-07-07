#ifndef CONFIG_FLASH_STORAGE_PRIVATE_H
#define CONFIG_FLASH_STORAGE_PRIVATE_H

#ifdef __cplusplus
extern "C" {
#endif

/** Returns true if the block at the given address has a valid checksum. */
bool config_block_is_valid(void *block);

/** Writes a header for the block at the address dst.
 *
 * @note The block first bytes (CONFIG_HEADER_SIZE) must be available to write
 * the checksum to the block. */
void config_write_block_header(void *dst, uint32_t len);

/** Returns the length of the pointed block. */
uint32_t config_block_get_length(void *block);

/** Returns a pointer to the first free usable area of the block. */
void *config_block_find_first_free(void *block);

/** Returns a pointer to the last used block. */
void *config_block_find_last_used(void *p);

#ifdef __cplusplus
}
#endif

#endif
