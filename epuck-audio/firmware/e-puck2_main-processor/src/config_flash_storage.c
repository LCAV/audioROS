#include <string.h>
#include "config_flash_storage.h"
#include "config_flash_storage_private.h"
#include "flash/flash.h"
#include "parameter/parameter_msgpack.h"
#include "cmp/cmp.h"
#include "cmp_mem_access/cmp_mem_access.h"
#include "crc/crc32.h"

/* We cannot use a CRC start value of 0 because CRC(0, 0xffffffff) = 0xffffffff
 * which makes empty flash pages valid. */
#define CRC_INITIAL_VALUE 0xdeadbeef


static size_t cmp_flash_writer(struct cmp_ctx_s *ctx, const void *data, size_t len)
{
    cmp_mem_access_t *mem = (cmp_mem_access_t*)ctx->buf;
    if (mem->index + len <= mem->size) {
        flash_write(&mem->buf[mem->index], data, len);
        mem->index += len;
        return len;
    } else {
        return 0;
    }
}

void config_erase(void *dst)
{
    flash_unlock();
    flash_sector_erase(dst);
    flash_lock();
}

static void err_mark_false(void *arg, const char *id, const char *err)
{
    (void) id;
    (void) err;

    bool *b = (bool *)arg;
    *b = false;
}

void config_save(void *dst, size_t dst_len, parameter_namespace_t *ns)
{
    cmp_ctx_t cmp;
    cmp_mem_access_t mem;
    uint32_t len;
    bool success = true;

    void *orig_dst = dst;

    flash_unlock();

    /* If there is no valid block, erase flash just to start from a pristine
     * state. */
    if (config_block_find_last_used(dst) == NULL) {
        flash_sector_erase(dst);
    }

    /* Find first available flash block. */
    dst = config_block_find_first_free(dst);

    len = dst_len - (dst - orig_dst);

    /* If the destination is too small to fit even the header, erase the block. */
    if (len <= CONFIG_HEADER_SIZE) {
        flash_sector_erase(orig_dst);
        dst = orig_dst;
        len = dst_len;
    }

    cmp_mem_access_init(&cmp, &mem,
                        dst + CONFIG_HEADER_SIZE, len - CONFIG_HEADER_SIZE);

    /* Replace the RAM writer with the special writer for flash. */
    cmp.write = cmp_flash_writer;

    /* Tries to write the config. If there is an error the callback will set
     * success to false. */
    parameter_msgpack_write_cmp(ns, &cmp, err_mark_false, &success);

    if (success == false) {
        flash_sector_erase(orig_dst);
        flash_lock();
        return config_save(orig_dst, dst_len, ns);
    }

    len = cmp_mem_access_get_pos(&mem);

    config_write_block_header(dst, len);

    flash_lock();
}

bool config_load(parameter_namespace_t *ns, void *src)
{
    int res;
    uint32_t src_len;

    src = config_block_find_last_used(src);

    /* If no valid block was found signal an error. */
    if (src == NULL) {
        return false;
    }

    src_len = config_block_get_length(src);

    res = parameter_msgpack_read(ns, src + CONFIG_HEADER_SIZE, src_len,
                                 NULL, NULL);

    if (res != 0) {
        return false;
    }

    return true;
}

bool config_block_is_valid(void *p)
{
    uint8_t *block = (uint8_t *)p;
    uint32_t crc, length;
    size_t offset = 0;

    /* Extract length checksum. */
    memcpy(&crc, &block[offset], sizeof(crc));
    offset += sizeof(crc);

    /* Extract length. */
    memcpy(&length, &block[offset], sizeof(length));
    offset += sizeof(length);

    /* Check that the length is valid. */
    if (crc != crc32(CRC_INITIAL_VALUE, &length, sizeof(length))) {
        return false;
    }

    /* Extract data checksum. */
    memcpy(&crc, &block[offset], sizeof(crc));
    offset += sizeof(crc);

    /* Check that the data checksum is valid. */
    if (crc != crc32(CRC_INITIAL_VALUE, &block[offset], length)) {
        return false;
    }

    return true;
}

void config_write_block_header(void *dst, uint32_t len)
{
    uint32_t crc;
    size_t offset = 0;

    /* First write length checksum. */
    crc = crc32(CRC_INITIAL_VALUE, &len, sizeof(uint32_t));
    flash_write(dst + offset, &crc, sizeof(uint32_t));
    offset += sizeof(uint32_t);

    /* Then write the length itself. */
    flash_write(dst + offset, &len, sizeof(uint32_t));
    offset += sizeof(uint32_t);

    /* Then write the data checksum. */
    crc = crc32(CRC_INITIAL_VALUE, dst + CONFIG_HEADER_SIZE, len);
    flash_write(dst + offset, &crc, sizeof(uint32_t));
}

uint32_t config_block_get_length(void *block)
{
    uint32_t *header = (uint32_t *) block;

    return header[1];
}

void *config_block_find_last_used(void *p)
{
    uint8_t *block = (uint8_t *)p;
    uint8_t *last = NULL;

    while (config_block_is_valid(block)) {
        last = block;
        block += config_block_get_length(block);
        block += CONFIG_HEADER_SIZE;
    }

    return last;
}

void *config_block_find_first_free(void *p)
{
    uint8_t *block = (uint8_t *)p;

    while (config_block_is_valid(block)) {
        block += config_block_get_length(block);
        block += CONFIG_HEADER_SIZE;
    }

    return block;
}
