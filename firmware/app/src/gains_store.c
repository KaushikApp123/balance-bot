#include "gains_store.h"

#include <string.h>

#include "board.h"
#include "protocol.h"

#define STORE_MAGIC    0x42414C31u   /* "BAL1" */
#define STORE_VERSION  1u
#define STORE_SECTOR   FLASH_SECTOR_1

extern const uint8_t __config_start[];   /* from the linker script */

typedef struct {
    uint32_t magic;
    uint32_t version;
    ctrl_gains_t gains;
    uint32_t crc;      /* CRC16 in the low half, padded to a word */
} store_t;

static uint32_t crc_of(const store_t *s)
{
    return crc16_ccitt((const uint8_t *)s, offsetof(store_t, crc), 0xFFFFu);
}

bool gains_store_load(ctrl_gains_t *out)
{
    const store_t *s = (const store_t *)(const void *)__config_start;
    if (s->magic != STORE_MAGIC || s->version != STORE_VERSION) return false;
    if (s->crc != crc_of(s)) return false;
    *out = s->gains;
    return true;
}

bool gains_store_save(const ctrl_gains_t *in)
{
    store_t s;
    memset(&s, 0, sizeof s);
    s.magic = STORE_MAGIC;
    s.version = STORE_VERSION;
    s.gains = *in;
    s.crc = crc_of(&s);

    /* Sector erase stalls flash reads (and so the whole CPU) for ~250 ms.
     * Kick the watchdog first; the caller guarantees the motors are off. */
    HAL_IWDG_Refresh(&hiwdg);

    FLASH_EraseInitTypeDef er = {
        .TypeErase = FLASH_TYPEERASE_SECTORS,
        .Sector = STORE_SECTOR,
        .NbSectors = 1,
        .VoltageRange = FLASH_VOLTAGE_RANGE_3,
    };
    uint32_t bad = 0;
    bool ok = false;

    HAL_FLASH_Unlock();
    if (HAL_FLASHEx_Erase(&er, &bad) == HAL_OK) {
        ok = true;
        const uint32_t *w = (const uint32_t *)&s;
        uint32_t addr = (uint32_t)__config_start;
        for (size_t i = 0; i < sizeof s / 4u && ok; i++, addr += 4u) {
            ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, w[i]) == HAL_OK;
        }
    }
    HAL_FLASH_Lock();
    HAL_IWDG_Refresh(&hiwdg);

    ctrl_gains_t check;
    return ok && gains_store_load(&check) && memcmp(&check, in, sizeof check) == 0;
}
