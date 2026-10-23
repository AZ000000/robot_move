/* =============================================================
 * crc16.c — CRC-16/Modbus
 * 多项式 0xA001，和 Modbus RTU 完全兼容
 * ============================================================= */
#include <stdint.h>
#include "crc16.h"

uint16_t crc16_modbus(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) crc = (crc >> 1) ^ 0xA001;
            else         crc >>= 1;
        }
    }
    return crc;
}

/* 流式版本（分块喂数据）*/
void crc16_start(crc16_t *state)  { *state = 0xFFFF; }
void crc16_feed (crc16_t *state, uint8_t byte)
{
    *state ^= byte;
    for (int j = 0; j < 8; j++) {
        if (*state & 1) *state = (*state >> 1) ^ 0xA001;
        else            *state >>= 1;
    }
}
uint16_t crc16_finish(crc16_t *state) { return *state; }
