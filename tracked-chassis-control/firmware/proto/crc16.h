#ifndef CRC16_H
#define CRC16_H

#include <stdint.h>

uint16_t crc16_modbus(const uint8_t *data, uint16_t len);
void     crc16_start(crc16_t *state);
void     crc16_feed (crc16_t *state, uint8_t byte);
uint16_t crc16_finish(crc16_t *state);

#endif
