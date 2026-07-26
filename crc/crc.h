#ifndef CRC_H
#define CRC_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* CRC-8/SMBUS: polynomial 0x07, initial value 0x00. */
uint8_t CRC8_Calculate(const uint8_t *data, size_t length);

/* CRC-16/MODBUS: reflected polynomial 0xA001, initial value 0xFFFF. */
uint16_t CRC16_Modbus(const uint8_t *data, size_t length);

#ifdef __cplusplus
}
#endif

#endif /* CRC_H */
