#ifndef CRC_H
#define CRC_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 无查表计算 CRC-8/SMBUS。
 * @param data 输入字节数组；仅当 length 为零时允许为 NULL。
 * @param length 输入长度，单位字节。
 * @return 多项式 0x07、初值 0x00 对应的 CRC。
 */
uint8_t CRC8_Calculate(const uint8_t *data, size_t length);

/**
 * @brief 无查表计算 CRC-16/MODBUS。
 * @param data 输入字节数组；仅当 length 为零时允许为 NULL。
 * @param length 输入长度，单位字节。
 * @return 反射多项式 0xA001、初值 0xFFFF 对应的 CRC。
 * @note Modbus RTU 发送时通常先放 CRC 低字节，再放高字节。
 */
uint16_t CRC16_Modbus(const uint8_t *data, size_t length);

#ifdef __cplusplus
}
#endif

#endif /* CRC_H */