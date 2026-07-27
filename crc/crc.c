#include "crc.h"

uint8_t CRC8_Calculate(const uint8_t *data, size_t length)
{
    uint8_t crc = 0U;
    size_t index;
    uint8_t bit;

    if ((data == NULL) && (length > 0U)) {
        return 0U;
    }

    /* 无查表逐位实现：节省 Flash，适合短报文和低频调用。 */
    for (index = 0U; index < length; ++index) {
        crc ^= data[index];
        for (bit = 0U; bit < 8U; ++bit) {
            if ((crc & 0x80U) != 0U) {
                crc = (uint8_t)((uint8_t)(crc << 1U) ^ 0x07U);
            } else {
                crc = (uint8_t)(crc << 1U);
            }
        }
    }
    return crc;
}

uint16_t CRC16_Modbus(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFFU;
    size_t index;
    uint8_t bit;

    if ((data == NULL) && (length > 0U)) {
        return 0U;
    }

    /* Modbus CRC 按最低有效位优先处理，发送时通常低字节在前。 */
    for (index = 0U; index < length; ++index) {
        crc ^= (uint16_t)data[index];
        for (bit = 0U; bit < 8U; ++bit) {
            if ((crc & 0x0001U) != 0U) {
                crc = (uint16_t)((crc >> 1U) ^ 0xA001U);
            } else {
                crc >>= 1U;
            }
        }
    }
    return crc;
}
