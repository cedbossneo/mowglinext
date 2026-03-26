/* SPDX-License-Identifier: GPL-3.0 */
/**
 * @file crc16.c
 * @brief CRC-16 CCITT-FALSE implementation (no lookup table).
 *
 * A byte-at-a-time, bit-by-bit computation is used deliberately. On an
 * STM32F103 with 256 KB Flash a 512-byte lookup table would consume ~0.2 % of
 * the total Flash budget with minimal throughput benefit at the packet rates
 * used here (< 100 Hz). The bit-loop approach is 16 instructions per byte,
 * fully deterministic, and leaves Flash free for application code.
 *
 * Algorithm (CRC-16/CCITT-FALSE):
 *   crc = 0xFFFF
 *   for each byte b:
 *     crc ^= (b << 8)
 *     for bit in 0..7:
 *       if crc & 0x8000:
 *         crc = (crc << 1) ^ 0x1021
 *       else:
 *         crc = crc << 1
 *   return crc & 0xFFFF
 */

#include "crc16.h"

uint16_t crc16_ccitt(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFFu;

    for (size_t i = 0u; i < len; ++i) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8u);

        for (uint8_t bit = 0u; bit < 8u; ++bit) {
            if (crc & 0x8000u) {
                crc = (uint16_t)((crc << 1u) ^ 0x1021u);
            } else {
                crc = (uint16_t)(crc << 1u);
            }
        }
    }

    return crc;
}
