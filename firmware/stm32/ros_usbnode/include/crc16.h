/* SPDX-License-Identifier: GPL-3.0 */
/**
 * @file crc16.h
 * @brief CRC-16 CCITT-FALSE checksum for the Mowgli packet protocol.
 *
 * Parameters:
 *   Width      : 16 bits
 *   Polynomial : 0x1021
 *   Initial    : 0xFFFF
 *   RefIn      : false  (input bytes are NOT bit-reversed)
 *   RefOut     : false  (output word is NOT bit-reversed)
 *   XorOut     : 0x0000
 *
 * This matches the crcCalc() used by the OpenMower STM32 firmware and the
 * crc16_ccitt() function in the ROS 2 bridge (mowgli_hardware/crc16.hpp).
 */

#ifndef MOWGLI_CRC16_H
#define MOWGLI_CRC16_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Compute CRC-16 CCITT-FALSE over @p len bytes starting at @p data.
 *
 * @param data Pointer to the byte buffer. Must not be NULL if len > 0.
 * @param len  Number of bytes to include in the CRC computation.
 * @return 16-bit CRC value.
 */
uint16_t crc16_ccitt(const uint8_t *data, size_t len);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* MOWGLI_CRC16_H */
