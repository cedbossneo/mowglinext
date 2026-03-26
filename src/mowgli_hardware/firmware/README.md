# Mowgli Firmware COBS Protocol

Replacement for `rosserial` in the Mowgli STM32F103 firmware.
These files implement the same binary protocol already used by the ROS 2 bridge
in `mowgli_hardware/include/mowgli_hardware/`.

---

## Files

| File | Purpose |
|------|---------|
| `mowgli_protocol.h` | Packet IDs, bitmask constants, and packed wire-format structs (shared between firmware and bridge) |
| `cobs.h / cobs.c` | Consistent Overhead Byte Stuffing encoder/decoder |
| `crc16.h / crc16.c` | CRC-16 CCITT-FALSE checksum (no lookup table, Flash-friendly) |
| `mowgli_comms.h / mowgli_comms.c` | Communication manager: RX frame assembly, COBS decode, CRC verify, handler dispatch, TX encode and transmit |

---

## Protocol Format

Every packet follows this wire format:

```
[0x00] [COBS-encoded payload] [0x00]
         ^^^^^^^^^^^^^^^^^^^^^^^^^
         payload = packed struct bytes (type + fields + CRC-16)
```

- `0x00` is the frame delimiter — it never appears inside the COBS payload.
- The CRC covers all bytes from the `type` field up to (but not including) the
  two CRC bytes at the end of the struct.
- All multi-byte fields are little-endian (native on Cortex-M3 and x86/ARM64).

### Packet flow

```
Firmware (STM32)                         Host (Raspberry Pi / ROS 2 bridge)
     |                                              |
     |-- PKT_ID_STATUS (0x01) ------------------>  |  LlStatus
     |-- PKT_ID_IMU    (0x02) ------------------>  |  LlImu
     |-- PKT_ID_UI_EVENT (0x03) --------------->   |  LlUiEvent
     |                                              |
     |<-- PKT_ID_HEARTBEAT   (0x42) -------------|  LlHeartbeat
     |<-- PKT_ID_HL_STATE    (0x43) -------------|  LlHighLevelState
     |<-- PKT_ID_CMD_VEL     (0x50) -------------|  LlCmdVel
```

---

## Integration Steps

### 1. Copy files into the firmware project

```
cp mowgli_protocol.h  <your_project>/Inc/
cp cobs.h             <your_project>/Inc/
cp cobs.c             <your_project>/Src/
cp crc16.h            <your_project>/Inc/
cp crc16.c            <your_project>/Src/
cp mowgli_comms.h     <your_project>/Inc/
cp mowgli_comms.c     <your_project>/Src/
```

### 2. Add source files to the build

In the STM32CubeIDE project or Makefile, add `cobs.c`, `crc16.c`, and
`mowgli_comms.c` to the `C_SOURCES` list.

### 3. Provide `usb_cdc_transmit()`

`mowgli_comms.h` declares `usb_cdc_transmit()` as an `extern`. Wire it to the
existing `CDC_Transmit()` in `usbd_cdc_if.c` by adding a thin wrapper in your
project:

```c
/* In usbd_cdc_if.c or a new glue file */
#include "mowgli_comms.h"

void usb_cdc_transmit(const uint8_t *buf, size_t len)
{
    CDC_Transmit((void *)buf, (uint32_t)len);
}
```

### 4. Initialise the comms layer

In `main.c`, after USB initialisation and before enabling the USB interrupt:

```c
#include "mowgli_comms.h"

/* ... */
MX_USB_DEVICE_Init();
mowgli_comms_init();
```

### 5. Wire up the USB CDC receive callback

Replace (or extend) `CDC_DataReceivedHandler` in `usbd_cdc_if.c`:

```c
uint8_t CDC_DataReceivedHandler(const uint8_t *Buf, uint32_t len)
{
    mowgli_comms_process_rx(Buf, (size_t)len);
    return CDC_RX_DATA_HANDLED;  /* prevents double-queuing */
}
```

### 6. Register handlers for host-to-firmware packets

```c
static void on_heartbeat(const uint8_t *data, size_t len)
{
    const pkt_heartbeat_t *hb = (const pkt_heartbeat_t *)data;
    if (hb->emergency_requested) {
        Emergency_Assert();
    } else if (hb->emergency_release_requested) {
        Emergency_Release();
    }
    watchdog_feed();  /* reset the heartbeat timeout */
}

static void on_cmd_vel(const uint8_t *data, size_t len)
{
    const pkt_cmd_vel_t *cmd = (const pkt_cmd_vel_t *)data;
    DriveMotors_SetVelocity(cmd->linear_x, cmd->angular_z);
}

/* In main init sequence, after mowgli_comms_init(): */
mowgli_comms_register_handler(PKT_ID_HEARTBEAT, on_heartbeat);
mowgli_comms_register_handler(PKT_ID_CMD_VEL,   on_cmd_vel);
mowgli_comms_register_handler(PKT_ID_HL_STATE,  on_hl_state);
```

### 7. Send packets from the firmware main loop

```c
/* Example: send status at 25 Hz */
pkt_status_t status = {
    .type             = PKT_ID_STATUS,
    .status_bitmask   = STATUS_BIT_INITIALIZED | STATUS_BIT_RASPI_POWER,
    .emergency_bitmask = 0,
    .v_charge         = ADC_ChargeVoltage(4),
    .v_system         = ADC_BatteryVoltage(4),
    .charging_current = ADC_ChargeCurrent(4),
    .batt_percentage  = Battery_GetPercent(),
    /* crc is computed automatically by mowgli_comms_send_status() */
};
for (uint8_t i = 0; i < MOWGLI_USS_COUNT; ++i) {
    status.uss_ranges_m[i] = USS_GetRange(i);
}
mowgli_comms_send_status(&status);
```

---

## Memory Budget (STM32F103)

| Item | Size |
|------|------|
| RX accumulation buffer (`s_rx_buf`) | 512 bytes RAM |
| COBS decode scratch (`s_decode_buf`) | 64 bytes RAM |
| Handler table (`s_handlers`, 8 entries) | ~40 bytes RAM |
| TX stack frame in `mowgli_comms_send()` | ~192 bytes stack (transient) |
| `cobs.c` + `crc16.c` code | < 400 bytes Flash (estimated) |
| `mowgli_comms.c` code | < 1 KB Flash (estimated) |

Total static RAM: ~620 bytes (< 1 % of 64 KB).

---

## Diagnostic Counters

```c
uint32_t overflows  = mowgli_comms_get_rx_overflow_count();
uint32_t crc_errors = mowgli_comms_get_crc_error_count();
```

Both counters increment monotonically from `mowgli_comms_init()`. Monitor
them via a debug UART or expose them in a future `PKT_ID_DIAG` packet.
