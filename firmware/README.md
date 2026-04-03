# Firmware

STM32 firmware for Mowgli robot mower — motor control, IMU, blade safety, and ROS2 serial bridge.

Forked from [cloudn1ne/Mowgli](https://github.com/cloudn1ne/Mowgli).

## Safety

> The custom firmware has no tilt sensing or emergency stop — **remove razor blades during development**.

## Structure

```
stm32/
├── ros_usbnode/        # Main firmware: ROS serial bridge over USB-CDC (STM32F103)
├── custom_panel_fw/    # Custom panel controller (STM32F0)
├── test_code/          # Hardware test/debug firmware
├── mainboard_firmware/ # Stock firmware backup/restore tools
└── panel_firmware/     # Stock panel firmware backup/restore tools
```

## Main Firmware (ros_usbnode)

The active firmware in `stm32/ros_usbnode/` runs on the STM32F103VCT6 and provides:

- Motor control (left/right wheels, blade)
- IMU reading (accelerometer, gyroscope, magnetometer)
- Battery voltage and charging state
- Rain sensor, emergency/stop buttons
- USB-CDC serial bridge to ROS2 (via COBS protocol)

### Building

Requires [PlatformIO](https://platformio.org/). Configure your board variant in `stm32/ros_usbnode/include/board.h`.

```bash
cd stm32/ros_usbnode
pio run
```

### Flashing

Flash via the GUI setup page (recommended) or with an ST-Link:

```bash
pio run --target upload
```

## Supported Hardware

- YardForce Classic 500 / 500B
- YardForce LUV1000Ri
- Adjust `board.h` for your specific model
