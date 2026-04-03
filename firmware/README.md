# Mowgli

<p align="center"><img src="./images/gforce.jpg" width="75%"/></p>
<br>
<h2 id="">
  About
</h2>

This repo is forked of https://github.com/cloudn1ne/Mowgli to folllow his great job. \
It should work on the Yardforce Classic 500 and LUV1000RI \
*Please adjust the configuration in [board.h](./stm32/ros_usbnode/include/board.h)*

The documentation on this fork is mostly not up to date - we will try to update it.

The current way to build a mowgli mower is to use [mowgli-docker](https://github.com/cedbossneo/mowgli-docker) and go to the Setup page in the GUI to flash your mainboard & gps (be sure your stlink is connected to your pi).

<br>
<br>
<br>
<h2 id="safety">
  Safety
</h2>
<br>
<table border=1>
   <tr>
      <td>
The custom firmware has no protections - no tilt sensing, no stop buttons will work. If you stick your finger in the wrong place, you will lose it.<p>
         <ul>
<li>Remove the razor blades
<li>Dont be stupid
<li> Dont blame me for your lost finger
         </ul>
      </td>
      <td>
         <img src="./images/finger.png" align="right" width="150px"/>
      </td>
   </tr>
</table>
<br>
<br>
<br>
<h1 id="">
  Every below this is not 100% up to date...
</h1>
<br>
<h2 id="">
  Whats done ?
</h2>
<br>

- [Basic overview of the mainboard](./Kicad), [Datasheets](./Datasheets) for ICs
- Motor Drivers for both Drive Motors and Blade Motor can be controlled
- Mainboard (GForce) Firmware [dump and restore](./stm32/mainboard_firmware) via an ST Link or other (e.g. JLink Segger) tools
- Panel (GForce) Firmware [dump and restore](./stm32/panel_firmware) via an ST Link or other (e.g. JLink Segger) tools
- [Demo Python code](./playground/) to control the Mower via a Joystick (needs "test_code" STM32 code flashed)
- [ROS Serial node](./stm32/ros_usbnode) via CDC USB Port with full hardware support
- Serial debugging on the unused (red) J18 header
- Software I2C on the unusued (red) J18 header - used for external IMU
- [Raspi, IMU, GPS Mounting Options](./Mounting)
- onboard Buzzer
- onboard IMU (accelerometer) for tilt protection
- Safety switches (Hall Sensors) for STOP button
- Rain Sensor

<br>
<h2 id="todo">
  TODO
</h2>
<br>

Do the correct topics to remove the mowgli proxy in openmower ros and so be plug & play with openmower.

<br>
<h2 id="updates">
  Updates
</h2>

see [Updates](UPDATES.md)

<br>
<h2 id="">
  rosserial node
</h2>
<br>

This is the software that needs to be compiled and flashed onto the STM32 on the Geforce Mainboard

- [See here how to use the ROS serial node](./stm32/ros_usbnode)


### in case a supported I2C IMU is connected to J18

Currently supported IMUs:
- [Pololu AltIMU 10v5](https://www.pololu.com/product/2739) other LSM6DS33 or LSM6DSO based should work too
- [WT901]()
- [MPU6050]()

Don't hesisate to ask if you want use another IMU

Check in board.h for the connected PIN, as I used PB3 for debug purposes, the pinning is different as the original repo.

## Published Topics

### Services

- /mowgli/MowerControlSrv - enabled/disable blade
- /mowgli/GetCfg - read SPI flash config var
- /mowgli/SetCfg - write SPI flash config var
- /mowgli/Reboot - reboot Mowgli
- /mowgli/ResetEmergency - Reset emergency state
- /mowgli/SetLed - enable LED (1-17), if the MSB is set (128), the bot will chirp too
- /mowgli/ClrLed - disabled LED (1-17), if the MSB is set (128), the bot will chirp too

### Subscribers

- /cmd_vel - geometry_msgs::Twist (compatible with teleop twist messages, so you can drive the bot)

## Serial Debugging

[Serial Debug for ros_usbnode](stm32/ros_usbnode#serial_debug)
