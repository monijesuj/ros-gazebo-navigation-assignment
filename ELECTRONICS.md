# Sensor connection and power on Raspberry Pi 5

These answers describe a practical integration approach. Exact supply voltage, current, pinout and software support must be checked for the particular Astra and GNSS receiver selected.

## 1 Connecting the sensors

**Orbbec Astra:** use USB, rather than the Pi's CSI camera connector. Confirm the exact Astra model and an ARM64-compatible Orbbec/OpenNI SDK, USB mode and available power. Put a high-current camera on a separately powered hub when needed; avoid a hub that backfeeds the Pi. The manufacturer's [Astra specifications](https://www.orbbec.com/products/structured-light-camera/astra-series/) identify the supported interface and operating ranges for its variants.

**Ultrasonic rangefinders:** for an HC-SR04-style device, use one GPIO output for Trigger and one GPIO input for Echo. Its 5 V Echo must pass through an appropriate level shifter or resistor divider to a 3.3 V Pi input. Check whether 3.3 V Trigger meets the chosen module's input threshold; translate it if necessary. Use common signal ground. Trigger several sensors sequentially to avoid acoustic crosstalk. For reliable timing, put the pulse measurement on a small microcontroller and send ranges to the Pi over USB serial, UART or I²C; Linux userspace timing is less deterministic. A rangefinder with a native 3.3 V I²C/UART interface is another option.

**GPS/GNSS:** choose either a USB GNSS receiver or a documented 3.3 V UART receiver, crossing TX/RX and connecting signal ground. Enable the chosen Pi UART and disable any conflicting serial login console; verify its device name on Pi 5. A module's supply rating does not establish its UART logic voltage. Feed PPS to a compatible GPIO if precise time synchronization is needed; connect the correct antenna.

The [Pi hardware documentation](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html#gpio) describes the GPIO interfaces and logic levels. Never feed a 5 V signal directly into a 3.3 V GPIO input.

## 2 One battery for the system

Use a protected battery appropriate to the motors, with a correctly rated BMS, charger, main fuse close to the battery, disconnect and branch fuses. Feed the motor driver from its compatible motor supply branch. Feed electronics from separate regulated DC/DC branches so motor startup current and electrical noise do not reset the computer.

Power the Pi 5 through USB-C from a regulator/PD source that supports its requirements, normally 5.1 V at 5 A. Budget camera, GNSS, USB hub, MCU and peak CPU current independently; a regulator label alone does not provide correct USB-C signalling. A compliant lower-power supply can limit the available USB peripheral budget. Check the [official Pi power guidance](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html#power-supply).

Provide the correct regulated voltage for each sensor and the powered USB hub. Use an intentional ground layout, short power wiring, adequate decoupling, filtering and motor transient suppression. Size wiring, fuses and converters for peak current, including motor stall current; verify these with measurements. Keep a motor emergency stop that leaves the control electronics powered. Use a battery monitor and shut the Pi down cleanly before low-voltage cutoff. Battery runtime can be estimated from usable watt-hours divided by measured total average power, allowing for DC/DC losses.

## 3 Testing without ROS

- **Astra:** check `lsusb`, USB link speed and kernel logs, then run the manufacturer's viewer or an SDK sample. Verify RGB and depth frame rates, valid depth pixels, and distances to measured targets. Use `v4l2-ctl` only for interfaces actually exposed as UVC; a proprietary depth stream needs the vendor SDK.
- **Ultrasonic:** check supply voltage with a multimeter. Observe Trigger/Echo with an oscilloscope or logic analyser. Use an MCU sketch or a Pi-compatible GPIO library with edge timestamping and explicit timeouts. Compare pulse duration against known distances; nominal distance is round-trip time × speed of sound / 2. Test no-echo conditions, oblique surfaces and crosstalk between units.
- **GPS:** check the USB/UART device and configured baud rate. Read NMEA/UBX with a serial terminal or the receiver's vendor tool; `gpsd` with `cgps` is useful for supported devices. Check valid fix status, coordinates, satellite count and timestamps outdoors with a clear sky view. A readable serial stream does not by itself prove a valid position fix. Observe PPS separately if used.

Finally test all devices together under CPU load and motor startup, while checking rail voltage, resets, USB disconnects and sensor rates. That catches power and interference problems that isolated tests miss.
