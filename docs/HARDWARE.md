# NavSight v2 hardware setup

How to wire the V2 hardware to a Raspberry Pi 5 and prepare Raspberry Pi OS (Bookworm) for it.
Pin numbers are BCM GPIO numbers, matching the V2 architecture document.

## Wiring

| Device | Device pin | Pi 5 pin |
|---|---|---|
| VL53L1X #1 (forward) | VIN, GND, SDA, SCL | 3.3 V, GND, GPIO 2, GPIO 3 |
| VL53L1X #2 (down) | VIN, GND, SDA, SCL, XSHUT | 3.3 V, GND, GPIO 2, GPIO 3, **GPIO 25** |
| BNO085 | VIN, GND, SDA, SCL | 3.3 V, GND, GPIO 2, GPIO 3 |
| INA219 | VCC, GND, SDA, SCL | 3.3 V, GND, GPIO 2, GPIO 3 |
| INA219 | VIN+, VIN- | in series with the 5 V line from the power bank (high side) |
| NEO-M9N | TX, RX, VCC, GND | GPIO 15 (RX), GPIO 14 (TX), 3.3 V, GND |
| Mode / Action / SOS buttons | one leg each, other leg to GND | GPIO 17 / GPIO 22 / GPIO 27 |
| MAX98357A | BCLK, LRC, DIN, VIN, GND | GPIO 18, GPIO 19, GPIO 21, 5 V, GND |
| Haptic motor driver (left / right) | logic input | GPIO 12 / GPIO 13 |
| Pi Camera Module 3 Wide | ribbon | CAM/DISP connector |

Notes:

- **Both VL53L1X boot at address 0x29.** Only the down sensor's XSHUT is wired; at start-up
  NavSight holds it in reset, moves the forward sensor out of the way, then gives the down sensor
  0x30 and the forward sensor back 0x29. Addresses reset at power-off, so this runs every start.
- **ERM motors draw far more current than a GPIO pin can supply.** Drive each motor through a
  logic-level N-MOSFET (or a DRV2605L/ULN2003-style driver) with a flyback diode across the motor.
  The GPIO only switches the transistor.
- **Buttons** are active-low with the Pi's internal pull-ups: no resistors needed.
- **Stereo bone-conduction audio** needs two MAX98357A boards (one with SD_MODE set for the left
  channel, one for the right) or a stereo I2S DAC. A single MAX98357A plays mono, so spatial
  panning then has no effect.
- **INA219** measures the 5 V output of the power bank, so voltage cannot show how full the bank
  is. NavSight counts the charge drawn instead; run `navsight --battery-full` after each charge.
- **GPS antenna**: keep the NEO-M9N's patch antenna facing the sky and away from the Pi's Wi-Fi.

## Raspberry Pi OS configuration

Add to `/boot/firmware/config.txt`, then reboot:

```ini
# I2C for the ToF pair, IMU and power monitor (400 kHz)
dtparam=i2c_arm=on
dtparam=i2c_arm_baudrate=400000

# UART on GPIO 14/15 for the GPS (and no serial console on it)
enable_uart=1
dtparam=uart0=on

# I2S audio DAC on GPIO 18/19/21
dtparam=audio=off
dtoverlay=max98357a

# Camera Module 3 Wide
camera_auto_detect=1
```

Free the UART from the login console:

```bash
sudo raspi-config nonint do_serial_cons 1
sudo raspi-config nonint do_serial_hw 0
```

Check the buses after the reboot:

```bash
sudo i2cdetect -y 1            # expect 29, 40 and 4a (30 appears once NavSight has run)
sudo cat /dev/serial0          # NMEA sentences such as $GNGGA from the NEO-M9N
aplay -l                       # the MAX98357A card
rpicam-hello --list-cameras    # the Camera Module 3 Wide
```

The NEO-M9N talks at **38400 baud** out of the box (the v1 NEO-8M used 9600). If yours was
reconfigured, set `"gps": {"baud_rate": ...}` in your config file.

## Software

```bash
sudo apt update
sudo apt install -y python3-venv python3-picamera2 python3-lgpio espeak-ng libportaudio2 i2c-tools
git clone https://github.com/robofan45/Newnavsight.git
cd Newnavsight
python3 -m venv --system-site-packages .venv   # sees the apt picamera2 and lgpio
.venv/bin/pip install -r requirements.txt
.venv/bin/pip install -e .
```

YOLO runs about twice as fast on the Pi 5 CPU when exported to NCNN:

```bash
.venv/bin/yolo export model=yolov8n.pt format=ncnn imgsz=416
```

Then set `"detector": {"model_path": "yolov8n_ncnn_model"}`. Ultralytics does not use the
Pi 5's VideoCore GPU, so the 15 FPS target depends on a small model and image size.
MiDaS depth is optional and heavy (PyTorch): leave `"depth": {"enabled": false}` unless you need
distances for camera-only objects.

## First run

```bash
cp config/navsight.example.json ~/navsight.json   # edit as needed
.venv/bin/navsight --config ~/navsight.json --battery-full   # after charging the power bank
.venv/bin/navsight --config ~/navsight.json
```

To start on boot, install the service in `systemd/navsight.service` (edit the user and paths).

## Calibration

- **IMU heading**: set `imu.forward_axis` to the BNO085 axis that points along the cane towards
  its tip, and `imu.heading_offset_deg` to your local magnetic declination. Wave the cane in a
  figure eight once after power-up so the BNO085 calibrates its magnetometer.
- **Down sensor**: if the down ToF is not mounted along the cane axis, set
  `hazard.down_sensor_offset_deg` to the angle between them.
- **Step length**: `pose.step_length_m` (default 0.65 m) sets dead-reckoning distance indoors.
