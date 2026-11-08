# NavSight v1 scripts

The original NavSight scripts for the v1 hardware: one HC-SR04 ultrasonic sensor on GPIO 23/24,
a NEO-6M/8M GPS at 9600 baud, the Pi camera with YOLOv8, and audio alerts.

| Script | Purpose |
|---|---|
| `combined_ultrasonic_yolo.py` | ultrasonic distance alerts plus YOLO announcements |
| `Ultrasonic.py` | ultrasonic sensor with beeps, Bluetooth audio setup |
| `yolo.py` | YOLO object detection with speech |
| `rpi_gps_controller.py` | GPS dashboard and NMEA parser |
| `GPS.py` | step-by-step GPS diagnostic |

They still run on v1 hardware (`python3 v1/combined_ultrasonic_yolo.py`). NavSight v2 replaces
them with the `navsight` package in the repository root; see the main README.
