#!/usr/bin/env python3
"""
Combined HC-SR04 Ultrasonic Sensor and YOLOv8 Object Detection
For Raspberry Pi 5 - Runs both systems simultaneously
"""

import lgpio
import cv2
import time
import os
import subprocess
import threading
import wave
import struct
import math
import sys
import queue
from dataclasses import dataclass
from enum import Enum
from ultralytics import YOLO

# Try to import necessary libraries
try:
    import numpy as np
except ImportError:
    print("Installing numpy...")
    os.system("pip3 install numpy")
    import numpy as np

try:
    from picamera2 import Picamera2
    PICAMERA_AVAILABLE = True
except ImportError:
    PICAMERA_AVAILABLE = False
    print("Picamera2 not available—falling back to USB camera")

# GPIO pins for ultrasonic sensor
TRIG_PIN = 23
ECHO_PIN = 24

class AlertType(Enum):
    """Types of alerts that can be generated"""
    ULTRASONIC = 1
    YOLO = 2

@dataclass
class Alert:
    """Alert data structure"""
    alert_type: AlertType
    message: str
    frequency: int = 1000
    duration: float = 0.1
    priority: int = 1  # Higher number = higher priority

class AudioController:
    """Unified audio controller for both systems"""
    
    def __init__(self):
        self.audio_method = None
        self.bluetooth_connected = False
        self.audio_queue = queue.PriorityQueue()
        self.audio_thread = None
        self.running = False
        self.last_yolo_audio = 0
        self.yolo_cooldown = 3.0  # Seconds between YOLO announcements
        self.setup_audio()
        self.start_audio_processor()
        
    def check_bluetooth_connection(self):
        """Check if Bluetooth device is connected"""
        try:
            result = subprocess.run(
                ["bluetoothctl", "devices", "Connected"],
                capture_output=True, text=True, timeout=2
            )
            if result.returncode == 0 and result.stdout.strip():
                print(f"Bluetooth devices connected: {result.stdout.strip()}")
                return True
            return False
        except:
            return False
    
    def setup_audio(self):
        """Setup and detect best audio method"""
        print("Setting up audio system...")
        
        # Ensure PulseAudio is running
        self.ensure_pulseaudio()
        
        # Check Bluetooth connection
        self.bluetooth_connected = self.check_bluetooth_connection()
        if self.bluetooth_connected:
            self.setup_bluetooth()
        
        # Test different audio methods
        methods = [
            ("pulse", self.test_pulseaudio),
            ("sox", self.test_sox),
            ("alsa", self.test_alsa),
            ("espeak", self.test_espeak)
        ]
        
        for name, test_func in methods:
            print(f"Testing {name}...")
            if test_func():
                self.audio_method = name
                print(f"✔ Using {name} for audio")
                break
        
        if not self.audio_method:
            print("⚠ No audio method available - will use visual alerts only")
    
    def ensure_pulseaudio(self):
        """Ensure PulseAudio is running"""
        try:
            result = subprocess.run(
                ["pactl", "info"],
                capture_output=True, timeout=1
            )
            if result.returncode != 0:
                print("Starting PulseAudio...")
                subprocess.run(["pulseaudio", "--start"], timeout=2)
                time.sleep(1)
        except:
            pass
    
    def setup_bluetooth(self):
        """Ensure Bluetooth audio is properly configured"""
        try:
            result = subprocess.run(
                ["pactl", "list", "short", "sinks"],
                capture_output=True, text=True, timeout=2
            )
            
            bluetooth_sink = None
            for line in result.stdout.split('\n'):
                if 'bluez' in line:
                    bluetooth_sink = line.split()[1]
                    break
            
            if bluetooth_sink:
                subprocess.run(
                    ["pactl", "set-default-sink", bluetooth_sink],
                    timeout=2
                )
                print(f"✔ Bluetooth audio sink set: {bluetooth_sink}")
                subprocess.run(["amixer", "set", "Master", "80%"], 
                             stderr=subprocess.DEVNULL)
                return True
            else:
                print("⚠ Bluetooth connected but no audio sink found")
                return False
        except Exception as e:
            print(f"⚠ Bluetooth setup error: {e}")
            return False
    
    def test_pulseaudio(self):
        """Test PulseAudio"""
        try:
            result = subprocess.run(
                ["which", "paplay"],
                capture_output=True, timeout=1
            )
            return result.returncode == 0
        except:
            return False
    
    def test_sox(self):
        """Test sox/play command"""
        try:
            result = subprocess.run(
                ["which", "play"],
                capture_output=True, timeout=1
            )
            return result.returncode == 0
        except:
            return False
    
    def test_alsa(self):
        """Test ALSA"""
        try:
            result = subprocess.run(
                ["which", "aplay"],
                capture_output=True, timeout=1
            )
            return result.returncode == 0
        except:
            return False
    
    def test_espeak(self):
        """Test espeak for TTS"""
        try:
            result = subprocess.run(
                ["which", "espeak"],
                capture_output=True, timeout=1
            )
            return result.returncode == 0
        except:
            return False
    
    def start_audio_processor(self):
        """Start the audio processing thread"""
        self.running = True
        self.audio_thread = threading.Thread(target=self._process_audio_queue)
        self.audio_thread.daemon = True
        self.audio_thread.start()
    
    def _process_audio_queue(self):
        """Process audio alerts from queue"""
        while self.running:
            try:
                # Get alert with timeout to allow checking running flag
                priority, alert = self.audio_queue.get(timeout=0.1)
                
                if alert.alert_type == AlertType.ULTRASONIC:
                    self._play_beep_internal(alert.frequency, alert.duration)
                elif alert.alert_type == AlertType.YOLO:
                    self._speak_text(alert.message)
                    
            except queue.Empty:
                continue
            except Exception as e:
                print(f"Audio processing error: {e}")
    
    def _play_beep_internal(self, frequency, duration):
        """Internal beep playing method"""
        if not self.audio_method:
            return
            
        try:
            if self.audio_method == "pulse":
                rate = 44100
                samples = np.sin(2 * np.pi * frequency * np.arange(rate * duration) / rate)
                data = (samples * 32767).astype(np.int16).tobytes()
                
                proc = subprocess.Popen(
                    ['paplay', '--raw', '--rate=44100', '--format=s16le', '--channels=1'],
                    stdin=subprocess.PIPE, stderr=subprocess.DEVNULL
                )
                proc.communicate(input=data, timeout=1)
                
            elif self.audio_method == "sox":
                subprocess.run([
                    "play", "-n", "-c1", "synth", str(duration), 
                    "sine", str(frequency), "vol", "0.5"
                ], stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL, timeout=2)
                
            elif self.audio_method == "alsa":
                # Create temporary WAV file
                wav_file = f"/tmp/beep_{frequency}_{int(time.time()*1000)}.wav"
                if self.create_beep_wav(wav_file, frequency, duration):
                    subprocess.run(
                        ["aplay", "-q", wav_file], 
                        stderr=subprocess.DEVNULL, timeout=2
                    )
                    try:
                        os.remove(wav_file)
                    except:
                        pass
        except Exception as e:
            print(f"Error playing beep: {e}")
    
    def _speak_text(self, text):
        """Speak text using espeak"""
        if self.audio_method == "espeak":
            try:
                subprocess.run(
                    ["espeak", "-v", "en", text], 
                    capture_output=True, timeout=3
                )
            except:
                print(f"TTS: {text}")
        else:
            print(f"Detection: {text}")
    
    def create_beep_wav(self, filename, frequency=1000, duration=0.1, volume=0.5):
        """Create a WAV file with a beep sound"""
        try:
            sample_rate = 44100
            num_samples = int(sample_rate * duration)
            
            samples = []
            for i in range(num_samples):
                sample = int(volume * 32767 * math.sin(2 * math.pi * frequency * i / sample_rate))
                samples.append(struct.pack('<h', sample))
            
            with wave.open(filename, 'wb') as wav_file:
                wav_file.setnchannels(1)
                wav_file.setsampwidth(2)
                wav_file.setframerate(sample_rate)
                wav_file.writeframes(b''.join(samples))
            return True
        except:
            return False
    
    def add_ultrasonic_alert(self, frequency, duration, priority=5):
        """Add ultrasonic beep to queue"""
        alert = Alert(
            alert_type=AlertType.ULTRASONIC,
            message="",
            frequency=frequency,
            duration=duration,
            priority=priority
        )
        # Use negative priority for queue (lower number = higher priority)
        self.audio_queue.put((-priority, alert))
    
    def add_yolo_alert(self, message, priority=3):
        """Add YOLO detection announcement to queue"""
        current_time = time.time()
        if current_time - self.last_yolo_audio >= self.yolo_cooldown:
            alert = Alert(
                alert_type=AlertType.YOLO,
                message=message,
                priority=priority
            )
            self.audio_queue.put((-priority, alert))
            self.last_yolo_audio = current_time
    
    def cleanup(self):
        """Clean up audio resources"""
        self.running = False
        if self.audio_thread:
            self.audio_thread.join(timeout=2)

class UltrasonicSensor:
    """Ultrasonic sensor controller"""
    
    def __init__(self, trig_pin, echo_pin):
        self.trig = trig_pin
        self.echo = echo_pin
        self.chip = None
        self.running = False
        self.thread = None
        self.current_distance = None
        self.audio_controller = None
        
        # Try different chip numbers for Pi 5
        for chip_num in [4, 0]:
            try:
                self.chip = lgpio.gpiochip_open(chip_num)
                print(f"✔ Ultrasonic using GPIO chip {chip_num}")
                break
            except:
                continue
        
        if self.chip is None:
            raise Exception("Failed to open GPIO chip for ultrasonic sensor")
        
        # Setup pins
        lgpio.gpio_claim_output(self.chip, self.trig)
        lgpio.gpio_claim_input(self.chip, self.echo)
        lgpio.gpio_write(self.chip, self.trig, 0)
        time.sleep(2)
        print(f"✔ Ultrasonic sensor initialized: Trig=GPIO{trig_pin}, Echo=GPIO{echo_pin}")
    
    def ultrasonic_measure(self):
        """Single distance measurement"""
        try:
            # Send trigger pulse
            lgpio.gpio_write(self.chip, self.trig, 0)
            time.sleep(0.000002)
            
            lgpio.gpio_write(self.chip, self.trig, 1)
            time.sleep(0.00001)
            lgpio.gpio_write(self.chip, self.trig, 0)
            
            # Wait for echo
            timeout = time.time() + 0.1
            pulse_start = time.time()
            while lgpio.gpio_read(self.chip, self.echo) == 0:
                pulse_start = time.time()
                if pulse_start > timeout:
                    return None
            
            timeout = time.time() + 0.1
            pulse_end = time.time()
            while lgpio.gpio_read(self.chip, self.echo) == 1:
                pulse_end = time.time()
                if pulse_end > timeout:
                    return None
            
            # Calculate distance
            pulse_duration = pulse_end - pulse_start
            distance_cm = pulse_duration * 17150
            
            if distance_cm < 2 or distance_cm > 400:
                return None
                
            return distance_cm
        except:
            return None
    
    def start_monitoring(self, audio_controller):
        """Start ultrasonic monitoring in background thread"""
        self.audio_controller = audio_controller
        self.running = True
        self.thread = threading.Thread(target=self._monitoring_loop)
        self.thread.daemon = True
        self.thread.start()
        print("✔ Ultrasonic monitoring started")
    
    def _monitoring_loop(self):
        """Main monitoring loop for ultrasonic sensor"""
        last_beep_time = 0
        
        while self.running:
            try:
                # Get distance reading
                readings = []
                for _ in range(3):
                    reading = self.ultrasonic_measure()
                    if reading:
                        readings.append(reading)
                    time.sleep(0.01)
                
                if readings:
                    readings.sort()
                    distance = readings[len(readings)//2]
                    self.current_distance = distance
                    
                    # Determine zone and beep parameters
                    current_time = time.time()
                    
                    if distance < 10:
                        beep_interval = 0.05
                        frequency = 2000
                        duration = 0.05
                        priority = 10
                    elif distance < 20:
                        beep_interval = 0.15
                        frequency = 1500
                        duration = 0.05
                        priority = 8
                    elif distance < 40:
                        beep_interval = 0.3
                        frequency = 1200
                        duration = 0.08
                        priority = 6
                    elif distance < 70:
                        beep_interval = 0.6
                        frequency = 1000
                        duration = 0.1
                        priority = 4
                    elif distance < 100:
                        beep_interval = 1.0
                        frequency = 800
                        duration = 0.1
                        priority = 2
                    else:
                        beep_interval = None
                    
                    # Add beep to queue if needed
                    if beep_interval is not None and self.audio_controller:
                        if current_time - last_beep_time >= beep_interval:
                            self.audio_controller.add_ultrasonic_alert(
                                frequency, duration, priority
                            )
                            last_beep_time = current_time
                
                time.sleep(0.05)
                
            except Exception as e:
                print(f"Ultrasonic monitoring error: {e}")
                time.sleep(0.1)
    
    def stop_monitoring(self):
        """Stop monitoring"""
        self.running = False
        if self.thread:
            self.thread.join(timeout=2)
    
    def cleanup(self):
        """Clean up resources"""
        self.stop_monitoring()
        if self.chip:
            lgpio.gpiochip_close(self.chip)

class YOLODetector:
    """YOLO object detection controller"""
    
    def __init__(self, model_path="yolov8n.pt", camera_type="csi", 
                 frame_width=640, frame_height=480):
        self.model_path = model_path
        self.camera_type = camera_type
        self.frame_width = frame_width
        self.frame_height = frame_height
        self.running = False
        self.thread = None
        self.current_detections = []
        self.audio_controller = None
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        
        # Load YOLO model
        print(f"Loading YOLO model from '{model_path}'...")
        try:
            self.model = YOLO(model_path)
            self.model.conf = 0.5  # Confidence threshold
            print("✔ YOLO model loaded successfully")
        except Exception as e:
            print(f"✗ Error loading YOLO model: {e}")
            raise
        
        # Initialize camera
        self._init_camera()
    
    def _init_camera(self):
        """Initialize camera"""
        print(f"Initializing {self.camera_type} camera...")
        
        if self.camera_type == "csi" and PICAMERA_AVAILABLE:
            try:
                self.picam2 = Picamera2()
                config = self.picam2.create_preview_configuration(
                    main={"size": (self.frame_width, self.frame_height), "format": "RGB888"}
                )
                self.picam2.configure(config)
                self.picam2.start()
                self.cap = self.picam2
                self.is_picam = True
                print("✔ CSI camera initialized")
            except Exception as e:
                print(f"CSI camera error: {e}. Falling back to USB.")
                self.camera_type = "usb"
                self._init_usb_camera()
        else:
            self._init_usb_camera()
    
    def _init_usb_camera(self):
        """Initialize USB camera"""
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        if not self.cap.isOpened():
            raise Exception("Could not open USB camera")
        self.is_picam = False
        print("✔ USB camera initialized")
    
    def start_detection(self, audio_controller):
        """Start object detection in background thread"""
        self.audio_controller = audio_controller
        self.running = True
        self.thread = threading.Thread(target=self._detection_loop)
        self.thread.daemon = True
        self.thread.start()
        print("✔ YOLO detection started")
    
    def _detection_loop(self):
        """Main detection loop"""
        while self.running:
            try:
                # Capture frame
                if self.is_picam:
                    frame = self.cap.capture_array()
                else:
                    success, frame = self.cap.read()
                    if not success:
                        time.sleep(0.1)
                        continue
                
                # Run detection
                results = self.model(frame, imgsz=self.frame_width, verbose=False)
                
                # Store detections
                self.current_detections = []
                if len(results[0].boxes) > 0:
                    for box in results[0].boxes:
                        class_id = int(box.cls)
                        confidence = float(box.conf)
                        class_name = self.model.names[class_id]
                        self.current_detections.append({
                            'class': class_name,
                            'confidence': confidence
                        })
                    
                    # Send audio alert for highest confidence detection
                    if self.current_detections and self.audio_controller:
                        top_detection = self.current_detections[0]
                        self.audio_controller.add_yolo_alert(
                            f"Detected {top_detection['class']}"
                        )
                
                # Store annotated frame
                annotated = results[0].plot()
                with self.frame_lock:
                    self.latest_frame = annotated
                
                time.sleep(0.033)  # ~30 FPS
                
            except Exception as e:
                print(f"YOLO detection error: {e}")
                time.sleep(0.1)
    
    def get_latest_frame(self):
        """Get the latest processed frame"""
        with self.frame_lock:
            return self.latest_frame.copy() if self.latest_frame is not None else None
    
    def stop_detection(self):
        """Stop detection"""
        self.running = False
        if self.thread:
            self.thread.join(timeout=2)
    
    def cleanup(self):
        """Clean up resources"""
        self.stop_detection()
        if self.is_picam:
            self.cap.stop()
        else:
            self.cap.release()

class CombinedSystem:
    """Main system combining ultrasonic and YOLO"""
    
    def __init__(self):
        self.audio = AudioController()
        self.ultrasonic = None
        self.yolo = None
        self.running = False
    
    def initialize(self):
        """Initialize all components"""
        print("\n" + "="*60)
        print("COMBINED ULTRASONIC & YOLO DETECTION SYSTEM")
        print("Raspberry Pi 5 Edition")
        print("="*60 + "\n")
        
        # Initialize ultrasonic sensor
        try:
            self.ultrasonic = UltrasonicSensor(TRIG_PIN, ECHO_PIN)
            self.ultrasonic.start_monitoring(self.audio)
        except Exception as e:
            print(f"Warning: Ultrasonic sensor initialization failed: {e}")
            print("Continuing with YOLO only...")
        
        # Initialize YOLO detector
        try:
            self.yolo = YOLODetector()
            self.yolo.start_detection(self.audio)
        except Exception as e:
            print(f"Warning: YOLO initialization failed: {e}")
            if not self.ultrasonic:
                print("Both systems failed to initialize!")
                return False
            print("Continuing with ultrasonic only...")
        
        return True
    
    def run(self):
        """Main run loop with display"""
        if not self.initialize():
            return
        
        print("\n" + "="*60)
        print("SYSTEM RUNNING")
        print("Press 'q' to quit")
        print("="*60 + "\n")
        
        self.running = True
        prev_time = time.time()
        
        try:
            while self.running:
                # Get current data
                distance = self.ultrasonic.current_distance if self.ultrasonic else None
                detections = self.yolo.current_detections if self.yolo else []
                
                # Create status display
                status_lines = []
                
                # Ultrasonic status
                if distance is not None:
                    if distance < 10:
                        zone = "DANGER!"
                        color = "\033[91m"  # Red
                    elif distance < 20:
                        zone = "VERY CLOSE"
                        color = "\033[91m"  # Red
                    elif distance < 40:
                        zone = "CLOSE"
                        color = "\033[93m"  # Yellow
                    elif distance < 70:
                        zone = "MEDIUM"
                        color = "\033[93m"  # Yellow
                    elif distance < 100:
                        zone = "FAR"
                        color = "\033[92m"  # Green
                    else:
                        zone = "SAFE"
                        color = "\033[92m"  # Green
                    
                    bars = int(min(distance / 10, 10))
                    bar_display = color + "█" * bars + "░" * (10 - bars) + "\033[0m"
                    status_lines.append(f"Distance: {distance:6.2f} cm [{zone:11s}] {bar_display}")
                else:
                    status_lines.append("Distance: No reading")
                
                # YOLO detections
                if detections:
                    det_str = ", ".join([f"{d['class']}({d['confidence']:.2f})" 
                                        for d in detections[:3]])
                    status_lines.append(f"Objects: {det_str}")
                else:
                    status_lines.append("Objects: None detected")
                
                # Display status
                print("\r" + " | ".join(status_lines) + "     ", end="", flush=True)
                
                # Show video if YOLO is running
                if self.yolo:
                    frame = self.yolo.get_latest_frame()
                    if frame is not None:
                        # Add FPS counter
                        new_time = time.time()
                        fps = 1 / (new_time - prev_time) if new_time != prev_time else 0
                        prev_time = new_time
                        
                        # Add distance overlay
                        if distance is not None:
                            cv2.putText(frame, f"Distance: {distance:.1f} cm", 
                                      (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                                      1, (0, 255, 0), 2, cv2.LINE_AA)
                        
                        cv2.putText(frame, f"FPS: {fps:.1f}", 
                                  (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 
                                  1, (0, 255, 0), 2, cv2.LINE_AA)
                        
                        cv2.imshow("Combined Detection System", frame)
                        
                        if cv2.waitKey(1) & 0xFF == ord('q'):
                            self.running = False
                else:
                    time.sleep(0.1)
                    
        except KeyboardInterrupt:
            print("\n\nStopping...")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up all resources"""
        print("\nCleaning up...")
        
        if self.ultrasonic:
            self.ultrasonic.cleanup()
        
        if self.yolo:
            self.yolo.cleanup()
        
        self.audio.cleanup()
        cv2.destroyAllWindows()
        
        print("✔ Cleanup complete. Goodbye!")

def install_dependencies():
    """Install all required dependencies"""
    print("\n" + "="*60)
    print("INSTALLING DEPENDENCIES")
    print("="*60)
    
    commands = [
        "sudo apt update",
        "sudo apt install -y pulseaudio pulseaudio-utils",
        "sudo apt install -y bluez pulseaudio-module-bluetooth",
        "sudo apt install -y sox libsox-fmt-all",
        "sudo apt install -y alsa-utils",
        "sudo apt install -y espeak",
        "sudo apt install -y python3-lgpio python3-pip",
        "sudo apt install -y python3-opencv",
        "pip3 install numpy lgpio ultralytics opencv-python"
    ]
    
    for cmd in commands:
        print(f"\nRunning: {cmd}")
        os.system(cmd)
    
    print("\n✔ Dependencies installed!")

def main():
    """Main entry point"""
    
    # Check for command line arguments
    if len(sys.argv) > 1:
        if sys.argv[1] == "--install":
            install_dependencies()
            return
        elif sys.argv[1] == "--help":
            print("Combined Ultrasonic & YOLO Detection System")
            print("\nUsage:")
            print("  python3 combined_system.py           # Run the system")
            print("  python3 combined_system.py --install # Install dependencies")
            print("  python3 combined_system.py --help    # Show this help")
            return
    
    # Run the combined system
    system = CombinedSystem()
    system.run()

if __name__ == "__main__":
    main()
