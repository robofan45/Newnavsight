#!/usr/bin/env python3
"""
HC-SR04 Ultrasonic Sensor with Bluetooth Audio Support
Complete solution for Raspberry Pi 5 - FIXED VERSION
"""

import lgpio
import time
import os
import subprocess
import threading
import wave
import struct
import math
import sys

# Try to import numpy, install if missing
try:
    import numpy as np
except ImportError:
    print("Installing numpy...")
    os.system("pip3 install numpy")
    import numpy as np

# GPIO pins
TRIG_PIN = 23
ECHO_PIN = 24

class AudioController:
    """Handles all audio output methods"""
    
    def __init__(self):
        self.audio_method = None
        self.bluetooth_connected = False
        self.setup_audio()
        
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
        
        # First, ensure PulseAudio is running
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
            ("wav", self.test_wav_file)
        ]
        
        for name, test_func in methods:
            print(f"Testing {name}...")
            if test_func():
                self.audio_method = name
                print(f"✓ Using {name} for audio")
                break
        
        if not self.audio_method:
            print("⚠ No audio method available - will use visual alerts only")
            print("  To fix: Install pulseaudio, sox, or alsa-utils")
    
    def ensure_pulseaudio(self):
        """Ensure PulseAudio is running"""
        try:
            # Check if PulseAudio is running
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
            # Get Bluetooth sink name
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
                # Set Bluetooth as default audio
                subprocess.run(
                    ["pactl", "set-default-sink", bluetooth_sink],
                    timeout=2
                )
                print(f"✓ Bluetooth audio sink set: {bluetooth_sink}")
                
                # Set volume
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
            # First check if paplay exists
            result = subprocess.run(
                ["which", "paplay"],
                capture_output=True, timeout=1
            )
            if result.returncode != 0:
                return False
            
            # Try to play a system sound if it exists
            test_sounds = [
                "/usr/share/sounds/alsa/Front_Center.wav",
                "/usr/share/sounds/freedesktop/stereo/bell.oga",
                "/usr/share/sounds/ubuntu/stereo/bell.ogg"
            ]
            
            for sound in test_sounds:
                if os.path.exists(sound):
                    cmd = ["paplay", sound]
                    result = subprocess.run(
                        cmd, stderr=subprocess.DEVNULL, timeout=2
                    )
                    if result.returncode == 0:
                        return True
            
            # If no system sounds, try generating one
            return self.test_pulse_generated()
        except:
            return False
    
    def test_pulse_generated(self):
        """Test PulseAudio with generated audio"""
        try:
            # Generate a short beep
            rate = 44100
            frequency = 1000
            duration = 0.1
            
            samples = np.sin(2 * np.pi * frequency * np.arange(rate * duration) / rate)
            data = (samples * 32767).astype(np.int16).tobytes()
            
            proc = subprocess.Popen(
                ['paplay', '--raw', '--rate=44100', '--format=s16le', '--channels=1'],
                stdin=subprocess.PIPE, stderr=subprocess.DEVNULL
            )
            proc.communicate(input=data, timeout=1)
            return proc.returncode == 0
        except:
            return False
    
    def test_sox(self):
        """Test sox/play command"""
        try:
            # Check if sox is installed
            result = subprocess.run(
                ["which", "play"],
                capture_output=True, timeout=1
            )
            if result.returncode != 0:
                return False
            
            cmd = ["play", "-n", "-c1", "synth", "0.1", "sine", "1000", "vol", "0.3"]
            result = subprocess.run(
                cmd, stderr=subprocess.DEVNULL, 
                stdout=subprocess.DEVNULL, timeout=2
            )
            return result.returncode == 0
        except:
            return False
    
    def test_alsa(self):
        """Test ALSA"""
        try:
            # Check if aplay exists
            result = subprocess.run(
                ["which", "aplay"],
                capture_output=True, timeout=1
            )
            if result.returncode != 0:
                return False
            
            # Create and play a test tone
            test_file = "/tmp/test_beep.wav"
            self.create_beep_wav(test_file, frequency=1000, duration=0.1)
            
            if os.path.exists(test_file):
                cmd = ["aplay", "-q", test_file]
                result = subprocess.run(cmd, stderr=subprocess.DEVNULL, timeout=2)
                os.remove(test_file)
                return result.returncode == 0
            return False
        except:
            return False
    
    def test_wav_file(self):
        """Test playing WAV file as fallback"""
        try:
            # This is basically the same as ALSA test
            return self.test_alsa()
        except:
            return False
    
    def create_beep_wav(self, filename, frequency=1000, duration=0.1, volume=0.5):
        """Create a WAV file with a beep sound"""
        try:
            sample_rate = 44100
            num_samples = int(sample_rate * duration)
            
            # Generate sine wave
            samples = []
            for i in range(num_samples):
                sample = int(volume * 32767 * math.sin(2 * math.pi * frequency * i / sample_rate))
                samples.append(struct.pack('<h', sample))
            
            # Write WAV file
            with wave.open(filename, 'wb') as wav_file:
                wav_file.setnchannels(1)  # Mono
                wav_file.setsampwidth(2)   # 2 bytes per sample
                wav_file.setframerate(sample_rate)
                wav_file.writeframes(b''.join(samples))
            return True
        except Exception as e:
            print(f"Error creating WAV: {e}")
            return False
    
    def play_beep(self, frequency=1000, duration=0.1):
        """Play beep using the best available method"""
        if not self.audio_method:
            return
        
        # Run in background thread to avoid blocking
        thread = threading.Thread(
            target=self._play_beep_internal, 
            args=(frequency, duration)
        )
        thread.daemon = True
        thread.start()
    
    def _play_beep_internal(self, frequency, duration):
        """Internal beep playing method"""
        try:
            if self.audio_method == "pulse":
                # Use PulseAudio with generated tone
                rate = 44100
                samples = np.sin(2 * np.pi * frequency * np.arange(rate * duration) / rate)
                data = (samples * 32767).astype(np.int16).tobytes()
                
                proc = subprocess.Popen(
                    ['paplay', '--raw', '--rate=44100', '--format=s16le', '--channels=1'],
                    stdin=subprocess.PIPE, stderr=subprocess.DEVNULL
                )
                proc.communicate(input=data)
                
            elif self.audio_method == "sox":
                subprocess.run([
                    "play", "-n", "-c1", "synth", str(duration), 
                    "sine", str(frequency), "vol", "0.5"
                ], stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL, timeout=2)
                
            elif self.audio_method == "alsa" or self.audio_method == "wav":
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

class UltrasonicSensorPi5:
    """Ultrasonic sensor controller for Raspberry Pi 5"""
    
    def __init__(self, trig_pin, echo_pin):
        self.trig = trig_pin
        self.echo = echo_pin
        self.chip = None
        
        # Try different chip numbers for Pi 5
        for chip_num in [4, 0]:
            try:
                self.chip = lgpio.gpiochip_open(chip_num)
                print(f"✓ Using GPIO chip {chip_num}")
                break
            except:
                continue
        
        if self.chip is None:
            raise Exception("Failed to open GPIO chip. Is lgpio installed?")
        
        # Setup pins
        lgpio.gpio_claim_output(self.chip, self.trig)
        lgpio.gpio_claim_input(self.chip, self.echo)
        lgpio.gpio_write(self.chip, self.trig, 0)
        time.sleep(2)
        print(f"✓ Sensor initialized: Trig=GPIO{trig_pin}, Echo=GPIO{echo_pin}")
    
    def ultrasonic_measure(self):
        """Single distance measurement"""
        try:
            # Send trigger pulse
            lgpio.gpio_write(self.chip, self.trig, 0)
            time.sleep(0.000002)
            
            lgpio.gpio_write(self.chip, self.trig, 1)
            time.sleep(0.00001)
            lgpio.gpio_write(self.chip, self.trig, 0)
            
            # Wait for echo start
            timeout = time.time() + 0.1
            pulse_start = time.time()
            while lgpio.gpio_read(self.chip, self.echo) == 0:
                pulse_start = time.time()
                if pulse_start > timeout:
                    return None
            
            # Wait for echo end
            timeout = time.time() + 0.1
            pulse_end = time.time()
            while lgpio.gpio_read(self.chip, self.echo) == 1:
                pulse_end = time.time()
                if pulse_end > timeout:
                    return None
            
            # Calculate distance
            pulse_duration = pulse_end - pulse_start
            distance_cm = pulse_duration * 17150
            
            # Validate reading
            if distance_cm < 2 or distance_cm > 400:
                return None
                
            return distance_cm
        except Exception as e:
            print(f"Measurement error: {e}")
            return None
    
    def get_distance_quick(self):
        """Get quick distance reading (3 samples)"""
        readings = []
        for _ in range(3):
            reading = self.ultrasonic_measure()
            if reading:
                readings.append(reading)
            time.sleep(0.01)
        
        if readings:
            # Return median for better accuracy
            readings.sort()
            return readings[len(readings)//2]
        return None
    
    def cleanup(self):
        """Clean up GPIO resources"""
        if self.chip:
            lgpio.gpiochip_close(self.chip)

class BluetoothAudioSetup:
    """Helper class for Bluetooth setup"""
    
    @staticmethod
    def interactive_setup():
        """Interactive Bluetooth setup"""
        print("\n" + "="*60)
        print("BLUETOOTH AUDIO SETUP WIZARD")
        print("="*60)
        
        # Check if Bluetooth service is running
        print("\n1. Checking Bluetooth service...")
        os.system("sudo systemctl status bluetooth --no-pager | head -5")
        
        print("\n2. Starting Bluetooth controller...")
        print("   Commands to run in bluetoothctl:")
        print("   - power on")
        print("   - agent on")
        print("   - scan on")
        print("   - pair XX:XX:XX:XX:XX:XX  (your device MAC)")
        print("   - trust XX:XX:XX:XX:XX:XX")
        print("   - connect XX:XX:XX:XX:XX:XX")
        print("   - exit")
        print("\nPress Enter to open bluetoothctl...")
        input()
        os.system("bluetoothctl")
        
        print("\n3. Setting up PulseAudio...")
        os.system("systemctl --user restart pulseaudio")
        time.sleep(2)
        
        print("\n4. Listing audio devices...")
        os.system("pactl list sinks short")
        
        print("\n5. Testing audio...")
        os.system("speaker-test -t wav -c 2 -l 1")
        
        print("\n" + "="*60)
        print("Setup complete!")
        print("="*60)

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
        "sudo apt install -y python3-lgpio python3-pip",
        "pip3 install numpy lgpio"
    ]
    
    for cmd in commands:
        print(f"\nRunning: {cmd}")
        os.system(cmd)
    
    print("\n✓ Dependencies installed!")

# Main Program
def main():
    print("\n" + "="*60)
    print("HC-SR04 ULTRASONIC SENSOR WITH BLUETOOTH AUDIO")
    print("Raspberry Pi 5 Edition - FIXED VERSION")
    print("="*60)
    
    # Check for setup options
    if len(sys.argv) > 1:
        if sys.argv[1] == "--install":
            install_dependencies()
            return
        elif sys.argv[1] == "--bluetooth":
            BluetoothAudioSetup.interactive_setup()
            return
    
    # Quick setup menu
    print("\nOptions:")
    print("1. Run sensor program")
    print("2. Setup Bluetooth audio")
    print("3. Install dependencies")
    print("\nChoice (1/2/3): ", end="")
    
    choice = input().strip()
    
    if choice == "2":
        BluetoothAudioSetup.interactive_setup()
        print("\nPress Enter to continue with sensor...")
        input()
    elif choice == "3":
        install_dependencies()
        print("\nPress Enter to continue with sensor...")
        input()
    
    # Initialize components
    print("\nInitializing components...")
    
    try:
        sensor = UltrasonicSensorPi5(TRIG_PIN, ECHO_PIN)
    except Exception as e:
        print(f"✗ Sensor initialization failed: {e}")
        print("\nTroubleshooting:")
        print("1. Check wiring: Trig->GPIO23, Echo->GPIO24")
        print("2. Install lgpio: sudo apt install python3-lgpio")
        print("3. Run as user (not root)")
        return
    
    audio = AudioController()
    
    # Test beep
    if audio.audio_method:
        print("\nTesting audio with 3 beeps...")
        for i in range(3):
            print(f"  Beep {i+1}...")
            audio.play_beep(frequency=1000 + (i * 200), duration=0.2)
            time.sleep(0.5)
        
        print("\nDid you hear the beeps? (y/n): ", end="")
        if input().lower() != 'y':
            print("\n⚠ Audio not working. Troubleshooting:")
            print("  1. Check Bluetooth speaker is on and paired")
            print("  2. Run: python3 ultrasonic_sensor_fixed.py --bluetooth")
            print("  3. Check volume on speaker and system")
            print("\nContinuing with visual alerts only...")
            audio.audio_method = None
    
    # Main monitoring loop
    last_beep_time = 0
    last_distance = None
    
    try:
        print("\n" + "="*60)
        print("STARTING ULTRASONIC MONITORING")
        print("Distance zones:")
        print("  <10cm:    DANGER - Continuous (2000Hz)")
        print("  10-20cm:  Very close - Fast (1500Hz)")
        print("  20-40cm:  Close - Medium (1200Hz)")
        print("  40-70cm:  Medium - Slow (1000Hz)")
        print("  70-100cm: Far - Very slow (800Hz)")
        print("  >100cm:   Safe - No beeping")
        print("\nPress Ctrl+C to stop")
        print("="*60 + "\n")
        
        while True:
            distance = sensor.get_distance_quick()
            
            if distance is not None:
                # Calculate change
                change = ""
                if last_distance:
                    diff = distance - last_distance
                    if abs(diff) > 2:
                        if diff > 0:
                            change = " ↗ Away"
                        else:
                            change = " ↘ Closer"
                    else:
                        change = " → Stable"
                last_distance = distance
                
                # Determine zone and beep parameters
                current_time = time.time()
                
                if distance < 10:
                    zone = "DANGER!"
                    beep_interval = 0.05
                    frequency = 2000
                    duration = 0.05
                    bar_color = "\033[91m"  # Red
                elif distance < 20:
                    zone = "VERY CLOSE"
                    beep_interval = 0.15
                    frequency = 1500
                    duration = 0.05
                    bar_color = "\033[91m"  # Red
                elif distance < 40:
                    zone = "CLOSE"
                    beep_interval = 0.3
                    frequency = 1200
                    duration = 0.08
                    bar_color = "\033[93m"  # Yellow
                elif distance < 70:
                    zone = "MEDIUM"
                    beep_interval = 0.6
                    frequency = 1000
                    duration = 0.1
                    bar_color = "\033[93m"  # Yellow
                elif distance < 100:
                    zone = "FAR"
                    beep_interval = 1.0
                    frequency = 800
                    duration = 0.1
                    bar_color = "\033[92m"  # Green
                else:
                    zone = "SAFE"
                    beep_interval = None
                    bar_color = "\033[92m"  # Green
                
                # Visual display
                bars = int(min(distance / 10, 10))
                bar_display = bar_color + "█" * bars + "░" * (10 - bars) + "\033[0m"
                
                # Clear line and print
                print(f"\rDistance: {distance:6.2f} cm [{zone:11s}] {bar_display} {change:10s}", 
                      end="", flush=True)
                
                # Play beep if needed
                if beep_interval is not None and audio.audio_method:
                    if current_time - last_beep_time >= beep_interval:
                        audio.play_beep(frequency=frequency, duration=duration)
                        last_beep_time = current_time
            else:
                print("\r⚠ No reading - Check sensor connection" + " "*30, end="", flush=True)
            
            time.sleep(0.05)  # Fast update rate
            
    except KeyboardInterrupt:
        print("\n\nStopping...")
    except Exception as e:
        print(f"\n✗ Error: {e}")
    finally:
        sensor.cleanup()
        print("✓ Cleanup complete. Goodbye!")

if __name__ == "__main__":
    main()
    
    os.system("which paplay > /dev/null 2>&1 || sudo apt-get install -y pulseaudio-utils >/dev/null 2>&1")
    
    main()
