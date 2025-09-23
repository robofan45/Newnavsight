#!/usr/bin/env python3
"""
HC-SR04 Ultrasonic Sensor with Bluetooth Audio Support
Complete solution for Raspberry Pi 5
"""

import lgpio
import time
import os
import subprocess
import threading
import wave
import struct
import math

# GPIO pins
TRIG_PIN = 23
ECHO_PIN = 24

class AudioController:
    """Handles all audio output methods"""
    
    def __init__(self):
        self.audio_method = None
        self.setup_audio()
        
    def setup_audio(self):
        """Setup and detect best audio method"""
        print("Setting up audio system...")
        
        # First, ensure Bluetooth is connected
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
    
    def setup_bluetooth(self):
        """Ensure Bluetooth audio is properly configured"""
        try:
            # Set Bluetooth as default audio
            subprocess.run([
                "pacmd", "set-default-sink",
                "$(pacmd list-sinks | grep -o 'bluez.*[>]' | sed 's/[<>]//g')"
            ], shell=True, stderr=subprocess.DEVNULL)
            
            # Set volume
            subprocess.run(["amixer", "set", "Master", "80%"], 
                         stderr=subprocess.DEVNULL)
            
            return True
        except:
            return False
    
    def test_pulseaudio(self):
        """Test PulseAudio"""
        try:
            cmd = ["paplay", "--device=default", "/usr/share/sounds/alsa/Front_Center.wav"]
            result = subprocess.run(cmd, stderr=subprocess.DEVNULL, timeout=2)
            return result.returncode == 0
        except:
            return False
    
    def test_sox(self):
        """Test sox/play command"""
        try:
            cmd = ["play", "-n", "-c1", "synth", "0.2", "sine", "1000", "vol", "0.5"]
            result = subprocess.run(cmd, stderr=subprocess.DEVNULL, 
                                  stdout=subprocess.DEVNULL, timeout=2)
            return result.returncode == 0
        except:
            return False
    
    def test_alsa(self):
        """Test ALSA"""
        try:
            # Create a temporary beep using speaker-test
            cmd = "timeout 0.2 speaker-test -t sine -f 1000 >/dev/null 2>&1"
            result = os.system(cmd)
            return result == 0
        except:
            return False
    
    def test_wav_file(self):
        """Test playing WAV file"""
        try:
            # Create a temporary WAV file
            self.create_beep_wav("/tmp/beep.wav", frequency=1000, duration=0.1)
            cmd = ["aplay", "-q", "/tmp/beep.wav"]
            result = subprocess.run(cmd, stderr=subprocess.DEVNULL, timeout=2)
            return result.returncode == 0
        except:
            return False
    
    def create_beep_wav(self, filename, frequency=1000, duration=0.1, volume=0.5):
        """Create a WAV file with a beep sound"""
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
    
    def play_beep(self, frequency=1000, duration=0.1):
        """Play beep using the best available method"""
        if not self.audio_method:
            return
        
        # Run in background thread to avoid blocking
        thread = threading.Thread(target=self._play_beep_internal, 
                                 args=(frequency, duration))
        thread.daemon = True
        thread.start()
    
    def _play_beep_internal(self, frequency, duration):
        """Internal beep playing method"""
        try:
            if self.audio_method == "pulse":
                # Use PulseAudio with generated tone
                subprocess.run([
                    "python3", "-c",
                    f"""
import numpy as np
import subprocess
import struct
rate = 44100
samples = np.sin(2 * np.pi * {frequency} * np.arange(rate * {duration}) / rate)
data = (samples * 32767).astype(np.int16).tobytes()
proc = subprocess.Popen(['paplay', '--raw', '--rate=44100', '--format=s16le', '--channels=1'],
                       stdin=subprocess.PIPE)
proc.communicate(input=data)
"""
                ], stderr=subprocess.DEVNULL, timeout=1)
                
            elif self.audio_method == "sox":
                subprocess.run([
                    "play", "-n", "-c1", "synth", str(duration), 
                    "sine", str(frequency), "vol", "0.5"
                ], stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL)
                
            elif self.audio_method == "alsa":
                os.system(f"timeout {duration} speaker-test -t sine -f {frequency} >/dev/null 2>&1")
                
            elif self.audio_method == "wav":
                wav_file = f"/tmp/beep_{frequency}.wav"
                self.create_beep_wav(wav_file, frequency, duration)
                subprocess.run(["aplay", "-q", wav_file], stderr=subprocess.DEVNULL)
        except:
            pass

class UltrasonicSensorPi5:
    """Ultrasonic sensor controller"""
    
    def __init__(self, trig_pin, echo_pin):
        self.trig = trig_pin
        self.echo = echo_pin
        
        try:
            self.chip = lgpio.gpiochip_open(4)
            print("Using GPIO chip 4 (Pi 5)")
        except:
            self.chip = lgpio.gpiochip_open(0)
            print("Using GPIO chip 0")
        
        lgpio.gpio_claim_output(self.chip, self.trig)
        lgpio.gpio_claim_input(self.chip, self.echo)
        lgpio.gpio_write(self.chip, self.trig, 0)
        time.sleep(2)
        print(f"Sensor initialized: Trig=GPIO{trig_pin}, Echo=GPIO{echo_pin}")
    
    def ultrasonic_measure(self):
        """Single distance measurement"""
        lgpio.gpio_write(self.chip, self.trig, 0)
        time.sleep(0.000002)
        
        lgpio.gpio_write(self.chip, self.trig, 1)
        time.sleep(0.00001)
        lgpio.gpio_write(self.chip, self.trig, 0)
        
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
        
        pulse_duration = pulse_end - pulse_start
        distance_cm = pulse_duration * 17150
        
        if distance_cm < 2 or distance_cm > 400:
            return None
            
        return distance_cm
    
    def get_distance_quick(self):
        """Get quick distance reading (3 samples)"""
        readings = []
        for _ in range(3):
            reading = self.ultrasonic_measure()
            if reading:
                readings.append(reading)
            time.sleep(0.01)
        
        if readings:
            return sum(readings) / len(readings)
        return None
    
    def cleanup(self):
        lgpio.gpiochip_close(self.chip)

class BluetoothAudioSetup:
    """Helper class for Bluetooth setup"""
    
    @staticmethod
    def setup_bluetooth_audio():
        """Complete Bluetooth audio setup"""
        print("\n" + "="*60)
        print("BLUETOOTH AUDIO SETUP")
        print("="*60)
        
        commands = [
            # Ensure Bluetooth is powered on
            ("bluetoothctl power on", "Powering on Bluetooth"),
            
            # List connected devices
            ("bluetoothctl devices Connected", "Checking connected devices"),
            
            # Restart PulseAudio
            ("systemctl --user restart pulseaudio", "Restarting PulseAudio"),
            
            # Wait for PulseAudio to start
            ("sleep 2", "Waiting for PulseAudio"),
            
            # List audio sinks
            ("pactl list sinks short", "Listing audio outputs"),
            
            # Set Bluetooth as default if available
            ("pacmd set-default-sink $(pacmd list-sinks | grep -o 'bluez.*[>]' | sed 's/[<>]//g' | head -1) 2>/dev/null", 
             "Setting Bluetooth as default"),
            
            # Set volume
            ("amixer set Master 85%", "Setting volume to 85%"),
            
            # Test sound
            ("speaker-test -c 2 -t wav -l 1", "Testing audio output")
        ]
        
        for cmd, description in commands:
            print(f"\n{description}...")
            os.system(cmd)
        
        print("\n" + "="*60)
        print("Setup complete! If you didn't hear test sound:")
        print("1. Check Bluetooth speaker is on and connected")
        print("2. Run: bluetoothctl connect XX:XX:XX:XX:XX:XX")
        print("3. Run this script again")
        print("="*60)

# Main Program
def main():
    print("\n" + "="*60)
    print("HC-SR04 ULTRASONIC SENSOR WITH BLUETOOTH AUDIO")
    print("Raspberry Pi 5 Edition")
    print("="*60)
    
    # Setup Bluetooth first
    print("\nDo you need to setup Bluetooth? (y/n): ", end="")
    if input().lower() == 'y':
        BluetoothAudioSetup.setup_bluetooth_audio()
        print("\nPress Enter to continue with sensor...")
        input()
    
    # Initialize components
    print("\nInitializing components...")
    sensor = UltrasonicSensorPi5(TRIG_PIN, ECHO_PIN)
    audio = AudioController()
    
    # Test beep
    print("\nTesting audio with 3 beeps...")
    for i in range(3):
        print(f"Beep {i+1}...")
        audio.play_beep(frequency=1000 + (i * 200), duration=0.2)
        time.sleep(0.5)
    
    print("\nDid you hear the beeps? (y/n): ", end="")
    if input().lower() != 'y':
        print("\nAudio not working. Check:")
        print("1. Bluetooth speaker is on")
        print("2. Speaker is paired and connected")
        print("3. Volume is up on speaker")
        print("\nContinuing with visual alerts only...")
        audio.audio_method = None
    
    # Main loop
    last_beep_time = 0
    last_distance = None
    
    try:
        print("\n" + "="*60)
        print("STARTING ULTRASONIC MONITORING")
        print("Distance zones:")
        print("  <10cm:    Continuous beep (2000Hz)")
        print("  10-20cm:  Very fast beeping (1500Hz)")
        print("  20-40cm:  Fast beeping (1200Hz)")
        print("  40-70cm:  Medium beeping (1000Hz)")
        print("  70-100cm: Slow beeping (800Hz)")
        print("  >100cm:   No beeping")
        print("\nPress Ctrl+C to stop")
        print("="*60 + "\n")
        
        while True:
            distance = sensor.get_distance_quick()
            
            if distance is not None:
                # Calculate change
                change = ""
                if last_distance:
                    diff = distance - last_distance
                    if diff > 2:
                        change = " ↗ Moving away"
                    elif diff < -2:
                        change = " ↘ Approaching"
                    else:
                        change = " → Stable"
                last_distance = distance
                
                # Determine zone and beep parameters
                current_time = time.time()
                beep_now = False
                
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
                bars = int(distance / 10) if distance < 100 else 10
                bar_display = bar_color + "█" * bars + "░" * (10 - bars) + "\033[0m"
                
                print(f"Distance: {distance:6.2f} cm [{zone:11s}] {bar_display} {change}")
                
                # Play beep if needed
                if beep_interval is not None:
                    if current_time - last_beep_time >= beep_interval:
                        audio.play_beep(frequency=frequency, duration=duration)
                        last_beep_time = current_time
            else:
                print("No reading - Check sensor connection")
            
            time.sleep(0.05)  # Fast update rate
            
    except KeyboardInterrupt:
        print("\n\nStopping...")
    finally:
        sensor.cleanup()
        print("Cleanup complete. Goodbye!")

if __name__ == "__main__":
    # Install required packages if missing
    print("Checking dependencies...")
    os.system("which play > /dev/null 2>&1 || sudo apt-get install -y sox libsox-fmt-all >/dev/null 2>&1")
    os.system("which paplay > /dev/null 2>&1 || sudo apt-get install -y pulseaudio-utils >/dev/null 2>&1")
    
    main()
