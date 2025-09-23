#!/usr/bin/env python3
"""
HC-SR04 Ultrasonic Sensor for Raspberry Pi 5
With noise filtering and distance-based beeping sound
Using lgpio library (works on Pi 5) and pygame for audio
"""

import lgpio
import time
import pygame
import threading
import numpy as np

# GPIO pins
TRIG_PIN = 23  # GPIO pin connected to TRIG pin of ultrasonic sensor
ECHO_PIN = 24  # GPIO pin connected to ECHO pin of ultrasonic sensor

# Distance thresholds for beeping (in cm)
DISTANCE_ZONES = {
    'very_close': 10,    # Continuous beep
    'close': 30,          # Fast beeping
    'medium': 60,         # Medium beeping
    'far': 100,          # Slow beeping
    'very_far': 200      # Very slow beeping
}

class BeepController:
    """Handles audio beeping based on distance"""
    
    def __init__(self):
        # Initialize pygame mixer for audio
        pygame.mixer.init(frequency=22050, size=-16, channels=2, buffer=512)
        self.beep_sound = self.create_beep_sound()
        self.last_beep_time = 0
        self.is_beeping = False
        
    def create_beep_sound(self, frequency=1000, duration=0.1):
        """Create a beep sound programmatically"""
        sample_rate = 22050
        frames = int(duration * sample_rate)
        arr = np.zeros(frames)
        for i in range(frames):
            arr[i] = np.sin(2 * np.pi * frequency * i / sample_rate)
        arr = (arr * 32767).astype(np.int16)
        arr = np.repeat(arr.reshape(frames, 1), 2, axis=1)
        sound = pygame.sndarray.make_sound(arr)
        return sound
    
    def get_beep_interval(self, distance):
        """Calculate beep interval based on distance"""
        if distance is None:
            return None
        elif distance <= DISTANCE_ZONES['very_close']:
            return 0  # Continuous beep
        elif distance <= DISTANCE_ZONES['close']:
            return 0.1  # Very fast
        elif distance <= DISTANCE_ZONES['medium']:
            return 0.3  # Fast
        elif distance <= DISTANCE_ZONES['far']:
            return 0.6  # Medium
        elif distance <= DISTANCE_ZONES['very_far']:
            return 1.2  # Slow
        else:
            return None  # No beep
    
    def beep(self, distance):
        """Play beep based on distance"""
        interval = self.get_beep_interval(distance)
        
        if interval is None:
            # No beep needed
            return
        
        current_time = time.time()
        
        if interval == 0:
            # Continuous beep for very close objects
            if not self.is_beeping:
                self.beep_sound.play(-1)  # Loop indefinitely
                self.is_beeping = True
        else:
            # Stop continuous beep if it was playing
            if self.is_beeping:
                self.beep_sound.stop()
                self.is_beeping = False
            
            # Intermittent beeping
            if current_time - self.last_beep_time >= interval:
                self.beep_sound.play()
                self.last_beep_time = current_time
    
    def stop(self):
        """Stop all sounds"""
        pygame.mixer.quit()

class UltrasonicSensorPi5:
    def __init__(self, trig_pin, echo_pin):
        self.trig = trig_pin
        self.echo = echo_pin
        
        # Open GPIO chip 4 for Raspberry Pi 5
        try:
            self.chip = lgpio.gpiochip_open(4)
            print("Using GPIO chip 4 (Pi 5)")
        except:
            self.chip = lgpio.gpiochip_open(0)
            print("Using GPIO chip 0")
        
        # Setup pins
        lgpio.gpio_claim_output(self.chip, self.trig)
        lgpio.gpio_claim_input(self.chip, self.echo)
        
        # Initialize trigger to LOW
        lgpio.gpio_write(self.chip, self.trig, 0)
        time.sleep(2)  # Let sensor settle
        print(f"Sensor initialized: Trig=GPIO{trig_pin}, Echo=GPIO{echo_pin}")
    
    def ultrasonic_measure(self):
        """Single distance measurement"""
        # Clear trigger
        lgpio.gpio_write(self.chip, self.trig, 0)
        time.sleep(0.000002)
        
        # Send 10μs trigger pulse
        lgpio.gpio_write(self.chip, self.trig, 1)
        time.sleep(0.00001)
        lgpio.gpio_write(self.chip, self.trig, 0)
        
        # Wait for echo start (with timeout)
        timeout = time.time() + 0.1
        pulse_start = time.time()
        while lgpio.gpio_read(self.chip, self.echo) == 0:
            pulse_start = time.time()
            if pulse_start > timeout:
                return None  # No echo received
        
        # Wait for echo end (with timeout)
        timeout = time.time() + 0.1
        pulse_end = time.time()
        while lgpio.gpio_read(self.chip, self.echo) == 1:
            pulse_end = time.time()
            if pulse_end > timeout:
                return None  # Echo too long
        
        # Calculate distance
        pulse_duration = pulse_end - pulse_start
        distance_cm = pulse_duration * 17150  # Speed of sound / 2
        
        # Filter out invalid readings
        if distance_cm < 2 or distance_cm > 400:
            return None
            
        return distance_cm
    
    def get_distance(self):
        """Get filtered distance using multiple measurements"""
        filter_array = []
        num_samples = 20
        
        # Take multiple measurements
        for _ in range(num_samples):
            measurement = self.ultrasonic_measure()
            if measurement is not None:  # Only add valid measurements
                filter_array.append(measurement)
            time.sleep(0.03)  # 30ms delay to avoid interference
        
        # Need at least 10 valid readings
        if len(filter_array) < 10:
            return None
        
        # Sort array in ascending order
        filter_array.sort()
        
        # Remove 25% smallest and 25% largest values (outliers)
        trim_count = len(filter_array) // 4
        if trim_count > 0:
            filtered_samples = filter_array[trim_count:-trim_count]
        else:
            filtered_samples = filter_array
        
        # Calculate average of remaining samples
        if filtered_samples:
            distance = sum(filtered_samples) / len(filtered_samples)
            return distance
        return None
    
    def cleanup(self):
        """Clean up GPIO resources"""
        lgpio.gpiochip_close(self.chip)

# Main program
if __name__ == "__main__":
    print("\n" + "="*60)
    print("HC-SR04 Ultrasonic Sensor with Audio Feedback")
    print("="*60)
    print("\nInitializing...")
    
    # Check and set up Bluetooth audio
    print("\nBluetooth Audio Setup:")
    print("1. Make sure your Bluetooth speaker is paired and connected")
    print("2. You can check with: bluetoothctl show")
    print("3. Set as default audio: pacmd set-default-sink <device_name>")
    print("\nPress Enter when Bluetooth speaker is ready...")
    input()
    
    sensor = UltrasonicSensorPi5(TRIG_PIN, ECHO_PIN)
    beeper = BeepController()
    
    try:
        print("\nStarting measurements with audio feedback")
        print("Distance zones:")
        print(f"  ≤{DISTANCE_ZONES['very_close']}cm: Continuous beep")
        print(f"  ≤{DISTANCE_ZONES['close']}cm: Very fast beeping")
        print(f"  ≤{DISTANCE_ZONES['medium']}cm: Fast beeping")
        print(f"  ≤{DISTANCE_ZONES['far']}cm: Medium beeping")
        print(f"  ≤{DISTANCE_ZONES['very_far']}cm: Slow beeping")
        print(f"  >{DISTANCE_ZONES['very_far']}cm: No beeping")
        print("\nPress Ctrl+C to stop")
        print("-" * 60)
        
        while True:
            distance = sensor.get_distance()
            
            if distance is not None:
                # Display distance
                print(f"Distance: {distance:.2f} cm", end="")
                
                # Show zone
                if distance <= DISTANCE_ZONES['very_close']:
                    zone = "VERY CLOSE!"
                elif distance <= DISTANCE_ZONES['close']:
                    zone = "Close"
                elif distance <= DISTANCE_ZONES['medium']:
                    zone = "Medium"
                elif distance <= DISTANCE_ZONES['far']:
                    zone = "Far"
                elif distance <= DISTANCE_ZONES['very_far']:
                    zone = "Very Far"
                else:
                    zone = "Out of range"
                
                print(f" [{zone}]", end="")
                
                # Visual distance indicator
                bars = int(distance / 10) if distance < 100 else 10
                print("  [" + "█" * bars + " " * (10 - bars) + "]")
                
                # Play beep sound
                beeper.beep(distance)
            else:
                print("Unable to get valid reading - check connections!")
            
            time.sleep(0.1)  # Faster update for responsive beeping
            
    except KeyboardInterrupt:
        print("\n\nMeasurement stopped by user")
    finally:
        sensor.cleanup()
        beeper.stop()