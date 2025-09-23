#!/usr/bin/env python3
"""
GPS FIX AND DEBUG TOOL
This will find and fix your GPS issues
"""

import os
import sys
import time
import subprocess

try:
    import serial
except:
    os.system("sudo apt install python3-serial -y")
    import serial

def step1_check_hardware():
    """Step 1: Check hardware connections"""
    print("\n" + "="*60)
    print("STEP 1: CHECKING HARDWARE")
    print("="*60)
    
    print("""
    ✓ CHECK YOUR WIRING:
    
    GPS Module  →  Raspberry Pi 5
    ─────────────────────────────
    VCC        →  Pin 1 (3.3V) or Pin 2 (5V) 
    GND        →  Pin 6 (Ground)
    TX         →  Pin 10 (GPIO15/RX)
    RX         →  Pin 8 (GPIO14/TX) [Optional]
    
    ❓ Questions to verify:
    1. Is the GPS LED blinking? (should blink every second)
    2. Is GPS antenna connected? (if external)
    3. Are you near a window or outside?
    """)
    
    input("Press ENTER when connections are verified...")
    return True

def step2_enable_serial():
    """Step 2: Enable serial port"""
    print("\n" + "="*60)
    print("STEP 2: ENABLING SERIAL PORT")
    print("="*60)
    
    commands = [
        "sudo raspi-config nonint do_serial 2",
        "sudo systemctl stop serial-getty@serial0.service",
        "sudo systemctl disable serial-getty@serial0.service",
        "sudo chmod 666 /dev/serial0 2>/dev/null",
    ]
    
    for cmd in commands:
        print(f"Running: {cmd}")
        os.system(cmd + " 2>/dev/null")
    
    # Check config file
    print("\nChecking boot config...")
    config_file = "/boot/firmware/config.txt"
    if not os.path.exists(config_file):
        config_file = "/boot/config.txt"
    
    with open(config_file, 'r') as f:
        config = f.read()
    
    if "enable_uart=1" not in config:
        print("Adding enable_uart=1 to config...")
        os.system(f"echo 'enable_uart=1' | sudo tee -a {config_file}")
    
    print("✅ Serial port enabled")
    return True

def step3_detect_gps_raw():
    """Step 3: Detect raw GPS data"""
    print("\n" + "="*60)
    print("STEP 3: DETECTING RAW GPS DATA")
    print("="*60)
    
    port = '/dev/serial0'
    if not os.path.exists(port):
        print(f"❌ {port} does not exist!")
        return False
    
    print(f"Reading from {port} for 10 seconds...")
    print("You should see NMEA sentences starting with $ below:")
    print("-"*40)
    
    found_gps = False
    found_fix = False
    
    try:
        ser = serial.Serial(port, 9600, timeout=1)
        ser.reset_input_buffer()
        
        end_time = time.time() + 10
        line_count = 0
        
        while time.time() < end_time:
            if ser.in_waiting:
                try:
                    raw_data = ser.read(ser.in_waiting)
                    decoded = raw_data.decode('ascii', errors='ignore')
                    
                    lines = decoded.strip().split('\n')
                    for line in lines:
                        if line and '$' in line:
                            line_count += 1
                            print(f"{line_count}: {line[:70]}")
                            found_gps = True
                            
                            # Check for GPS fix
                            if 'GPGGA' in line or 'GNGGA' in line:
                                parts = line.split(',')
                                if len(parts) > 7:
                                    fix = parts[6] if parts[6] else '0'
                                    sats = parts[7] if parts[7] else '0'
                                    if fix != '0':
                                        found_fix = True
                                        print(f"    └─> FIX STATUS: Quality={fix}, Satellites={sats}")
                except:
                    pass
            
            time.sleep(0.1)
        
        ser.close()
        
    except Exception as e:
        print(f"Error: {e}")
        return False
    
    print("-"*40)
    
    if not found_gps:
        print("❌ NO GPS DATA RECEIVED!")
        print("\nPossible issues:")
        print("1. TX/RX wires may be swapped - try reversing them")
        print("2. GPS module may need different voltage (try 5V instead of 3.3V)")
        print("3. Baud rate may be different (not 9600)")
        return False
    
    if found_gps and not found_fix:
        print("⚠️ GPS IS WORKING but NO POSITION FIX")
        print("\nThis means:")
        print("• GPS module is connected correctly ✓")
        print("• GPS needs clear sky view to get satellites")
        print("• Move near window or go outside")
        print("• Wait 30-60 seconds for first fix")
    
    if found_fix:
        print("✅ GPS IS WORKING WITH POSITION FIX!")
    
    return found_gps

def step4_get_location():
    """Step 4: Get actual GPS location"""
    print("\n" + "="*60)
    print("STEP 4: GETTING GPS LOCATION")
    print("="*60)
    
    print("Waiting for GPS position (up to 60 seconds)...")
    print("For best results: GO NEAR A WINDOW or OUTSIDE")
    print("-"*40)
    
    try:
        ser = serial.Serial('/dev/serial0', 9600, timeout=1)
        
        start_time = time.time()
        timeout = 60
        
        while (time.time() - start_time) < timeout:
            if ser.in_waiting:
                line = ser.readline().decode('ascii', errors='ignore').strip()
                
                # Show activity
                if '$' in line:
                    sentence_type = line[1:6] if len(line) > 6 else line
                    print(f"\rReceiving: {sentence_type} | Time: {int(time.time()-start_time)}s", end="")
                
                # Parse GPGGA for location
                if ('$GPGGA' in line or '$GNGGA' in line):
                    parts = line.split(',')
                    
                    if len(parts) >= 10:
                        lat_raw = parts[2]
                        lat_dir = parts[3]
                        lon_raw = parts[4]
                        lon_dir = parts[5]
                        fix = parts[6]
                        sats = parts[7]
                        alt = parts[9]
                        
                        # Show satellite count even without fix
                        if sats and sats != '00':
                            print(f"\r📡 Satellites visible: {sats} | Fix quality: {fix}     ", end="")
                        
                        # Check for valid fix
                        if fix != '0' and lat_raw and lon_raw:
                            # Convert to decimal degrees
                            try:
                                # Latitude DDMM.MMMM
                                lat = float(lat_raw[:2]) + float(lat_raw[2:])/60
                                if lat_dir == 'S': lat = -lat
                                
                                # Longitude DDDMM.MMMM
                                lon = float(lon_raw[:3]) + float(lon_raw[3:])/60
                                if lon_dir == 'W': lon = -lon
                                
                                # SUCCESS! Show location
                                print("\n\n" + "="*60)
                                print("🎉 GPS LOCATION FOUND!")
                                print("="*60)
                                print(f"\n📍 COORDINATES:")
                                print(f"   Latitude:  {lat:.8f}°")
                                print(f"   Longitude: {lon:.8f}°")
                                
                                if alt:
                                    print(f"   Altitude:  {alt} meters")
                                
                                print(f"   Satellites: {sats}")
                                print(f"   Fix Quality: {fix}")
                                
                                # Generate Google Maps link
                                maps_url = f"https://maps.google.com/?q={lat},{lon}"
                                print(f"\n🗺️  GOOGLE MAPS LINK:")
                                print(f"   {maps_url}")
                                
                                print("\n📋 COPY THIS LINK TO YOUR BROWSER:")
                                print(f"   {maps_url}")
                                
                                print("="*60)
                                
                                ser.close()
                                return True
                                
                            except ValueError:
                                pass
            
            time.sleep(0.1)
        
        ser.close()
        print(f"\n\n❌ No GPS fix after {timeout} seconds")
        
    except Exception as e:
        print(f"\nError: {e}")
        return False
    
    return False

def step5_troubleshoot():
    """Step 5: Advanced troubleshooting"""
    print("\n" + "="*60)
    print("STEP 5: TROUBLESHOOTING")
    print("="*60)
    
    print("Testing different baud rates...")
    
    baudrates = [9600, 4800, 38400, 57600, 115200]
    
    for baud in baudrates:
        print(f"\nTesting {baud} baud...")
        try:
            ser = serial.Serial('/dev/serial0', baud, timeout=0.5)
            
            # Read for 2 seconds
            found = False
            for _ in range(20):
                if ser.in_waiting:
                    data = ser.read(ser.in_waiting)
                    decoded = data.decode('ascii', errors='ignore')
                    if '$' in decoded and 'GP' in decoded:
                        print(f"  ✅ GPS FOUND at {baud} baud!")
                        print(f"  Sample: {decoded[:50]}")
                        found = True
                        break
                time.sleep(0.1)
            
            ser.close()
            
            if found:
                print(f"\n⚠️ Your GPS uses {baud} baud, not 9600!")
                print(f"Update your code to use: serial.Serial('/dev/serial0', {baud})")
                return True
                
        except:
            pass
    
    print("\n" + "="*60)
    print("📝 FINAL TROUBLESHOOTING STEPS:")
    print("="*60)
    print("""
    If GPS still not working:
    
    1. SWAP TX/RX WIRES
       - Most common issue!
       - GPS TX → Pi RX (pin 10)
       - GPS RX → Pi TX (pin 8)
    
    2. CHECK POWER
       - Try 5V instead of 3.3V
       - Check if LED is blinking
    
    3. ANTENNA/LOCATION
       - MUST have clear sky view
       - Won't work indoors well
       - External antenna helps
    
    4. COLD START TIME
       - First fix: 30-60 seconds
       - Warm start: 1-5 seconds
       - Be patient!
    
    5. REBOOT AFTER FIXES
       sudo reboot
    """)

def main():
    print("""
╔════════════════════════════════════════════════════╗
║          GPS COMPLETE FIX & LOCATION TOOL         ║
║                Raspberry Pi 5                      ║
╚════════════════════════════════════════════════════╝
    """)
    
    print("\nThis tool will fix your GPS and show location on Google Maps")
    print("Follow each step carefully...")
    
    # Run through steps
    step1_check_hardware()
    step2_enable_serial()
    
    if step3_detect_gps_raw():
        if not step4_get_location():
            print("\nGPS is connected but can't get location.")
            print("SOLUTION: Move near a window or go outside and run again!")
    else:
        step5_troubleshoot()
        print("\nAfter fixing issues above, run this script again!")

if __name__ == "__main__":
    main()