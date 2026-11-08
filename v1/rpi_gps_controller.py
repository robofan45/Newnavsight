import serial
import threading
import time
import logging
from collections import deque
from datetime import datetime, timezone
from typing import Optional, Dict, Tuple, List
from functools import reduce
import operator

# --- [IMPROVED] Structured Configuration ---
class GPSConfig:
    """Centralized configuration for the GPS controller."""
    SERIAL_PORT: str = '/dev/ttyAMA0'
    BAUD_RATE: int = 9600
    TIMEOUT_S: float = 2.0
    MAX_RETRY_ATTEMPTS: int = 3
    UPDATE_RATE_HZ: float = 1.0
    LOG_FILE: str = 'gps_log.log'
    LOG_LEVEL: int = logging.INFO
    STALE_DATA_THRESHOLD_S: float = 5.0

# --- Setup Logging ---
logging.basicConfig(
    level=GPSConfig.LOG_LEVEL,
    format='%(asctime)s - %(levelname)s - %(threadName)s - %(message)s',
    handlers=[
        logging.FileHandler(GPSConfig.LOG_FILE),
        logging.StreamHandler()
    ]
)

class GPSError(Exception):
    """Custom exception for GPS-related errors."""
    pass

class GPSData:
    """A data class to hold parsed GPS information."""
    def __init__(self):
        self.timestamp: Optional[datetime.time] = None
        self.latitude: float = 0.0
        self.longitude: float = 0.0
        self.altitude: float = 0.0
        self.fix_quality: int = 0
        self.satellites_in_view: int = 0
        self.satellites_in_use: int = 0
        self.hdop: float = 0.0
        self.speed_kmh: float = 0.0
        self.track_angle: float = 0.0
        self.is_valid: bool = False
        self.satellite_data: Dict[str, Tuple[Optional[str], Optional[str], Optional[str]]] = {}
        self.last_updated_utc: Optional[datetime] = None

    def get_fix_quality_str(self) -> str:
        """Returns a human-readable string for the fix quality."""
        qualities = {
            0: "No Fix", 1: "GPS (SPS)", 2: "DGPS", 3: "PPS",
            4: "Real Time Kinematic", 5: "Float RTK"
        }
        return qualities.get(self.fix_quality, "Unknown")

# --- [NEW] Dedicated NMEA Parser Class for Robustness and Separation of Concerns ---
class NMEAParser:
    """
    Handles parsing of NMEA 0183 sentences.
    This dedicated class makes the system more modular and easier to test/extend.
    """
    def __init__(self):
        self.parse_errors = 0

    def parse(self, sentence: str, gps_data: GPSData) -> bool:
        """
        Parses a single NMEA sentence and updates the provided GPSData object.
        Returns True on successful parse, False otherwise.
        """
        if not self._validate_checksum(sentence):
            logging.debug(f"Invalid checksum for sentence: {sentence}")
            self.parse_errors += 1
            return False

        try:
            parts = sentence.split('*')[0].split(',')
            # [IMPROVED] Handle talker IDs like $GP, $GN, etc. by looking at the last 3 chars
            msg_type = parts[0][-3:]

            if msg_type == 'GGA':
                self._parse_gga(parts, gps_data)
            elif msg_type == 'RMC':
                self._parse_rmc(parts, gps_data)
            elif msg_type == 'GSV':
                self._parse_gsv(parts, gps_data)
            
            # If any valid data was processed, update the timestamp
            gps_data.last_updated_utc = datetime.now(timezone.utc)
            return True

        except (IndexError, ValueError) as e:
            logging.debug(f"Could not parse NMEA sentence: {sentence} - Error: {e}")
            self.parse_errors += 1
            return False

    def _validate_checksum(self, sentence: str) -> bool:
        try:
            data, checksum = sentence.strip().split('*', 1)
            calculated_checksum = reduce(operator.xor, (ord(c) for c in data[1:]), 0)
            return int(checksum, 16) == calculated_checksum
        except (ValueError, IndexError):
            return False

    def _convert_lat_lon(self, value: str, direction: str) -> float:
        if not value: return 0.0
        deg_part, min_part_str = value.split('.')
        degrees = int(deg_part[:-2])
        minutes = int(deg_part[-2:]) + float(f"0.{min_part_str}")
        decimal_degrees = degrees + (minutes / 60)
        if direction in ['S', 'W']:
            decimal_degrees *= -1
        return decimal_degrees

    def _parse_gga(self, parts: List[str], data: GPSData):
        if parts[1]: data.timestamp = datetime.strptime(parts[1].split('.')[0], '%H%M%S').time()
        data.latitude = self._convert_lat_lon(parts[2], parts[3])
        data.longitude = self._convert_lat_lon(parts[4], parts[5])
        data.fix_quality = int(parts[6]) if parts[6] else 0
        data.satellites_in_use = int(parts[7]) if parts[7] else 0
        data.hdop = float(parts[8]) if parts[8] else 0.0
        data.altitude = float(parts[9]) if parts[9] else 0.0
        data.is_valid = data.fix_quality > 0

    def _parse_rmc(self, parts: List[str], data: GPSData):
        if parts[7]: data.speed_kmh = float(parts[7]) * 1.852
        if parts[8]: data.track_angle = float(parts[8])

    def _parse_gsv(self, parts: List[str], data: GPSData):
        if parts[2] == '1': data.satellite_data.clear()
        if parts[3]: data.satellites_in_view = int(parts[3])
        sats_in_msg = (len(parts) - 4) // 4
        for i in range(sats_in_msg):
            base_idx = 4 + (i * 4)
            prn = parts[base_idx]
            if prn:
                data.satellite_data[prn] = (
                    parts[base_idx + 1], parts[base_idx + 2], parts[base_idx + 3]
                )

class GPSController:
    """
    Manages the connection, reading, and parsing of data from a GPS module.
    """
    def __init__(self, config: GPSConfig):
        self.config = config
        self.serial_connection: Optional[serial.Serial] = None
        self.running: bool = False
        self.thread: Optional[threading.Thread] = None
        self.gps_data = GPSData()
        self.raw_data_buffer = deque(maxlen=100)
        self.data_lock = threading.Lock()
        self.parser = NMEAParser() # Use the dedicated parser

    def start(self):
        """Starts the GPS data reading thread."""
        if self.running:
            logging.warning("GPS Controller is already running.")
            return

        self._initialize_connection()
        self.running = True
        self.thread = threading.Thread(target=self._read_data_loop, name="GPSReadThread", daemon=True)
        self.thread.start()
        logging.info("GPS data reading thread started.")

    def stop(self):
        """Stops the GPS data reading thread and cleans up resources."""
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join()
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()
            logging.info("GPS serial connection closed.")

    def _initialize_connection(self):
        """Initializes the serial connection with retry logic."""
        for attempt in range(self.config.MAX_RETRY_ATTEMPTS):
            try:
                logging.info(f"Attempting to connect to GPS on {self.config.SERIAL_PORT}...")
                self.serial_connection = serial.Serial(
                    self.config.SERIAL_PORT, self.config.BAUD_RATE, timeout=self.config.TIMEOUT_S
                )
                logging.info("GPS serial connection established.")
                return
            except serial.SerialException as e:
                logging.error(f"Attempt {attempt + 1}: Failed to open serial port: {e}")
                time.sleep(2)
        raise GPSError(f"Could not establish serial connection after {self.config.MAX_RETRY_ATTEMPTS} attempts.")

    def _read_data_loop(self):
        """The main loop for the reading thread with improved resilience."""
        while self.running:
            try:
                if not self.serial_connection or not self.serial_connection.is_open:
                    raise serial.SerialException("Port is not open.")

                line = self.serial_connection.readline()
                if line:
                    sentence = line.decode('ascii', errors='ignore').strip()
                    with self.data_lock:
                        self.raw_data_buffer.append(sentence)
                        if sentence.startswith('$'):
                            self.parser.parse(sentence, self.gps_data)

            except serial.SerialException as e:
                logging.error(f"Serial error: {e}. Attempting to reconnect in 5s...")
                if self.serial_connection: self.serial_connection.close()
                time.sleep(5)
                try:
                    self._initialize_connection()
                except GPSError:
                    logging.error("Failed to reconnect. Will retry.")
            except Exception as e:
                logging.error(f"An unexpected error occurred in the read loop: {e}", exc_info=True)
                time.sleep(1)

    # --- [NEW] Automatic NMEA Checksum Calculation for User-Friendly Commands ---
    def send_command(self, command_body: str):
        """
        Sends a command to the GPS module, automatically calculating the checksum.
        Example: send_command("PMTK220,1000") for a 1Hz update rate.
        """
        checksum = reduce(operator.xor, (ord(c) for c in command_body), 0)
        full_command = f"${command_body}*{checksum:02X}\r\n".encode('ascii')

        if self.serial_connection and self.serial_connection.is_open:
            logging.info(f"Sending command to GPS: {full_command.strip().decode()}")
            self.serial_connection.write(full_command)
            self.serial_connection.flush()
        else:
            logging.error("Cannot send command, serial connection is not open.")

    def get_current_data(self) -> GPSData:
        with self.data_lock: return self.gps_data
    
    def get_raw_buffer(self) -> List[str]:
        with self.data_lock: return list(self.raw_data_buffer)

    def get_parser_errors(self) -> int:
        with self.data_lock: return self.parser.parse_errors

def display_dashboard(gps: GPSController, config: GPSConfig, start_time: float):
    """Clears the screen and displays a formatted dashboard."""
    data = gps.get_current_data()
    raw_sentences = gps.get_raw_buffer()
    parser_errors = gps.get_parser_errors()

    print("\033[H\033[J", end="") # Clear console
    print("--- Raspberry Pi GPS Dashboard ---")

    # --- [IMPROVED] More Specific Status Reporting ---
    status, color = "ACQUIRING FIX...", "\033[93m" # Yellow
    if data.is_valid:
        status, color = "OPERATIONAL (FIX ACQUIRED)", "\033[92m" # Green
    elif data.last_updated_utc is None and (time.time() - start_time) > 15:
        status, color = "NO DATA RECEIVED (CHECK WIRING)", "\033[91m" # Red
    elif data.last_updated_utc and (datetime.now(timezone.utc) - data.last_updated_utc).total_seconds() > config.STALE_DATA_THRESHOLD_S:
        status, color = "STALE DATA (NO SIGNAL?)", "\033[91m" # Red

    print(f"Status: {color}{status}\033[0m")
    print(f"Elapsed: {int(time.time() - start_time)}s | Parser Errors: {parser_errors}")
    print("-" * 40)
    print(f"Timestamp: {data.timestamp or 'N/A'} | Lat/Lon: {data.latitude:.6f}, {data.longitude:.6f}")
    print(f"Altitude: {data.altitude:.2f}m | Fix: {data.get_fix_quality_str()} ({data.fix_quality})")
    print(f"Speed: {data.speed_kmh:.2f} km/h | Heading: {data.track_angle:.2f}° | HDOP: {data.hdop}")
    print("-" * 40)
    print(f"Satellites (Used/View): {data.satellites_in_use}/{data.satellites_in_view}")
    if data.satellite_data:
        print("PRN | Elv | Azi | SNR (Signal)")
        sorted_sats = sorted(data.satellite_data.items(), key=lambda item: int(item[1][2] or 0), reverse=True)
        for prn, (elv, azi, snr) in sorted_sats[:6]: # Display top 6 strongest
            snr_val = int(snr or 0)
            snr_bar = '█' * (snr_val // 3)
            print(f"{prn:>3} | {elv:>3}°| {azi:>3}°| {snr:>3} {snr_bar}")
    
    if not data.is_valid and (time.time() - start_time) > 90:
        print("-" * 40)
        print("\033[91m--- TROUBLESHOOTING TIPS ---\033[0m")
        print("1. CRITICAL: Move antenna to a location with a clear, unobstructed sky view.")
        print("2. PATIENCE: A 'cold start' can take 5-15 minutes to get the first fix.")
        print("3. CONNECTION: Ensure antenna is securely connected to the GPS module.")

    print("-" * 40)
    print("--- Raw NMEA Sentences (Last 5) ---")
    for sentence in raw_sentences[-5:]: print(sentence)

def main():
    config = GPSConfig()
    gps = GPSController(config)
    try:
        gps.start()
        print("GPS controller started. Waiting for satellite fix...")
        # Example of user-friendly command sending
        # gps.send_command("PMTK220,1000") # Set 1Hz update rate
        time.sleep(2)
        start_time = time.time()
        while True:
            display_dashboard(gps, config, start_time)
            time.sleep(config.UPDATE_RATE_HZ)
    except GPSError as e:
        logging.critical(f"A critical GPS error occurred: {e}")
    except KeyboardInterrupt:
        print("\nProgram interrupted by user.")
    finally:
        print("Shutting down GPS controller...")
        gps.stop()
        print("Shutdown complete.")

if __name__ == "__main__":
    main()