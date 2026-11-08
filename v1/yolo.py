#!/usr/bin/env python3
import cv2
import time
import subprocess  # For system audio (espeak for TTS)
from ultralytics import YOLO
try:
    from picamera2 import Picamera2  # For CSI camera (Raspberry Pi Camera Module)
    PICAMERA_AVAILABLE = True
except ImportError:
    PICAMERA_AVAILABLE = False
    print("Picamera2 not available—falling back to USB camera. Install with: pip install picamera2")

def speak_text(text, voice="en"):
    """
    Simple TTS using espeak (pre-installed on Raspberry Pi OS).
    """
    try:
        subprocess.run(["espeak", "-v", voice, text], check=True, capture_output=True)
    except subprocess.CalledProcessError:
        print(f"TTS fallback: {text}")  # Silent fallback if espeak fails

def main(model_path="yolov8n.pt", camera_type="csi", frame_width=640, frame_height=480, save_video=False, enable_audio=True):
    """
    Enhanced real-time YOLOv8 object detection for Raspberry Pi with audio feedback.
    Supports CSI (Picamera2) or USB camera. Speaks class names on detection.
    """
    # --- 1. Load YOLOv8 Model ---
    print(f"Loading YOLO model from '{model_path}'...")
    try:
        model = YOLO(model_path)
        model.conf = 0.5  # Filter low-confidence detections for cleaner output
        print("Model loaded successfully.")
    except Exception as e:
        print(f"Error loading model: {e}")
        return

    # --- 2. Initialize Camera ---
    print(f"Initializing {camera_type} camera...")
    if camera_type == "csi" and PICAMERA_AVAILABLE:
        # Picamera2 for CSI (official Pi Camera)
        try:
            picam2 = Picamera2()
            config = picam2.create_preview_configuration(
                main={"size": (frame_width, frame_height), "format": "RGB888"}
            )
            picam2.configure(config)
            picam2.start()
            cap = picam2  # Use as capture source
            is_picam = True
            print("CSI camera (Picamera2) initialized.")
        except Exception as e:
            print(f"CSI camera error: {e}. Falling back to USB.")
            camera_type = "usb"
    else:
        # Fallback to USB/OpenCV
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)
        if not cap.isOpened():
            print("Error: Could not open USB camera.")
            return
        is_picam = False
        print("USB camera initialized.")

    # Optional: Video Writer for saving output
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = None
    if save_video:
        out = cv2.VideoWriter('output.mp4', fourcc, 10.0, (frame_width, frame_height))

    print("Press 'q' to exit. Audio feedback enabled." if enable_audio else "Press 'q' to exit.")

    # --- 3. FPS and Audio State Variables ---
    prev_time = time.time()
    last_audio_time = 0  # Throttle audio to avoid spam
    audio_cooldown = 2.0  # Seconds between audio feedback

    # --- 4. Main Loop ---
    try:
        while True:
            # Capture frame
            if is_picam:
                frame = cap.capture_array()  # RGB NumPy array
            else:
                success, frame = cap.read()
                if not success:
                    print("Warning: Failed to grab frame, skipping...")
                    time.sleep(0.1)
                    continue

            # --- Object Detection ---
            results = model(frame, imgsz=frame_width, verbose=False)

            # --- Audio Feedback (if enabled) ---
            if enable_audio:
                current_time = time.time()
                if current_time - last_audio_time > audio_cooldown:
                    detections = results[0].boxes
                    if len(detections) > 0:
                        # Get the top (most confident) detection class
                        top_class = model.names[int(detections[0].cls)]
                        speak_text(f"Detected {top_class}")
                        last_audio_time = current_time

            # --- Annotate Frame ---
            annotated_frame = results[0].plot()

            # --- FPS Calculation ---
            new_time = time.time()
            fps = 1 / (new_time - prev_time) if new_time - prev_time > 0 else 0
            prev_time = new_time
            fps_text = f"FPS: {fps:.1f}"
            cv2.putText(annotated_frame, fps_text, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

            # Save if enabled
            if save_video:
                out.write(annotated_frame)

            # Display
            cv2.imshow("YOLOv8 Real-Time Detection", annotated_frame)

            # Exit on 'q'
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        # --- Cleanup ---
        print("\nCleaning up...")
        if is_picam:
            cap.stop()
        else:
            cap.release()
        if out:
            out.release()
        cv2.destroyAllWindows()
        print("Application exited.")

if __name__ == "__main__":
    # Configuration (tweak as needed)
    MODEL_PATH = "yolov8n.pt"  # Nano for speed; try 'yolov8s.pt' for accuracy
    CAMERA_TYPE = "csi"        # "csi" for Pi Camera, "usb" for webcam
    FRAME_WIDTH = 640          # Lower to 320 for ~2x FPS boost
    FRAME_HEIGHT = 480
    SAVE_VIDEO = False         # Set True to save 'output.mp4'
    ENABLE_AUDIO = True        # Set False to disable TTS
    
    main(model_path=MODEL_PATH, camera_type=CAMERA_TYPE,
         frame_width=FRAME_WIDTH, frame_height=FRAME_HEIGHT,
         save_video=SAVE_VIDEO, enable_audio=ENABLE_AUDIO)
