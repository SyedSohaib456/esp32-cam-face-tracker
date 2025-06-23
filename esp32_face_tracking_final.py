import cv2
import numpy as np
import requests
import time
import json
from cvzone.FaceDetectionModule import FaceDetector

# ESP32 Configuration
ESP32_CAM_URL = "http://192.168.1.9"
STREAM_URL = f"{ESP32_CAM_URL}/stream"
CONTROL_URL = f"{ESP32_CAM_URL}/control"

class ESP32FaceTracker:
    def __init__(self):
        self.detector = FaceDetector()
        self.servoPos = [90, 90]  # [X, Y] servo positions
        self.ws, self.hs = 800, 600  # Target resolution
        
        print("🎯 ESP32 Face Tracker initialized")
        print(f"ESP32 URL: {ESP32_CAM_URL}")
        print(f"Stream URL: {STREAM_URL}")
        print(f"Control URL: {CONTROL_URL}")
    
    def send_servo_command(self, x_angle, y_angle):
        """Send servo positions to ESP32"""
        try:
            servo_data = {
                "servoX": int(x_angle),
                "servoY": int(y_angle)
            }
            
            response = requests.post(CONTROL_URL, 
                                   json=servo_data,  # Use json parameter instead of data+headers
                                   timeout=1)
            
            if response.status_code == 200:
                print(f"Servos moved: X={int(x_angle)}°, Y={int(y_angle)}°")
                return True
            else:
                print(f"Servo control failed: {response.status_code}")
                return False
                
        except Exception as e:
            print(f"Servo error: {e}")
            return False
    
    def get_frame_from_stream(self):
        """Get a frame from ESP32 multipart stream"""
        try:
            response = requests.get(STREAM_URL, stream=True, timeout=10)
            if response.status_code != 200:
                return None
            
            bytes_data = b''
            for chunk in response.iter_content(chunk_size=1024):
                bytes_data += chunk
                
                # Look for JPEG frame boundaries
                start = bytes_data.find(b'\xff\xd8')  # JPEG start
                end = bytes_data.find(b'\xff\xd9')    # JPEG end
                
                if start != -1 and end != -1 and end > start:
                    # Extract JPEG data
                    jpg_data = bytes_data[start:end+2]
                    bytes_data = bytes_data[end+2:]
                    
                    # Decode to OpenCV image
                    img_array = np.frombuffer(jpg_data, dtype=np.uint8)
                    frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
                    
                    if frame is not None:
                        response.close()
                        return frame
                
                # Prevent buffer overflow
                if len(bytes_data) > 100000:
                    bytes_data = bytes_data[-50000:]
                    
        except Exception as e:
            print(f"Stream error: {e}")
            return None
        
        return None
    
    def run_tracking(self):
        """Main face tracking loop"""
        print("\n🚀 Starting face tracking...")
        print("📺 Connecting to ESP32 stream...")
        print("Press 'q' in the tracking window to quit")
        print("-" * 50)
        
        frame_count = 0
        last_servo_time = 0
        
        print("✅ Starting frame capture...")
        
        while True:
            # Get frame from ESP32
            frame = self.get_frame_from_stream()
            
            if frame is None:
                print("📷 Waiting for frame...")
                time.sleep(0.5)
                continue
            
            frame_count += 1
            if frame_count % 10 == 0:
                print(f"📸 Processed {frame_count} frames")
            
            # Resize frame to target size
            frame = cv2.resize(frame, (self.ws, self.hs))
            
            # Detect faces
            frame, bboxs = self.detector.findFaces(frame, draw=False)
            
            if bboxs:
                # Get the first (largest) face
                face = bboxs[0]
                fx, fy = face["center"][0], face["center"][1]
                
                # Convert face position to servo angles
                # X servo: 0° = right, 180° = left
                # Y servo: 0° = down, 180° = up
                servoX = np.interp(fx, [0, self.ws], [180, 0])
                servoY = np.interp(fy, [0, self.hs], [0, 180])
                
                # Constrain servo angles
                servoX = max(0, min(180, servoX))
                servoY = max(0, min(180, servoY))
                
                # Send servo commands with improved responsiveness
                current_time = time.time()
                # Calculate movement difference
                move_diff_x = abs(servoX - self.servoPos[0])
                move_diff_y = abs(servoY - self.servoPos[1])
                
                # Send command if significant movement or enough time passed
                if (move_diff_x > 3 or move_diff_y > 3 or current_time - last_servo_time > 1.0):
                    if self.send_servo_command(servoX, servoY):
                        self.servoPos[0] = servoX
                        self.servoPos[1] = servoY
                        last_servo_time = current_time
                
                # Draw tracking visualization
                cv2.circle(frame, (fx, fy), 80, (0, 255, 0), 3)
                cv2.circle(frame, (fx, fy), 15, (0, 255, 0), -1)
                cv2.line(frame, (0, fy), (self.ws, fy), (0, 255, 0), 2)
                cv2.line(frame, (fx, 0), (fx, self.hs), (0, 255, 0), 2)
                
                # Display face info
                cv2.putText(frame, f"Face: ({fx}, {fy})", (fx + 20, fy - 20),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(frame, "TARGET LOCKED", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                
            else:
                # No face detected
                cv2.putText(frame, "NO TARGET", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                
                # Draw center crosshair
                center_x, center_y = self.ws // 2, self.hs // 2
                cv2.circle(frame, (center_x, center_y), 80, (128, 128, 128), 2)
                cv2.circle(frame, (center_x, center_y), 15, (128, 128, 128), -1)
                cv2.line(frame, (0, center_y), (self.ws, center_y), (128, 128, 128), 1)
                cv2.line(frame, (center_x, 0), (center_x, self.hs), (128, 128, 128), 1)
            
            # Display servo positions and frame info
            cv2.putText(frame, f"Servo X: {int(self.servoPos[0])}deg", (10, self.hs - 80),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(frame, f"Servo Y: {int(self.servoPos[1])}deg", (10, self.hs - 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(frame, f"Frame: {frame_count}", (10, self.hs - 40),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(frame, "ESP32-CAM Face Tracking", (10, self.hs - 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
            # Show frame
            cv2.imshow("ESP32 Face Tracking", frame)
            
            # Check for quit key
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
        
        # Cleanup
        cv2.destroyAllWindows()
        print("✅ Face tracking stopped")
        return True

def main():
    print("🎯 ESP32 Face Tracking System")
    print("=" * 50)
    print("🔧 Configuration:")
    print(f"  ESP32 IP: 192.168.1.9")
    print(f"  WiFi Network: destroy.bat")
    print(f"  Servo X (Pan): GPIO 12")
    print(f"  Servo Y (Tilt): GPIO 13")
    print()
    
    # Test ESP32 connection
    print("🔍 Testing ESP32 connection...")
    try:
        response = requests.get(ESP32_CAM_URL, timeout=5)
        if response.status_code == 200:
            print("✅ ESP32 is reachable")
        else:
            print(f"⚠️ ESP32 responded with status: {response.status_code}")
    except Exception as e:
        print(f"❌ Cannot reach ESP32: {e}")
        print("💡 Check if ESP32 is powered on and connected to WiFi")
        
        choice = input("Do you want to continue anyway? (y/n): ").lower()
        if choice != 'y':
            return
    
    # Initialize and start tracking
    tracker = ESP32FaceTracker()
    
    try:
        success = tracker.run_tracking()
        if not success:
            print("❌ Face tracking failed to start")
    except KeyboardInterrupt:
        print("\n🛑 Stopped by user (Ctrl+C)")
    except Exception as e:
        print(f"❌ Error: {e}")

if __name__ == "__main__":
    main() 