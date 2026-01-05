import cv2
import requests
import threading
import time
from ultralytics import YOLO

# -------- CONFIG --------
STREAM_URL = 'http://192.168.18.57:8080/video'
ESP32_IP = 'http://192.168.18.164'

FORWARD_URL = f"{ESP32_IP}/forward"
LEFT_URL = f"{ESP32_IP}/left"
RIGHT_URL = f"{ESP32_IP}/right"
STOP_URL = f"{ESP32_IP}/stop"
DISTANCE_URL = f"{ESP32_IP}/distance"

# Video settings
FRAME_WIDTH = 480
FRAME_HEIGHT = 360

# Control parameters - WIDER CENTER ZONE
CENTER_TOLERANCE = 80
SMOOTHING_FRAMES = 5
MIN_PERSON_HEIGHT = 50
MIN_CONFIDENCE = 0.4

COMMAND_INTERVAL = 0.2

# ULTRASONIC-BASED DISTANCES
STOP_DISTANCE = 70
CHASE_DISTANCE = 80
MAX_CHASE_DISTANCE = 350

# OBSTACLE AVOIDANCE
OBSTACLE_THRESHOLD = 100 # Distance below which obstacle avoidance triggers
OBSTACLE_LEFT_TURN = 0.45
OBSTACLE_FORWARD = 0.8
OBSTACLE_RIGHT_TURN = 0.8

# TURN PARAMETERS
TURN_DURATION = 0.06
FORWARD_DURATION = 0.15

# Load YOLO model
model = YOLO("yolov8n.pt")
model.fuse()

last_action = "IDLE"
last_cmd_time = 0
last_cmd_sent = None
command_lock = threading.Lock()
obstacle_avoidance_active = False
obstacle_avoidance_completed = False  # Flag to ensure it runs only once

# -------- SEND COMMAND TO ESP32 --------
def send_cmd(url, action_text, hold_duration=None):
    global last_action, last_cmd_time, last_cmd_sent
    now = time.time()
    
    with command_lock:
        # Prevent duplicate commands too quickly
        if last_cmd_sent == url and (now - last_cmd_time) < COMMAND_INTERVAL:
            return False
        
        last_cmd_time = now
        last_cmd_sent = url
    
    last_action = action_text
    print(f"[{time.strftime('%H:%M:%S')}] {action_text}")

    def send_request():
        try:
            response = requests.get(url, timeout=2.0)
            if hold_duration:
                time.sleep(hold_duration)
                requests.get(STOP_URL, timeout=2.0)
        except requests.exceptions.Timeout:
            pass
        except requests.exceptions.ConnectionError:
            pass
        except Exception:
            pass
    
    threading.Thread(target=send_request, daemon=True).start()
    return True

# -------- OBSTACLE AVOIDANCE MANEUVER --------
def perform_obstacle_avoidance():
    """Execute zig-zag avoidance pattern: left turn -> stop -> forward -> stop -> right turn -> stop"""
    global obstacle_avoidance_active, obstacle_avoidance_completed
    
    obstacle_avoidance_active = True
    
    # STEP 1: Turn LEFT
    send_cmd(LEFT_URL, f"🔄 OBSTACLE AVOID - TURN LEFT ({OBSTACLE_LEFT_TURN}s)", OBSTACLE_LEFT_TURN)
    time.sleep(OBSTACLE_LEFT_TURN + 0.15)
    
    # STEP 2: STOP after left turn
    send_cmd(STOP_URL, "🛑 OBSTACLE AVOID - STOP (after left turn)", 0.2)
    time.sleep(0.3)
    
    # STEP 3: Move FORWARD
    send_cmd(FORWARD_URL, f"➡️  OBSTACLE AVOID - FORWARD ({OBSTACLE_FORWARD}s)", OBSTACLE_FORWARD)
    time.sleep(OBSTACLE_FORWARD + 0.15)
    
    # STEP 4: STOP after forward
    send_cmd(STOP_URL, "🛑 OBSTACLE AVOID - STOP (after forward)", 0.2)
    time.sleep(0.3)
    
    # STEP 5: Turn RIGHT
    send_cmd(RIGHT_URL, f"🔄 OBSTACLE AVOID - TURN RIGHT ({OBSTACLE_RIGHT_TURN}s)", OBSTACLE_RIGHT_TURN)
    time.sleep(OBSTACLE_RIGHT_TURN + 0.15)
    
    # STEP 6: Final STOP
    send_cmd(STOP_URL, "🛑 OBSTACLE AVOID - STOP (final)", 0.2)
    time.sleep(0.3)
    
    obstacle_avoidance_active = False
    obstacle_avoidance_completed = True  # Mark as completed - won't run again

# -------- GET ULTRASONIC DISTANCE --------
distance_cache = {'value': None, 'time': 0, 'history': []}
DISTANCE_SMOOTHING = 2

def get_dist():
    global distance_cache
    now = time.time()
    
    if now - distance_cache['time'] < 0.2:
        return distance_cache['value']
    
    try:
        r = requests.get(DISTANCE_URL, timeout=2.0)
        dist = float(r.text.strip())
        
        if dist > 0 and 2 <= dist <= 400:
            distance_cache['history'].append(dist)
            if len(distance_cache['history']) > DISTANCE_SMOOTHING:
                distance_cache['history'].pop(0)
            
            smoothed = sum(distance_cache['history']) / len(distance_cache['history'])
            distance_cache['value'] = smoothed
        
        distance_cache['time'] = now
        return distance_cache['value']
    except:
        return distance_cache['value']

# -------- VIDEO STREAM CLASS --------
class VideoStream:
    def __init__(self, src):
        self.cap = cv2.VideoCapture(src, cv2.CAP_FFMPEG)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.ret, self.frame = self.cap.read()
        self.stop_flag = False
        self.lock = threading.Lock()
        threading.Thread(target=self.update, daemon=True).start()
        time.sleep(0.5)

    def update(self):
        while not self.stop_flag:
            ret, frame = self.cap.read()
            if ret:
                with self.lock:
                    self.ret = ret
                    self.frame = frame
            time.sleep(0.005)

    def read(self):
        with self.lock:
            return self.ret, self.frame.copy() if self.frame is not None else (False, None)

    def release(self):
        self.stop_flag = True
        time.sleep(0.05)
        self.cap.release()

# -------- MAIN --------
def main():
    print("\n" + "="*60)
    print("🤖 PERSON FOLLOWING ROBOT WITH OBSTACLE AVOIDANCE")
    print("="*60)
    print(f"📱 Camera: {STREAM_URL}")
    print(f"🎮 ESP32: {ESP32_IP}")
    print(f"🎯 Center Tolerance: {CENTER_TOLERANCE}px (2x WIDER)")
    print(f"📡 Ultrasonic Distances:")
    print(f"   🛑 Stop: <{STOP_DISTANCE}cm")
    print(f"   🚀 Chase: >{CHASE_DISTANCE}cm")
    print(f"   ⚠️  Max: <{MAX_CHASE_DISTANCE}cm")
    print(f"🚧 Obstacle Avoidance Threshold: {OBSTACLE_THRESHOLD}cm")
    print(f"   Left Turn: {OBSTACLE_LEFT_TURN}s | Forward: {OBSTACLE_FORWARD}s | Right Turn: {OBSTACLE_RIGHT_TURN}s")
    print(f"⚡ Normal Turn: {TURN_DURATION}s | Forward: {FORWARD_DURATION}s")
    print("\nPress 'q' to quit\n")
    print("="*60 + "\n")

    vs = VideoStream(STREAM_URL)
    smooth = []
    no_person_frames = 0
    frame_count = 0

    try:
        while True:
            ret, frame = vs.read()
            if not ret or frame is None:
                time.sleep(0.05)
                continue

            frame_count += 1

            # Rotate and resize
            frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
            frame = cv2.resize(frame, (FRAME_WIDTH, FRAME_HEIGHT))

            # YOLO detection
            results = model.predict(frame, verbose=False, conf=MIN_CONFIDENCE, imgsz=320)
            boxes = results[0].boxes

            largest = None
            max_area = 0

            # Find largest person
            for b in boxes:
                if int(b.cls[0]) != 0:  # Only person class
                    continue

                x1, y1, x2, y2 = map(int, b.xyxy[0])
                w, h = x2 - x1, y2 - y1

                if h < MIN_PERSON_HEIGHT or w < 20:
                    continue

                area = w * h
                if area > max_area:
                    max_area = area
                    largest = (x1, y1, w, h)

            # Get ultrasonic distance
            dist = get_dist()
            dist_str = f"{dist:.0f}cm" if dist else "N/A"

            # ========== OBSTACLE AVOIDANCE CHECK ==========
            if not obstacle_avoidance_active and not obstacle_avoidance_completed and dist is not None and dist < OBSTACLE_THRESHOLD:
                # Trigger obstacle avoidance in separate thread (only once)
                threading.Thread(target=perform_obstacle_avoidance, daemon=True).start()

            # Process detection (skip normal logic during obstacle avoidance)
            if not obstacle_avoidance_active:
                if largest and dist is not None:
                    x, y, w, h = largest
                    cx = x + w // 2
                    no_person_frames = 0

                    # Smooth center position
                    smooth.append(cx)
                    if len(smooth) > SMOOTHING_FRAMES:
                        smooth.pop(0)

                    scx = sum(smooth) // len(smooth)
                    center = FRAME_WIDTH // 2
                    offset = scx - center

                    # Draw detection box
                    cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                    cv2.circle(frame, (cx, y+h//2), 5, (255, 0, 0), -1)

                    # ========== DECISION LOGIC ==========
                    
                    # PRIORITY 1: Safety - Too Close
                    if dist < STOP_DISTANCE:
                        send_cmd(STOP_URL, f"🛑 STOP | Dist: {dist_str} (TOO CLOSE!)")
                    
                    # PRIORITY 2: Out of Range - Too Far
                    elif dist > MAX_CHASE_DISTANCE:
                        send_cmd(STOP_URL, f"⚠️  STOP | Dist: {dist_str} (TOO FAR)")
                    
                    # PRIORITY 3: Not Centered - Turn to Align
                    elif abs(offset) > CENTER_TOLERANCE:
                        if offset < 0:
                            send_cmd(RIGHT_URL, f"🔄 TURN RIGHT | Offset: {offset}px | Dist: {dist_str}", TURN_DURATION)
                        else:
                            send_cmd(LEFT_URL, f"🔄 TURN LEFT | Offset: +{offset}px | Dist: {dist_str}", TURN_DURATION)
                    
                    # PRIORITY 4: Centered - Move Forward Based on Distance
                    else:
                        if dist > CHASE_DISTANCE:
                            send_cmd(FORWARD_URL, f"➡️  MOVE FORWARD | Dist: {dist_str}", FORWARD_DURATION)
                        else:
                            send_cmd(FORWARD_URL, f"🐢 SLOW FORWARD | Dist: {dist_str}", FORWARD_DURATION)

                elif largest and dist is None:
                    send_cmd(STOP_URL, "⚠️  STOP | Distance sensor offline")

                else:
                    no_person_frames += 1
                    if no_person_frames == 3:
                        smooth.clear()
                        send_cmd(STOP_URL, "🛑 STOP | No person detected")

            # -------- DRAW UI --------
            cv2.line(frame, (FRAME_WIDTH//2, 0), (FRAME_WIDTH//2, FRAME_HEIGHT), 
                    (255, 255, 255), 2)
            
            left_bound = FRAME_WIDTH//2 - CENTER_TOLERANCE
            cv2.line(frame, (left_bound, 0), (left_bound, FRAME_HEIGHT), 
                    (0, 255, 255), 2)
            
            right_bound = FRAME_WIDTH//2 + CENTER_TOLERANCE
            cv2.line(frame, (right_bound, 0), (right_bound, FRAME_HEIGHT), 
                    (0, 255, 255), 2)
            
            status = f"Frame: {frame_count} | Dist: {dist_str} | Person: {'YES' if largest else 'NO'} | ObstAvoid: {'ACTIVE' if obstacle_avoidance_active else 'IDLE'}"
            cv2.putText(frame, status, (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            
            action_color = (0, 255, 255) if "STOP" in last_action else (0, 255, 0) if "OBSTACLE" in last_action else (0, 165, 255)
            cv2.putText(frame, f"Action: {last_action}", (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, action_color, 2)

            cv2.imshow("Person Following Robot", frame)

            print(f"[FRAME {frame_count}] Action: {last_action} | ObstAvoid: {'ACTIVE' if obstacle_avoidance_active else 'IDLE'}")

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("\n⚠️  Interrupted by user")
    
    finally:
        print("\n🛑 Shutting down robot...")
        send_cmd(STOP_URL, "🛑 EMERGENCY STOP")
        time.sleep(0.3)
        vs.release()
        cv2.destroyAllWindows()
        print("✅ Robot stopped safely")

if __name__ == "__main__":
    main()