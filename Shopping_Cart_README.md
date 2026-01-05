# Automated Shopping Cart with YOLO and ESP32

## Project Overview

This project presents an **Automated Shopping Cart** that autonomously follows a person using real-time computer vision and sensor-based safety mechanisms. The system combines **YOLO-based human detection**, **ESP32 motion control**, and **ultrasonic distance sensing** to achieve smooth and safe navigation.

The cart detects a person through a camera feed, determines their position in the frame, and adjusts its movement accordingly while maintaining a safe distance and avoiding obstacles.

---

## Author

- **Muhammad Bin Shahid**

---

## Key Features

- Real-time person detection using YOLO  
- Live video stream processing from an IP camera  
- ESP32-based motion control (forward, left, right, stop)  
- Ultrasonic sensor for distance measurement  
- Automatic obstacle avoidance behavior  
- Smooth person tracking using center alignment  
- Safety stop when the person is too close or too far  
- Responsive control using multithreaded command handling  

---

## System Architecture

The system is composed of the following logical modules:

- **Image Processing Module**  
  Captures and resizes frames from the camera stream.

- **Object Detection Module**  
  Identifies humans in each frame using YOLO.

- **Tracking & Decision Module**  
  Computes the horizontal offset of the detected person and decides movement direction.

- **Ultrasonic Safety Module**  
  Continuously monitors distance to ensure safe operation.

- **Control Module**  
  Sends movement commands to the ESP32 over HTTP.

---

## How It Works

1. The camera captures a live video stream.  
2. Each frame is processed to detect a person.  
3. The largest detected person is selected as the target.  
4. The system calculates the person’s position relative to the frame center.  
5. Ultrasonic sensor data is used to maintain a safe following distance.  
6. The cart moves forward, turns left/right, or stops based on alignment and distance.  
7. If no person is detected, the cart stops automatically.  
8. Obstacle avoidance is triggered when an object is detected too close.

---

## Challenges and Learnings

- Handling camera stream latency  
- Improving tracking stability with smoothing techniques  
- Filtering noisy ultrasonic sensor readings  
- Synchronizing movement commands safely  
- Resolving hardware integration issues  

---

## Demonstration

A demonstration video of the working prototype is included in the original project submission.

---

## Conclusion

This project successfully demonstrates the integration of **computer vision**, **embedded systems**, and **sensor-based safety** to build an intelligent autonomous shopping cart. The system reliably follows a person, adapts to movement, and maintains safety through distance monitoring and obstacle avoidance.

---

## References

- Ultralytics YOLO Documentation  
- ESP32 HTTP Server Documentation  
- OpenCV Documentation  
