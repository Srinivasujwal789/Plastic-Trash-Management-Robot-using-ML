# Plastic-Trash-Management-Robot-using-ML

 **Overview**

This project features a Raspberry Pi-powered robot designed to detect and collect plastic trash autonomously using YOLOv8, a state-of-the-art machine learning model
for object detection. The robot identifies plastic waste (trained on a dataset from [Roboflow](https://roboflow.com/)), moves toward it, stopping 0.5 meters away, and uses 
a user-controlled robotic arm (via an MIT App Inventor app) to pick up the plastic and deposit it into an onboard bin. This system aims to assist in automated waste 
management and environmental cleanup.

 **Features**

- Plastic Detection: Uses YOLOv8 to detect plastic objects in real-time via a camera.
- Autonomous Movement: Robot navigates to within 0.5m of detected plastic using Raspberry Pi GPIO-controlled motors.
- User-Controlled Arm: Robotic arm operated through an MIT App Inventor mobile app for precise pickup.
- Dataset: Plastic detection model trained on a custom dataset from Roboflow.
- Hardware Integration: Combines Raspberry Pi, camera, motors, and servo-based robotic arm.

 **Prerequisites**

**Hardware**
- Raspberry Pi 4 (with Raspbian OS installed)
- USB Camera
- DC Motors (with motor driver, e.g., L298N) for robot movement
- Servo Motors(for robotic arm)
- Ultrasonic Sensor(e.g., HC-SR04) for distance measurement
- Power Supply: Battery pack or adapter for Raspberry Pi and motors
- Trash Bin: Attached to the robot for plastic collection

 **Software**
- Python 3.8
- Git
- MIT App Inventor: For the mobile app (see [App Inventor Project](#mit-app-inventor))
- Roboflow Account: For dataset access
