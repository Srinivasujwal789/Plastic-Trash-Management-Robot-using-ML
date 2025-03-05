import cv2
import numpy as np
from ultralytics import YOLO
import RPi.GPIO as GPIO
import time

# GPIO Setup for DC Motors
GPIO.setmode(GPIO.BCM)
# Motor 1 (Left)
IN1 = 17  # GPIO 17
IN2 = 18  # GPIO 18
ENA = 27  # GPIO 27 (PWM for speed)
# Motor 2 (Right)
IN3 = 22  # GPIO 22
IN4 = 23  # GPIO 23
ENB = 24  # GPIO 24 (PWM for speed)

# Initialize GPIO pins
GPIO.setup([IN1, IN2, ENA, IN3, IN4, ENB], GPIO.OUT)
pwm_left = GPIO.PWM(ENA, 100)  # 100 Hz PWM
pwm_right = GPIO.PWM(ENB, 100)
pwm_left.start(0)
pwm_right.start(0)

# Motor control functions
def move_forward(speed=50):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_left.ChangeDutyCycle(speed)
    pwm_right.ChangeDutyCycle(speed)

def move_left(speed=50):
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    pwm_left.ChangeDutyCycle(speed)
    pwm_right.ChangeDutyCycle(speed)

def move_right(speed=50):
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)
    pwm_left.ChangeDutyCycle(speed)
    pwm_right.ChangeDutyCycle(speed)

def stop():
    GPIO.output([IN1, IN2, IN3, IN4], GPIO.LOW)
    pwm_left.ChangeDutyCycle(0)
    pwm_right.ChangeDutyCycle(0)

# Load YOLOv8 model
model = YOLO("yolov8n.pt")  # Replace with your custom model path if trained for plastic

# Camera Setup
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# Constants for navigation
TARGET_CLASS = "plastic"  # Adjust based on your model's class name
FRAME_CENTER = 320  # Half of 640px width
MIN_DISTANCE_PX = 50  # Approximate pixel distance for 0.5 cm (calibrate this)

# Main loop
try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        # Run YOLOv8 inference
        results = model(frame)

        # Process detections
        plastic_detected = False
        target_x, target_y, target_width = 0, 0, 0
        for result in results:
            boxes = result.boxes
            for box in boxes:
                cls = int(box.cls[0])
                conf = float(box.conf[0])
                label = model.names[cls]

                if label == TARGET_CLASS and conf > 0.7:  # Confidence threshold
                    plastic_detected = True
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    target_x = (x1 + x2) // 2  # Center of bounding box
                    target_y = (y1 + y2) // 2
                    target_width = x2 - x1

                    # Draw bounding box and label
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(frame, f"{label} {conf:.2f}", (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                    break  # Process only the first detected plastic

        # Navigation logic
        if plastic_detected:
            print(f"Plastic detected at x: {target_x}, width: {target_width}")
            # Estimate distance (simplified: larger width = closer)
            distance_px = target_width
            if distance_px > MIN_DISTANCE_PX:  # Too far
                if target_x < FRAME_CENTER - 50:  # Plastic is left
                    move_left(40)
                    print("Turning left")
                elif target_x > FRAME_CENTER + 50:  # Plastic is right
                    move_right(40)
                    print("Turning right")
                else:  # Plastic is centered
                    move_forward(40)
                    print("Moving forward")
            else:  # Close enough (approx 0.5 cm)
                stop()
                print("Stopped near plastic")
        else:
            stop()
            print("No plastic detected")

        # Display frame
        cv2.imshow("Plastic Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("Stopped by user")

finally:
    cap.release()
    cv2.destroyAllWindows()
    stop()
    pwm_left.stop()
    pwm_right.stop()
    GPIO.cleanup()
