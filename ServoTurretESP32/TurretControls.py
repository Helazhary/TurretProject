import cv2
import serial
import time

# Adjust the port name as needed (e.g., COM3 on Windows or /dev/ttyUSB0 on Linux)
ser = serial.Serial('/dev/ttyUSB0', 115200)
time.sleep(2)  # Wait for ESP32 to reboot

# Load Haar Cascade for person detection
person_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_fullbody.xml')

# Start webcam
cap = cv2.VideoCapture(2)
frame_width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
frame_height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)

def map_range(value, in_min, in_max, out_min, out_max):
    # Simple map function
    return int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    people = person_cascade.detectMultiScale(gray, 1.1, 4)

    # Assume only 1 target for simplicity
    for (x, y, w, h) in people:
        cx = x + w // 2
        cy = y + h // 2

        # Map center coordinates to servo angles (0-180)
        pan = map_range(cx, 0, frame_width, 0, 180)
        tilt = map_range(cy, 0, frame_height, 0, 180)

        # Send command to ESP32
        command = f"pan:{pan} tilt:{tilt} fire:1\n"
        ser.write(command.encode())

        # Draw detection
        cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)
        break
    else:
        # No person found → turn off water
        ser.write(b"pan:90 tilt:90 fire:0\n")

    cv2.imshow('Tracking', frame)
    if cv2.waitKey(1) == 27:  # ESC to quit
        break

cap.release()
cv2.destroyAllWindows()
ser.close()
