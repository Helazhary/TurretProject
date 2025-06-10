import serial
import time
import cv2

# --- Serial Setup ---
ser = serial.Serial('/dev/ttyACM0', 9600)
time.sleep(2)

# --- Camera Setup ---
cam = cv2.VideoCapture(2) 
frame_width = int(cam.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))

# --- Face Detection ---
face_classifier = cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_frontalface_default.xml")

# --- Servo Settings ---
PAN_MIN = 40
PAN_MAX = 110
TILT_MIN = 50
TILT_MAX = 140

# Offset compensation (adjust these experimentally)
CAM_OFFSET_X = 30  # negative: camera is left of turret
CAM_OFFSET_Y = -30 # positive: camera is below turret

def map_range(value, in_min, in_max, out_min, out_max):
    # Clamp and map a value from one range to another
    value = max(min(value, in_max), in_min)
    return int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

while True:
    ret, frame = cam.read()
    if not ret:
        continue

    frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    faces = face_classifier.detectMultiScale(
        gray, scaleFactor=1.1, minNeighbors=8, minSize=(50, 50)
    )

    if len(faces) > 0:
        (x, y, w, h) = faces[0]

        cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)

        # Face center in frame
        face_cx = x + w // 2
        face_cy = y + h // 2

        # Map to servo angles
        pan_angle = map_range(face_cx, 0, frame.shape[1], PAN_MAX, PAN_MIN)
        tilt_angle = map_range(face_cy, 0, frame.shape[0], TILT_MAX, TILT_MIN)

        # Apply offsets
        pan_angle += CAM_OFFSET_X
        tilt_angle += CAM_OFFSET_Y

        # Clamp after offset
        pan_angle = max(min(pan_angle, PAN_MAX), PAN_MIN)
        tilt_angle = max(min(tilt_angle, TILT_MAX), TILT_MIN)

        print(f"Face center: ({face_cx}, {face_cy}) -> Pan: {pan_angle}, Tilt: {tilt_angle}")

        try:
            ser.write(f"{pan_angle},{tilt_angle}\n".encode())
        except Exception as e:
            print("Serial write error:", e)

    cv2.imshow("Face Tracking Turret", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cam.release()
cv2.destroyAllWindows()
ser.close()

