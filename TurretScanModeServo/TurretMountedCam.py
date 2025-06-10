import serial
import time
import cv2

# --- Serial Setup ---
ser = serial.Serial('/dev/ttyACM0', 9600)
time.sleep(2)

# --- Camera Setup ---
cam = cv2.VideoCapture(2)  # Adjust index if needed
frame_width = int(cam.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))
center_x = frame_width // 2
center_y = frame_height // 2

# --- Face Detection ---
face_classifier = cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_frontalface_default.xml")

# --- Servo Angles ---
pan_angle = 90
tilt_angle = 90
pan_smoothed = 90
tilt_smoothed = 90
last_seen = time.time()
sweep_direction = 1

# --- Control Parameters ---
deadzone = 35
scan_delay = 2
smooth_factor = 0.7  # Face input smoothing
servo_smoothing = 0.7  # Output angle smoothing
kp = 0.03
kd = 0.1

# --- Tracking Memory ---
prev_face_x = center_x
prev_face_y = center_y
pan_error_prev = 0
tilt_error_prev = 0

while True:
    ret, frame = cam.read()
    if not ret:
        continue
    frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    faces = face_classifier.detectMultiScale(
        gray, scaleFactor=1.1, minNeighbors=8, minSize=(65, 65)
    )

    found_face = False

    for (x, y, w, h) in faces:
        raw_x = x + w // 2
        raw_y = y + h // 2

        # Smooth input
        face_x = int(smooth_factor * prev_face_x + (1 - smooth_factor) * raw_x)
        face_y = int(smooth_factor * prev_face_y + (1 - smooth_factor) * raw_y)
        prev_face_x = face_x
        prev_face_y = face_y

        # Visuals
        cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)
        cv2.circle(frame, (face_x, face_y), 5, (0, 255, 0), -1)
        cv2.line(frame, (center_x, 0), (center_x, frame_height), (0, 255, 255), 1)
        cv2.line(frame, (0, center_y), (frame_width, center_y), (0, 255, 255), 1)

        # Offsets
        offset_x = face_x - center_x
        offset_y = face_y - center_y

        # PD Control
        pan_error = offset_x
        tilt_error = offset_y
        d_pan = pan_error - pan_error_prev
        d_tilt = tilt_error - tilt_error_prev

        pan_adjust = kp * pan_error + kd * d_pan
        tilt_adjust = kp * tilt_error + kd * d_tilt

        if abs(offset_x) > deadzone:
            pan_angle -= int(pan_adjust)

        if abs(offset_y) > deadzone:
            tilt_angle -= int(tilt_adjust)

        pan_angle = max(0, min(180, pan_angle))
        tilt_angle = max(60, min(120, tilt_angle))

        # Smooth output before sending
        pan_smoothed = int(servo_smoothing * pan_smoothed + (1 - servo_smoothing) * pan_angle)
        tilt_smoothed = int(servo_smoothing * tilt_smoothed + (1 - servo_smoothing) * tilt_angle)

        try:
            ser.write(f"{pan_smoothed},{tilt_smoothed}\n".encode())
        except:
            pass

        pan_error_prev = pan_error
        tilt_error_prev = tilt_error
        last_seen = time.time()
        found_face = True
        break

    # --- Sentry Mode ---
    if not found_face and time.time() - last_seen > scan_delay:
        tilt_smoothed = 90
        pan_smoothed += 2 * sweep_direction
        if pan_smoothed >= 180:
            pan_smoothed = 180
            sweep_direction = -1
        elif pan_smoothed <= 0:
            pan_smoothed = 0
            sweep_direction = 1

        try:
            ser.write(f"{pan_smoothed},{tilt_smoothed}\n".encode())
        except:
            pass

    cv2.imshow("Face Tracking Turret", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# --- Cleanup ---
cam.release()
cv2.destroyAllWindows()
ser.close()
