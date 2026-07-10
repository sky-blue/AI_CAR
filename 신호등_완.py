# ===============================
# Line Tracing Robot (Raspberry Pi)
# boost2.py baseline + traffic color control
# ===============================

import time

import cv2
import numpy as np
import RPi.GPIO as GPIO
from gpiozero import TonalBuzzer

import mycamera


# -------------------------------
# GPIO Pin Definition (BCM)
# -------------------------------
PWMA = 18
AIN1 = 22
AIN2 = 27
PWMB = 23
BIN1 = 25
BIN2 = 24
BUZZER_PIN = 12


# -------------------------------
# Motor Control Functions
# -------------------------------
def motor_drive(left_speed: int, right_speed: int, left_dir=True, right_dir=True):
    left_speed = max(0, min(100, abs(left_speed)))
    right_speed = max(0, min(100, abs(right_speed)))

    L_Motor.ChangeDutyCycle(left_speed)
    GPIO.output(AIN2, left_dir)
    GPIO.output(AIN1, not left_dir)
    R_Motor.ChangeDutyCycle(right_speed)
    GPIO.output(BIN2, right_dir)
    GPIO.output(BIN1, not right_dir)


def motor_go(speed: int):
    motor_drive(speed, speed, True, True)


def motor_right_pivot(speed: int):
    motor_drive(speed, speed, True, False)


def motor_left_pivot(speed: int):
    motor_drive(speed, speed, False, True)


# -------------------------------
# Buzzer Control
# -------------------------------
horn_remaining = 0
horn_on = False
horn_next_time = 0.0


def horn_start(count: int):
    global horn_remaining, horn_on, horn_next_time
    horn_remaining = max(0, count)
    horn_on = False
    horn_next_time = time.time()
    BUZZER.stop()


def horn_update(on_time: float = 0.10, off_time: float = 0.10):
    global horn_remaining, horn_on, horn_next_time

    if horn_remaining <= 0 and not horn_on:
        return

    now = time.time()
    if now < horn_next_time:
        return

    if horn_on:
        BUZZER.stop()
        horn_on = False
        horn_remaining -= 1
        horn_next_time = now + off_time
    elif horn_remaining > 0:
        BUZZER.play(261)
        horn_on = True
        horn_next_time = now + on_time


# -------------------------------
# Color Cube Detection
# -------------------------------
def _largest_blob(mask):
    mask = cv2.erode(mask, kernel=None, iterations=1)
    mask = cv2.dilate(mask, kernel=None, iterations=2)
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return 0, None
    contour = max(cnts, key=cv2.contourArea)
    return cv2.contourArea(contour), cv2.boundingRect(contour)


def detect_color_cube(cube_roi):
    hsv = cv2.cvtColor(cube_roi, cv2.COLOR_BGR2HSV)

    red_mask1 = cv2.inRange(hsv, np.array([0, 120, 80]), np.array([10, 255, 255]))
    red_mask2 = cv2.inRange(hsv, np.array([160, 120, 80]), np.array([179, 255, 255]))
    red_mask = cv2.bitwise_or(red_mask1, red_mask2)

    # Yellow only. The orange floor is avoided by keeping the hue range narrow.
    yellow_mask = cv2.inRange(hsv, np.array([26, 110, 110]), np.array([35, 255, 255]))
    green_mask = cv2.inRange(hsv, np.array([40, 80, 80]), np.array([90, 255, 255]))

    blobs = {
        "red": _largest_blob(red_mask),
        "yellow": _largest_blob(yellow_mask),
        "green": _largest_blob(green_mask),
    }
    color, (color_area, bbox) = max(blobs.items(), key=lambda item: item[1][0])

    # A floor cube is much smaller than a traffic light ROI. Tune this first if needed.
    if color_area >= 120:
        return color, bbox, color_area
    return None, None, color_area


# -------------------------------
# Line Analysis
# -------------------------------
def get_cx_cy_angle(roi):
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    _, thresh = cv2.threshold(blur, 130, 255, cv2.THRESH_BINARY)

    mask = cv2.erode(thresh, kernel=None, iterations=2)
    mask = cv2.dilate(mask, kernel=None, iterations=2)

    cnts, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if cnts:
        c = max(cnts, key=cv2.contourArea)
        M = cv2.moments(c)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])

            _, _, w, h = cv2.boundingRect(c)
            if w > h or w > (roi.shape[1] * 0.4):
                line_type = "horizontal"
            else:
                line_type = "vertical"
            return cx, cy, line_type
    return None, None, None


def apply_line_drive(speed: int, frame, roi_top, roi_bottom):
    w_b = roi_bottom.shape[1]
    center_x_b = w_b // 2

    cx_b, cy_b, type_b = get_cx_cy_angle(roi_bottom)
    cx_t, cy_t, _ = get_cx_cy_angle(roi_top)

    deadband = 15
    kp = 0.5
    corner_threshold = 35

    if cx_b is None:
        motor_go(0)
        return cx_b, cy_b, cx_t, cy_t

    error = center_x_b - cx_b
    abs_err = abs(error)

    if cx_t is not None and abs_err <= deadband and abs(center_x_b - cx_t) <= deadband:
        motor_go(speed)
        cv2.putText(frame, "BOOST STRAIGHT!", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    elif cx_t is not None and abs(cx_t - cx_b) > corner_threshold:
        if cx_t < cx_b:
            motor_left_pivot(speed)
        else:
            motor_right_pivot(speed)
    elif type_b == "vertical":
        if abs_err <= deadband:
            motor_go(speed)
        else:
            steering_p = int(kp * abs_err)

            if error > 0:
                left_target = speed - steering_p
                right_target = speed
                left_dir = left_target >= 0
                motor_drive(abs(left_target), right_target, left_dir, True)
            else:
                left_target = speed
                right_target = speed - steering_p
                right_dir = right_target >= 0
                motor_drive(left_target, abs(right_target), True, right_dir)
    elif type_b == "horizontal":
        motor_go(int(speed * 0.8))

    return cx_b, cy_b, cx_t, cy_t


# -------------------------------
# GPIO / PWM Setup
# -------------------------------
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup([AIN2, AIN1, PWMA, BIN1, BIN2, PWMB], GPIO.OUT, initial=GPIO.LOW)

L_Motor = GPIO.PWM(PWMA, 500)
R_Motor = GPIO.PWM(PWMB, 500)
L_Motor.start(0)
R_Motor.start(0)
BUZZER = TonalBuzzer(BUZZER_PIN)


# -------------------------------
# Main Loop
# -------------------------------
def main():
    camera = mycamera.MyPiCamera(640, 480)

    base_speed = 100
    slow_speed = int(base_speed * 0.8)
    traffic_state = "GO"
    frame_count = 0

    try:
        while camera.isOpened():
            ret, frame = camera.read()
            if not ret:
                break

            frame = cv2.flip(frame, -1)
            frame_count += 1
            horn_update()

            cube_roi_x = 80
            cube_roi_y = 120
            cube_roi = frame[cube_roi_y:420, cube_roi_x:560]
            roi_top = frame[160:320, :]
            roi_bottom = frame[320:, :]

            detected_color, detected_bbox, detected_area = detect_color_cube(cube_roi)

            if detected_color == "red" and traffic_state != "STOP":
                traffic_state = "STOP"
                horn_start(1)
            elif detected_color == "green":
                traffic_state = "GO"
            elif detected_color == "yellow" and traffic_state != "STOP" and traffic_state != "SLOW":
                traffic_state = "SLOW"
                horn_start(5)

            if traffic_state == "STOP":
                motor_go(0)
                cv2.putText(frame, "RED STOP", (20, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            else:
                current_speed = slow_speed if traffic_state == "SLOW" else base_speed
                cx_b, cy_b, cx_t, cy_t = apply_line_drive(current_speed, frame, roi_top, roi_bottom)

                if traffic_state == "SLOW":
                    cv2.putText(frame, "YELLOW SLOW", (20, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
                else:
                    cv2.putText(frame, "GREEN GO", (20, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

                if cx_b is not None:
                    cv2.circle(roi_bottom, (cx_b, cy_b), 5, (0, 0, 255), -1)
                if cx_t is not None:
                    cv2.circle(roi_top, (cx_t, cy_t), 5, (255, 0, 0), -1)

            cv2.rectangle(frame, (80, cube_roi_y), (560, 420), (255, 255, 0), 1)
            if detected_color is not None:
                x, y, w, h = detected_bbox
                x1 = cube_roi_x + x
                y1 = cube_roi_y + y
                x2 = x1 + w
                y2 = y1 + h
                box_color = {
                    "red": (0, 0, 255),
                    "yellow": (0, 255, 255),
                    "green": (0, 255, 0),
                }[detected_color]
                cv2.rectangle(frame, (x1, y1), (x2, y2), box_color, 3)
                cv2.putText(frame, f"CUBE: {detected_color} area={int(detected_area)}", (20, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.8, box_color, 2)
            else:
                cv2.putText(frame, f"CUBE: none area={int(detected_area)}", (20, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (200, 200, 200), 2)

            cv2.imshow("Race Color Frame", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    finally:
        motor_go(0)
        L_Motor.stop()
        R_Motor.stop()
        BUZZER.stop()
        if hasattr(camera, "release"):
            camera.release()
        GPIO.cleanup()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
