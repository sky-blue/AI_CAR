# ===============================
# Line Tracing Robot (Raspberry Pi)
# Camera + OpenCV + PWM Motor Control
# Version: 2.2 (Dual black line center tracking)
# Features:
# - 두 검은 선 사이의 가상 중심선을 따라감
# - 한쪽 선만 보일 때 반대쪽 선이 보이도록 회전
# - ROI를 하단부에 집중해 라인 추적 안정화
# ===============================

import cv2
import RPi.GPIO as GPIO
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


# -------------------------------
# Motor Control Functions
# -------------------------------
def motor_go(speed: int):
    L_Motor.ChangeDutyCycle(speed)
    R_Motor.ChangeDutyCycle(speed)

    GPIO.output(AIN1, False)
    GPIO.output(AIN2, True)

    GPIO.output(BIN1, False)
    GPIO.output(BIN2, True)


def motor_right(speed: int):
    L_Motor.ChangeDutyCycle(speed)
    R_Motor.ChangeDutyCycle(speed)

    GPIO.output(AIN1, False)
    GPIO.output(AIN2, True)

    GPIO.output(BIN1, True)
    GPIO.output(BIN2, False)


def motor_left(speed: int):
    L_Motor.ChangeDutyCycle(speed)
    R_Motor.ChangeDutyCycle(speed)

    GPIO.output(AIN1, True)
    GPIO.output(AIN2, False)

    GPIO.output(BIN1, False)
    GPIO.output(BIN2, True)


# -------------------------------
# GPIO / PWM Setup
# -------------------------------
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup([AIN2, AIN1, PWMA, BIN1, BIN2, PWMB], GPIO.OUT, initial=GPIO.LOW)

L_Motor = GPIO.PWM(PWMA, 100)
R_Motor = GPIO.PWM(PWMB, 100)
L_Motor.start(0)
R_Motor.start(0)


# -------------------------------
# Main Loop
# -------------------------------
def main():
    camera = mycamera.MyPiCamera(1280, 720)

    BASE_SPEED = 70
    DEADBAND = 18
    KP = 0.30
    MIN_LINE_AREA = 250

    def detect_lines(roi):
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        _, thresh = cv2.threshold(blur, 100, 255, cv2.THRESH_BINARY_INV)

        mask = cv2.erode(thresh, None, iterations=1)
        mask = cv2.dilate(mask, None, iterations=2)

        contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detected = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < MIN_LINE_AREA:
                continue

            M = cv2.moments(contour)
            if M['m00'] == 0:
                continue

            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            detected.append((cx, cy, area))

        if not detected:
            return []

        detected.sort(key=lambda item: item[2], reverse=True)
        detected = detected[:2]
        detected.sort(key=lambda item: item[0])
        return detected

    try:
        while camera.isOpened():
            ret, frame = camera.read()
            if not ret:
                break

            frame = cv2.flip(frame, -1)

            roi = frame[360:, :]
            h, w = roi.shape[:2]
            center_x = w // 2

            lines = detect_lines(roi)

            if len(lines) >= 2:
                left_cx = lines[0][0]
                right_cx = lines[1][0]
                target_cx = (left_cx + right_cx) // 2
                error = center_x - target_cx

                if abs(error) <= DEADBAND:
                    motor_go(BASE_SPEED)
                elif error > 0:
                    steer_speed = int(min(100, BASE_SPEED + KP * abs(error)))
                    motor_left(steer_speed)
                else:
                    steer_speed = int(min(100, BASE_SPEED + KP * abs(error)))
                    motor_right(steer_speed)

                print(f"two lines -> target={target_cx}, error={error}")

            elif len(lines) == 1:
                single_cx = lines[0][0]
                if single_cx < center_x:
                    motor_right(BASE_SPEED)
                    print("one line on left -> rotate right to find the other line")
                else:
                    motor_left(BASE_SPEED)
                    print("one line on right -> rotate left to find the other line")

            else:
                motor_go(0)
                print("no line detected")

            # -------------------
            # 디버그 화면
            # -------------------
            for cx, cy, _ in lines:
                cv2.circle(roi, (cx, cy), 8, (0, 255, 0), -1)

            if len(lines) >= 2:
                left_cx = lines[0][0]
                right_cx = lines[1][0]
                target_cx = (left_cx + right_cx) // 2
                cv2.line(roi, (target_cx, 0), (target_cx, h), (255, 0, 0), 2)
                cv2.line(roi, (left_cx, 0), (left_cx, h), (0, 0, 255), 1)
                cv2.line(roi, (right_cx, 0), (right_cx, h), (0, 0, 255), 1)

            cv2.imshow('camera', frame)
            cv2.imshow('ROI', roi)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        motor_go(0)
        L_Motor.stop()
        R_Motor.stop()
        GPIO.cleanup()
        cv2.destroyAllWindows()


# -------------------------------
# Entry Point
# -------------------------------
if __name__ == "__main__":
    main()
