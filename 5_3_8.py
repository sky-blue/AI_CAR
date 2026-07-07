# ===============================
# Line Tracing Robot (Raspberry Pi)
# Camera + OpenCV + PWM Motor Control
# Version: 1.5
# Features: 
# - 해상도 1280x720
# - ROI 상/하 분리
# - 90도 코너 감지
# - 횡선 따라가기
# - 하단 ROI 주행 + 상단 ROI 코너 통합
# ===============================

import cv2
import numpy as np
import RPi.GPIO as GPIO
import mycamera

# -------------------------------
# GPIO Pin Definition (BCM)
# -------------------------------
PWMA = 18; AIN1 = 22; AIN2 = 27
PWMB = 23; BIN1 = 25; BIN2 = 24

# -------------------------------
# Motor Control Functions
# -------------------------------
def motor_go(speed: int):
    L_Motor.ChangeDutyCycle(speed)
    GPIO.output(AIN2, True); GPIO.output(AIN1, False)
    R_Motor.ChangeDutyCycle(speed)
    GPIO.output(BIN2, True); GPIO.output(BIN1, False)

def motor_right(speed: int):
    L_Motor.ChangeDutyCycle(speed)
    GPIO.output(AIN2, True); GPIO.output(AIN1, False)
    R_Motor.ChangeDutyCycle(0)
    GPIO.output(BIN2, False); GPIO.output(BIN1, True)

def motor_left(speed: int):
    L_Motor.ChangeDutyCycle(0)
    GPIO.output(AIN2, False); GPIO.output(AIN1, True)
    R_Motor.ChangeDutyCycle(speed)
    GPIO.output(BIN2, True); GPIO.output(BIN1, False)

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

    BASE_SPEED = 100
    DEADBAND = 12
    KP = 0.25
    CORNER_THRESHOLD = 40  # 상단 ROI와 하단 ROI cx 차이 임계값

    try:
        while camera.isOpened():
            ret, frame = camera.read()
            if not ret:
                break

            frame = cv2.flip(frame, -1)

            # -------------------
            # ROI 분할
            # -------------------
            roi_top = frame[240:480, :]    # 상단 ROI (미래 라인)
            roi_bottom = frame[480:, :]    # 하단 ROI (현재 라인)
            h_b, w_b = roi_bottom.shape[:2]
            center_x_b = w_b // 2

            # -------------------
            # cx 계산 함수 + 각도 판단 (세로/횡 구분)
            # -------------------
            def get_cx_cy_angle(roi):
                gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
                blur = cv2.GaussianBlur(gray, (5,5), 0)
                _, thresh = cv2.threshold(blur, 130, 255, cv2.THRESH_BINARY)
                mask = cv2.erode(thresh, None, iterations=2)
                mask = cv2.dilate(mask, None, iterations=2)
                cnts, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
                if cnts:
                    c = max(cnts, key=cv2.contourArea)
                    M = cv2.moments(c)
                    if M['m00'] != 0:
                        cx = int(M['m10']/M['m00'])
                        cy = int(M['m01']/M['m00'])
                        # 최소 외접 직사각형으로 각도 구하기
                        rect = cv2.minAreaRect(c)
                        width, height = rect[1]
                        angle = rect[2]
                        # 가로선/세로선 판단
                        if width > height:
                            line_type = 'horizontal'
                        else:
                            line_type = 'vertical'
                        return cx, cy, line_type
                return None, None, None

            cx_b, cy_b, type_b = get_cx_cy_angle(roi_bottom)
            cx_t, cy_t, type_t = get_cx_cy_angle(roi_top)

            # -------------------
            # 조향 판단
            # -------------------
            if cx_b is not None:
                error = center_x_b - cx_b
                abs_err = abs(error)

                corner_detected = False
                if cx_t is not None and abs(cx_t - cx_b) > CORNER_THRESHOLD:
                    corner_detected = True
                    if cx_t < cx_b:
                        motor_left(BASE_SPEED)
                        print(f"Corner LEFT detected | bottom_cx={cx_b}, top_cx={cx_t}")
                    else:
                        motor_right(BASE_SPEED)
                        print(f"Corner RIGHT detected | bottom_cx={cx_b}, top_cx={cx_t}")

                if not corner_detected:
                    if type_b == 'vertical':
                        # 세로선 → 기존 좌우 조향
                        if abs_err <= DEADBAND:
                            motor_go(BASE_SPEED)
                        elif error > 0:
                            steer_speed = int(min(100, BASE_SPEED + KP * abs_err))
                            motor_left(steer_speed)
                        else:
                            steer_speed = int(min(100, BASE_SPEED + KP * abs_err))
                            motor_right(steer_speed)
                    elif type_b == 'horizontal':
                        # 횡선 → 횡선 따라 이동
                        # cx 기준으로 좌우 조향 그대로 사용하거나 속도 줄여 직진
                        motor_go(BASE_SPEED)
                        print(f"Horizontal line detected → following | bottom_cx={cx_b}")

            else:
                motor_go(0)

            # -------------------
            # 디버그 화면
            # -------------------
            cv2.imshow('normal', frame)
            cv2.imshow('top ROI', roi_top)
            cv2.imshow('bottom ROI', roi_bottom)

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
