# ===============================
# Line Tracing Robot (Raspberry Pi)
# Camera + OpenCV + PWM Motor Control
# Version: 1.7 (최대 속도 최적화 버전)
# Features: 
# - 해상도 640x480 (연산 속도 극대화로 모터 지연 최소화)
# - BASE_SPEED 100 (최대 출력 주행)
# - ROI 상/하 분리 및 고속 차동 조향
# ===============================

import cv2
import numpy as np
import RPi.GPIO as GPIO
from gpiozero import TonalBuzzer
import mycamera
import time

# -------------------------------
# GPIO Pin Definition (BCM)
# -------------------------------
PWMA = 18; AIN1 = 22; AIN2 = 27
PWMB = 23; BIN1 = 25; BIN2 = 24
BUZZER_PIN = 12

# -------------------------------
# Motor Control Functions
# -------------------------------
def motor_drive(left_speed: int, right_speed: int):
    """양쪽 모터를 모두 전진 방향으로 두고 속도만 차등 제어"""
    L_Motor.ChangeDutyCycle(left_speed)
    GPIO.output(AIN2, True); GPIO.output(AIN1, False)
    R_Motor.ChangeDutyCycle(right_speed)
    GPIO.output(BIN2, True); GPIO.output(BIN1, False)

def motor_go(speed: int):
    motor_drive(speed, speed)

def motor_right_pivot(speed: int):
    """90도 급커브용 제자리 우회전"""
    L_Motor.ChangeDutyCycle(speed)
    GPIO.output(AIN2, True); GPIO.output(AIN1, False)
    R_Motor.ChangeDutyCycle(speed)
    GPIO.output(BIN2, False); GPIO.output(BIN1, True)

def motor_left_pivot(speed: int):
    """90도 급커브용 제자리 좌회전"""
    L_Motor.ChangeDutyCycle(speed)
    GPIO.output(AIN2, False); GPIO.output(AIN1, True)
    R_Motor.ChangeDutyCycle(speed)
    GPIO.output(BIN2, True); GPIO.output(BIN1, False)

# -------------------------------
# Buzzer Control
# -------------------------------
buzzer_active = False
buzzer_deadline = 0.0


def buzzer_beep(duration: float = 0.2):
    global buzzer_active, buzzer_deadline
    if buzzer_active:
        return

    BUZZER.play(261)
    buzzer_active = True
    buzzer_deadline = time.time() + duration


def buzzer_update():
    global buzzer_active
    if buzzer_active and time.time() >= buzzer_deadline:
        BUZZER.stop()
        buzzer_active = False


def detect_traffic_light(light_roi):
    hsv = cv2.cvtColor(light_roi, cv2.COLOR_BGR2HSV)
    red_mask1 = cv2.inRange(hsv, np.array([0, 120, 80]), np.array([10, 255, 255]))
    red_mask2 = cv2.inRange(hsv, np.array([160, 120, 80]), np.array([179, 255, 255]))
    red_mask = cv2.bitwise_or(red_mask1, red_mask2)
    orange_mask = cv2.inRange(hsv, np.array([10, 120, 80]), np.array([25, 255, 255]))
    green_mask = cv2.inRange(hsv, np.array([40, 80, 80]), np.array([90, 255, 255]))

    red_area = cv2.countNonZero(red_mask)
    orange_area = cv2.countNonZero(orange_mask)
    green_area = cv2.countNonZero(green_mask)
    area = light_roi.shape[0] * light_roi.shape[1]
    threshold = area * 0.02

    if red_area > threshold:
        return 'red'
    if orange_area > threshold:
        return 'orange'
    if green_area > threshold:
        return 'green'
    return None


def get_cx_cy_angle(roi):
    M = cv2.moments(roi)
    if M['m00'] == 0:
        return None, None, None

    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])

    pts = cv2.findNonZero(roi)
    if pts is None:
        return None, None, None

    x, y, width, height = cv2.boundingRect(pts)
    line_type = 'horizontal' if width > height or width > (roi.shape[1] * 0.4) else 'vertical'
    return cx, cy, line_type

# -------------------------------
# GPIO / PWM Setup
# -------------------------------
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup([AIN2, AIN1, PWMA, BIN1, BIN2, PWMB], GPIO.OUT, initial=GPIO.LOW)

# 모터 응답성을 높이기 위해 주파수를 100Hz에서 500Hz로 상향
L_Motor = GPIO.PWM(PWMA, 500)
R_Motor = GPIO.PWM(PWMB, 500)
L_Motor.start(0)
R_Motor.start(0)
BUZZER = TonalBuzzer(BUZZER_PIN)

# -------------------------------
# Main Loop
# -------------------------------
def main():
    # 고속 프레임 처리를 위해 해상도를 320x240으로 낮춰 연산량을 줄임
    camera = mycamera.MyPiCamera(320, 240)

    BASE_SPEED = 100       # 최대 속도 설정
    DEADBAND = 20         # 고속 주행 시 자잘한 흔들림 방지를 위해 데드밴드 약간 확장
    KP = 0.4              # 고속 조향을 위한 감도 조정
    CORNER_THRESHOLD = 35 # 320x240 해상도에 맞게 코너 임계값 유지
    last_light_state = None
    last_beep_time = 0.0
    frame_count = 0

    try:
        while camera.isOpened():
            ret, frame = camera.read()
            if not ret:
                break

            frame = cv2.flip(frame, -1)
            frame_count += 1

            buzzer_update()

            # -------------------
            # ROI 분할 (320x240 해상도 기준 최적화)
            # -------------------
            light_roi = frame[0:80, :]
            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            _, binary_frame = cv2.threshold(gray_frame, 130, 255, cv2.THRESH_BINARY_INV)
            binary_frame = cv2.erode(binary_frame, None, iterations=1)
            binary_frame = cv2.dilate(binary_frame, None, iterations=1)
            roi_top = binary_frame[80:160, :]    # 상단 ROI
            roi_bottom = binary_frame[160:, :]   # 하단 ROI
            h_b, w_b = roi_bottom.shape[:2]
            center_x_b = w_b // 2

            current_light = last_light_state
            if frame_count % 10 == 0:
                current_light = detect_traffic_light(light_roi)

            cx_b, cy_b, type_b = get_cx_cy_angle(roi_bottom)
            cx_t, cy_t, type_t = get_cx_cy_angle(roi_top)

            # -------------------
            # 신호등 제어 및 라인트레이싱 유지
            # -------------------
            if current_light == 'red':
                if last_light_state != 'red':
                    buzzer_beep(0.2)
                    last_beep_time = time.time()
                motor_go(0)
            elif current_light == 'orange':
                now = time.time()
                if now - last_beep_time >= 1.0:
                    buzzer_beep(0.1)
                    last_beep_time = now

            # 초록/노랑 모두 라인트레이싱 유지
            if current_light != 'red':
                if cx_b is not None:
                    error = center_x_b - cx_b
                    abs_err = abs(error)

                    # 상황 1: 직각 코너 감지 시 피벗 턴
                    if cx_t is not None and abs(cx_t - cx_b) > CORNER_THRESHOLD:
                        if cx_t < cx_b:
                            motor_left_pivot(BASE_SPEED)
                        else:
                            motor_right_pivot(BASE_SPEED)

                    # 상황 2: 일반 세로선 주행 (최대 속도 유지형 차동 조향)
                    elif type_b == 'vertical':
                        if abs_err <= DEADBAND:
                            motor_go(BASE_SPEED)
                        else:
                            # 한쪽 속도를 깎아서 방향을 틀 때, 기본 속도가 100이므로 
                            # 한쪽 바퀴만 감속하여 고속으로 회전하도록 유도
                            if error > 0: # 라인이 왼쪽에 있음 -> 좌회전 필요 (왼쪽 바퀴 감속)
                                left_target = BASE_SPEED - int(KP * abs_err)
                                right_target = BASE_SPEED
                            else:         # 라인이 오른쪽에 있음 -> 우회전 필요 (오른쪽 바퀴 감속)
                                left_target = BASE_SPEED
                                right_target = BASE_SPEED - int(KP * abs_err)

                            left_target = max(0, min(100, left_target))
                            right_target = max(0, min(100, right_target))

                            motor_drive(left_target, right_target)

                    # 상황 3: 횡선(교차로) 감지 시 (고속 돌파를 위해 감속 비율 완화)
                    elif type_b == 'horizontal':
                        motor_go(int(BASE_SPEED * 0.8)) # 80%의 속도로 빠르게 통과

                else:
                    motor_go(0)

            last_light_state = current_light

           # -------------------
            # 디버그 화면 출력
            # -------------------
            cv2.imshow('Camera', frame)
            cv2.imshow('Binary', binary_frame)
            cv2.imshow('Top ROI', roi_top)
            cv2.imshow('Bottom ROI', roi_bottom)
            cv2.imshow('Traffic ROI', light_roi)

            if cx_b is not None:
                cv2.circle(frame, (cx_b, cy_b + 160), 5, (0, 0, 255), -1)
            if cx_t is not None:
                cv2.circle(frame, (cx_t, cy_t + 80), 5, (255, 0, 0), -1)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        motor_go(0)
        L_Motor.stop()
        R_Motor.stop()
        BUZZER.stop()
        GPIO.cleanup()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()