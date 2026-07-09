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

def motor_back(speed: int):
    """후진"""
    L_Motor.ChangeDutyCycle(speed)
    GPIO.output(AIN2, False); GPIO.output(AIN1, True)
    R_Motor.ChangeDutyCycle(speed)
    GPIO.output(BIN2, False); GPIO.output(BIN1, True)

# -------------------------------
# Buzzer Control
# -------------------------------
buzzer_active = False
buzzer_deadline = 0.0


def buzzer_beep(duration: float = 0.2):
    pass
    # global buzzer_active, buzzer_deadline
    # if buzzer_active:
    #     return

    # BUZZER.play(261)
    # buzzer_active = True
    # buzzer_deadline = time.time() + duration


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
    if roi.ndim == 3:
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    else:
        gray = roi

    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    _, thresh = cv2.threshold(blur, 130, 255, cv2.THRESH_BINARY)
    mask = cv2.erode(thresh, None, iterations=1)
    mask = cv2.dilate(mask, None, iterations=1)

    cnts, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not cnts:
        return None, None, None

    c = max(cnts, key=cv2.contourArea)
    M = cv2.moments(c)
    if M['m00'] == 0:
        return None, None, None

    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])

    x, y, w, h = cv2.boundingRect(c)
    if w > h or w > (roi.shape[1] * 0.4):
        line_type = 'horizontal'
    else:
        line_type = 'vertical'

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
    camera = mycamera.MyPiCamera(640, 480)

    BASE_SPEED = 100       # 최대 속도 설정
    DEADBAND = 30         # 고속 주행 시 자잘한 흔들림 방지를 위해 데드밴드 약간 확장
    KP = 0.2              # 고속 조향을 위한 감도 조정
    CORNER_THRESHOLD = 35 # 320x240 해상도에 맞게 코너 임계값 유지
    last_light_state = None
    last_beep_time = 0.0
    frame_count = 0
    state = 'FOLLOW'
    back_start = 0.0
    search_start = 0.0
    found_count = 0
    last_turn_dir = 'left'
    search_dir = True

    try:
        while camera.isOpened():
            ret, frame = camera.read()
            if not ret:
                break

            frame = cv2.flip(frame, -1)
            frame_count += 1

            buzzer_update()

            # -------------------
            # ROI 분할 (640x480 해상도 기준 최적화)
            # -------------------
            light_roi = frame[0:120, :]
            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            blur_frame = cv2.GaussianBlur(gray_frame, (5, 5), 0)
            _, binary_frame = cv2.threshold(blur_frame, 100, 255, cv2.THRESH_BINARY)
            binary_frame = cv2.erode(binary_frame, None, iterations=1)
            binary_frame = cv2.dilate(binary_frame, None, iterations=1)

            roi_top_frame = frame[160:320, :]
            roi_bottom_frame = frame[320:, :]
            roi_top = binary_frame[160:320, :]
            roi_bottom = binary_frame[320:, :]
            h_b, w_b = roi_bottom.shape[:2]
            center_x_b = w_b // 2

            current_light = last_light_state
            if frame_count % 10 == 0:
                current_light = detect_traffic_light(light_roi)

            cx_b, cy_b, type_b = get_cx_cy_angle(roi_bottom_frame)
            cx_t, cy_t, type_t = get_cx_cy_angle(roi_top_frame)

            # -------------------
            # 신호등 제어 및 상태 기반 라인트레이싱 유지
            # -------------------
            if current_light == 'red':
                if last_light_state != 'red':
                    buzzer_beep(0.2)
                    last_beep_time = time.time()
                state = 'STOP'
                motor_go(0)
            elif current_light == 'orange':
                now = time.time()
                if now - last_beep_time >= 1.0:
                    buzzer_beep(0.1)
                    last_beep_time = now

            if state == 'STOP':
                motor_go(0)
            elif state == 'BACK':
                motor_back(60)
                if time.time() - back_start > 0.3:
                    state = 'SEARCH'
                    search_start = time.time()
                    found_count = 0
            elif state == 'SEARCH':
                if last_turn_dir == 'left':
                    motor_left_pivot(60)
                else:
                    motor_right_pivot(60)

                if cx_b is not None:
                    found_count += 1
                else:
                    found_count = 0

                if found_count >= 3:
                    state = 'FOLLOW'
                    found_count = 0
                elif time.time() - search_start > 2.5:
                    state = 'STOP'
                    motor_go(0)
            else:
                if cx_b is not None:
                    error = center_x_b - cx_b
                    abs_err = abs(error)

                    # 상황 1: 직각 코너 감지 시 피벗 턴
                    if cx_t is not None and abs(cx_t - cx_b) > CORNER_THRESHOLD:
                        if cx_t < cx_b:
                            motor_left_pivot(BASE_SPEED)
                            last_turn_dir = 'left'
                        else:
                            motor_right_pivot(BASE_SPEED)
                            last_turn_dir = 'right'

                    # 상황 2: 일반 세로선 주행 (최대 속도 유지형 차동 조향)
                    elif type_b == 'vertical':
                        if abs_err <= DEADBAND:
                            motor_go(BASE_SPEED)
                        else:
                            if error > 0:
                                left_target = BASE_SPEED - int(KP * abs_err)
                                right_target = BASE_SPEED
                                last_turn_dir = 'left'
                            else:
                                left_target = BASE_SPEED
                                right_target = BASE_SPEED - int(KP * abs_err)
                                last_turn_dir = 'right'

                            left_target = max(0, min(100, left_target))
                            right_target = max(0, min(100, right_target))

                            motor_drive(left_target, right_target)

                    # 상황 3: 횡선(교차로) 감지 시 (고속 돌파를 위해 감속 비율 완화)
                    elif type_b == 'horizontal':
                        motor_go(int(BASE_SPEED * 0.95))
                else:
                    if search_dir:
                        motor_drive(60, 100)
                    else:
                        motor_drive(100, 60)

                    if frame_count % 5 == 0:
                        search_dir = not search_dir

                    if frame_count % 20 == 0:
                        state = 'BACK'
                        back_start = time.time()
                        found_count = 0

            if state == 'STOP' and current_light != 'red':
                state = 'FOLLOW'

            last_light_state = current_light

           # -------------------
            # 디버그 화면 출력
            # -------------------
            cv2.imshow('Binary', binary_frame)
            cv2.imshow('Bottom ROI', roi_bottom)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            # if cx_b is not None:
            #     cv2.circle(frame, (cx_b, cy_b + 320), 5, (0, 0, 255), -1)
            # if cx_t is not None:
            #     cv2.circle(frame, (cx_t, cy_t + 160), 5, (255, 0, 0), -1)

    finally:
        motor_go(0)
        L_Motor.stop()
        R_Motor.stop()
        BUZZER.stop()
        GPIO.cleanup()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()