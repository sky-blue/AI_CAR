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


def largest_blob(mask):

    kernel = np.ones((5,5),np.uint8)

    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_CLOSE,
        kernel
    )

    mask = cv2.erode(mask,None,iterations=1)
    mask = cv2.dilate(mask,None,iterations=2)


    cnts, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    if not cnts:
        return 0, None

    c = max(cnts, key=cv2.contourArea)

    return cv2.contourArea(c), cv2.boundingRect(c)



def detect_traffic_light(light_roi):

    hsv = cv2.cvtColor(light_roi, cv2.COLOR_BGR2HSV)


    red1 = cv2.inRange(
        hsv,
        np.array([0,120,80]),
        np.array([10,255,255])
    )

    red2 = cv2.inRange(
        hsv,
        np.array([160,120,80]),
        np.array([179,255,255])
    )

    masks = {
        "red": cv2.bitwise_or(red1,red2),

        "orange": cv2.inRange(
            hsv,
            np.array([15,70,170]),
            np.array([30,130,220])
        ),

        "green": cv2.inRange(
            hsv,
            np.array([35,60,60]),
            np.array([90,255,255])
        )
    }


    result = {}

    for color,mask in masks.items():
        result[color] = largest_blob(mask)


    color,(area,bbox)=max(
        result.items(),
        key=lambda x:x[1][0]
    )


    if area > 150:
        return color,bbox,area


    return None,None,area

def get_cx_cy_angle(mask):

    cnts, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_NONE
    )

    if not cnts:
        return None, None, None

    c = max(cnts, key=cv2.contourArea)

    M = cv2.moments(c)

    if M["m00"] == 0:
        return None, None, None

    cx = int(M["m10"]/M["m00"])
    cy = int(M["m01"]/M["m00"])

    x, y, w, h = cv2.boundingRect(c)

    if w > h or w > mask.shape[1] * 0.4:
        line_type = "horizontal"
    else:
        line_type = "vertical"

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
    KP = 0.25      # 기존보다 약간 증가
    KD = 0.45      # 미분 게인
    DEADBAND = 15  # 기존 30 → 15 추천
    last_error = 0
    last_light_box = None
    last_light_time = 0
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
            light_roi = frame[90:200, 250:390]
            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            blur_frame = cv2.GaussianBlur(gray_frame, (5, 5), 0)
            _, binary_frame = cv2.threshold(blur_frame, 100, 255, cv2.THRESH_BINARY)
            binary_frame = cv2.erode(binary_frame, None, iterations=1)
            binary_frame = cv2.dilate(binary_frame, None, iterations=1)

            roi_top_frame = binary_frame[240:360, :]
            roi_bottom_frame = binary_frame[360:480, :]
            roi_top = frame[240:360, :]
            roi_bottom = frame[360:480, :]
            h_b, w_b = roi_bottom.shape[:2]
            center_x_b = w_b // 2

            current_light = last_light_state
            

            if frame_count % 5 == 0:
                detected_light, detected_box, area = detect_traffic_light(light_roi)

                if detected_light is not None:
                    current_light = detected_light
                    last_light_box = detected_box
                    last_light_time = time.time()
                if time.time() - last_light_time > 1.0:
                    last_light_box = None

            action = "NONE"

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
                action = "STOP"
            elif current_light == 'orange':
                now = time.time()

                if now - last_beep_time >= 1.0:
                    buzzer_beep(0.1)
                    last_beep_time = now

                action = "BUZZER"
            elif current_light == 'green':
                action = "GO"
            else:
                action = "NONE"

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
                    last_error = 0
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

                        # 현재 오차
                        error = center_x_b - cx_b

                        # 변화량(D)
                        d_error = error - last_error

                        # PD 계산
                        steering = KP * error + KD * d_error

                        # 다음 프레임을 위해 저장
                        last_error = error

                        # 데드밴드
                        if abs(error) <= DEADBAND:
                            motor_go(BASE_SPEED)

                        else:

                            left_target = BASE_SPEED - steering
                            right_target = BASE_SPEED + steering

                            left_target = int(max(0, min(100, left_target)))
                            right_target = int(max(0, min(100, right_target)))

                            if error > 0:
                                last_turn_dir = 'left'
                            else:
                                last_turn_dir = 'right'

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
                last_error = 0

            last_light_state = current_light

           # -------------------
            # 디버그 화면 출력
            # -------------------
            # cv2.imshow('Binary', binary_frame)

            if cx_b is not None:
                cv2.circle(frame, (cx_b, cy_b + 360), 5, (0, 0, 255), -1)

            if cx_t is not None:
                cv2.circle(frame, (cx_t, cy_t + 240), 5, (255, 0, 0), -1)


            if last_light_box is not None:
                x, y, w, h = last_light_box

                # ROI 좌표 -> 원본 프레임 좌표 변환
                roi_x_offset = 250
                roi_y_offset = 90

                x += roi_x_offset
                y += roi_y_offset

                color_map = {
                    "red": (0,0,255),
                    "orange": (0,165,255),
                    "green": (0,255,0)
                }

                cv2.rectangle(
                    frame,
                    (x,y),
                    (x+w,y+h),
                    color_map.get(current_light, (255,255,255)),
                    2
                )

            text_color = {
                "red": (0,0,255),
                "orange": (0,165,255),
                "green": (0,255,0),
                None: (255,255,255)
            }

            cv2.putText(
                frame,
                f"LIGHT : {current_light}",
                (10,30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                text_color.get(current_light, (255,255,255)),
                2
            )

            cv2.putText(
                frame,
                f"ACTION : {action}",
                (10,60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                text_color.get(current_light, (255,255,255)),
                2
            )
            # -------------------
            # ROI 박스 표시
            # -------------------

            # 신호등 영역
            cv2.rectangle(
                frame,
                (250, 90),
                (390, 200),
                (255, 255, 0),
                2
            )

            # Top ROI
            cv2.rectangle(
                frame,
                (0, 240),
                (640, 360),
                (255, 0, 0),
                2
            )

            # Bottom ROI
            cv2.rectangle(
                frame,
                (0, 360),
                (640, 480),
                (0, 0, 255),
                2
            )
            cv2.imshow('Camera', frame)  
            cv2.imshow("Bottom ROI", roi_bottom_frame)

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