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
import RPi.GPIO as GPIO
import mycamera
import time

# -------------------------------
# GPIO Pin Definition (BCM)
# -------------------------------
PWMA = 18; AIN1 = 22; AIN2 = 27
PWMB = 23; BIN1 = 25; BIN2 = 24
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

def motor_back_drive(left_speed: int, right_speed: int):
    """양쪽 모터를 후진 방향으로 차동 제어"""
    L_Motor.ChangeDutyCycle(left_speed)
    GPIO.output(AIN2, False); GPIO.output(AIN1, True)
    R_Motor.ChangeDutyCycle(right_speed)
    GPIO.output(BIN2, False); GPIO.output(BIN1, True)

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

# -------------------------------
# Main Loop
# -------------------------------
def old_auto_main():
    # 자동주행 코드는 사용하지 않음
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