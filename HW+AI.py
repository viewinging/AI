from flask import Flask, request, jsonify
import cv2 as cv
from ultralytics import YOLO
import RPi.GPIO as GPIO
import time
import threading
import requests

app = Flask(__name__)

# 글로벌 변수 초기화
should_detect = False
get_value = None

# URL 설정 (Flask 서버에서 label 전송)
label_url = "http://10.150.150.181:8080/label" # 라즈베리파이 -> Flask
auto_url = "http://10.150.150.181:8080/auto-trash" # Flask -> 라즈베리파이

# YOLO 모델 로드
model = YOLO('viewing.pt') # viewing.pt는 학습된 모델명

# GPIO 및 스텝 모터 핀 설정
GPIO.setmode(GPIO.BCM)
relay_pin = 18
GPIO.setup(relay_pin, GPIO.OUT)

step_pins_can = [5, 6, 13, 19]
for pin in step_pins_can:
    GPIO.setup(pin, GPIO.OUT)

step_pins_open = [17, 27, 22, 10]
for pin in step_pins_open:
    GPIO.setup(pin, GPIO.OUT)

step_pins_plastic = [1, 7, 8, 25]
for pin in step_pins_plastic:
    GPIO.setup(pin, GPIO.OUT)

# IN1, IN2 핀 설정 (vinyl)
IN1 = 20
IN2 = 21
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)

# 스텝 모터 시퀀스 설정
step_sequence = [
    [1, 0, 0, 1],
    [1, 0, 0, 0],
    [1, 1, 0, 0],
    [0, 1, 0, 0],
    [0, 1, 1, 0],
    [0, 0, 1, 0],
    [0, 0, 1, 1],
    [0, 0, 0, 1]
]

# 스텝 모터 동작
def step_motor_with_steps(step_pins, steps, delay, direction='forward'):
    sequence = step_sequence if direction == 'forward' else step_sequence[::-1]
    for _ in range(steps):
        for step in sequence:
            for pin in range(4):
                GPIO.output(step_pins[pin], step[pin])
            time.sleep(delay)
    for pin in step_pins:
        GPIO.output(pin, False)

# 라벨에 따른 하드웨어 동작
def perform_action_based_on_label(label):
    steps_per_revolution = 128
    steps_per_revolution1 = 256

    if label == 'vinyl':
        step_motor_with_steps(step_pins_open, steps_per_revolution, 0.001, direction='forward')
        GPIO.output(relay_pin, GPIO.HIGH)
        time.sleep(1.5)
        GPIO.output(relay_pin, GPIO.LOW)
        time.sleep(2)
        step_motor_with_steps(step_pins_open, steps_per_revolution, 0.001, direction='reverse')
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
        time.sleep(5)
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.LOW)

    elif label == 'plastic':
        step_motor_with_steps(step_pins_open, steps_per_revolution, 0.001, direction='forward')
        GPIO.output(relay_pin, GPIO.HIGH)
        time.sleep(3.5)
        GPIO.output(relay_pin, GPIO.LOW)
        time.sleep(2)
        step_motor_with_steps(step_pins_open, steps_per_revolution, 0.001, direction='reverse')
        time.sleep(1)
        step_motor_with_steps(step_pins_plastic, steps_per_revolution1, 0.001, direction='reverse')
        time.sleep(1)
        step_motor_with_steps(step_pins_plastic, steps_per_revolution1, 0.001, direction='forward')

    elif label == 'can':
        step_motor_with_steps(step_pins_open, steps_per_revolution, 0.001, direction='forward')
        GPIO.output(relay_pin, GPIO.HIGH)
        time.sleep(4)
        GPIO.output(relay_pin, GPIO.LOW)
        time.sleep(2)
        step_motor_with_steps(step_pins_open, steps_per_revolution, 0.001, direction='reverse')
        time.sleep(1)
        step_motor_with_steps(step_pins_can, steps_per_revolution1, 0.001, direction='forward')
        time.sleep(1)
        step_motor_with_steps(step_pins_can, steps_per_revolution1, 0.001, direction='forward')

    elif label == 'general':
        step_motor_with_steps(step_pins_open, steps_per_revolution, 0.001, direction='forward')
        GPIO.output(relay_pin, GPIO.HIGH)
        time.sleep(4)
        GPIO.output(relay_pin, GPIO.LOW)
        time.sleep(1)
        step_motor_with_steps(step_pins_open, steps_per_revolution, 0.001, direction='reverse')

# 물체 인식 및 화면에 그리기(수동)
def detect_manual(frame):
    global get_value, should_detect
    results = model(frame)
    # 쓰레기가 인식이 되지 않았을 때(일반쓰레기)
    if not results:
        label = "general"
    else:
        label = None
        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                label = model.names[int(box.cls)]
                cv.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv.putText(frame, label, (x1, y1 - 10), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)

                print(f"label:{label}")
                
                break
                
    if label is None:
        lable = "general"
    
    # 인식된 라벨 전송
    try:
        response = requests.post(label_url, json={"label": label})
        if response.status_code == 200:
            print("Successfully sent:", response.json())
        else:
            print("Failed to send label:", response.status_code)
    except Exception as e:
        print("Error sending label:", e)

    # 물체 인식 후 get_value 초기화 및 should_detect False 설정
    get_value = None
    should_detect = False

    # 라벨에 따른 동작 수행
    perform_action_based_on_label(label)

    return  # 한 번 인식한 후 함수 종료

# 물체 인식 및 화면에 그리기(자동)
def detect_auto(frame):
    global get_value, should_detect
    results = model(frame)
    # 쓰레기가 인식이 되지 않았을 때(일반쓰레기)
    if not results:
        label = "general"
    else:
        label = None
        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                label = model.names[int(box.cls)]
                cv.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv.putText(frame, label, (x1, y1 - 10), cv.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)

                print(f"label:{label}")
                
                break
                
    if label is None:
        lable = "general"
    
    # 인식된 라벨 전송
    try:
        response = requests.post(label_url, json={"label": label})
        if response.status_code == 200:
            print("Successfully sent:", response.json())
        else:
            print("Failed to send label:", response.status_code)
    except Exception as e:
        print("Error sending label:", e)

    # 물체 인식 후 get_value 초기화 및 should_detect False 설정
    get_value = None
    should_detect = False

    # 라벨에 따른 동작 수행
    perform_action_based_on_label(label)

    return  # 한 번 인식한 후 함수 종료
        
# 라즈베리 파이에서 값을 받을 엔드포인트 설정
@app.route('/receive-autovalue', methods=['POST'])
def receive_get_value():
    global should_detect, get_value
    data = request.get_json()
    get_value = data.get('get_value')

    if get_value is not None:
        print(f"Received get_value: {get_value}")

        if get_value == 'automatic' or get_value == 'manual': # 값을 받았을 때
            should_detect = True  # 물체 인식 시작
            return jsonify({'message': "Auto value 'start' received, detection started."}), 200
        else:
            should_detect = False  # 물체 인식 중지
            return jsonify({'message': "Auto value received, detection stopped."}), 200
    else:
        return jsonify({'message': 'Auto value is missing'}), 400

# 객체 인식 스레드
def object_detection():
    cap = cv.VideoCapture(0)
    while True:
        ret, frame = cap.read()
        if ret and should_detect:  # should_detect가 True일 때만 물체 인식
            if get_value == 'automatic': # 자동
                detect_auto(frame)
            elif get_value == 'manual': # 수동
                detect_manual(frame)

        cv.imshow('viewing', frame)
        if cv.waitKey(1) & 0xFF == 27:  # ESC 키
            break

    cap.release()
    cv.destroyAllWindows()

if __name__ == '__main__':
    detection_thread = threading.Thread(target=object_detection)
    detection_thread.start()
    app.run(host='0.0.0.0', port=5000)  # Flask 서버 시작