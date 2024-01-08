import cv2
import socket
import threading
import serial
import RPi.GPIO as GPIO
import time

# 소켓을 생성하고 영상 전송을 시작
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET,socket.SO_SNDBUF,10000000)
server_ip = "192.168.45.204"
server_port = 6666
cap = cv2.VideoCapture(-1)
cap.set(3, 640)
cap.set(4, 480)

# 블루투스와 GPIO 초기화
bleSerial = serial.Serial("/dev/ttyS0", baudrate=9600, timeout=1.0)
gData = ""
data_lock = threading.Lock()

SWs = [5, 6, 13, 19]  # 스위치핀
LEDs = [26, 16, 20, 21]  # LED 핀
MotorPins = [(18, 22, 27), (23, 25, 24)]  # 모터핀 (PWM, IN1, IN2)
BUZZER = 12  # 부저핀

# GPIO 초기화
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)  # Use BCM pin numbering

# 스위치핀 입력으로 설정
for pin in SWs:
    GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# LED 핀을 출력으로 설정
for pin in LEDs:
    GPIO.setup(pin, GPIO.OUT)

# 모터핀 출력으로 설정
for pwm, in1, in2 in MotorPins:
    GPIO.setup(pwm, GPIO.OUT)
    GPIO.setup(in1, GPIO.OUT)
    GPIO.setup(in2, GPIO.OUT)

# 모터 돌리기 위한 PWM 초기화
L_Motor = GPIO.PWM(MotorPins[0][0], 500)  # PWM 초기화 (500Hz)
L_Motor.start(0)  # 0% duty cycle로 시작
R_Motor = GPIO.PWM(MotorPins[1][0], 500)  # PWM 초기화 (500Hz)
R_Motor.start(0)  # 0% duty cycle로 시작

# 부저 초기화
GPIO.setup(BUZZER, GPIO.OUT)  # 부저핀 추력으로 설정
p = GPIO.PWM(BUZZER, 391)  # pwm 391HZ으로 초기화
p.stop()  # s

##부저 울리기 위한 udp 소켓 thread
def pi_server_thread():
    pi_server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    pi_server_socket.bind(('0.0.0.0', 6667))
    pi_server_socket.listen(1)
    conn, addr = pi_server_socket.accept()
    
    while True:
        data = conn.recv(1024)
        if data == b'armed_person_detected':
            # 부저를 작동시킵니다.
            p.start(50)
            p.ChangeFrequency(391)
            time.sleep(1)
            p.stop()
            
def set_leds(*states):
    for pin, state in zip(LEDs, states):
        GPIO.output(pin, state)


def motor_control(L_A1, L_A2, R_A1, R_A2, speed):
    GPIO.output(MotorPins[0][1], L_A1)
    GPIO.output(MotorPins[0][2], L_A2)
    L_Motor.ChangeDutyCycle(speed)
    GPIO.output(MotorPins[1][1], R_A1)
    GPIO.output(MotorPins[1][2], R_A2)
    R_Motor.ChangeDutyCycle(speed)

def serial_thread():
    global gData
    global ArdData
    while True:
        try: 
            ## 시리얼 통신으로 들어온 바이트를 문자열로 디코딩함
            data = bleSerial.readline().decode()
            ## 문자열 중 공백을 제거함
            with data_lock:
                gData = data.strip()
                print("data")
        except Exception as e:
            print(f"Error in serial_thread: {e}")

def video_thread():
    try:
        while cap.isOpened():
            ret, img = cap.read()
            img = cv2.flip(img, -1)
            img = cv2.resize(img, (640, 480))
            ret, buffer = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 50])
            data = buffer.tobytes()
            
            if data:
                s.sendto(data, (server_ip, server_port))
            
            if cv2.waitKey(1) & 0xFF == 27:
                break
    except Exception as e:
        print(f"Video error: {e}")
    finally:
        cv2.destroyAllWindows()
        cap.release()


def main():
    global gData
    global ArdData

    task1 = threading.Thread(target=video_thread)
    task2 = threading.Thread(target=serial_thread)
    task3 = threading.Thread(target=pi_server_thread)
    
    task1.start()
    task2.start()
    task3.start()

    try:
        while True:
            with data_lock:
                local_data = gData
                
            if "go" in local_data:
                print("ok go")
                motor_control(0, 1, 0, 1, 60)
                set_leds(GPIO.HIGH, GPIO.HIGH, GPIO.LOW, GPIO.LOW)
                with data_lock:
                    gData = ""
            elif "back" in local_data:
                print("ok back")
                motor_control(1, 0, 1, 0, 60)
                set_leds(GPIO.LOW, GPIO.LOW, GPIO.HIGH, GPIO.HIGH)
                with data_lock:
                    gData = ""
            elif "left" in local_data:
                print("ok left")
                motor_control(1, 0, 0, 1, 60)
                set_leds(GPIO.HIGH, GPIO.LOW, GPIO.HIGH, GPIO.LOW)
                with data_lock:
                    gData = ""
            elif "right" in local_data:
                print("ok right")
                motor_control(0, 1, 1, 0, 60)
                set_leds(GPIO.LOW, GPIO.HIGH, GPIO.LOW, GPIO.HIGH)
                with data_lock:
                    gData = ""
            elif "stop" in local_data:
                print("ok stop")
                motor_control(0, 1, 0, 1, 0)
                set_leds(GPIO.LOW, GPIO.LOW, GPIO.LOW, GPIO.LOW)
                with data_lock:
                    gData = ""
            elif "bz_on" in local_data:
                print("ok buzzer on")
                p.start(50)
                p.ChangeFrequency(391)
                with data_lock:
                    gData = ""
            elif "bz_off" in local_data:
                print("ok buzzer off")
                p.stop()
                with data_lock:
                    gData = ""
            elif "route1" in local_data:
                print("ok, route1")
                
                # 출발지에서 도착지까지 이동하는 코드
                motor_control(0, 1, 0, 1, 60)  # 전진
                set_leds(GPIO.HIGH, GPIO.HIGH, GPIO.LOW, GPIO.LOW)
                time.sleep(10)  # 10초간 전진
                
                motor_control(1, 0, 0, 1, 60)  # 왼쪽으로 회전
                set_leds(GPIO.HIGH, GPIO.LOW, GPIO.HIGH, GPIO.LOW)
                time.sleep(3)  # 3초간 회전
                
                motor_control(0, 1, 0, 1, 60)  # 전진
                set_leds(GPIO.HIGH, GPIO.HIGH, GPIO.LOW, GPIO.LOW)
                time.sleep(10)  # 10초간 전진
                
                motor_control(0, 1, 1, 0, 60)  # 오른쪽으로 회전
                set_leds(GPIO.LOW, GPIO.HIGH, GPIO.LOW, GPIO.HIGH)
                time.sleep(3)  # 3초간 회전
                
                motor_control(0, 1, 0, 1, 60)  # 전진
                set_leds(GPIO.HIGH, GPIO.HIGH, GPIO.LOW, GPIO.LOW)
                time.sleep(10)  # 10초간 전진

                # 도착지에서 출발지로 되돌아오는 코드
                motor_control(0, 1, 1, 0, 60)  # 오른쪽으로 회전
                set_leds(GPIO.LOW, GPIO.HIGH, GPIO.LOW, GPIO.HIGH)
                time.sleep(3)  # 3초간 회전
                
                motor_control(0, 1, 0, 1, 60)  # 전진
                set_leds(GPIO.HIGH, GPIO.HIGH, GPIO.LOW, GPIO.LOW)
                time.sleep(10)  # 10초간 전진
                
                motor_control(1, 0, 0, 1, 60)  # 왼쪽으로 회전
                set_leds(GPIO.HIGH, GPIO.LOW, GPIO.HIGH, GPIO.LOW)
                time.sleep(3)  # 3초간 회전
                
                motor_control(0, 1, 0, 1, 60)  # 전진
                set_leds(GPIO.HIGH, GPIO.HIGH, GPIO.LOW, GPIO.LOW)
                time.sleep(10)  # 10초간 전진
                
                motor_control(0, 1, 0, 1, 0)  # 정지
                set_leds(GPIO.LOW, GPIO.LOW, GPIO.LOW, GPIO.LOW)
                
                with data_lock:
                    gData = ""
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        bleSerial.close()
        p.stop()
        GPIO.cleanup()
    
if __name__ == '__main__':
    main()
