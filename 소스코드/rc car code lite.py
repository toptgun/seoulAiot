import threading
import serial
import time
import RPi.GPIO as GPIO

bleSerial = serial.Serial("/dev/ttyS0", baudrate=9600, timeout=1.0)
gData = ""

SWs = [5, 6, 13, 19]
LEDs = [26, 16, 20, 21]
MotorPins = [(18, 22, 27), (23, 25, 24)]
BUZZER = 12

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

for pin in SWs:
    GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

for pin in LEDs:
    GPIO.setup(pin, GPIO.OUT)

for pwm, in1, in2 in MotorPins:
    GPIO.setup(pwm, GPIO.OUT)
    GPIO.setup(in1, GPIO.OUT)
    GPIO.setup(in2, GPIO.OUT)

L_Motor = GPIO.PWM(MotorPins[0][0], 500)
L_Motor.start(0)
R_Motor = GPIO.PWM(MotorPins[1][0], 500)
R_Motor.start(0)

GPIO.setup(BUZZER, GPIO.OUT)
p = GPIO.PWM(BUZZER, 391)
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
    while True:
        data = bleSerial.readline().decode()
        gData = data.strip()

def main():
    global gData
    task1 = threading.Thread(target=serial_thread)
    task1.start()

    try:
        while True:
            if "go" in gData:
                print("ok go")
                motor_control(0, 1, 0, 1, 50)
                set_leds(GPIO.HIGH, GPIO.HIGH, GPIO.LOW, GPIO.LOW)
                gData = ""
            elif "back" in gData:
                print("ok back")
                motor_control(1, 0, 1, 0, 50)
                set_leds(GPIO.LOW, GPIO.LOW, GPIO.HIGH, GPIO.HIGH)
                gData = ""
            elif "left" in gData:
                print("ok left")
                motor_control(1, 0, 0, 1, 50)
                set_leds(GPIO.HIGH, GPIO.LOW, GPIO.HIGH, GPIO.LOW)
                gData = ""
            elif "right" in gData:
                print("ok right")
                motor_control(0, 1, 1, 0, 50)
                set_leds(GPIO.LOW, GPIO.HIGH, GPIO.LOW, GPIO.HIGH)
                gData = ""
            elif "stop" in gData:
                print("ok stop")
                motor_control(0, 1, 0, 1, 0)
                set_leds(GPIO.LOW, GPIO.LOW, GPIO.LOW, GPIO.LOW)
                gData = ""
            elif "bz_on" in gData:
                print("ok buzzer on")
                p.start(50)
                p.ChangeFrequency(391)
                gData = ""
            elif "bz_off" in gData:
                print("ok buzzer off")
                p.stop()
                gData = ""

    except KeyboardInterrupt:
        pass

    bleSerial.close()
    GPIO.cleanup()

if __name__ == '__main__':
    main()
