from __future__ import division
import RPi.GPIO as GPIO
import Adafruit_PCA9685
import threading
import random
import time
from multiprocessing import Queue

pwm = Adafruit_PCA9685.PCA9685()
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
high_speed = 4000  # Max pulse length out of 4096
mid_speed = 2500  # Middle pulse length out of 4096
low_speed = 2000  # low pulse length out of 4096
short_delay=0.2
long_delay=0.25
extra_long_delay=0.6

# Define GPIO to use on Pi
GPIO_TRIGGER = 20
GPIO_ECHO    = 21
#define L298N(Model-Pi motor drive board) GPIO pins
IN1 = 23  #left motor direction pin
IN2 = 24  #left motor direction pin
IN3 = 27  #right motor direction pin
IN4 = 22  #right motor direction pin
ENA = 0  #left motor speed PCA9685 port 0
ENB = 1  #right motor speed PCA9685 port 1

servo_lft = 420 #ultrasonic sensor facing right
servo_ctr = 300 #ultrasonic sensor facing front
servo_rgt = 180 #ultrasonic sensor facing left
# Set pins as output and input
GPIO.setup(GPIO_TRIGGER,GPIO.OUT)  # Trigger
GPIO.setup(GPIO_ECHO,GPIO.IN)      # Echo
GPIO.setup(IN1, GPIO.OUT)   
GPIO.setup(IN2, GPIO.OUT) 
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)

queue = Queue()

pwm.set_pwm_freq(60)
print('Moving servo on channel 0, press Ctrl-C to quit...')
sts1=0
sts2=0
sts3=0
ob_range=30
pwm.set_pwm(15, 0, servo_lft)
time.sleep(1)
pwm.set_pwm(15, 0, servo_rgt)
time.sleep(1)
pwm.set_pwm(15, 0, servo_ctr)
time.sleep(5)

def changespeed(speed_left,speed_right):
	pwm.set_pwm(ENA, 0, speed_left)
	pwm.set_pwm(ENB, 0, speed_right)

def stopcar():
	GPIO.output(IN1, GPIO.LOW)
	GPIO.output(IN2, GPIO.LOW)
	GPIO.output(IN3, GPIO.LOW)
	GPIO.output(IN4, GPIO.LOW)
	changespeed(0,0)
stopcar()

def backward(speed_left,speed_right):
	GPIO.output(IN1, GPIO.HIGH)
	GPIO.output(IN2, GPIO.LOW)
	GPIO.output(IN3, GPIO.HIGH)
	GPIO.output(IN4, GPIO.LOW)
	changespeed(speed_left,speed_right)
 
	#following two lines can be removed if you want car make continuous movement without pause
	#time.sleep(short_delay) 
	#stopcar()
	
def forward(speed_left,speed_right):
	GPIO.output(IN2, GPIO.HIGH)
	GPIO.output(IN1, GPIO.LOW)
	GPIO.output(IN4, GPIO.HIGH)
	GPIO.output(IN3, GPIO.LOW)
	changespeed(speed_left,speed_right)
	#following two lines can be removed if you want car make continuous movement without pause
	#time.sleep(short_delay) 
	#stopcar()
	
def turnRight(speed_left,speed_right):
	GPIO.output(IN1, GPIO.LOW)
	GPIO.output(IN2, GPIO.HIGH)
	GPIO.output(IN3, GPIO.HIGH)
	GPIO.output(IN4, GPIO.LOW)
	changespeed(speed_left,speed_right)
	#following two lines can be removed if you want car make continuous movement without pause
	#time.sleep(short_delay) 
	#stopcar()
	
def turnLeft(speed_left,speed_right):
	GPIO.output(IN1, GPIO.HIGH)
	GPIO.output(IN2, GPIO.LOW)
	GPIO.output(IN3, GPIO.LOW)
	GPIO.output(IN4, GPIO.HIGH)
	changespeed(speed_left,speed_right)	
	#following two lines can be removed if you want car make continuous movement without pause
	#time.sleep(short_delay) 
	#stopcar()

def threadone():
    for i in range(10):
        x = [servo_lft, servo_rgt, servo_ctr]
        rand = random.randint(0, 2)
        pwm.set_pwm(15, 0, x[rand])
        time.sleep(1)

def measure():
    # This function measures a distance
    GPIO.output(GPIO_TRIGGER, True)
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)
    start = time.time()
    while GPIO.input(GPIO_ECHO)==0:
        start = time.time()
    while GPIO.input(GPIO_ECHO)==1:
        stop = time.time()
    elapsed = stop-start
    distance = (elapsed * 34300)/2
    return distance

def funct1():
    while True:
        pwm.set_pwm(15, 0, servo_lft)
        time.sleep(0.2)
        distance = measure()
        sts1 =  0 if distance>ob_range else 1

        pwm.set_pwm(15, 0, servo_ctr)
        time.sleep(0.2)
        distance = measure()
        sts2 =  0 if distance>ob_range else 1

        pwm.set_pwm(15, 0, servo_rgt)
        time.sleep(0.2)
        distance = measure()
        sts3 =  0 if distance>ob_range else 1
        sensorval = ''.join([str(sts1), str(sts2), str(sts3)])

        queue.put(sensorval)

def funct2():
    while True:
        if queue.empty():
            data = queue.get()

            if  data=="100":
                print("100 slight right")
                forward(1000,0) #slight right turn
                #time.sleep(long_delay)  
                #stopcar()
                time.sleep(short_delay)	
                
            if  data=="001":
                print("001 slight left")
                forward(0,1000) #slight left turn
                #time.sleep(long_delay)  
                #stopcar()
                time.sleep(short_delay)	

            if  data=="110":
                print("110 sharp right")
                turnRight(1500,500) #shart right turn
                #time.sleep(long_delay)  
                #stopcar()
                time.sleep(short_delay) 
                
            if  data=="011" or data=="010":	
                print(data+" sharp left")
                turnLeft(500,1500)   #sharp left turn
                #time.sleep(long_delay)  
                #stopcar()
                time.sleep(short_delay) 
                
            if  data=="111" or data=="101":	
                print(data+" back to left")
                turnRight(-1500,-1500) #back to left side 
                #time.sleep(extra_long_delay)  
                #stopcar()
                time.sleep(short_delay) 
                
            if  data=="000":
                print(data+" forward")
                forward(1000,1000) #go forward
                #time.sleep(long_delay)  
                #stopcar()
                time.sleep(short_delay)	


t1 = threading.Thread(target=funct1, args=())
t2 = threading.Thread(target=funct2, args=())

t1.start()

t2.start()

t1.join()

t2.join()

print("DONE")