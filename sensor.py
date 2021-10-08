# servomotor control
# reference 'digikey/en/maker/blogs/2021/how-to-control-servo-motors-with-a-raspberry-pi'
# ultra sonic sensor
#ref 'https://tutorials-raspberrypi.com/raspberry-pi-ultrasonic-sensor-hc-sr04/'
# thingsboard ref 'temp_monitor.py'
from gpiozero import Servo
from time import sleep
import numpy as np
import RPi.GPIO as GPIO
import time
import os
import sys
import Adafruit_DHT as dht
import json
import paho.mqtt.client as mqtt

# thingboard
THINGSBOARD_HOST = 'thingsboard.cloud'
ACCESS_TOKEN ='AL96dMXXU2esIKbqeEf2'

client =mqtt.Client()
client.username_pw_set(ACCESS_TOKEN)


#GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BCM)

#set GPIO Pins
GPIO_TRIGGER = 18
GPIO_ECHO = 24

#set GPIO direction (IN / OUT)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)
 
def distance():
    # set Trigger to HIGH
    GPIO.output(GPIO_TRIGGER, True)
 
    # set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)
 
    StartTime = time.time()
    StopTime = time.time()
 
    # save StartTime
    while GPIO.input(GPIO_ECHO) == 0:
        StartTime = time.time()
 
    # save time of arrival
    while GPIO.input(GPIO_ECHO) == 1:
        StopTime = time.time()
 
    # time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = (TimeElapsed * 34300) / 2
 
    return distance

# motor parameter
servo = Servo(25)
val = np.linspace(-1,1,21)

#sensor data
sensor_data ={'distance':0,'angle':0,}
#connect to cloud
client.connect(THINGSBOARD_HOST,1883,60)
client.loop_start()

try:
    while True:
        i=0
        #CW rotation
        while i<len(val):
            servo.value = val[i]
            dist = distance()
            sensor_data['distance'] = dist
            sensor_data['angle'] = val[i]
            client.publish('v1/devices/me/telemetry',json.dumps(sensor_data),1)
            i+=1
            sleep(0.2)
            
        #CCW rotation
        i = len(val)
        while i>0:
            i-=1
            servo.value =val[i]
            dist = distance()
            sensor_data['distance'] = dist
            sensor_data['angle'] = val[i]

            client.publish('v1/devices/me/telemetry',json.dumps(sensor_data),1)
            sleep(0.2)
            
        
        
except KeyboardInterrupt:
    print('stoped')
client.loop_stop()
client.disconnect()