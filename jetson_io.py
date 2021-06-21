import time
import datetime

import Jetson.GPIO as GPIO

GPIO.setmode(GPIO.BCM)

running = True

class IO:
    def __init__(self):
        self.manual_open_pin = 20
        self.manual_close_pin = 21
        self.visual_open_pin = 20
        self.visual_close_pin = 21
        self.auto_open_pin = 23
        self.auto_close_pin = 24

        GPIO.setup(self.manual_open_pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.manual_close_pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.auto_open_pin, GPIO.IN)
        GPIO.setup(self.auto_close_pin, GPIO.IN)
    
    def write_log(self, content):
        with open('/home/dev-admin/Desktop/logs.txt', 'a') as log:
            log.write(f'[{datetime.datetime.now().strftime("%Y/%m/%d %H:%M:%S")}] {content}\n')


    def get_auto_open(self):
        global running
        value = GPIO.input(self.auto_open_pin)
        if value and not running:
            running = True
            self.write_log('Auto open!')
            return True
        else:
            return False

    def get_auto_close(self):
        global running
        value = GPIO.input(self.auto_close_pin)
        if value and running:
            running = False
            self.write_log('Auto close!')
            return True
        else:
            return False

    def push_manual_open(self):
        global running
        running = True
        GPIO.output(self.manual_open_pin, GPIO.HIGH)
        time.sleep(0.5)
        GPIO.output(self.manual_open_pin, GPIO.LOW)
        self.write_log('Manual open!')

    def push_manual_close(self):
        global running
        running = False
        GPIO.output(self.manual_close_pin, GPIO.HIGH)
        time.sleep(0.5)
        GPIO.output(self.manual_close_pin, GPIO.LOW)
        self.write_log('Manual close!')
    
    def push_visual_open(self):
        global running
        running = True
        GPIO.output(self.visual_open_pin, GPIO.HIGH)
        time.sleep(0.5)
        GPIO.output(self.visual_open_pin, GPIO.LOW)
        self.write_log('Visual open!')

    def push_visual_close(self):
        global running
        running = False
        GPIO.output(self.visual_close_pin, GPIO.HIGH)
        time.sleep(0.5)
        GPIO.output(self.visual_close_pin, GPIO.LOW)
        self.write_log('Visual close!')
