import time
import datetime

import Jetson.GPIO as GPIO

GPIO.setmode(GPIO.BCM)

def write_log(content):
    with open('/home/dev-admin/Desktop/logs.txt', 'a') as log:
        log.write(f'[{datetime.datetime.now().strftime("%Y/%m/%d %H:%M:%S")}] {content}\n')

class IO:
    def __init__(self):
        self.running = True
        self.status = True
        self.manual_open_pin = 20
        self.manual_close_pin = 21
        self.visual_open_pin = 20
        self.visual_close_pin = 21
        self.visual_alarm_pin = 16
        self.auto_open_pin = 8
        self.auto_close_pin = 7
        self.reset_pin = 12

        GPIO.setup(self.manual_open_pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.manual_close_pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.visual_alarm_pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.auto_open_pin, GPIO.IN)
        GPIO.setup(self.auto_close_pin, GPIO.IN)
        GPIO.setup(self.reset_pin, GPIO.IN)

    def get_auto_open(self):
        value = GPIO.input(self.auto_open_pin)
        if value and not self.running:
            self.running = True
            write_log('Auto open!')
            return True
        else:
            return False

    def get_auto_close(self):
        value = GPIO.input(self.auto_close_pin)
        if value and self.running:
            self.running = False
            write_log('Auto close!')
            return True
        else:
            return False

    def get_pin_reset(self):
        value = GPIO.input(self.reset_pin)
        if value and not self.status:
            self.status = True
            return True
        else:
            return False

    def push_manual_open(self):
        self.running = True
        GPIO.output(self.manual_open_pin, GPIO.HIGH)
        GPIO.output(self.manual_close_pin, GPIO.LOW)
        GPIO.output(self.visual_alarm_pin, GPIO.LOW)
        write_log('Manual open!')

    def push_manual_close(self):
        self.running = False
        GPIO.output(self.manual_close_pin, GPIO.HIGH)
        GPIO.output(self.manual_open_pin, GPIO.LOW)
        GPIO.output(self.visual_alarm_pin, GPIO.LOW)
        write_log('Manual close!')
    
    def push_visual_open(self):
        self.running = True
        GPIO.output(self.visual_open_pin, GPIO.HIGH)
        GPIO.output(self.visual_close_pin, GPIO.LOW)
        GPIO.output(self.visual_alarm_pin, GPIO.LOW)
        write_log('Visual open!')

    def push_visual_close(self):
        self.running = False
        GPIO.output(self.visual_close_pin, GPIO.HIGH)
        GPIO.output(self.visual_open_pin, GPIO.LOW)
        GPIO.output(self.visual_alarm_pin, GPIO.LOW)
        write_log('Visual close!')

    def push_visual_alarm(self):
        self.running = False
        self.status = False
        GPIO.output(self.visual_alarm_pin, GPIO.HIGH)
        GPIO.output(self.visual_open_pin, GPIO.LOW)
        write_log('Visual alarm!')
