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
        self.manual = False
        self.auto = True
        self.manual_open_pin = 20
        self.manual_close_pin = 21
        self.visual_open_pin = 20
        self.visual_close_pin = 21
        self.visual_alarm_pin = 19
        self.auto_open_pin = 8
        self.auto_close_pin = 7
        self.reset_pin = 17

        GPIO.setup(self.manual_open_pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.manual_close_pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.visual_alarm_pin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.auto_open_pin, GPIO.IN)
        GPIO.setup(self.auto_close_pin, GPIO.IN)
        GPIO.setup(self.reset_pin, GPIO.IN)

    def change_manual_auto(self):
        self.manual = False
        self.auto = True

    def get_auto_open(self):
        value = GPIO.input(self.auto_open_pin)
        if value:
            self.change_manual_auto()

        if value and not self.running and self.status and not self.manual and self.auto:
            self.running = True
            write_log('Auto open!')
            GPIO.output(self.manual_open_pin, GPIO.HIGH)
            GPIO.output(self.manual_close_pin, GPIO.LOW)
            # GPIO.output(self.visual_alarm_pin, GPIO.LOW)
            return True
        else:
            return False

    def get_auto_close(self):
        value = GPIO.input(self.auto_open_pin)
        if not value and self.running and not self.manual and self.auto:
            self.running = False
            write_log('Auto close!')
            GPIO.output(self.manual_close_pin, GPIO.HIGH)
            GPIO.output(self.manual_open_pin, GPIO.LOW)
            # GPIO.output(self.visual_alarm_pin, GPIO.LOW)
            return True
        else:
            return False

    def get_pin_reset(self):
        value = GPIO.input(self.reset_pin)
        if value and not self.status:
            self.manual = False
            GPIO.output(self.manual_close_pin, GPIO.HIGH)
            GPIO.output(self.manual_open_pin, GPIO.LOW)
            GPIO.output(self.visual_alarm_pin, GPIO.LOW)
            self.status = True
            write_log('Alarm reset!')
            return True
        else:
            return False

    def push_manual_open(self):
        if self.status:
            self.running = True
            self.manual = True
            self.auto = False
            GPIO.output(self.manual_open_pin, GPIO.HIGH)
            GPIO.output(self.manual_close_pin, GPIO.LOW)
            GPIO.output(self.visual_alarm_pin, GPIO.LOW)
            write_log('Manual open!')

    def push_manual_close(self):
        self.running = False
        self.manual = True
        self.auto = False
        GPIO.output(self.manual_close_pin, GPIO.HIGH)
        GPIO.output(self.manual_open_pin, GPIO.LOW)
        GPIO.output(self.visual_alarm_pin, GPIO.LOW)
        write_log('Manual close!')
    
    def push_visual_open(self):
        if self.status:
            self.running = True
            self.manual = True
            self.auto = False
            GPIO.output(self.manual_open_pin, GPIO.HIGH)
            GPIO.output(self.manual_close_pin, GPIO.LOW)
            GPIO.output(self.visual_alarm_pin, GPIO.LOW)
            write_log('Manual open!')

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
        GPIO.output(self.visual_close_pin, GPIO.LOW)
        GPIO.output(self.visual_open_pin, GPIO.LOW)
        write_log('Visual alarm!')

    def clean(self):
        GPIO.cleanup()


if __name__ == '__main__':
    io = IO()
    
    while True:
        act = input("Please enter 'r' for read, 'out1', 'out2', 'out3' for output or 'q' for exit: ")

        if act == 'out1':
            io.push_manual_open()

        if act == 'out2':
            io.push_manual_close()

        if act == 'out3':
            io.push_visual_alarm()

        if act == 'r':
            if io.get_auto_open():
                print('auto_open')

            if io.get_auto_close():
                print('auto_close')

            if io.get_pin_reset():
                print('reset')
        
        if act == 'q':
            break

