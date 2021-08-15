from configparser import ConfigParser
import time

from kivy.app import App
from kivy.config import Config
from kivy.uix.floatlayout import FloatLayout
from kivy.uix.image import Image
from kivy.uix.label import Label
from kivy.uix.button import Button
from kivy.uix.textinput import TextInput
from kivy.uix.slider import Slider
from kivy.uix.screenmanager import ScreenManager, Screen, NoTransition
from kivy.uix.widget import Widget
from kivy.lang import Builder
from kivy.clock import Clock
from kivy.graphics import Line
from kivy.graphics import Color
from kivy.graphics.texture import Texture
from kivy.core.window import Window

import cv2

from usbcam_tracking import Tracking
from jetson_io import ArduinoIO

Window.size = (800, 600)

io = ArduinoIO()

track = Tracking(io)
track.start()
global_frame = track.main()

running = True

track.status(running)
sm = ScreenManager(transition=NoTransition())

config = ConfigParser()
config.read('config.ini')
settings = config['setting']

PASSWORD = settings['password']
START_PASSWORD = settings['start_password']
STOP_PASSWORD = settings['stop_password']

def write_config():

    with open('config.ini', 'w') as configfile:
        config.write(configfile)

Builder.load_string("""
<SettingWidget>:
    FloatLayout

<RootWidget>:
    FloatLayout

<PassWidget>:
    FloatLayout

<StartPassWidget>:
    FloatLayout

<StopPassWidget>:
    FloatLayout

<RangeSettingWidget>:
    FloatLayout

<ChangePassWidget>:
    FloatLayout

<ChangeSettingPass>:
    FloatLayout

<ChangeStartPass>:
    FloatLayout

<ChangeStopPass>:
    FloatLayout
""")

class PassWidget(Screen):
    def __init__(self, **kwargs):
        super(PassWidget, self).__init__(**kwargs) 
        self.checkBtn = Button(
                text='CHECK',
                font_size=24,
                pos_hint={'x': 0.6, 'y': 0.1}, 
                size_hint=(.1, .1),
                background_color=(0,1,0,1)
                )
        self.checkBtn.bind(on_press=self.check)
        self.add_widget(self.checkBtn)

        self.backBtn = Button(
                text='BACK',
                font_size=24,
                pos_hint={'x': 0.3, 'y': 0.1}, 
                size_hint=(.1, .1),
                background_color=(0,1,0,1)
                )
        self.backBtn.bind(on_press=self.back)
        self.add_widget(self.backBtn)
        
        self.label = Label(
                text='Password:',
                font_size=24,
                pos_hint={'x': -0.2, 'y': 0.05},
                )
        self.add_widget(self.label)

        self.passInput = TextInput(
                multiline=False,
                font_size=24,
                pos_hint={'x': 0.4, 'y': 0.525},
                size_hint=(.4, .05),
                password=True
                )
        self.passInput.bind(text=self.on_text)
        self.passInput.bind(on_text_validate=self.check)
        self.add_widget(self.passInput)
    
    def reset_pwInput(self):
        self.passInput.text = ''

    def on_text(self, instance, value):
        try:
            self.pw = value
        except IndexError:
            pass

    def check(self, instance):
        if self.passInput.text == PASSWORD:
            self.reset_pwInput()
            sm.current = 'setting'
        else:
            self.reset_pwInput()

    def back(self, instance):
        self.reset_pwInput()
        sm.current = 'root'

class StartPassWidget(Screen):
    def __init__(self, **kwargs):
        super(StartPassWidget, self).__init__(**kwargs) 
        self.checkBtn = Button(
                text='CHECK',
                font_size=24,
                pos_hint={'x': 0.6, 'y': 0.1}, 
                size_hint=(.1, .1),
                background_color=(0,1,0,1)
                )
        self.checkBtn.bind(on_press=self.check)
        self.add_widget(self.checkBtn)

        self.backBtn = Button(
                text='BACK',
                font_size=24,
                pos_hint={'x': 0.3, 'y': 0.1}, 
                size_hint=(.1, .1),
                background_color=(0,1,0,1)
                )
        self.backBtn.bind(on_press=self.back)
        self.add_widget(self.backBtn)
        
        self.label = Label(
                text='Password:',
                font_size=24,
                pos_hint={'x': -0.2, 'y': 0.05},
                )
        self.add_widget(self.label)

        self.passInput = TextInput(
                multiline=False,
                font_size=24,
                pos_hint={'x': 0.4, 'y': 0.525},
                size_hint=(.4, .05),
                password=True
                )
        self.passInput.bind(on_text_validate=self.check)
        self.add_widget(self.passInput)
    
    def reset_pwInput(self):
        self.passInput.text = ''

    def check(self, instance):
        global running
        if self.passInput.text == START_PASSWORD:
            running = True
            track.status(running)
            io.push_manual_open()
            self.reset_pwInput()
            sm.current = 'root'
        else:
            self.reset_pwInput()

    def back(self, instance):
        self.reset_pwInput()
        sm.current = 'root'

class StopPassWidget(Screen):
    def __init__(self, **kwargs):
        super(StopPassWidget, self).__init__(**kwargs)
        self.checkBtn = Button(
                text='CHECK',
                font_size=24,
                pos_hint={'x': 0.6, 'y': 0.1}, 
                size_hint=(.1, .1),
                background_color=(0,1,0,1)
                )
        self.checkBtn.bind(on_press=self.check)
        self.add_widget(self.checkBtn)

        self.backBtn = Button(
                text='BACK',
                font_size=24,
                pos_hint={'x': 0.3, 'y': 0.1}, 
                size_hint=(.1, .1),
                background_color=(0,1,0,1)
                )
        self.backBtn.bind(on_press=self.back)
        self.add_widget(self.backBtn)
        
        self.label = Label(
                text='Password:',
                font_size=24,
                pos_hint={'x': -0.2, 'y': 0.05},
                )
        self.add_widget(self.label)

        self.passInput = TextInput(
                multiline=False,
                font_size=24,
                pos_hint={'x': 0.4, 'y': 0.525},
                size_hint=(.4, .05),
                password=True
                )
        self.passInput.bind(on_text_validate=self.check)
        self.add_widget(self.passInput)
    
    def reset_pwInput(self):
        self.passInput.text = ''

    def check(self, instance):
        global running
        if self.passInput.text == STOP_PASSWORD:
            running = False
            track.status(running)
            io.push_manual_close()
            self.reset_pwInput()
            sm.current = 'root'
        else:
            self.reset_pwInput()

    def back(self, instance):
        self.reset_pwInput()
        sm.current = 'root'


class ChangeSettingPass(Screen):
    def __init__(self, **kwargs):
        super(ChangeSettingPass, self).__init__(**kwargs)
        self.checkBtn = Button(
                text='CHECK',
                font_size=24,
                pos_hint={'x': 0.6, 'y': 0.1}, 
                size_hint=(.1, .1),
                background_color=(0,1,0,1)
                )
        self.checkBtn.bind(on_press=self.check)
        self.add_widget(self.checkBtn)

        self.backBtn = Button(
                text='BACK',
                font_size=24,
                pos_hint={'x': 0.3, 'y': 0.1}, 
                size_hint=(.1, .1),
                background_color=(0,1,0,1)
                )
        self.backBtn.bind(on_press=self.back)
        self.add_widget(self.backBtn)
        
        self.label0 = Label(
                text='Current Password:',
                font_size=24,
                pos_hint={'x': -0.2, 'y': 0.2},
                )
        self.add_widget(self.label0)

        self.cuPassInput = TextInput(
                multiline=False,
                font_size=24,
                pos_hint={'x': 0.45, 'y': 0.675},
                size_hint=(.4, .05),
                password=True
                )
        self.add_widget(self.cuPassInput)
        
        self.label1 = Label(
                text='New Password:',
                font_size=24,
                pos_hint={'x': -0.2, 'y': 0.05},
                )
        self.add_widget(self.label1)

        self.nePassInput = TextInput(
                multiline=False,
                font_size=24,
                pos_hint={'x': 0.45, 'y': 0.525},
                size_hint=(.4, .05),
                password=True
                )
        self.add_widget(self.nePassInput)
        
        self.label2 = Label(
                text='Confirm Password:',
                font_size=24,
                pos_hint={'x': -0.2, 'y': -0.1},
                )
        self.add_widget(self.label2)

        self.coPassInput = TextInput(
                multiline=False,
                font_size=24,
                pos_hint={'x': 0.45, 'y': 0.375},
                size_hint=(.4, .05),
                password=True
                )
        self.add_widget(self.coPassInput)
    
    def reset_pwInput(self):
        self.cuPassInput.text = ''
        self.nePassInput.text = ''
        self.coPassInput.text = ''

    def check(self, instance):
        global PASSWORD
        if self.cuPassInput.text == PASSWORD and self.nePassInput.text == self.coPassInput.text:
            settings['password'] = self.nePassInput.text
            PASSWORD = self.nePassInput.text
            write_config()
            self.reset_pwInput()
            sm.current = 'setting'
        else:
            self.reset_pwInput()

    def back(self, instance):
        self.reset_pwInput()
        sm.current = 'setting'

class ChangeStartPass(Screen):
    def __init__(self, **kwargs):
        super(ChangeStartPass, self).__init__(**kwargs)
        self.checkBtn = Button(
                text='CHECK',
                font_size=24,
                pos_hint={'x': 0.6, 'y': 0.1}, 
                size_hint=(.1, .1),
                background_color=(0,1,0,1)
                )
        self.checkBtn.bind(on_press=self.check)
        self.add_widget(self.checkBtn)

        self.backBtn = Button(
                text='BACK',
                font_size=24,
                pos_hint={'x': 0.3, 'y': 0.1}, 
                size_hint=(.1, .1),
                background_color=(0,1,0,1)
                )
        self.backBtn.bind(on_press=self.back)
        self.add_widget(self.backBtn)
        
        self.label0 = Label(
                text='Current Password:',
                font_size=24,
                pos_hint={'x': -0.2, 'y': 0.2},
                )
        self.add_widget(self.label0)

        self.cuPassInput = TextInput(
                multiline=False,
                font_size=24,
                pos_hint={'x': 0.45, 'y': 0.675},
                size_hint=(.4, .05),
                password=True
                )
        self.add_widget(self.cuPassInput)
        
        self.label1 = Label(
                text='New Password:',
                font_size=24,
                pos_hint={'x': -0.2, 'y': 0.05},
                )
        self.add_widget(self.label1)

        self.nePassInput = TextInput(
                multiline=False,
                font_size=24,
                pos_hint={'x': 0.45, 'y': 0.525},
                size_hint=(.4, .05),
                password=True
                )
        self.add_widget(self.nePassInput)
        
        self.label2 = Label(
                text='Confirm Password:',
                font_size=24,
                pos_hint={'x': -0.2, 'y': -0.1},
                )
        self.add_widget(self.label2)

        self.coPassInput = TextInput(
                multiline=False,
                font_size=24,
                pos_hint={'x': 0.45, 'y': 0.375},
                size_hint=(.4, .05),
                password=True
                )
        self.add_widget(self.coPassInput)
    
    def reset_pwInput(self):
        self.cuPassInput.text = ''
        self.nePassInput.text = ''
        self.coPassInput.text = ''

    def check(self, instance):
        global START_PASSWORD
        if self.cuPassInput.text == START_PASSWORD and self.nePassInput.text == self.coPassInput.text:
            settings['start_password'] = self.nePassInput.text
            START_PASSWORD = self.nePassInput.text
            write_config()
            self.reset_pwInput()
            sm.current = 'setting'
        else:
            self.reset_pwInput()

    def back(self, instance):
        self.reset_pwInput()
        sm.current = 'setting'

class ChangeStopPass(Screen):
    def __init__(self, **kwargs):
        super(ChangeStopPass, self).__init__(**kwargs)
        self.checkBtn = Button(
                text='CHECK',
                font_size=24,
                pos_hint={'x': 0.6, 'y': 0.1}, 
                size_hint=(.1, .1),
                background_color=(0,1,0,1)
                )
        self.checkBtn.bind(on_press=self.check)
        self.add_widget(self.checkBtn)

        self.backBtn = Button(
                text='BACK',
                font_size=24,
                pos_hint={'x': 0.3, 'y': 0.1}, 
                size_hint=(.1, .1),
                background_color=(0,1,0,1)
                )
        self.backBtn.bind(on_press=self.back)
        self.add_widget(self.backBtn)
        
        self.label0 = Label(
                text='Current Password:',
                font_size=24,
                pos_hint={'x': -0.2, 'y': 0.2},
                )
        self.add_widget(self.label0)

        self.cuPassInput = TextInput(
                multiline=False,
                font_size=24,
                pos_hint={'x': 0.45, 'y': 0.675},
                size_hint=(.4, .05),
                password=True
                )
        self.add_widget(self.cuPassInput)
        
        self.label1 = Label(
                text='New Password:',
                font_size=24,
                pos_hint={'x': -0.2, 'y': 0.05},
                )
        self.add_widget(self.label1)

        self.nePassInput = TextInput(
                multiline=False,
                font_size=24,
                pos_hint={'x': 0.45, 'y': 0.525},
                size_hint=(.4, .05),
                password=True
                )
        self.add_widget(self.nePassInput)
        
        self.label2 = Label(
                text='Confirm Password:',
                font_size=24,
                pos_hint={'x': -0.2, 'y': -0.1},
                )
        self.add_widget(self.label2)

        self.coPassInput = TextInput(
                multiline=False,
                font_size=24,
                pos_hint={'x': 0.45, 'y': 0.375},
                size_hint=(.4, .05),
                password=True
                )
        self.add_widget(self.coPassInput)
    
    def reset_pwInput(self):
        self.cuPassInput.text = ''
        self.nePassInput.text = ''
        self.coPassInput.text = ''

    def check(self, instance):
        global STOP_PASSWORD
        if self.cuPassInput.text == STOP_PASSWORD and self.nePassInput.text == self.coPassInput.text:
            settings['stop_password'] = self.nePassInput.text
            STOP_PASSWORD = self.nePassInput.text
            write_config()
            self.reset_pwInput()
            sm.current = 'setting'
        else:
            self.reset_pwInput()

    def back(self, instance):
        self.reset_pwInput()
        sm.current = 'setting'

class ChangePassWidget(Screen):
    def __init__(self, **kwargs):
        super(ChangePassWidget, self).__init__(**kwargs)
        
        self.changePassBtn = Button(
                text='Change Setting Password',
                font_size=24,
                pos_hint={'x': 0.3, 'y': 0.6}, 
                size_hint=(.4, .1),
                background_color=(0,1,0,1)
                )
        self.changePassBtn.bind(on_press=self.changePass)
        self.add_widget(self.changePassBtn)
        
        self.changeStartPassBtn = Button(
                text='Change Start Password',
                font_size=24,
                pos_hint={'x': 0.3, 'y': 0.45}, 
                size_hint=(.4, .1),
                background_color=(0,1,0,1)
                )
        self.changeStartPassBtn.bind(on_press=self.changeStartPass)
        self.add_widget(self.changeStartPassBtn)
        
        self.changeStopPassBtn = Button(
                text='Change Stop Password',
                font_size=24,
                pos_hint={'x': 0.3, 'y': 0.3}, 
                size_hint=(.4, .1),
                background_color=(0,1,0,1)
                )
        self.changeStopPassBtn.bind(on_press=self.changeStopPass)
        self.add_widget(self.changeStopPassBtn)
        
        self.backBtn = Button(
                text='BACK',
                font_size=24,
                pos_hint={'x': 0.3, 'y': 0.1}, 
                size_hint=(.1, .1),
                background_color=(0,1,0,1)
                )
        self.backBtn.bind(on_press=self.back)
        self.add_widget(self.backBtn)
    
    def changePass(self, instance):
        sm.current = 'chps'

    def changeStartPass(self, instance):
        sm.current = 'chstartps'

    def changeStopPass(self, instance):
        sm.current = 'chstopps'

    def back(self, instance):
        sm.current = 'setting'

class RangeSettingWidget(Screen):
    def __init__(self, **kwargs):
        super(RangeSettingWidget, self).__init__(**kwargs)
        self.scanAx = float(settings['scanax']) * Window.size[0]
        self.scanAy = (1- float(settings['scanay'])) * Window.size[1]
        self.scanBx = float(settings['scanbx']) * Window.size[0]
        self.scanBy = (1- float(settings['scanby'])) * Window.size[1]
        self.new_scanAx = float(settings['scanAx'])
        self.new_scanAy = 1 - float(settings['scanAy'])
        self.new_scanBx = float(settings['scanBx'])
        self.new_scanBy = 1 - float(settings['scanBy'])

        self.img = Image(size=Window.size, allow_stretch=True)
        Clock.schedule_interval(self.update, 1.0/30.0)
        
        self.add_widget(self.img)
        
        self._keyboard = Window.request_keyboard(self._keyboard_closed, self)
        self._keyboard.bind(on_key_down=self._on_keyboard_down)

        with self.canvas:
            self.rect = Line(points=[self.scanAx, self.scanAy, self.scanAx, self.scanBy, self.scanBx, self.scanBy, self.scanBx, self.scanAy, self.scanAx, self.scanAy])

    def update(self, dt):
        global global_frame, running
        if track.get_status():
            running = True
        else:
            running = False
        
        if running and io.status:
            frame = next(global_frame)
            buf1 = cv2.flip(frame, 0)
            buf = buf1.tostring()
            texture1 = Texture.create(size=(frame.shape[1], frame.shape[0]), colorfmt='bgr')
            texture1.blit_buffer(buf, colorfmt='bgr', bufferfmt='ubyte')
            self.img.texture = texture1

    def on_touch_down(self, touch):
        self.scanAx = touch.pos[0]
        self.scanAy = touch.pos[1]
        self.new_scanAx = touch.spos[0]
        self.new_scanAy = 1- touch.spos[1]

    def on_touch_move(self, touch):
        self.scanBx = touch.pos[0]
        self.scanBy = touch.pos[1]
        self.rect.points=[self.scanAx, self.scanAy, self.scanAx, self.scanBy, self.scanBx, self.scanBy, self.scanBx, self.scanAy, self.scanAx, self.scanAy]

    def on_touch_up(self, touch):
        self.new_scanBx = touch.spos[0]
        self.new_scanBy = 1 - touch.spos[1]
        

    def _keyboard_closed(self):
        pass
        #self._keyboard.unbind(on_key_down=self._on_keyboard_down)
        #self._keyboard = None

    def _on_keyboard_down(self, keyboard, keycode, text, modifiers):
        if keycode[1] == 'enter' and sm.current == 'range':
            settings['scanax'] = f'{self.new_scanAx}'
            settings['scanay'] = f'{self.new_scanAy}'
            settings['scanbx'] = f'{self.new_scanBx}'
            settings['scanby'] = f'{self.new_scanBy}'
            write_config()
            sm.current = 'root'

class SettingWidget(Screen):
    def __init__(self, **kwargs):
        super(SettingWidget, self).__init__(**kwargs)
        
        self.sens_title = Label(
                text='Sensitivity:',
                font_size=24,
                pos_hint={'x':0, 'y':0.4}
                )
        
        self.sens1 = Label(
                text='1',
                font_size=24,
                pos_hint={'x':-0.1, 'y':0.3}
                )

        self.sens100 = Label(
                text='100',
                font_size=24,
                pos_hint={'x':0.1, 'y':0.3}
                )

        self.sens = Slider(
                min=1,
                max=100,
                value=int(settings['sensitivity']),
                step=1,
                pos_hint={'x': 0.35 , 'y': 0.8},
                size_hint=(.3, .1)
                )

        self.record_title = Label(
                text='Record Range:',
                font_size=24,
                pos_hint={'x': -0.06, 'y': 0.2}
                )

        self.record_time = TextInput(
                text=settings['record'],
                font_size=24,
                pos_hint={'x': 0.55, 'y': 0.67},
                size_hint=(.13, .06)
                )
        
        self.rangeBtn = Button(
                text='SCAN RANGE',
                font_size=24,
                pos_hint={'x': 0.4, 'y': 0.5}, 
                size_hint=(.25, .1),
                background_color=(0,1,0,1)
                )
        self.rangeBtn.bind(on_press=self.range_setting)
        
        self.changePassBtn = Button(
                text='Change Passowrd',
                font_size=24,
                pos_hint={'x': 0.4, 'y': 0.3}, 
                size_hint=(.25, .1),
                background_color=(0,1,0,1)
                )
        self.changePassBtn.bind(on_press=self.change_password)
        
        self.checkBtn = Button(
                text='CHECK',
                font_size=24,
                pos_hint={'x': 0.6, 'y': 0.1}, 
                size_hint=(.1, .1),
                background_color=(0,1,0,1)
                )
        self.checkBtn.bind(on_press=self.check)
        
        self.backBtn = Button(
                text='BACK',
                font_size=24,
                pos_hint={'x': 0.3, 'y': 0.1}, 
                size_hint=(.1, .1),
                background_color=(0,1,0,1)
                )
        self.backBtn.bind(on_press=self.back)
        
        self.add_widget(self.sens_title)
        self.add_widget(self.sens1)
        self.add_widget(self.sens100)
        self.add_widget(self.sens)
        self.add_widget(self.record_title)
        self.add_widget(self.record_time)
        self.add_widget(self.rangeBtn)
        self.add_widget(self.changePassBtn)
        self.add_widget(self.checkBtn)
        self.add_widget(self.backBtn)

    def range_setting(self, instance):
        sm.current = 'range'

    def change_password(self, instance):
        sm.current = 'change'
    
    def check(self, instance):
        global settings
        settings['sensitivity'] = str(self.sens.value)
        settings['record'] = self.record_time.text
        write_config()
        sm.current = 'root'

    def back(self, instance):
        sm.current = 'root'


class RootWidget(Screen):
    def __init__(self, **kwargs):
        super(RootWidget, self).__init__(**kwargs)
        self.img = Image(
                pos_hint={'x': 0.2, 'y': 0},
                size_hint=(.8, .8)
                )
        self.add_widget(self.img)
        self.frame = track.main()
        Clock.schedule_interval(self.update, 1.0/30.0)
        
        self.running = True 
        
        self.startBtn = Button(
                text='START',
                font_size=24,
                pos_hint={'x': 0.35, 'y': 0.83}, 
                size_hint=(.15, .15),
                background_color=(0,1,0,1)
                )
        self.startBtn.bind(on_press=self.start)
        self.stopBtn = Button(
                text='STOP',
                font_size=24,
                pos_hint={'x': 0.75, 'y': 0.83},
                size_hint=(.15, .15),
                background_color=(1,1,1,1)
                )
        self.stopBtn.bind(on_press=self.stop)

        self.settingBtn = Button(
                text='Admin',
                font_size=24,
                pos_hint={'x': 0.05, 'y': 0.1},
                size_hint=(.1, .1),
                background_color=(1,1,1,1)
                )
        self.settingBtn.bind(on_press=self.setting)
        self.logBtn = Button(
                text='Log',
                font_size=24,
                pos_hint={'x': 0.05, 'y': 0.3},
                size_hint=(.1, .1),
                background_color=(1,1,1,1)
                )
        self.logBtn.bind(on_press=self.logs)
        
        self.add_widget(self.startBtn)
        self.add_widget(self.stopBtn)
        self.add_widget(self.settingBtn)
        #self.add_widget(self.logBtn)


    def update(self, dt):
        global running, global_frame

        io.get_pin_reset()
        
        if track.get_status():
            running = True
        else:
            running = False

        if io.get_auto_open():
            running = True
            track.status(running)

        elif io.get_auto_close():
            running = False
            track.status(running)
        
        if io.status:
            running = True
            frame = next(global_frame)
            self.startBtn.background_color=(0,1,0,1)
            self.stopBtn.background_color=(1,1,1,1)
            buf1 = cv2.flip(frame, 0)
            buf = buf1.tostring()
            texture1 = Texture.create(size=(frame.shape[1], frame.shape[0]), colorfmt='bgr')
            texture1.blit_buffer(buf, colorfmt='bgr', bufferfmt='ubyte')
            self.img.texture = texture1
        else:
            running = False
            self.startBtn.background_color=(1,1,1,1)
            self.stopBtn.background_color=(1,0,0,1)

    
    def start(self, instance):
        sm.current = 'startpass'

    def stop(self, instance):
        sm.current = 'stoppass'
    
    def setting(self, instance):
        sm.current = 'pass'
        
    def logs(self, instance):
        sm.current = 'pass'


class CVCamApp(App):
    def build(self):
        sm.add_widget(RootWidget(name='root'))
        sm.add_widget(SettingWidget(name='setting'))
        sm.add_widget(ChangePassWidget(name='change'))
        sm.add_widget(ChangeSettingPass(name='chps'))
        sm.add_widget(ChangeStartPass(name='chstartps'))
        sm.add_widget(ChangeStopPass(name='chstopps'))
        sm.add_widget(PassWidget(name='pass'))
        sm.add_widget(StartPassWidget(name='startpass'))
        sm.add_widget(StopPassWidget(name='stoppass'))
        sm.add_widget(RangeSettingWidget(name='range'))
        sm.current = 'root'
        return sm

   
def main():   
    Config.set('graphics', 'fullscreen', False)
    Config.set('graphics', 'resizeable', False)
    Config.set('graphics', 'window_state', 'maximized')
    Config.set('graphics', 'width', '800')
    Config.set('graphics', 'height', '600')
    Config.write()
    CVCamApp().run()
    track.stop()
    io.clean()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
