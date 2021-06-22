import time
import datetime
import threading
import configparser

import cv2

config = configparser.ConfigParser()

class Record:
    def __init__(self, condition, s_img):
        self.condition = condition
        self.s_img = s_img

    def record_video(condition):
        global s_img
        config.read('config.ini')
        record_time = float(config['setting']['record'])
        end_time = datetime.datetime.now() + datetime.timedelta(seconds=record_time * 60)

        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        out = cv2.VideoWriter(f'/home/dev-admin/Desktop/video/{datetime.datetime.now().strftime("%Y-%m-%d-%H:%M:%S")}.avi', fourcc, 20.0, (640, 480))

        while end_time >= datetime.datetime.now():
            with self.condition:
                if self.condition.wait(timeout=120):
                    img = s_img
            out.write(img)
        print('Finish record!')

    def main(self):
        record_job = threading.Thread(target=self.record_video)
        record_job.start()
        print('Finish thread!!!')

if __name__ == '__main__':
    cam = cv2.VideoCapture(0)
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    record = Record(cam)
    record.main()
    cam.release()
    cv2.destroyAllWindows()

