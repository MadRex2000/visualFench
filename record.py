import time
import datetime
import threading
import configparser

import cv2

config = configparser.ConfigParser()

#recording = False

class Record:
    def __init__(self, cap):
        self.cap = cap
        self.recording = False

    def counter(self):
        self.recording = True
    
        config.read('config.ini')
        record_time = float(config['setting']['record'])

        time.sleep(record_time * 60)
        #time.sleep(10)
        self.recording = False

    def record_video(self):
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        out = cv2.VideoWriter(f'/home/dev-admin/Desktop/video/{datetime.datetime.now().strftime("%Y-%m-%d-%H:%M:%S")}.avi', fourcc, 20.0, (640, 480))
        #out = cv2.VideoWriter(f'/home/dev-admin/project/visualFench/video/{datetime.datetime.now().strftime("%Y-%m-%d-%H:%M:%S")}.avi', fourcc, 20.0, (640, 480))
        #out = cv2.VideoWriter('/home/dev-admin/project/visualFench/video/test.avi', fourcc, 20.0, (640, 480))

        while self.recording:
            ret, frame = self.cap.read()
            out.write(frame)
        out.release()
        print('Finish record!')

    def main(self):
        counter_job = threading.Thread(target=self.counter)
        counter_job.start()
        record_job = threading.Thread(target=self.record_video)
        record_job.start()
        #record_job.join()
        #counter_job.join()

if __name__ == '__main__':
    cam = cv2.VideoCapture(0)
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    record = Record(cam)
    record.main()
    cam.release()
    cv2.destroyAllWindows()

