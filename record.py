import time
import datetime
import threading
import configparser

import cv2

config = configparser.ConfigParser()

recordind = False

def counter():
    global recording

    recording = True
    
    config.read('config.ini')
    record_time = float(config['setting']['record'])

    time.sleep(record_time * 60)
    recording = False

def record_video(cam):
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter(f'/home/dev-admin/Desktop/video/{datetime.datetime.now().strftime("%Y-%m-%d-%H:%M:%S")}.avi', fourcc, 20.0, (640, 480))
    #out = cv2.VideoWriter(f'/home/dev-admin/project/visualFench/video/{datetime.datetime.now().strftime("%Y-%m-%d-%H:%M:%S")}.avi', fourcc, 20.0, (640, 480))
    #out = cv2.VideoWriter('/home/dev-admin/project/visualFench/video/test.avi', fourcc, 20.0, (640, 480))

    while cam.isOpened():
        ret, frame = cam.read()
        if ret:
            out.write(frame)
            if not recording:
                print("Time's up")
                break
        else:
            break


def main(cam):
    counter_job = threading.Thread(target=counter)
    counter_job.start()
    record_job = threading.Thread(target=record_video, args=(cam,))
    record_job.start()
    record_job.join()
    counter_job.join()
    print('Finish record!')

if __name__ == '__main__':
    cam = cv2.VideoCapture(0)
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    main(cam)
    cam.release()
    cv2.destroyAllWindows()

