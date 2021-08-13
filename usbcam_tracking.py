import cv2
import numpy as np
import collections
import threading
import pycuda.driver as cuda

from utils.ssd import TrtSSD
from filterpy.kalman import KalmanFilter
from numba import jit
from sklearn.utils.linear_assignment_ import linear_assignment

import warnings
import configparser
import datetime
import time
import math
import glob

device_paths = glob.glob('/dev/video*')

config = configparser.ConfigParser()

warnings.filterwarnings("ignore", category=DeprecationWarning)

s_img, s_boxes = None, None
INPUT_HW = (300, 300)
MAIN_THREAD_TIMEOUT = 120.0  # 20 seconds

check_count = 0


running = True

# SORT Multi object tracking

#iou
@jit
def iou(bb_test, bb_gt):
    xx1 = np.maximum(bb_test[0], bb_gt[0])
    yy1 = np.maximum(bb_test[1], bb_gt[1])
    xx2 = np.minimum(bb_test[2], bb_gt[2])
    yy2 = np.minimum(bb_test[3], bb_gt[3])
    w = np.maximum(0., xx2 - xx1)
    h = np.maximum(0., yy2 - yy1)
    wh = w * h
    o = wh / ((bb_test[2] - bb_test[0]) * (bb_test[3] - bb_test[1])
              + (bb_gt[2] - bb_gt[0]) * (bb_gt[3] - bb_gt[1]) - wh)
    return o

#[x1, y1, x2, y2] -> [u, v, s, r]
def convert_bbox_to_z(bbox):
    """
    Takes a bounding box in the form [x1,y1,x2,y2] and returns z in the form
      [u,v,s,r] where u,v is the centre of the box and s is the scale/area and r is
      the aspect ratio
    """
    w = bbox[2] - bbox[0]
    h = bbox[3] - bbox[1]
    x = bbox[0] + w / 2.
    y = bbox[1] + h / 2.
    s = w * h  # scale is just area
    r = w / float(h)
    return np.array([x, y, s, r]).reshape((4, 1))

#[u, v, s, r] -> [x1, y1, x2, y2]
def convert_x_to_bbox(x, score=None):
    """
    Takes a bounding box in the centre form [x,y,s,r] and returns it in the form
      [x1,y1,x2,y2] where x1,y1 is the top left and x2,y2 is the bottom right
    """
    w = np.sqrt(x[2] * x[3])
    h = x[2] / w
    if score == None:
        return np.array([x[0] - w / 2., x[1] - h / 2., x[0] + w / 2., x[1] + h / 2.]).reshape((1, 4))
    else:
        return np.array([x[0] - w / 2., x[1] - h / 2., x[0] + w / 2., x[1] + h / 2., score]).reshape((1, 5))

#
class KalmanBoxTracker(object):
    """
    This class represents the internel state of individual tracked objects observed as bbox.
    """
    count = 0

    def __init__(self, bbox):
        """
        Initialises a tracker using initial bounding box.
        """
        # define constant velocity model
        self.kf = KalmanFilter(dim_x=7, dim_z=4)
        self.kf.F = np.array(
            [[1, 0, 0, 0, 1, 0, 0], [0, 1, 0, 0, 0, 1, 0], [0, 0, 1, 0, 0, 0, 1], [0, 0, 0, 1, 0, 0, 0],
             [0, 0, 0, 0, 1, 0, 0], [0, 0, 0, 0, 0, 1, 0], [0, 0, 0, 0, 0, 0, 1]])
        self.kf.H = np.array(
            [[1, 0, 0, 0, 0, 0, 0], [0, 1, 0, 0, 0, 0, 0], [0, 0, 1, 0, 0, 0, 0], [0, 0, 0, 1, 0, 0, 0]])

        self.kf.R[2:, 2:] *= 10.
        self.kf.P[4:, 4:] *= 1000.  # give high uncertainty to the unobservable initial velocities
        self.kf.P *= 10.
        self.kf.Q[-1, -1] *= 0.01
        self.kf.Q[4:, 4:] *= 0.01

        self.kf.x[:4] = convert_bbox_to_z(bbox)
        self.time_since_update = 0
        self.id = KalmanBoxTracker.count
        KalmanBoxTracker.count += 1
        self.history = []
        self.hits = 0
        self.hit_streak = 0
        self.age = 0

    def update(self, bbox):
        """
        Updates the state vector with observed bbox.
        """
        self.time_since_update = 0
        self.history = []
        self.hits += 1
        self.hit_streak += 1
        self.kf.update(convert_bbox_to_z(bbox))

    def predict(self):
        """
        Advances the state vector and returns the predicted bounding box estimate.
        """
        if (self.kf.x[6] + self.kf.x[2]) <= 0:
            self.kf.x[6] *= 0.0
        self.kf.predict()
        self.age += 1
        if self.time_since_update > 0:
            self.hit_streak = 0
        self.time_since_update += 1
        self.history.append(convert_x_to_bbox(self.kf.x))
        return self.history[-1]

    def get_state(self):
        """
        Returns the current bounding box estimate.
        """
        return convert_x_to_bbox(self.kf.x)


def associate_detections_to_trackers(detections, trackers, iou_threshold=0.3):
    """
    Assigns detections to tracked object (both represented as bounding boxes)

    Returns 3 lists of matches, unmatched_detections and unmatched_trackers
    """
    if len(trackers) == 0:
        return np.empty((0, 2), dtype=int), np.arange(len(detections)), np.empty((0, 5), dtype=int)
    iou_matrix = np.zeros((len(detections), len(trackers)), dtype=np.float32)

    for d, det in enumerate(detections):
        for t, trk in enumerate(trackers):
            iou_matrix[d, t] = iou(det, trk)

    #Hungarian Algorithm
    matched_indices = linear_assignment(-iou_matrix)

    unmatched_detections = []
    for d, det in enumerate(detections):
        if d not in matched_indices[:, 0]:
            unmatched_detections.append(d)
    unmatched_trackers = []
    for t, trk in enumerate(trackers):
        if t not in matched_indices[:, 1]:
            unmatched_trackers.append(t)

    # filter out matched with low IOU
    matches = []
    for m in matched_indices:
        if iou_matrix[m[0], m[1]] < iou_threshold:
            unmatched_detections.append(m[0])
            unmatched_trackers.append(m[1])
        else:
            matches.append(m.reshape(1, 2))
    if len(matches) == 0:
        matches = np.empty((0, 2), dtype=int)
    else:
        matches = np.concatenate(matches, axis=0)

    return matches, np.array(unmatched_detections), np.array(unmatched_trackers)

#TensorRT Detection
class TrtThread(threading.Thread):
    def __init__(self, condition, cam, model, conf_th):
        """__init__

        # Arguments
            condition: the condition variable used to notify main
                       thread about new frame and detection result
            cam: the camera object for reading input image frames
            model: a string, specifying the TRT SSD model
            conf_th: confidence threshold for detection
        """
        threading.Thread.__init__(self)
        self.condition = condition
        self.cam = cam
        self.model = model
        self.conf_th = conf_th
        self.cuda_ctx = None  # to be created when run
        self.trt_ssd = None   # to be created when run
        self.running = False

    def modify_contrast_brightness(self, img):

        config.read('config.ini')
        
        brightness = float(config['image']['brightness'])
        contrast = float(config['image']['contrast'])

        B = brightness / 255.0
        c = contrast / 255.0 
        k = math.tan((45 + 44 * c) / 180 * math.pi)

        img = (img - 127.5 * (1 - B)) * k + 127.5 * (1 + B)
        img = np.clip(img, 0, 255).astype(np.uint8)

        return img

    def refresh_cam(self): 
        new_paths = glob.glob('/dev/video*')
        # new_paths.remove(device_paths[0])
        if new_paths:
            self.cam = cv2.VidoeCapture(new_paths[0])

    def run(self):
        global s_img, s_boxes, running
        
        print('TrtThread: loading the TRT SSD engine...')
        self.cuda_ctx = cuda.Device(0).make_context()  # GPU 0
        self.trt_ssd = TrtSSD(self.model, INPUT_HW)
        print('TrtThread: start running...')
        time.sleep(5)
        self.running = True
        while self.running:
            ret, img = self.cam.read()
            if img is None:
                # self.refresh_cam()
                continue
            # img = self.modify_contrast_brightness(img)
            boxes, confs, clss = self.trt_ssd.detect(img, self.conf_th)
            with self.condition:
                s_img, s_boxes = img, boxes
                self.condition.notify()
        del self.trt_ssd
        self.cuda_ctx.pop()
        del self.cuda_ctx
        print('TrtThread: stopped...')

    def stop(self):
        self.running = False
        self.join()

def record_video(condition):
    global s_img
    config.read('config.ini')
    record_time = float(config['setting']['record'])
    end_time = datetime.datetime.now() + datetime.timedelta(seconds=record_time * 60)

    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter(f'/home/dev-admin/Desktop/video/{datetime.datetime.now().strftime("%Y-%m-%d-%H:%M:%S")}.avi', fourcc, 20.0, (640, 480))

    while end_time >= datetime.datetime.now():
        with condition:
            if condition.wait(timeout=120):
                img = s_img
        out.write(img)
    print('Finish record!')


def get_frame(condition, io, cam):
    frame = 0
    max_age = 10
    
    trackers = []

    global s_img, s_boxes, check_count, running

    
    io.push_visual_open()
    
    print("frame number ", frame)
    frame += 1
    idstp = collections.defaultdict(list)
    idcnt = []
    incnt, outcnt = 0, 0
    alarm = False

    while running:
        with condition:
            if condition.wait(timeout=MAIN_THREAD_TIMEOUT):
                img, boxes = s_img, s_boxes
            else:
                raise SystemExit('ERROR: timeout waiting for img from child')
        boxes = np.array(boxes)

        H, W = img.shape[:2]
        
        config.read('config.ini')
        sens = 101 - int(config['setting']['sensitivity'])
        scanAx = int(float(config['setting']['scanax']) * 640)
        scanAy = int(float(config['setting']['scanay']) * 480)
        scanBx = int(float(config['setting']['scanbx']) * 640)
        scanBy = int(float(config['setting']['scanby']) * 480)

        trks = np.zeros((len(trackers), 5))
        to_del = []

        for t, trk in enumerate(trks):
            pos = trackers[t].predict()[0]
            trk[:] = [pos[0], pos[1], pos[2], pos[3], 0]
            if np.any(np.isnan(pos)):
                to_del.append(t)
        trks = np.ma.compress_rows(np.ma.masked_invalid(trks))
        for t in reversed(to_del):
            trackers.pop(t)
        
        if len(trackers) > 10:
            trackers.clear()
        matched, unmatched_dets, unmatched_trks = associate_detections_to_trackers(boxes, trks)

        # update matched trackers with assigned detections
        for t, trk in enumerate(trackers):
            if t not in unmatched_trks:
                d = matched[np.where(matched[:, 1] == t)[0], 0]
                trk.update(boxes[d, :][0])
                xmin, ymin, xmax, ymax = boxes[d, :][0]
                cx = int((xmin + xmax) / 2)
                cy = int((ymin + ymax) / 2)
                
                #IN count
                #if scanAx <  cx < scanBx and scanAy < cy < scanBy:
                if (scanAx < xmin + sens < scanBx or scanAx < xmax - sens < scanBx or (scanAx > xmin + sens and scanBx < xmax - sens)) and (scanAy < ymin + sens < scanBy or scanAy < ymax - sens < scanBy or (scanAy > ymin + sens and scanBy < ymax - sens)):
                #if (360 <  xmin + sens < 640 or 360 < xmax - sens < 640) or (160 < ymin + sens < 340 or 160 < ymax - sens < 340):
                    incnt += 1
                    check_count += 1
                    alarm = True
                    if check_count >= sens:
                        running = False
                        io.push_visual_alarm()
                        recording_job = threading.Thread(target=record_video, args=(condition, ))
                        recording_job.start()
                        check_count = 0
                    #print("id: " + str(trk.id) + " - IN ")
                    idcnt.append(trk.id)
                else:
                    alarm = False
                #OUT count
                '''elif idstp[trk.id][0][1] > H // 2 and cy < H // 2 and trk.id not in idcnt:
                    outcnt += 1
                    print("id: " + str(trk.id) + " - OUT ")
                    idcnt.append(trk.id)'''

                cv2.rectangle(img, (xmin, ymin), (xmax, ymax), (0, 0, 255), 2)
                #cv2.putText(img, "id: " + str(trk.id), (int(xmin) - 10, int(ymin) - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        #Total, IN, OUT count & Line
        #cv2.putText(img, "Total: " + str(len(trackers)), (15, 25), cv2.FONT_HERSHEY_DUPLEX, 0.7, (255, 255, 255), 1)
        if alarm:
            cv2.putText(img, "禁制區遭受入侵中！！", (15, 25), cv2.FONT_HERSHEY_DUPLEX, 1.2, (255, 0, 0), 1)

        #cv2.rectangle(img, (360, 160), (640, 340), (255, 0, 0), 3)
        cv2.rectangle(img, (scanAx, scanAy), (scanBx, scanBy), (255, 0, 0), 3)
        #cv2.putText(img, f"IN: {incnt}", (15, 40), cv2.FONT_HERSHEY_DUPLEX, 0.5, (255, 255, 255), 1)
        # cv2.putText(img, f"OUT: {outcnt}", (10, H // 2 + 20), cv2.FONT_HERSHEY_DUPLEX, 0.5, (255, 255, 255), 1)

        # create and initialise new trackers for unmatched detections
        for i in unmatched_dets:
            trk = KalmanBoxTracker(boxes[i, :])
            trackers.append(trk)

            trk.id = len(trackers)

            #new tracker id & u, v
            u, v = trk.kf.x[0], trk.kf.x[1]
            idstp[trk.id].append([u, v])

            if trk.time_since_update > max_age:
                trackers.pop(i)


        #cv2.imshow("dst",img)
        #if cv2.waitKey(1) & 0xFF == ord('q'):
        #   break
        
        yield img
        

class Tracking:
    def __init__(self, io):
        self.io = io

    def reset_count(self):
        global check_count
        check_count = 0

    def start(self):
        global s_img
        self.model = 'ssd_mobilenet_v1_coco'
        self.cam = cv2.VideoCapture(device_paths[0])
        # self.cam = cv2.VideoCapture(0)
        self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        # self.cam = cv2.VideoCapture('video/8.mp4')
        ret, s_img = self.cam.read()
        
        if not self.cam.isOpened():
            raise SystemExit('ERROR: failed to open camera!')
    
        cuda.init()  # init pycuda driver
    
        self.condition = threading.Condition()
        self.trt_thread = TrtThread(self.condition, self.cam, self.model, conf_th=0.5)
        self.trt_thread.start()  # start the child thread
    
    def main(self):
        return get_frame(self.condition, self.io, self.cam)

    def local_main(self):
        get_frame(self.condition, self.io, self.cam)

    def status(self, status=False):
        global running
        running = status

    def get_status(self):
        return running

    def stop(self):
        self.trt_thread.stop()   # stop the child thread
    
        self.cam.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    from jetson_io import IO
    io = IO()
    track = Tracking(io)
    track.start()
    track.local_main()
    track.stop()
