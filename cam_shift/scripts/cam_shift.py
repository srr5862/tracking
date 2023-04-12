#!/usr/bin/env python3

'''
Camshift node
================
This is a ros node that shows mean-shift based tracking
You select a color objects such as your face and it tracks it.
This subscrib from "/image" topic for reading image,
and publish the information of target to "/TargetPositionSize" 
or "/roi" topic.
The position and size have been normalized in "/TargetPositionSize".
http://www.robinhewitt.com/research/track/camshift.html
Usage:
------
    To initialize tracking, select the object with mouse
Keys:
-----
    ESC/q  - exit
    b           - toggle back-projected probability visualization
    s            - save roi to file
    l             - load roi from file to calculate hist
'''

# Python 2/3 compatibility
from __future__ import print_function
import sys
PY3 = sys.version_info[0] == 3

if PY3:
    xrange = range

import numpy as np
import cv2
import time
import os

# debug with pudb
# import pudb; pu.db

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, RegionOfInterest, CameraInfo
# from cam_shift.msg import  TargetPositionSize

class App:

    def __init__(self):
        self.roi_file = os.path.expanduser("~/roi.jpg")

        cv2.namedWindow('camshift', 1)
        cv2.setMouseCallback('camshift', self.onmouse)

        self.frame = None
        self.vis = None
        self.vis_roi = None
        self.selection = None
        self.drag_start = None
        self.show_backproj = False
        self.track_window = None
        self.track_box = None   #rotated rect
        self.expand_ratio = 0.2
        self.hist = None
        self.last_track = None
        self.fps = 0
        self.fps_values = list()
        self.fps_n_values = 10
        self.time_star = time.time()

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            "rgb/image_raw", Image, self.callback)
        # self.target_pub = rospy.Publisher(
        #     "/TargetPositionSize", TargetPositionSize)
        self.roi_pub = rospy.Publisher("/roi", RegionOfInterest,queue_size=100)

    def onmouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.drag_start = (x, y)
            self.track_window = None
        if event == cv2.EVENT_LBUTTONUP:
            self.drag_start = None
            self.track_window = self.selection
        if self.drag_start:
            xmin = min(x, self.drag_start[0])
            ymin = min(y, self.drag_start[1])
            xmax = max(x, self.drag_start[0])
            ymax = max(y, self.drag_start[1])
            self.selection = (xmin, ymin, xmax - xmin + 1, ymax - ymin + 1)

    def show_hist(self):
        bin_count = self.hist.shape[0]
        bin_w = 24
        img = np.zeros((256, bin_count * bin_w, 3), np.uint8)
        for i in xrange(bin_count):
            h = int(self.hist[i])
            cv2.rectangle(img, (i * bin_w + 2, 255), ((i + 1) * bin_w -
                                                      2, 255 - h), (int(180.0 * i / bin_count), 255, 255), -1)
        img = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)
        cv2.imshow('hist', img)

    def show_hist_new(self):
        bin_count = self.hist.shape[0]
        bin_w = 1
        img = np.zeros((256, bin_count * bin_w, 3), np.uint8)
        for i in xrange(bin_count):
            h = int(self.hist[i])
            cv2.rectangle(img, (i * bin_w, 255), ((i + 1) * bin_w,
                                                  255 - h), (int(180.0 * i / bin_count), 255, 255), -1)
        img = cv2.cvtColor(img, cv2.COLOR_HSV2BGR)
        cv2.imshow('hist', img)

    def expand_window(self, last_track):
        x, y, w, h = last_track
        row, col = self.frame.shape[:2]
        n_x0 = np.maximum(0, x - int(w * self.expand_ratio) - 1)
        n_y0 = np.maximum(0, y - int(h * self.expand_ratio) - 1)
        n_w = np.minimum(col, w + int(w * self.expand_ratio * 2) + 1)
        n_h = np.minimum(row, h + int(h * self.expand_ratio * 2) + 1)
        return (n_x0, n_y0, n_w, n_h)

    def cvBox2D_to_cvRect(self, roi):
        try:
            if len(roi) == 3:
                (center, size, angle) = roi
                pt1 = (
                    int(center[0] - size[0] / 2), int(center[1] - size[1] / 2))
                pt2 = (
                    int(center[0] + size[0] / 2), int(center[1] + size[1] / 2))
                rect = [pt1[0], pt1[1], pt2[0] - pt1[0], pt2[1] - pt1[1]]
            else:
                rect = list(roi)
        except:
            return [0, 0, 0, 0]
        return rect

    # def publish_target(self):
    #     target = TargetPositionSize()
    #     height, width = self.frame.shape[:2]
    #     x, y, w, h = self.track_window
    #     target.center_x = (x + w / 2.0) / width * 2 - 1
    #     target.center_y = 1 - (y + h / 2.0) / height * 2
    #     target.size_x = float(w) / width
    #     target.size_y = float(h) / height
    #     self.target_pub.publish(target)

    def publish_roi(self):
        roi_box = self.track_window
        # roi_box = self.track_box
        try:
            roi_box = self.cvBox2D_to_cvRect(roi_box)
        except:
            return

        # Watch out for negative offsets
        roi_box[0] = max(0, roi_box[0])
        roi_box[1] = max(0, roi_box[1])

        try:
            roi = RegionOfInterest()
            roi.x_offset = int(roi_box[0])
            roi.y_offset = int(roi_box[1])
            roi.width = int(roi_box[2])
            roi.height = int(roi_box[3])
            self.roi_pub.publish(roi)
        except:
            rospy.loginfo("Publishing ROI failed")

    def display_fps(self):
        time_end = time.time()
        img_fps = int(1 / (time_end - self.time_star))
        self.time_star = time_end
        self.fps_values.append(img_fps)
        if len(self.fps_values) > self.fps_n_values:
            self.fps_values.pop(0)
        self.fps = int(sum(self.fps_values) / len(self.fps_values))
        cv2.putText(self.vis, "FPS: " + str(self.fps), (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0))

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        self.frame = np.array(cv_image, dtype=np.uint8)
        self.vis = self.frame.copy()
        hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(
            hsv, np.array((0., 60., 32.)), np.array((180., 255., 255.)))

        if self.selection:
            x0, y0, w, h = self.selection
            hsv_roi = hsv[y0:y0 + h, x0:x0 + w]
            mask_roi = mask[y0:y0 + h, x0:x0 + w]
            self.hist = cv2.calcHist([hsv_roi], [0], mask_roi, [16], [0, 180])
            # self.hist = cv2.calcHist([hsv_roi], [0], mask_roi, [360], [0, 180])
            cv2.normalize(self.hist, self.hist, 0, 255, cv2.NORM_MINMAX)
            self.hist = self.hist.reshape(-1)
            self.show_hist()
            # self.show_self.hist_new(self.hist)

            self.vis_roi = self.vis[y0:y0 + h, x0:x0 + w]
            cv2.bitwise_not(self.vis_roi, self.vis_roi)
            # highlight befitting object when selecting
            # self.vis[mask == 0] = 0

        if self.track_window:
            # lost the target, expand last valid track window
            if self.track_window == (0, 0, 0, 0):
                self.track_window = self.expand_window(self.last_track)
                # print("Re-search at :     ", self.track_window)
            self.last_track = self.track_window
            self.selection = None
            prob = cv2.calcBackProject([hsv], [0], self.hist, [0, 180], 1)
            prob &= mask
            term_crit = (
                cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1)
            self.track_box, self.track_window = cv2.CamShift(
                prob, self.track_window, term_crit)

            # publish position and size of target, has been normalized.
            # self.publish_target()
            self.publish_roi()

            if self.show_backproj:
                self.vis[:] = prob[..., np.newaxis]
            try:
                cv2.ellipse(self.vis, self.track_box, (0, 0, 255), 2)
            except:
                print(self.track_box)

        # Compute the FPS and display in image
        self.display_fps()
        cv2.imshow('camshift', self.vis)
        ch = 0xFF & cv2.waitKey(1)
        if ch == 27 or ch == ord('q'):
            os._exit(0)
        if ch == ord('b'):
            self.show_backproj = not self.show_backproj
        if ch == ord('s'):
            if self.track_window == None:
                print("There has no tracked object!")
                return
            x, y, w, h = self.track_window
            cv2.imwrite(self.roi_file, self.frame[y:y+h, x:x+w])
            print("Saved to ", self.roi_file)
        if ch == ord('l'):
            if not os.path.isfile(self.roi_file):
                print(self.roi_file, " is not exist!")
                return
            roi = cv2.imread(self.roi_file)
            print("Loaded from ", self.roi_file)
            roi_hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            roi_mask = cv2.inRange(
            roi_hsv, np.array((0., 60., 32.)), np.array((180., 255., 255.)))
            self.hist = cv2.calcHist([roi_hsv], [0], roi_mask, [16], [0, 180])
            cv2.normalize(self.hist, self.hist, 0, 255, cv2.NORM_MINMAX)
            self.hist = self.hist.reshape(-1)
            self.show_hist()
            row, col = self.frame.shape[:2]
            self.track_window = (0, 0, col, row)


if __name__ == '__main__':
    rospy.init_node('cam_shift', anonymous=True)
    cs = App()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()