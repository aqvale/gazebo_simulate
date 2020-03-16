#!/usr/bin/env python2.7

import numpy as np
import cv2, PIL

import rospy
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class ArucoProcess:
  def __init__(self):
    rospy.loginfo("Init aruco")
    rospy.init_node("robot_aruco", anonymous=True)
    self.bridge = CvBridge()
    self.pub_image = rospy.Publisher('/camera/aruco', Int32, queue_size=1)

  def aruco_detection(self, frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
    parameters =  cv2.aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    # frame_markers = cv2.aruco.drawDetectedMarkers(frame.copy(), corners, ids)
    return ids

  def image_callback(self, data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "passthrough")
      msg_int = Int32()
      aux = self.aruco_detection(cv_image)
      try:
        msg_int = aux[0][0]
      except Exception as e:
        msg_int.data = 0
      self.pub_image.publish(msg_int)
    except CvBridgeError, e:
      rospy.logerr("CvBridge Error: {0}".format(e))


  def run(self):
    self.sub_image = rospy.Subscriber("/mybot/camera/image_raw", Image, self.image_callback)

if __name__ == "__main__":
  rospy.loginfo("Init Aruco")
  aruco = ArucoProcess()
  aruco.run()
  while not rospy.is_shutdown():
    rospy.spin()  