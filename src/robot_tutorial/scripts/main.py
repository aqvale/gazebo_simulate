#!/usr/bin/env python2.7

import rospy
from rospy.timer import time

from robot import Robot
from actionlib_msgs.msg import (
  GoalStatusArray
) 
from std_msgs.msg import Int32

class StateMachine:
  state = 0
  move_base_info = None
  aruco_id = None
  case = []
  
  def __init__(self):
    rospy.init_node("machine", anonymous=True)
    rospy.Subscriber("/camera/aruco", Int32, self.callback_aruco)
    self.robot = Robot('odom')
    self.case.append({'x': 6.6, 'y': 0.9, 'yaw': 0})
    self.case.append({'x': 3.4, 'y': 6.6, 'yaw': 1.0})
    self.case.append({'x': -6.8, 'y': 6.8, 'yaw': -3.14})
    self.case.append({'x': -6.5, 'y': -6.8, 'yaw': -1.57})
    self.case.append({'x': 6.4, 'y': -7.8, 'yaw': 0})
    self.case.append({'x': 0, 'y': 0, 'yaw': 0})
    rospy.timer.sleep(1)

  def callback_main(self, data):
    input_ = data.status_list[0].status if data.status_list else 2
    self.FSM(input_)

  def callback_aruco(self, data):
    self.aruco_id = data.data

  def ED0(self, input_):
    case = self.case[0]
    self.robot.move_to_goal(x=case['x'], y=case['y'])
    self.state = 1
    self.time_old = time.time()

  def ED1(self, input_):
    if (time.time() - self.time_old) > 10 and input_ == 3:
      self.state = 2 

  def ED2(self, input_):
    if input_ == 3 and self.aruco_id != 0:
      self.robot.stop_move()
      self.robot.move_to_goal(x=self.case[self.aruco_id]['x'], y=self.case[self.aruco_id]['y'], yaw=self.case[self.aruco_id]['yaw'])
      self.state = 1
      self.time_old = time.time()
    
    if self.aruco_id == 5:
      self.state = 3
    
  def ED3(self, input_):
    if input_ != 1:
      rospy.loginfo("Finish my jorney!")

  def FSM(self, input_):
    switch = {
      0 : self.ED0,
      1 : self.ED1,
      2 : self.ED2,
      3 : self.ED3,
    }
    func = switch.get(self.state, lambda: None)
    func(input_)

  def run(self):
    rospy.Subscriber("/move_base/status", GoalStatusArray, self.callback_main)

if __name__ == "__main__":
  rospy.loginfo("Start Machine")
  state_machine = StateMachine()
  state_machine.run()
  while not rospy.is_shutdown():
    rospy.spin() 