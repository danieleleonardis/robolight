#!/usr/bin/env python
# coding: utf-8
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from math import sqrt
import numpy as np

rate_hz = 20 #control rate
scale_lin_vel = rospy.get_param('/joy_to_twist/scale_lin_vel', 0.25)
scale_ang_vel = rospy.get_param('/joy_to_twist/scale_ang_vel', np.pi/5.0)
local_joy= rospy.get_param('/joy_to_twist/local_joy',False)
use_keyboard = rospy.get_param('/joy_to_twist/use_keyboard',False)

######### WATCHDOG ON /joy #########
# watchdog max value (cycles)
watchdog_max = 50;
# watchdog counter init
watchdog_cnt = 0;

def callback(data):
    global scale_lin_vel, scale_ang_vel, watchdog_cnt, watchdog_max
    if use_keyboard:
        x = scale_lin_vel * data.axes[0]
        theta = scale_ang_vel * data.axes[1]
    elif local_joy:
        x = scale_lin_vel * data.axes[1]
        theta = scale_ang_vel * data.axes[2]
    else:
        x = scale_lin_vel * data.axes[1]
        theta = scale_ang_vel * data.axes[3]

    twist = Twist()
    twist.linear.x = x
    twist.angular.z = theta
    watchdog_cnt = watchdog_max
    pub.publish(twist)

rospy.init_node('joy_node', anonymous=True)
rospy.Subscriber("joy", Joy, callback)
pub = rospy.Publisher('cmd_vel_joy', Twist, queue_size=10)

r = rospy.Rate(rate_hz)
while not rospy.is_shutdown():
    if watchdog_cnt>0:
        watchdog_cnt-=1
    else:
        pub.publish(Twist())
    r.sleep()
