#!/usr/bin/env python


import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

import sys, select, os
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

MAX_AXIS = 1.0
MIN_AXIS = -1.0
STEP_AXIS = 0.1


msg = """

Key to joy by Daniele
---------------------------

        w
   a    s    d
        x

Axis 1 : tasti w, x
Axis 2 : tasti a, d
Axis 3 : tasti u,j
Axis 4 : tasti h,k
Button 0 (A) tasto r
Button 1 (B) tasto f
Button 2 (X) tasto r
Button 3 (Y) tasto f
Button 4 (LB) tasto r
Button 5 (RB) tasto f
Button 7 (start) tasto f
Button 8 (power) tasto f
Button 10 (UP) tasto f
Button 11 (DOWN) tasto f
Button 12 (LEFT) tasto f
Button 13 (RIGHT) tasto f
Button 14 (back) tasto f

Spazio o tasto s: azzera assi
CTRL-C per chiudere
"""

e = """
Communications Failed
"""

def getKey():
    if os.name == 'nt':
      return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def logcurrent():
    print 'axes   :\t ', joy.axes[0],'\t ',joy.axes[1],'\t ',joy.axes[2],'\t ',joy.axes[3]
    print "buttons:\t ", joy.buttons[0],"\t ",joy.buttons[1],"\t ",joy.buttons[2],"\t ",joy.buttons[3]

def axisIncrement(u, step, maxval, minval):
    y = u+step
    if y > maxval:
        y = maxval
    if y < minval:
        y = minval
    return y


if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('key_to_joy')
    pub = rospy.Publisher('joy', Joy, queue_size=10)
    rate = rospy.Rate(20) # ROS Rate at 5Hz

    joy = Joy();
    #for i in range(15):
    #	joy.buttons.push_back(0)
    #for i in range(2):
    #	joy.axes.push_back(0.0)
    joy.header.stamp = rospy.Time.now()
    joy.buttons = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    joy.axes = [0.0, 0.0, 0.0, 0.0]



    try:
		print(msg)
		while not rospy.is_shutdown():
			#reset buttons states
			joy.buttons[0] = 0
			joy.buttons[1] = 0
			joy.buttons[2] = 0
			joy.buttons[3] = 0
			joy.buttons[4] = 0
			joy.buttons[5] = 0
			joy.buttons[6] = 0
			joy.buttons[7] = 0
			    
			key = getKey()
			if key == 'w' :
			    joy.axes[0] = axisIncrement(joy.axes[0],STEP_AXIS, MAX_AXIS, MIN_AXIS)
			elif key == 'x' :
			    joy.axes[0] = axisIncrement(joy.axes[0],-STEP_AXIS, MAX_AXIS, MIN_AXIS)
			elif key == 'a' :
			    joy.axes[1] = axisIncrement(joy.axes[1],STEP_AXIS, MAX_AXIS, MIN_AXIS)
			elif key == 'd' :
			    joy.axes[1] = axisIncrement(joy.axes[1],-STEP_AXIS, MAX_AXIS, MIN_AXIS)
			if key == 'u' :
			    joy.axes[2] = axisIncrement(joy.axes[2],STEP_AXIS, MAX_AXIS, MIN_AXIS)
			elif key == 'j' :
			    joy.axes[2] = axisIncrement(joy.axes[2],-STEP_AXIS, MAX_AXIS, MIN_AXIS)
			elif key == 'h' :
			    joy.axes[3] = axisIncrement(joy.axes[3],STEP_AXIS, MAX_AXIS, MIN_AXIS)
			elif key == 'k' :
			    joy.axes[3] = axisIncrement(joy.axes[3],-STEP_AXIS, MAX_AXIS, MIN_AXIS)
			elif key == '1' :
			    joy.buttons[0] = 1
			elif key == '2' :
			    joy.buttons[1] = 1
			elif key == '3' :
			    joy.buttons[2] = 1
			elif key == '4' :
			    joy.buttons[3] = 1
			elif key == '5' :
			    joy.buttons[4] = 1
			elif key == '6' :
			    joy.buttons[5] = 1
			elif key == '7' :
			    joy.buttons[6] = 1
			elif key == '8' :
			    joy.buttons[7] = 1
			elif key == '9' :
			    joy.buttons[8] = 1
			elif key == 'r' :
			    joy.buttons[9] = 1
			elif key == 'f' :
			    joy.buttons[10] = 1
			elif key == 't' :
			    joy.buttons[11] = 1
			elif key == 'g' :
			    joy.buttons[12] = 1
			elif key == 'y' :
			    joy.buttons[13] = 1                                                
			elif key == ' ' or key == 's' :
			    joy.axes[0] = 0.0
			    joy.axes[1] = 0.0
			    joy.axes[2] = 0.0
			    joy.axes[3] = 0.0			    
			else:
			    if (key == '\x03'):
			        break
			if key != '' :
				logcurrent()

			joy.header.stamp = rospy.Time.now()
			pub.publish(joy) 
			rate.sleep()                

    except:
        print(e)

    finally:
		joy.axes[0] = 0.0
		joy.axes[1] = 0.0
		joy.axes[2] = 0.0
		joy.axes[3] = 0.0		
		joy.buttons[0] = 0.0
		joy.buttons[1] = 0.0
		joy.buttons[2] = 0.0
		joy.buttons[2] = 0.0
		joy.buttons[4] = 0.0
		joy.buttons[5] = 0.0
		joy.buttons[6] = 0.0
		joy.buttons[7] = 0.0
		joy.buttons[8] = 0.0
		joy.buttons[9] = 0.0
		joy.buttons[10] = 0.0
		joy.buttons[11] = 0.0
		joy.buttons[12] = 0.0
		joy.buttons[13] = 0.0
		joy.buttons[14] = 0.0

		pub.publish(joy)

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
