#!/usr/bin/env python

import rospy
import roslaunch
import csv
from tf import TransformListener
import tf2_ros
import tf
import math
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, PointStamped


waypoints_filepath = rospy.get_param('/navigation_light/filepath','/home/xavier/maps/waypoints_record.csv')
#map_filepath = rospy.get_param('/navigation_light/filepath','/home/pluto/maps/autosaved_map')
map_frame_id = rospy.get_param('/navigation_light/map_frame_id',"map")
base_frame_id = rospy.get_param('/navigation_light/base_frame_id',"base_link")
new_waypoint_thr = rospy.get_param('/navigation_light/new_waypoint_thr', 0.3)
waypoints = []
cur_record_i = 0
new_distance = 0.0
new_heading = 0.0
cur_x = 0.0
cur_y = 0.0
ref_x = 0.0
ref_y = 0.0



def updatePosition():

	global map_frame_id, base_frame_id, cur_x, cur_y, tf_listener

	try:
		#tf_listener = tf.TransformListener()
		#now = rospy.Time.now()
		#tf_listener.waitForTransform(map_frame_id, base_frame_id, now, rospy.Duration(.02))
		#trans,rot = tf_listener.lookupTransform(map_frame_id,base_frame_id, now)
		trans = tfBuffer.lookup_transform(map_frame_id, base_frame_id, rospy.Time())

		cur_x = trans.transform.translation.x
		cur_y = trans.transform.translation.y
		return 1
	except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
		#print "error: no " +map_frame_id+" to "+base_frame_id+" tf retrieved"
		return 0


def checkNewWaypoint():
	
	global waypoints, cur_distance, cur_record_i, cur_x, cur_y, ref_x, ref_y, new_waypoint_thr, poseArray_publisher

	delta_x = cur_x-ref_x;
	delta_y = cur_y-ref_y;
	new_distance = math.sqrt(pow(delta_x,2)+pow(delta_y,2)) #distanza dal vecchio waypoint
	#print 'new distance ', new_distance
	if(new_distance>new_waypoint_thr):
		cur_waypoint = PoseWithCovarianceStamped() 
		cur_waypoint.pose.pose.position.x = cur_x
		cur_waypoint.pose.pose.position.y = cur_y
		cur_waypoint.pose.pose.position.z = 0.0
		if(delta_y!=0) or (delta_x!=0):
			quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, math.atan2(delta_y, delta_x))
		else:
			quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)
		cur_waypoint.pose.pose.orientation.x = quaternion[0]
		cur_waypoint.pose.pose.orientation.y = quaternion[1]
		cur_waypoint.pose.pose.orientation.z = quaternion[2]
		cur_waypoint.pose.pose.orientation.w = quaternion[3]
		waypoints.append(cur_waypoint)
		cur_record_i=cur_record_i+1
		print 'new waypoint added: X ',cur_x,', Y ', cur_y  
		ref_x = cur_x
		ref_y = cur_y
		#salva waypoints su file
		saveWaypointsToFile()
		#pubblica waypoint per rviz
		poseArray_publisher.publish(convert_PoseWithCovArray_to_PoseArray(waypoints))

		return 1
	return 0


def saveWaypointsToFile():
	global waypoints, waypoints_filepath

	with open(waypoints_filepath, 'w') as file:
		for current_pose in waypoints:
			file.write(str(current_pose.pose.pose.position.x) + ',' + str(current_pose.pose.pose.position.y) + ',' + str(current_pose.pose.pose.position.z) + ',' + str(current_pose.pose.pose.orientation.x) + ',' + str(current_pose.pose.pose.orientation.y) + ',' + str(current_pose.pose.pose.orientation.z) + ',' + str(current_pose.pose.pose.orientation.w)+ '\n')
			print str(current_pose.pose.pose.position.x) + ',' + str(current_pose.pose.pose.position.y) + ',' + str(current_pose.pose.pose.position.z) + ',' + str(current_pose.pose.pose.orientation.x) + ',' + str(current_pose.pose.pose.orientation.y) + ',' + str(current_pose.pose.pose.orientation.z) + ',' + str(current_pose.pose.pose.orientation.w)+ '\n'
		rospy.loginfo('poses written to '+ waypoints_filepath)
		print "Waypoints saved to file"
		return 1
	return 0

def convert_PoseWithCovArray_to_PoseArray(waypoints):
	global map_frame_id
	"""Used to publish waypoints as pose array so that you can see them in rviz, etc."""
	poses = PoseArray()
	poses.header.frame_id = map_frame_id
	poses.poses = [pose.pose.pose for pose in waypoints]
	return poses




rospy.init_node('record_waypoints')
rate = rospy.Rate(2)
poseArray_publisher = rospy.Publisher('/path_waypoints', PoseArray, queue_size=1)
tfBuffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tfBuffer)

print "node record waypoints started \n"
try:
	while not rospy.is_shutdown():
		updatePosition()
		checkNewWaypoint()
		#print "debug: in main loop"
		rate.sleep()
except:
		print("record_waypoints node: error")

finally:
	print "exiting"
