#!/usr/bin/env python

import rospy
import csv
from tf import TransformListener
import tf2_ros
import tf
import math
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, PointStamped
from std_msgs.msg import Float32

waypoint_filepath = rospy.get_param('/navigation_light/waypoint_filepath',"/home/xavier/maps/waypoints_record.csv")
map_frame_id = rospy.get_param('/navigation_light/map_frame_id',"map")
base_frame_id = rospy.get_param('/navigation_light/base_frame_id',"base_link")
waypoint_reached_thr = rospy.get_param('/navigation_light/waypoint_reached_thr', 0.1)
waypoints = []
goal = PoseStamped()
cur_record_i = 0
n_waypoints = 0
new_distance = 0.0
new_heading = 0.0
cur_x = 0.0
cur_y = 0.0
ref_x = 0.0
ref_y = 0.0




def updatePosition():

	global map_frame_id, base_frame_id, cur_x, cur_y
	try:
		#tf_listener = tf.TransformListener()
		#now = rospy.Time.now()
		#tf_listener.waitForTransform(map_frame_id, base_frame_id, now, rospy.Duration(2.0))
		#trans,rot = tf_listener.lookupTransform(map_frame_id,base_frame_id, now)
		trans = tfBuffer.lookup_transform(map_frame_id, base_frame_id, rospy.Time())

		cur_x = trans.transform.translation.x
		cur_y = trans.transform.translation.y
		return 1
	except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
		#print "error: no " +map_frame_id+" to "+base_frame_id+" tf retrieved"
		return 0


def checkNewWaypoint():
	
	global cur_distance, cur_x, cur_y, ref_x, ref_y, waypoint_reached_thr, cur_record_i

	delta_x = cur_x-ref_x;
	delta_y = cur_y-ref_y;
	new_distance = math.sqrt(pow(delta_x,2)+pow(delta_y,2)) #distanza dal waypoint
	#print "current distance: ", new_distance
	if(new_distance<waypoint_reached_thr):
		if(cur_record_i<(n_waypoints-1)):
			cur_record_i = cur_record_i+1
			setCurrentWaypoint()
			goal_publisher.publish(goal)
	return 0


def setCurrentWaypoint():
	global waypoints, cur_record_i, ref_x, ref_y, goal
	ref_x = waypoints[cur_record_i].pose.pose.position.x
	ref_y = waypoints[cur_record_i].pose.pose.position.y
	goal = PoseStamped() #PoseStamped()
	goal.header.frame_id = map_frame_id
	goal.header.stamp = rospy.Time.now()
	goal.pose.position.x = ref_x
	goal.pose.position.y = ref_y
	quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)
	goal.pose.orientation.x = quaternion[0]
	goal.pose.orientation.y = quaternion[1]
	goal.pose.orientation.z = quaternion[2]
	goal.pose.orientation.w = quaternion[3]



	#print 'Target waypoint: X ',ref_x,', Y ', ref_y, ' i ', cur_record_i
	return 0

def loadWaypointsFromFile():
	global waypoints, waypoint_filepath, cur_record_i,ref_x, ref_y, n_waypoints, poseArray_publisher

	print "loading waypoint file: ", waypoint_filepath
	with open(waypoint_filepath, 'r') as file:
		reader = csv.reader(file, delimiter = ',')
		cur_record_i = 0
		waypoints = []	 
		for row in reader:
			print row
			current_pose = PoseWithCovarianceStamped()
			current_pose.pose.pose.position.x    = float(row[0])
			current_pose.pose.pose.position.y    = float(row[1])
			current_pose.pose.pose.position.z    = float(row[2])
			current_pose.pose.pose.orientation.x = float(row[3])
			current_pose.pose.pose.orientation.y = float(row[4])
			current_pose.pose.pose.orientation.z = float(row[5])
			current_pose.pose.pose.orientation.w = float(row[6])
			waypoints.append(current_pose)
			cur_record_i = cur_record_i+1
			#poseArray_publisher.publish(convert_PoseWithCovArray_to_PoseArray(waypoints))

		n_waypoints = cur_record_i
		print n_waypoints, "waypoints read"
		cur_record_i = 0
		setCurrentWaypoint()
		return 1
	return 0


def convert_PoseWithCovArray_to_PoseArray(waypoints):
	global map_frame_id
	"""Used to publish waypoints as pose array so that you can see them in rviz, etc."""
	poses = PoseArray()
	poses.header.frame_id = map_frame_id
	poses.poses = [pose.pose.pose for pose in waypoints]
	return poses




rospy.init_node('replay_waypoints')
rate = rospy.Rate(10)
poseArray_publisher = rospy.Publisher('/path_waypoints', PoseArray, queue_size=1)
goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
#test_publisher = rospy.Publisher('/test_number', Float32, queue_size=1)
tfBuffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tfBuffer)
cnt = 0
print "node replay waypoints started \n"
try:
	if(loadWaypointsFromFile()):
		while not rospy.is_shutdown():
			updatePosition()
			checkNewWaypoint()

			cnt = cnt +1
			if(cnt>10):
				#test_publisher.publish(a) #debug
				cnt = 0
				goal_publisher.publish(goal)
			#poseArray_publisher.publish(convert_PoseWithCovArray_to_PoseArray(waypoints))
			rate.sleep()
except:
		print("replay_waypoints node: error")
