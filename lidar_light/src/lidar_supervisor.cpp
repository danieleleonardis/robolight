#include <iostream>
#define _USE_MATH_DEFINES
#include <math.h>

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/LaserScan.h>
//#include <tf/transform_listener.h>
//#include <move_base_msgs/MoveBaseAction.h>

#define PI 3.1415926535

//#define DEBUG_LIDAR
//#define DEBUG_LOOP
#define MIN_INTENSITY 10.0 //in lumen
#define MIN_RANGE 0.05 //in metri 
//#define TF_WATCHDOG_COUNT 20 				// [counts], cycles of tf aging
#define MAX_LIDAR_POINTS 2000 		// turtlebot lidar:360 points, YDLIDAR G4: 1524 points
//#define CLUTTER_ANGLE 100 					// [degs], points filtered
//#define CLUTTER_RANGE 5 					// [m], points filtered

//std::string s
//n.param<std::string>("my_param", s, "default_value")
float frontal_dist_thr = 0.3;
float frontal_dist_thr_2 = 0.6;
float frontal_width = 0.3;
float frontal_width_2 = 0.4;



ros::Subscriber lidar_subscriber;
ros::Publisher lidar_publisher;
ros::Publisher coeff_publisher;

sensor_msgs::LaserScan scan_msg;
std_msgs::Float32 dist_coeff;

float lidar_dist_x[MAX_LIDAR_POINTS];
float lidar_dist_y[MAX_LIDAR_POINTS];
float lidar_angle[MAX_LIDAR_POINTS];
float lidar_angle_min;
float lidar_angle_inc;
float lidar_angle_rad;
float min_frontal_dist = 1.0;
float lidar_angle_offset = 0.0;

//float scaling_const = frontal_dist_thr_2/5000;
int mode = 0;


void lidar_callback(const sensor_msgs::LaserScan::ConstPtr& msg) {
	
	scan_msg = *msg;
	
	int i=0;
	int n_points = (scan_msg.angle_max-scan_msg.angle_min)/scan_msg.angle_increment;
	int point_mid = n_points/2;
	float half_frontal_width = frontal_width*0.5;
	float half_frontal_width_2 = frontal_width_2*0.5;

	#ifdef DEBUG_LIDAR
		ROS_INFO("Received: \n");
		ROS_INFO("angle min %f", scan_msg.angle_min);
		ROS_INFO("angle max %f", scan_msg.angle_max);
		ROS_INFO("angle increment %f", scan_msg.angle_increment);
		ROS_INFO("n points %d", n_points);
		ROS_INFO("test value %f", scan_msg.ranges[20]);
		i=0;
		ROS_INFO("At i = %d, front: %f, intensity: %f", i, scan_msg.ranges[i], scan_msg.intensities[i]);
		i=n_points/4;
		ROS_INFO("At i = %d, front: %f, intensity: %f", i, scan_msg.ranges[i], scan_msg.intensities[i]);
		i=n_points/2;
		ROS_INFO("At i = %d, front: %f, intensity: %f", i, scan_msg.ranges[i], scan_msg.intensities[i]);
		i=n_points*3/4;
		ROS_INFO("At i = %d, front: %f, intensity: %f", i, scan_msg.ranges[i], scan_msg.intensities[i]);


	#endif

	min_frontal_dist=1.0;
	int i_min = -1;
	
	for(i=0;i<n_points;i++) {
		//convert coordinates from polar to cartesian representation 
		//lidar_angle[i] = scan_msg.angle_min + i*scan_msg.angle_increment; //per lidar robolight
		lidar_angle[i] = scan_msg.angle_min + i*scan_msg.angle_increment +lidar_angle_offset; //per lidar danbot offset -PI
		lidar_dist_x[i] = cos(lidar_angle[i])*scan_msg.ranges[i];
		lidar_dist_y[i] = sin(lidar_angle[i])*scan_msg.ranges[i];
		if((scan_msg.intensities[i] > MIN_INTENSITY) && (scan_msg.ranges[i]>MIN_RANGE))
		{
			//Adjust Intensities
			if((lidar_dist_y[i]<half_frontal_width_2)&&(lidar_dist_y[i]>-half_frontal_width_2)&&(lidar_dist_x[i]<frontal_dist_thr_2)) 
			{
				//scan_msg.intensities[i] = lidar_dist_x[i]/scaling_const;
				if(lidar_dist_x[i]<min_frontal_dist) {
				   min_frontal_dist = lidar_dist_x[i];	
				   i_min = i;	
				}
			}	
		}
	}
	
	if(min_frontal_dist<=frontal_dist_thr){
		dist_coeff.data=0;
	} else if ((min_frontal_dist>frontal_dist_thr)&&(min_frontal_dist<=frontal_dist_thr_2)) {
		dist_coeff.data= (min_frontal_dist-frontal_dist_thr)/(frontal_dist_thr_2-frontal_dist_thr);
	} else if (min_frontal_dist>frontal_dist_thr_2){
		dist_coeff.data=1.0;
	} else {
		dist_coeff.data=0;
	}

	float d = dist_coeff.data;
	#ifdef DEBUG_LIDAR
		ROS_INFO("MIN FRONTAL DIST: %f, ANGLE: %f, RANGE: %f, X: %f, Y: %f, INDEX: %d, INTENSITY: %f, DIST_COEFF %f",min_frontal_dist, lidar_angle[i_min], scan_msg.ranges[i_min], lidar_dist_x[i_min], lidar_dist_y[i_min], i_min, scan_msg.intensities[i_min], d);
	#endif

	//lidar_publisher.publish(scan_msg);

	coeff_publisher.publish(dist_coeff);
}

int main(int argc, char **argv){
	
	ros::init(argc,argv,"lidar_supervisor");
	ros::NodeHandle node_obj;

	std::string s;
	ros::param::param<std::string>("/lidar_supervisor/scan_topic", s,"/scan");
	lidar_subscriber = node_obj.subscribe(s, 10, lidar_callback);
	coeff_publisher = node_obj.advertise<std_msgs::Float32>("/dist_coeff",5);
	#ifdef PUBLISH_SCAN_PLOT
	lidar_publisher = node_obj.advertise<sensor_msgs::LaserScan>("/scan_supervisor_plot",4);
	#endif

    ros::Rate rate(5.0);

	ros::param::param<float>("/lidar_supervisor/frontal_dist_thr", frontal_dist_thr,(float)0.3);
	ros::param::param<float>("/lidar_supervisor/frontal_dist_thr_2", frontal_dist_thr_2,(float)0.6);
	ros::param::param<float>("/lidar_supervisor/frontal_width", frontal_width,(float)0.3);
	ros::param::param<float>("/lidar_supervisor/frontal_width_2", frontal_width_2,(float)0.4);
	ros::param::param<float>("/lidar_supervisor/lidar_angle_offset", lidar_angle_offset,(float)0.0);
	
    while (node_obj.ok()) {

		#ifdef DEBUG_LOOP
			ROS_INFO("LIDAR_SUPERVISOR: loop test min_dist %f", min_frontal_dist);	
			ROS_INFO("LIDAR_SUPERVISOR: frontal_dist_thr %f", frontal_dist_thr);
		#endif
		rate.sleep();
		ros::spinOnce();
    }
}

