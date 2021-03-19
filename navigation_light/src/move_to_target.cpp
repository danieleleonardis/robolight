#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <iostream>
#include <geometry_msgs/Twist.h>
//#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>

#define PI 3.1415926535
#define PI2 6.283185307


//#define DEBUG_LOG
#define TF_WATCHDOG_COUNT 40 //counts, numero di cicli invecchiamento tf
#define GOAL_WATCHDOG_COUNT 100 //counts, cicli invecchiamento target goal

#define CHANGE_ROTATION_HYST 0.2 //in radianti, isteresi per cambio direzione di rotazione (ostacolo a 180°)
float nav_ang_speed; // 0.3
float nav_lin_speed; // 0.05
float nav_rot_thr; //0.5 in radianti, angolo di errore di heading oltre il quale blocca l'avanzamento (prima rotazione sul posto)
float goal_pos_thr; //0.1 n metri, soglia di distanza sotto la quale il target è raggiunto
float goal_pos_hyst; // isteresi su goal_pos_thr una volta raggiunto il target

ros::Publisher cmd_vel_publisher;
ros::Subscriber goal_subscriber;
geometry_msgs::Twist twist_ref_auto, twist_ref;
geometry_msgs::PoseStamped goal_msg;

tf::StampedTransform tf_map2base;
tf::StampedTransform tf_map2target;

int count_tf_ok = 0;
int count_goal_ok = 0;
int mode = 0;
int target_ok = 0; //flag target raggiunto

float pos_err_x, pos_err_y, pos_err, ang_err;
float goal_thr;

double old_ang_err = 0;

void goal_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	goal_msg = *msg;	
	ROS_INFO("New Goal received x=%f y = %f, w=%f", msg->pose.position.x, msg->pose.position.y, msg->pose.orientation.w);
	count_goal_ok = GOAL_WATCHDOG_COUNT;
}


int main(int argc, char **argv){
	ros::init(argc,argv,"move_to_target");
	ros::NodeHandle node_obj;

	//ros::Subscriber number_subscriber = node_obj.subscribe("/numbers",10,number_callback);
    goal_subscriber = node_obj.subscribe("/move_base_simple/goal", 1, goal_callback);
    tf::TransformListener listener;

	cmd_vel_publisher = node_obj.advertise<geometry_msgs::Twist>("/cmd_vel_auto",1);
  
	ros::Rate rate(20.0);

	goal_thr = goal_pos_thr;
	int param_update_cnt = 0;
	//ROS_INFO("avvio loop");
	while (node_obj.ok())
	{
		//update params counter
    	if(param_update_cnt>0)
    		param_update_cnt--;
    	else
    	{
    		param_update_cnt = 50;
	    	ros::param::param<float>("/move_to_target/nav_ang_speed", nav_ang_speed,(float)0.3);
	    	ros::param::param<float>("/move_to_target/nav_lin_speed", nav_lin_speed,(float)0.05);
	    	ros::param::param<float>("/move_to_target/nav_rot_thr", nav_rot_thr,(float)0.5);
	    	ros::param::param<float>("/move_to_target/goal_pos_thr", goal_pos_thr,(float)0.1);
	    	ros::param::param<float>("/move_to_target/goal_pos_hyst", goal_pos_hyst,(float)0.1);
    	}

		try
		{
			listener.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(2.0));
			listener.lookupTransform("map", "base_link", ros::Time(0), tf_map2base);
			#ifdef DEBUG_LOG		
				ROS_INFO("ricevuta TF map2base");
			 #endif
			 count_tf_ok = TF_WATCHDOG_COUNT;
		}
		catch (tf::TransformException ex)
		{
			#ifdef DEBUG_LOG
			 ROS_INFO("errore ricezione TF");
			 ROS_ERROR("%s",ex.what());
			#endif		
			 mode = 0;         
		}
		if(count_tf_ok > 0)
		{
			count_tf_ok--;
		}
		if(count_goal_ok > 0)
		{
			count_tf_ok--;
		}
		#ifdef DEBUG_LOG
			 ROS_INFO("count tf: %d, count goal: %d", count_tf_ok, count_goal_ok);
		#endif	

		
		if((count_tf_ok>0)&&(count_goal_ok>1)) //SE ricevuto target e localizzazione robot valida
		{ 
				//calcolo errore posizione (solo traslazione)  
				pos_err_x =  goal_msg.pose.position.x - tf_map2base.getOrigin().x();		
				pos_err_y =  goal_msg.pose.position.y - tf_map2base.getOrigin().y();
				pos_err = sqrt(pos_err_x*pos_err_x+pos_err_y*pos_err_y);
				//ang_err =  goal_msg.pose.orientation.w - tf_map2base.getRotation().w(); //controlla quaternione se va bene finchè robot planare
				double roll, pitch, yaw;
				double ang_goal;
				double ang_err_test;
				ang_goal = atan2(pos_err_y, pos_err_x);
				tf::Matrix3x3(tf_map2base.getRotation()).getRPY(roll, pitch, yaw);
				

				ang_err = ang_goal - yaw;
				if(ang_err>0)
					ang_err_test = ang_err-PI2;
				else
					ang_err_test = ang_err+PI2;

				//DEBUGGARE ALGORITMO ISTERESI (NECESSARIO PER ANGOLI DI ERRORE DI 180°)
				if(abs(ang_err_test)<abs(ang_err))
				{
					ang_err = ang_err_test;	
				}
				//old_ang_err = ang_err;
				#ifdef DEBUG_LOG
				ROS_INFO("pos_err_x: %f pos_err_y %f, goal_ang %f, current_ang %f, ang_err %f", pos_err_x, pos_err_y, atan2(pos_err_y, pos_err_x), yaw, ang_err);
				#endif
		}
			
		//DEFAULT CMD_VEL
		twist_ref.linear.x = 0;
		twist_ref.angular.z = 0;
		//CALCOLO CMD_VEL
		if((count_tf_ok>0)&&(count_goal_ok>1))	
		{
			if(target_ok==0) //if target not reached yet
			{
				//set linear
				if(abs(ang_err)<nav_rot_thr)
					twist_ref.linear.x = nav_lin_speed;
				else
					twist_ref.linear.x = 0;

				//set twist
				twist_ref.angular.z = 1.0*ang_err; //segno meno riferimenti angolari inversi  fra TF e cmd_vel

			if(twist_ref.angular.z > nav_ang_speed)
				twist_ref.angular.z = nav_ang_speed;
			if(twist_ref.angular.z < -nav_ang_speed)
				twist_ref.angular.z = -nav_ang_speed;
			}
		}

		if(pos_err<goal_thr)
		{
			target_ok = 1;
			goal_thr = goal_pos_thr+goal_pos_hyst;
		}
		else
		{
			target_ok = 0;
			goal_thr = goal_pos_thr;
		}


		cmd_vel_publisher.publish(twist_ref);

		rate.sleep();
		ros::spinOnce();
    }


}

