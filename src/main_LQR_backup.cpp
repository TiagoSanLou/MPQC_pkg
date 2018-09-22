#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <stdlib.h>
#include <iostream>
#include <tf/transform_broadcaster.h>
#include <angles/angles.h>
#include <mavros_msgs/ActuatorControl.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <qpOASES.hpp>
#include <Eigen/Dense>

#include "definicoes.hpp"
#include "lqrdados.hpp"
#include "funcoes_auxiliares.hpp"

using namespace std;
using namespace ros;

const float pi = 3.1416;

double roll, pitch, yaw;
MatrizX stateVector = MatrizX::Zero(6,1);
double height;
double height_sp = 3.0;
double height_err;
double prev_height_err;
double sum_height_err;

double K_p = 0.65;
double K_i = 0.0003;
double K_d = 0.4;

double roll_cmd;
double pitch_cmd;
double yaw_cmd;
double throttle_cmd;

int lqr_switch      = 0;
int start_mpc_timer = 0;

VetorX u;

mavros_msgs::State current_state;

void state_cb(const mavros_msgs::State::ConstPtr& msg);
void imu_cb(const sensor_msgs::Imu::ConstPtr& msg);
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);


/*
Main Loop
*/
int main(int argc, char **argv)
{
	cout << "========================== \n" << endl;
	cout << "== BEGIN LQR CONTROLLER == \n" << endl;
    cout << "========================== \n" << endl;
    
    cout << "== Load LQR Matrices == \n" << endl;
   	LQRDados LQR;
   	//LQR.imprime_LQR();
   	
	cout << "== Set ROS == \n" << endl;
	/*
	ROS DECLARATIONS
	*/
	init(argc, argv, "main_LQR");
	NodeHandle nh;
	
	//Subscribers & Publishers
	Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>
	    ("/mavros/imu/data", 100 ,&imu_cb);
    Subscriber state_sub = nh.subscribe<mavros_msgs::State>
        ("mavros/state", 10, state_cb);
    Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
        ("/mavros/local_position/pose", 10, pose_cb);        
    ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
        ("mavros/cmd/arming");        
    ServiceClient takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>
        ("mavros/cmd/takeoff");
	ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
		("mavros/set_mode");        
    Publisher actuator_controls_pub = nh.advertise<mavros_msgs::ActuatorControl>
	    ("/mavros/actuator_control", 1000);
	Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
		("mavros/setpoint_position/local", 10);	    
    
    //Loop Rate
	Rate loop_rate(250.0);
	
	// Messages
	mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    
    mavros_msgs::CommandTOL takeoff_cmd;
    takeoff_cmd.request.latitude = 0;
    takeoff_cmd.request.longitude = 0;
    takeoff_cmd.request.altitude = 5.0;
    takeoff_cmd.request.yaw = 0;
    takeoff_cmd.request.min_pitch = 0;
    
    mavros_msgs::ActuatorControl actuator_control_msg;
    
    geometry_msgs::PoseStamped pose;
	pose.pose.position.x = 0;
	pose.pose.position.y = 0;
	pose.pose.position.z = 2;
	
	mavros_msgs::SetMode offb_set_mode;
	offb_set_mode.request.custom_mode = "OFFBOARD";

	// wait for FCU connection
	while(ok() && current_state.connected){
		ros::spinOnce();
		loop_rate.sleep();
	cout << "waiting for FCU connection" << endl;
	}
	
    //send a few setpoints before starting
	for(int i = 100; ros::ok() && i > 0; --i){
		local_pos_pub.publish(pose);
		ros::spinOnce();
		loop_rate.sleep();
	}

	//Arming Request Timer
	Time last_request = ros::Time::now();
	Time mpc_start_time;

    /*
     * Control Loop	 
    */
    int i = 0;
    while (ok())
    {
		/*
   	    if( current_state.mode != "OFFBOARD"){          
			ROS_INFO("Offboard NOT enabled...");           
        } else {
			if( !current_state.armed && (Time::now() - last_request > Duration(5.0))){
				if( arming_client.call(arm_cmd) &&
					arm_cmd.response.success){
                    ROS_INFO("Vehicle armed!");
                    i = 0;
                }
                last_request = Time::now();
            }
        }        
        */      
        
        if( current_state.mode != "OFFBOARD" &&
			(ros::Time::now() - last_request > ros::Duration(5.0))){
			if( set_mode_client.call(offb_set_mode) &&
			offb_set_mode.response.success){
				ROS_INFO("Offboard enabled");
			}
			last_request = ros::Time::now();
		} else {
			if( !current_state.armed && (Time::now() - last_request > Duration(5.0))){
				if( arming_client.call(arm_cmd) && arm_cmd.response.success){
					start_mpc_timer = 0;
					ROS_INFO("Vehicle armed!");
					//if( takeoff_client.call(takeoff_cmd) && takeoff_cmd.response.success){
						//ROS_INFO("Vehicle taking off!");
					//}
				}
                last_request = Time::now();
			} 
		}       	
			
		if( current_state.armed){
			if(start_mpc_timer==0){
			mpc_start_time = Time::now();
			start_mpc_timer = 1;
			ROS_INFO("LQR waiting...");
			}
		}	
			
		if(current_state.armed && 
			(Time::now() - mpc_start_time > Duration(5.0)) && lqr_switch != 1){
				ROS_INFO("LQR started!");
				lqr_switch = 1;
		}		
		
		if(lqr_switch != 1){
			local_pos_pub.publish(pose);				
		}else{	
		
			//cout << "== LQR optimal control action == \n" << endl;
			// LQR control law
			u = LQR.K_N*stateVector;
			
			// Integrate command
			u = u + LQR.ulast;	
			LQR.ulast = u;
			
		    //cout << "== Allocate commands == \n" << endl;
			//Command Calculation
			roll_cmd = u(0);
			pitch_cmd = u(1);
			yaw_cmd = u(2);
			
			//Calculate throttle command (PID)
			height_err = (height_sp - height);
			throttle_cmd = K_p*height_err + 
							K_i*(height_err + sum_height_err) + 
								K_d*(height_err - prev_height_err);
			prev_height_err = height_err;
			sum_height_err =+ height_err;
			// Throttle command saturation 			
			if(throttle_cmd > 0.8) throttle_cmd = 0.8;
			if(throttle_cmd < 0.55) throttle_cmd = 0.55;			

			//Control message alocation
			actuator_control_msg.header.stamp = ros::Time::now();
			actuator_control_msg.group_mix = 0;
	        actuator_control_msg.controls[0] = roll_cmd;
	    	actuator_control_msg.controls[1] = pitch_cmd;
	    	actuator_control_msg.controls[2] = yaw_cmd;
	    	actuator_control_msg.controls[3] = throttle_cmd;
	    	actuator_control_msg.controls[4] = 0.0;
	    	actuator_control_msg.controls[5] = 0.0;
	    	actuator_control_msg.controls[6] = 0.0;
	    	actuator_control_msg.controls[7] = 0.0;
			
			//Control message publish
	        actuator_controls_pub.publish(actuator_control_msg);
	        cout << "Command Published!" << endl;
		}
        //Print States
        //cout << "ROLL[deg] = "  << roll/pi*180 << endl;
        //cout << "PITCH[deg] = " << pitch/pi*180 << endl;
        //cout << "YAW[deg] = "   << yaw/pi*180 << endl;
        //cout << "stateVector = " << stateVector << endl;
        //cout << "height = " << height << endl;
		
		if(lqr_switch == 1){
			//cout << " U = "  << u << endl;
			//cout << " throttle_cmd = "  << throttle_cmd << endl;

			//cout << " --------" << endl;
		}
                
        spinOnce();
        loop_rate.sleep();
        i++;
    }
    return 0; 
}

/* 
Subscriber Callback Functionss
*/

//State msg callback
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

//Imu data msg callback
void imu_cb(const sensor_msgs::Imu::ConstPtr& msg)
{
	//double roll, pitch, yaw; // comment if global variable

	tf::Quaternion quater;

	tf::quaternionMsgToTF(msg->orientation, quater);

	tf::Matrix3x3(quater).getRPY(roll,pitch,yaw);
	
	stateVector(0,0) = roll;
	stateVector(2,0) = pitch;
	stateVector(4,0) = yaw;
	
	stateVector(1,0) = msg->angular_velocity.x;
	stateVector(3,0) = msg->angular_velocity.y;
	stateVector(5,0) = msg->angular_velocity.z;

	//roll = angles::normalize_angle_positive(roll);
	//pitch = angles::normalize_angle_positive(pitch);
	//yaw = angles::normalize_angle_positive(yaw);

	cout << "Rotation around the z axis is [rad]" << yaw << endl;
	//cout << "Rotation around the x axis is [deg]" << roll/pi*180 << endl;
	//cout << "Rotation around the y axis is [deg]" << pitch/pi*180 << endl;
	//cout << "Rotation around the z axis is [deg]" << yaw/pi*180 << endl;
}

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	height = msg->pose.position.z;
}
