//roslaunch mavros px4.launch fcu_url:=udp://:14540@14557
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
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3.h>
#include <Eigen/Dense>

#include "definicoes.hpp"
#include "lqrdados.hpp"
#include "funcoes_auxiliares.hpp"

using namespace std;
using namespace ros;

const float pi = 3.1416;

double roll, roll_rate, pitch, pitch_rate, yaw, yaw_rate, z, z_vel;
MatrizX stateVector = MatrizX::Zero(6,1);

double z_sp  = 6.0;
double z_err = 0.0;
double prev_z_err = 0.0;

double z_vel_sp = 0.5;
double z_vel_err = 0.0;
double prev_z_vel_err = 0.0;
double sum_z_vel_err = 0.0;

double z_K_p = 1.0;
double z_K_d = 0.001;

double z_vel_K_p = 1.1;
double z_vel_K_i = 0.01;
double z_vel_K_d = 0.1;

double roll_cmd;
double pitch_cmd;
double yaw_cmd;
double throttle_cmd;

double MAX_TORQUE = 10.0;
float  LQR_DELAY  = 3;

LQRDados LQR;

int lqr_switch           = 0;
int start_lqr_timer      = 0;
int actuator_msg_updated = 0;
int is_cb_first_run = 1;
double dt;
Time last_imu_cb;

VetorX u;

mavros_msgs::State current_state;
geometry_msgs::Vector3 att_msg;
mavros_msgs::ActuatorControl actuator_control_msg;

void state_cb(const mavros_msgs::State::ConstPtr& msg);
void imu_cb(const sensor_msgs::Imu::ConstPtr& msg);
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
void velocity_cb(const geometry_msgs::TwistStamped::ConstPtr& msg);
double map_torque(double x, double in_min, double in_max, double out_min, double out_max);

/*
Main Loop
*/
int main(int argc, char **argv)
{
	cout << "== BEGIN LQR CONTROLLER == \n" << endl;
   	
	cout << "== Set ROS == \n" << endl;
	/*
	ROS DECLARATIONS
	*/
	init(argc, argv, "main_LQR");
	NodeHandle nh;
	
	//Subscribers & Publishers
	Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>
	    ("/mavros/imu/data", 1 ,&imu_cb);
    Subscriber state_sub = nh.subscribe<mavros_msgs::State>
        ("mavros/state", 10, state_cb);
    Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
        ("/mavros/local_position/pose", 1, pose_cb);  
    Subscriber velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>
        ("/mavros/local_position/velocity", 1, velocity_cb);      
    ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
        ("mavros/cmd/arming"); 
	ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
		("mavros/set_mode");
	Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
		("mavros/setpoint_position/local", 1);	   
	Publisher att_pub = nh.advertise<geometry_msgs::Vector3>
		("mavros/attitude_RPY/", 1);
	Publisher actuator_controls_pub = nh.advertise<mavros_msgs::ActuatorControl>
		("/mavros/actuator_control", 1); 
    
    //Loop Rate
	Rate loop_rate(250.0);
	
	// Messages
	mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;  
    
    geometry_msgs::PoseStamped pose;
	pose.pose.position.x = 0;
	pose.pose.position.y = 0;
	pose.pose.position.z = 5;
	
	mavros_msgs::SetMode offb_set_mode;
	offb_set_mode.request.custom_mode = "OFFBOARD";

	// wait for FCU connection
	while(ok() && current_state.connected){
		ros::spinOnce();
		loop_rate.sleep();
		cout << "waiting for FCU connection" << endl;
	}
	
    //send a few setpoints before starting
	for(int i = 100; ok() && i > 0; --i){
		local_pos_pub.publish(pose);
		ros::spinOnce();
		loop_rate.sleep();
	}

	//Arming Request Timer
	Time last_request = Time::now();
	Time lqr_start_time;
	Time last_print = Time::now();

    /*
     * Control Loop	 
    */
    int i = 0;
    while (ok())
    {        
        if( current_state.mode != "OFFBOARD" &&
			(Time::now() - last_request > Duration(5.0))){
			if( set_mode_client.call(offb_set_mode) &&
			offb_set_mode.response.success){
				ROS_INFO("Offboard enabled");
			}
			last_request = ros::Time::now();
		} else {
			if( !current_state.armed && (Time::now() - last_request > Duration(5.0))){
				if( arming_client.call(arm_cmd) && arm_cmd.response.success){
					start_lqr_timer = 0;
					ROS_INFO("Vehicle armed!");
				}
                last_request = Time::now();
			} 
		}       	
			
		if( current_state.armed){
			if(start_lqr_timer==0){
			lqr_start_time = Time::now();
			start_lqr_timer = 1;
			ROS_INFO("LQR waiting...");
			}
		}	
			
		if(current_state.armed && 
			(Time::now() - lqr_start_time > Duration(LQR_DELAY)) && lqr_switch != 1){
				ROS_INFO("LQR started!");
				lqr_switch = 1;
		}		
		
		if(lqr_switch != 1){
			local_pos_pub.publish(pose);				
		}else{
			if(actuator_msg_updated == 1){
				//LQR control message publish
				actuator_controls_pub.publish(actuator_control_msg);
				//cout << "Command Published!" << endl;
				actuator_msg_updated = 0;
			}						
		}
		
		if(Time::now() - last_print > Duration(1.0) && lqr_switch == 1){					
			cout << "ROLL[deg] = "  << roll/pi*180 << endl;
			cout << "Roll Rate[/s] = "   << roll_rate << endl;
			cout << "PITCH[deg] = " << pitch/pi*180 << endl;
			cout << "Pitch Rate[/s] = "   << pitch_rate << endl;
			cout << "YAW[deg] = "   << yaw/pi*180 << endl;
			cout << "Yaw Rate[/s] = "   << yaw_rate << endl;			
			cout << " z = " << z << endl;		
			cout << " z_vel = " << z_vel << endl;
			cout << "\n" << endl;	
			cout << " u = "  << u << endl;
			cout << " roll_cmd = "  << roll_cmd << endl;
			cout << " pitch_cmd = "  << pitch_cmd << endl;
			cout << " yaw_cmd = "  << yaw_cmd << endl;
			cout << " throttle_cmd = "  << throttle_cmd << endl;
			cout << " -------------------------------" << endl;	
			last_print = Time::now();		
		}
		
		att_msg.x = roll;
		att_msg.y = pitch;
		att_msg.z = yaw;
   		//att_pub.publish(att_msg);
		
        spinOnce();
        loop_rate.sleep();
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
	tf::Quaternion quater;
	tf::quaternionMsgToTF(msg->orientation, quater);
	tf::Matrix3x3(quater).getRPY(roll,pitch,yaw);
	
	roll_rate  = msg->angular_velocity.x;
	pitch_rate = msg->angular_velocity.y;
	yaw_rate   = msg->angular_velocity.z;
	
	stateVector(0,0) = roll;
	stateVector(1,0) = roll_rate;
	stateVector(2,0) = pitch;
	stateVector(3,0) = pitch_rate;
	stateVector(4,0) = yaw;		
	stateVector(5,0) = yaw_rate;
	
	// Measure time since last callback
	if(is_cb_first_run == 1){
		dt = 0.02;
		is_cb_first_run = 0;
	}else{
		dt = (Time::now() - last_imu_cb).toSec();
		last_imu_cb = Time::now();
	}
	//cout << "dt = " << dt << endl;
	
	if(lqr_switch == 1){
		// LQR control law
	    u = LQR.K_N*stateVector;
		//u = u*dt + LQR.ulast;	
		//u = u + LQR.ulast;
		LQR.ulast = u;
		
		//cout << "== Allocate commands == \n" << endl;
		//Command mapping
		roll_cmd  = map_torque(u(0),-MAX_TORQUE,MAX_TORQUE,-1.0, 1.0);
		pitch_cmd = map_torque(u(1),-MAX_TORQUE,MAX_TORQUE, 1.0,-1.0);
		yaw_cmd   = map_torque(u(2),-MAX_TORQUE,MAX_TORQUE, 1.0,-1.0);	
		
		/*
		 * Calculate throttle command (PID)
		 */		
		// Z tracking	
		z_err = z_sp - z;
		//cout << " z_err = "  << z_err << endl;
		z_vel_sp = z_K_p*z_err + 
					z_K_d*(z_err - prev_z_err)/dt;
		//cout << " z_vel_sp = "  << z_vel_sp << endl;
		prev_z_err = z_err; 		
		
		// Z velocity tracking
		z_vel_err = (z_vel_sp - z_vel);
		//cout << " z_vel_err = "  << z_vel_err << endl;
		sum_z_vel_err = sum_z_vel_err + z_vel_err*dt;
		//cout << " sum_z_vel_err = "  << sum_z_vel_err << endl;
		throttle_cmd = z_vel_K_p*z_vel_err + 
						z_vel_K_i*sum_z_vel_err + 
							z_vel_K_d*(z_vel_err - prev_z_vel_err)/dt;
		prev_z_vel_err = z_vel_err;
		//cout << " throttle_cmd = "  << throttle_cmd << endl;		
		
		if(sum_z_vel_err > 0.6/z_vel_K_i){
			sum_z_vel_err = 0.6/z_vel_K_i;
			//cout << "== Z ERROR INTEGRATION Upper SATURATION !!! == \n" << endl;
		}else if(sum_z_vel_err < -0.1/z_vel_K_i){
			sum_z_vel_err = -0.1/z_vel_K_i;
			//cout << "== Z ERROR INTEGRATION Lower SATURATION !!! == \n" << endl;
		}
		
		// Throttle command saturation 			
		if(throttle_cmd > 1.0) throttle_cmd = 1.0;
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
		
		actuator_msg_updated = 1;
	}	
}

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	z = msg->pose.position.z;
	//cout << " Read Height..." << endl;
}

void velocity_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
	z_vel = msg->twist.linear.z;
	//cout << " Read z velocity..." << endl;
}

double map_torque(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
