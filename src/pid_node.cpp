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

using namespace std;
using namespace ros;

const float pi = 3.1416;

double roll, pitch, yaw, z, z_vel;

double roll_sp = 0.0;
double roll_err = 0.0;
double prev_roll_err = 0.0;
double sum_roll_err = 0.0;

double pitch_sp = 0.0;
double pitch_err = 0.0;
double prev_pitch_err = 0.0;
double sum_pitch_err = 0.0;

double yaw_sp = 0.0;
double yaw_err = 0.0;
double prev_yaw_err = 0.0;
double sum_yaw_err = 0.0;

double z_sp  = 6.0;
double z_err = 0.0;
double prev_z_err = 0.0;

double z_vel_sp = 0.0;
double z_vel_err = 0.0;
double prev_z_vel_err = 0.0;
double sum_z_vel_err = 0.0;

int is_cb_first_run = 1;
Time last_imu_cb;

double roll_K_p = 0.050;
double roll_K_i = 0.0;
double roll_K_d = 0.001;

double pitch_K_p = 0.05;
double pitch_K_i = 0.0;
double pitch_K_d = 0.001;

double yaw_K_p = 0.01;
double yaw_K_i = 0.0;
double yaw_K_d = 0.001;

double z_K_p = 1.0;
double z_K_d = 0.001;

double z_vel_K_p = 1.0;
double z_vel_K_i = 0.01;
double z_vel_K_d = 0.1;

double roll_cmd;
double pitch_cmd;
double yaw_cmd;
double throttle_cmd;

int pid_switch           = 0;
int start_pid_timer      = 0;
int actuator_msg_updated = 0;
double dt;

mavros_msgs::State current_state;
geometry_msgs::Vector3 att_msg;
mavros_msgs::ActuatorControl actuator_control_msg;

void state_cb(const mavros_msgs::State::ConstPtr& msg);
void imu_cb(const sensor_msgs::Imu::ConstPtr& msg);
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
void velocity_cb(const geometry_msgs::TwistStamped::ConstPtr& msg);

/*
Main Loop
*/
int main(int argc, char **argv)
{
	cout << "== BEGIN PID CONTROLLER == \n" << endl;
   	
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
	Time pid_start_time;
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
					start_pid_timer = 0;
					ROS_INFO("Vehicle armed!");
				}
                last_request = Time::now();
			} 
		}       	
			
		if( current_state.armed){
			if(start_pid_timer==0){
			pid_start_time = Time::now();
			start_pid_timer = 1;
			ROS_INFO("PID waiting...");
			}
		}	
			
		if(current_state.armed && 
			(Time::now() - pid_start_time > Duration(3.0)) && pid_switch != 1){
				ROS_INFO("PID started!");
				pid_switch = 1;
		}		
		
		if(pid_switch != 1){
			local_pos_pub.publish(pose);				
		}else{
			if(actuator_msg_updated == 1){
				//PID control message publish
				actuator_controls_pub.publish(actuator_control_msg);
				//cout << "Command Published!" << endl;				
				actuator_msg_updated = 0;
			}						
		}		
		
		if(Time::now() - last_print > Duration(1.0) && pid_switch == 1){					
			cout << "ROLL[deg] = "  << roll/pi*180 << endl;
			cout << "PITCH[deg] = " << pitch/pi*180 << endl;
			cout << "YAW[deg] = "   << yaw/pi*180 << endl;
			cout << " z = " << z << endl;		
			cout << " z_vel = " << z_vel << endl;
			cout << "\n" << endl;			
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
		
		att_pub.publish(att_msg);
		
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
	/*
	 * Allocate states 
	*/
	//tf::Quaternion quater;
	//tf::quaternionMsgToTF(msg->orientation, quater);
	//tf::Matrix3x3(quater).getRPY(roll,pitch,yaw);	
	
	//stateVector(1,0) = msg->angular_velocity.x;
	//stateVector(3,0) = msg->angular_velocity.y;
	//stateVector(5,0) = msg->angular_velocity.z;

	//cout << " Read IMU..." << endl;	
	
	roll  = msg->angular_velocity.x;
	pitch = msg->angular_velocity.y;
	yaw   = msg->angular_velocity.z;	
	
	// Measure time since last callback
	if(is_cb_first_run == 1){
		dt = 0.02;
		is_cb_first_run = 0;
	}else{
		dt = (Time::now() - last_imu_cb).toSec();
		last_imu_cb = Time::now();
	}
	//cout << "dt = " << dt << endl;
	
	if(pid_switch == 1){
		/*
		 * Calculate roll command (PID)
		 */	
		roll_err = (roll_sp - roll);
		sum_roll_err = sum_roll_err + roll_err*dt;
		roll_cmd = roll_K_p*roll_err + 
						roll_K_i*sum_roll_err + 
							roll_K_d*(roll_err - prev_roll_err)/dt;
		prev_roll_err = roll_err;
		
		if(sum_roll_err > 0.1/roll_K_i){
			sum_roll_err = 0.1/roll_K_i;
			//cout << "== Z ERROR INTEGRATION Upper SATURATION !!! == \n" << endl;
		}else if(sum_roll_err < -0.1/roll_K_i){
			sum_roll_err = -0.1/roll_K_i;
			//cout << "== Z ERROR INTEGRATION Lower SATURATION !!! == \n" << endl;
		}
		// Roll command saturation 			
		if(roll_cmd > 1.0) roll_cmd = 1.0;
		if(roll_cmd < -1.0) roll_cmd = -1.0;
		//cout << " roll_cmd = "  << roll_cmd << endl;
		
		/*
		 * Calculate pitch command (PID)
		 */	
		pitch_err = (pitch_sp - pitch);
		sum_pitch_err = sum_pitch_err + pitch_err*dt;
		pitch_cmd = pitch_K_p*pitch_err + 
						pitch_K_i*sum_pitch_err + 
							pitch_K_d*(pitch_err - prev_pitch_err)/dt;
		prev_pitch_err = pitch_err;
		
		if(sum_pitch_err > 0.1/pitch_K_i){
			sum_pitch_err = 0.1/pitch_K_i;
			//cout << "== Z ERROR INTEGRATION Upper SATURATION !!! == \n" << endl;
		}else if(sum_pitch_err < -0.1/pitch_K_i){
			sum_pitch_err = -0.1/pitch_K_i;
			//cout << "== Z ERROR INTEGRATION Lower SATURATION !!! == \n" << endl;
		}
		// pitch command saturation 			
		if(pitch_cmd > 1.0) pitch_cmd = 1.0;
		if(pitch_cmd < -1.0) pitch_cmd = -1.0;
		
		pitch_cmd = - pitch_cmd;
		//cout << " pitch_cmd = "  << pitch_cmd << endl;	

		/*
		 * Calculate yaw command (PID)
		 */	
		yaw_err = (yaw_sp - yaw);
		sum_yaw_err = sum_yaw_err + yaw_err*dt;
		yaw_cmd = yaw_K_p*yaw_err + 
						yaw_K_i*sum_yaw_err + 
							yaw_K_d*(yaw_err - prev_yaw_err)/dt;
		prev_yaw_err = yaw_err;
		
		if(sum_yaw_err > 0.1/yaw_K_i){
			sum_yaw_err = 0.1/yaw_K_i;
			//cout << "== Z ERROR INTEGRATION Upper SATURATION !!! == \n" << endl;
		}else if(sum_yaw_err < -0.1/yaw_K_i){
			sum_yaw_err = -0.1/yaw_K_i;
			//cout << "== Z ERROR INTEGRATION Lower SATURATION !!! == \n" << endl;
		}
		// yaw command saturation 			
		if(yaw_cmd > 1.0) yaw_cmd = 1.0;
		if(yaw_cmd < -1.0) yaw_cmd = -1.0;

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
