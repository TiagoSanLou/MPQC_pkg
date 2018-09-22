// roslaunch mavros px4.launch 
#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <stdlib.h>
#include <tf/transform_broadcaster.h>
#include <angles/angles.h>
#include <mavros_msgs/ActuatorControl.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/RCIn.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <qpOASES.hpp>
#include <Eigen/Dense>
#include <ctime>
#include <string>

#include "definicoes.hpp"
#include "mpcdados.hpp"
#include "expParam.hpp"
#include "funcoes_auxiliares.hpp"

using namespace std;
using namespace ros;

const float pi = 3.1416;

double roll, roll_rate, pitch, pitch_rate, yaw, yaw_rate, z, z_vel;
float roll_sp, roll_rate_sp, pitch_sp, pitch_rate_sp, yaw_sp, yaw_rate_sp;
float throttle_sp;
MatrizX stateVector       = MatrizX::Zero(6,1);
MatrizX pred_stateVector  = MatrizX::Zero(6,1);
MatrizX setpointVector = MatrizX::Zero(6,1); // Mudar de acordo com o MPC.nr
VetorX u;	// Command vector

/*
 * 
*/ 
float z_sp  = 6.0;
float z_err = 0.0;
float prev_z_err = 0.0;

float z_vel_sp = 0.5;
float z_vel_err = 0.0;
float prev_z_vel_err = 0.0;
float sum_z_vel_err = 0.0;

float z_K_p = 1.0;
float z_K_d = 0.001;

float z_vel_K_p = 1.1;
float z_vel_K_i = 0.01;
float z_vel_K_d = 0.1;

float roll_torque_cmd;
float pitch_torque_cmd;
float yaw_torque_cmd;

float roll_cmd;
float pitch_cmd;
float yaw_cmd;
float throttle_cmd;

//
MPCDados MPC;
int nu = MPC.B.cols();
int N = MPC.N;
float MAX_TORQUE = MPC.MAX_TORQUE;


//
expParam PAR(&MPC);
int np = PAR.np;

//
int mpc_switch       = 0;
int isFirstRun		 = 1;
int imu_msg_updated  = 0;
float dt;
float node_run_time = 0.0;
Time last_imu_cb;

//
mavros_msgs::State current_state;
mavros_msgs::ActuatorControl actuator_control_msg;

// Subscriber Callback Functions
void state_cb(const mavros_msgs::State::ConstPtr& msg);
void imu_cb(const sensor_msgs::Imu::ConstPtr& msg);
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
void velocity_cb(const geometry_msgs::TwistStamped::ConstPtr& msg);
void rc_in_cb(const mavros_msgs::RCIn::ConstPtr& msg);

//Torque Mapping Function (Normalization) 
float map_torque(float x, float in_min, float in_max, float out_min, float out_max);

/*
Main Loop
*/
int main(int argc, char **argv)
{
	USING_NAMESPACE_QPOASES		
	
	/*
	 * Open telemetry files
	 * */
	time_t now = time(0);
	tm *ltm = localtime(&now);
	// print various components of tm structure.
	//cout << "Year" << 1900 + ltm->tm_year<<endl;
	//cout << "Month: "<< 1 + ltm->tm_mon<< endl;
	//cout << "Day: "<<  ltm->tm_mday << endl;
	//cout << "Time: "<< 1 + ltm->tm_hour << ":";
	//cout << 1 + ltm->tm_min << ":";
	//cout << 1 + ltm->tm_sec << endl;
	
	//string hour = to_string(ltm->tm_hour);
	//string minute = to_string(ltm->tm_min);
	
	//string actuator_filename_str     =  "outputs/" + hour + "_" + minute + 
	//"_actuator_control.csv";					
	
	//const char * actuator_filename = actuator_filename_str.c_str();					
						
	cout << "== CREATING OUTPUT FILES (.TXT) == \n" << endl;    
	// System variables are printed in txt files
	ofstream param_file;
	ofstream time_file;
	ofstream attitude_file;
    ofstream attitude_sp_file;
	ofstream prediction_file;
	ofstream command_file;
	
	param_file.open ("/home/odroid/catkin_ws/src/mpc_quad_px4/outputs/sim_params.txt");
	time_file.open ("/home/odroid/catkin_ws/src/mpc_quad_px4/outputs/time_vec_c.txt");
	attitude_file.open ("/home/odroid/catkin_ws/src/mpc_quad_px4/outputs/attitude.txt");	
    attitude_sp_file.open ("/home/odroid/catkin_ws/src/mpc_quad_px4/outputs/attitude_sp.txt");	
	prediction_file.open ("/home/odroid/catkin_ws/src/mpc_quad_px4/outputs/prediction.txt");		
	command_file.open ("/home/odroid/catkin_ws/src/mpc_quad_px4/outputs/commands.txt");
	
	// Print simulation set up
	param_file << MPC.N << "\t" << MPC.test_duration <<  "\t "<< MPC.tau;
	param_file.close();
	
	//cout << "== BEGIN MPC CONTROLLER == \n" << endl; 	
	ROS_INFO("BEGIN MPC CONTROLLER");

	/* 
	 * Setting up QP object. 
	*/
	//QProblem attQP( N*nu , MPC.Bineq.rows() );
	QProblem attQP( np , PAR.Br.rows() ); //PARAMETRIZED
	
	Options options;
	attQP.setOptions( options );	
	
	// Declare Control Sequence (Utilde)
	VetorX xSol;
	xSol.resize(N*nu);	
	
	// Parametrized Control sequence (p)
	VetorX pSol;
	pSol.resize(np);	
	
	/*
	ROS DECLARATIONS
	*/
	//cout << "== Set ROS == \n" << endl;
	ROS_INFO("Set ROS");
	
	init(argc, argv, "main_MPC");
	NodeHandle nh;
	last_imu_cb = Time::now(); // ???
	
	//Subscribers & Publishers
	Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>
	    ("/mavros/imu/data", 1 , &imu_cb);
    Subscriber state_sub = nh.subscribe<mavros_msgs::State>
        ("mavros/state", 10, state_cb);
    Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
        ("/mavros/local_position/pose", 1, pose_cb);  
    Subscriber velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>
        ("/mavros/local_position/velocity", 1, velocity_cb); 
    Subscriber rc_in_sub = nh.subscribe<mavros_msgs::RCIn> 
        ("/mavros/rc/in", 1, rc_in_cb);                   
    ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
        ("mavros/cmd/arming");        
	ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
		("mavros/set_mode");   
	Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
		("mavros/setpoint_position/local", 1);		     
    Publisher actuator_controls_pub = nh.advertise<mavros_msgs::ActuatorControl>
	    ("/mavros/actuator_control", 1);  	   	
	Publisher attitude_euler_pub = nh.advertise<geometry_msgs::Point>
		("mavros/attitude_euler", 1);
	Publisher attitude_euler_sp_pub = nh.advertise<geometry_msgs::Point>
		("mavros/attitude_euler_sp", 1);   
    
    //Loop Rate
	Rate loop_rate(200.0);
	
	// Arm Cmd, set mode and Initial Position Messages
	mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true; 
    
    mavros_msgs::SetMode offb_set_mode;
	offb_set_mode.request.custom_mode = "OFFBOARD";     
    
    geometry_msgs::PoseStamped pose;
	pose.pose.position.x = 0;
	pose.pose.position.y = 0;
	pose.pose.position.z = 0;	
	
	geometry_msgs::Point euler_angles;
	
	geometry_msgs::Point euler_angles_sp;	

	// Wait for FCU connection
	while(ok() && current_state.connected){
		ros::spinOnce();
		loop_rate.sleep();
		cout << "waiting for FCU connection" << endl;
	}
	
    //Send a few setpoints before starting
	for(int i = 100; ok() && i > 0; --i){
		local_pos_pub.publish(pose);
		ros::spinOnce();
		loop_rate.sleep();
	}

	//Arming Request Timer
	Time last_request = Time::now();
	//Time mpc_start_time;
	Time last_print = Time::now();

    /*
     * Control Loop	 
    */
    //cout << "== Begin control loop == \n" << endl;
    ROS_INFO("BEGIN CONTROL LOOP");
    int i = 0; // Loop count
    while (ok())
    {
   	    if( current_state.mode != "OFFBOARD"){          
			//ROS_INFO("Offboard NOT enabled..."); 
			mpc_switch = 0;          
        } else {
			if( !current_state.armed && (Time::now() - last_request > Duration(5.0))){
				if( arming_client.call(arm_cmd) &&
					arm_cmd.response.success){
                    ROS_INFO("Vehicle armed!");
                    mpc_switch = 1;				
                }
                last_request = Time::now();
            }
        }      	
		if(mpc_switch != 1){
			//ROS_INFO("MPC turned off...");
			local_pos_pub.publish(pose);						
		}else{
			if(imu_msg_updated == 1){	
				//ROS_INFO("Publishing MPC Commands");		
				
				//cout << "== Project Ytilde == \n" << endl;
				// Yref projection into Prediction Horizon
				for(int j=0; j<N; j++){
					MPC.yref_pred.segment( j*MPC.ny, MPC.ny ) = setpointVector;
				}			
							
				//cout << "== Calculate Gradient Matrix == \n" << endl;
				// Calculating Gradient Matrix (MPC.F / qpOASES.g) (ADICIONAR MPC->F3*U_D !!!)
				MPC.F = MPC.F1*stateVector + MPC.F2*MPC.yref_pred;	
			
				//cout << "== Calculate Upper-bound == \n" << endl;
				// Calculating Upper-bound (MPC.Bineq / qpOASES.ubA)		
				MPC.Bineq = MPC.G1*stateVector + MPC.G2*MPC.ulast + MPC.G3;					
				
				/*
				 * PARAMETRIZED CONVERTION BEGIN
				 * */
				PAR.compute_reduced_matrices(&MPC);	
				
				// Converting 'PAR.Hr' to qpOASES H
				real_t H[np*np];
				for(int j=0; j < np*np ; j++){
					int liNum,colNum;
					liNum  = j/np;
					colNum = j-j/np*np;
					H[j] = PAR.Hr(liNum,colNum);
				}	

				// Converting 'PAR.Ar' to qpOASES Ar
				real_t A[PAR.Ar.rows()*PAR.Ar.cols()];
				for(int j=0; j < PAR.Ar.rows()*PAR.Ar.cols() ; j++){
					int liNum,colNum;
					liNum  = j/PAR.Ar.cols();
					colNum = j-j/PAR.Ar.cols()*PAR.Ar.cols();
					A[j] = PAR.Ar(liNum,colNum);
				}
				
				// Converting 'PAR.Fr' to qpOASES format Gr
				real_t G[np];
				for(int j=0 ; j < np ; j++){
					G[j] = PAR.Fr(j,0);
				}

				// Converting 'PAR.Br' to qpOASES format ubAr
				real_t ubA[PAR.Br.rows()];
				for(int j=0 ; j < PAR.Br.rows() ; j++){
					ubA[j] = PAR.Br(j,0);
				}				
				/*
				 * PARAMETRIZED CONVERTION END
				 * */			
							
				/*
				 * Solve QP 
				*/	
				int nWSR = 10;
				real_t cpu_time = 0.015;
				real_t xOpt[N*nu];				
				if(isFirstRun == 1){	
					//cout << "== Solve first QP == \n" << endl;					
					//attQP.init(H,G,A,lb,ub,0,ubA,nWSR,&cpu_time);
					attQP.init(H,G,A,0,0,0,ubA, nWSR, &cpu_time); // PARAMETRIZED
					// Get and print solution of first QP
					attQP.getPrimalSolution( xOpt );							
					isFirstRun = 0;
				} else {	
					//cout << "== Solve Subsequent QPs == \n" << endl;
					//attQP.hotstart(G,lb,ub,0,ubA,nWSR,&cpu_time);
					attQP.hotstart(G,0,0,0,ubA, nWSR, &cpu_time); // PARAMETRIZED
					// Get and print solution of QP
					attQP.getPrimalSolution( xOpt );				 	
				}	
				
				// Convert PARAMETRIZED Solution to Eigen format
				for(int j=0 ; j< np ; j++){
					pSol(j,0) = xOpt[j];
				}
				
				// Calculate the control sequence from PARAMETRIZED
				xSol = PAR.Pi_e*pSol;										
			
				// Separate first control action
				u = P_i(0,nu,MPC.N)*xSol;
				
				MPC.ulast = u;
				
				//Command mapping 
				roll_torque_cmd = u(0);
				pitch_torque_cmd = u(1);
				yaw_torque_cmd = u(2);
				
				roll_cmd  = map_torque(u(0),-MAX_TORQUE,MAX_TORQUE,-1.0, 1.0);
				pitch_cmd = map_torque(u(1),-MAX_TORQUE,MAX_TORQUE, 1.0,-1.0);
				yaw_cmd   = map_torque(u(2),-MAX_TORQUE,MAX_TORQUE, 1.0,-1.0);				
				
				/*
				 * Calculate throttle command (PID)
				 */		
				// Z tracking	
				
				/*
				z_err = z_sp - z;
				z_vel_sp = z_K_p*z_err + 
							z_K_d*(z_err - prev_z_err)/dt;
				prev_z_err = z_err; 	
				*/
				
				z_vel_sp = throttle_sp;
				
				// Z velocity tracking
				z_vel_err = (z_vel_sp - z_vel);
				sum_z_vel_err = sum_z_vel_err + z_vel_err*dt;
				throttle_cmd = z_vel_K_p*z_vel_err + 
								z_vel_K_i*sum_z_vel_err + 
									z_vel_K_d*(z_vel_err - prev_z_vel_err)/dt;
				prev_z_vel_err = z_vel_err;
				
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

				/*
				 * Control message alocation & publish
				 * */
				actuator_control_msg.header.stamp = ros::Time::now();
				actuator_control_msg.group_mix = 0;
				actuator_control_msg.controls[0] = roll_cmd;
				actuator_control_msg.controls[1] = pitch_cmd;
				actuator_control_msg.controls[2] = yaw_cmd;
				//actuator_control_msg.controls[3] = throttle_cmd;  // PID command
				actuator_control_msg.controls[3] = throttle_sp;		// Radio mapped command
								
				actuator_controls_pub.publish(actuator_control_msg);
				imu_msg_updated = 0;
				//cout << "Command Published!" << endl;					
			}			
		}
		/*
		 * Print data to output files 
		*/ 
		time_file << i << "\t" << dt << "\t" << node_run_time << "\n";
		attitude_file <<
			roll << "\t" << roll_rate << "\t" <<
			pitch << '\t' << pitch_rate << '\t' << 
		    yaw << '\t' << yaw_rate << "\n";
		    
		attitude_sp_file << 
			setpointVector(0,0) << "\t" << setpointVector(1,0)<< "\t" <<
			setpointVector(2,0) << "\t" << setpointVector(3,0)<< "\t" <<
			setpointVector(4,0) << "\t" << setpointVector(5,0)<<  "\n";
		
		prediction_file << pred_stateVector.transpose()/pi*180 << "\n";
		command_file << roll_torque_cmd << "\t" << pitch_torque_cmd<< "\t" << yaw_torque_cmd << "\n";
				 
		//Print to screen
		if(Time::now() - last_print > Duration(0.5)){ //&& mpc_switch == 1	
			cout << "Current Mode = " << current_state.mode << endl;
			cout << "MPC Switch = " << mpc_switch << endl;	
			cout << " \n " << endl;					
			cout << "ROLL[deg] = "  << roll/pi*180 << endl;
			//cout << "Roll Rate[/s] = "   << roll_rate << endl;
			cout << "PITCH[deg] = " << pitch/pi*180 << endl;
			//cout << "Pitch Rate[/s] = "   << pitch_rate << endl;
			cout << "YAW[deg] = "   << yaw/pi*180 << endl;
			//cout << "Yaw Rate[/s] = "   << yaw_rate << endl;
			//cout << "State Vector = "   << stateVector << endl;		
			//cout << " z = " << z << endl;		
			//cout << " z_vel = " << z_vel << endl;
			cout << "\n" << endl;	
			cout << "roll_sp = "  << roll_sp*180.0/pi << endl;
			cout << "pitch_sp = " << pitch_sp*180.0/pi << endl;
			cout << "yaw_sp = "   << yaw_sp*180.0/pi << endl;
			cout << "throttle_sp = "   << throttle_sp << endl;
			//cout << "roll_rate_sp = "  << roll_rate_sp << endl;
			//cout << "pitch_rate_sp = " << pitch_rate_sp << endl;
			//cout << "yaw_rate_sp = "   << yaw_rate_sp << endl;
			cout << "\n" << endl;	
			cout << " u = "  << u << endl;	
			cout << " roll_cmd = "  << roll_cmd << endl;
			cout << " pitch_cmd = "  << pitch_cmd << endl;
			cout << " yaw_cmd = "  << yaw_cmd << endl;
			//cout << " throttle_cmd = "  << throttle_cmd << endl;
			cout << " -------------------------------" << endl;
			//cout << " Cpu_time" << cpu_time << endl;
			//cout << " pSol = "  << pSol << endl;
			//cout << " xSol = "  << xSol << endl;			
			//cout << "PAR.Fr = " << PAR.Fr << endl;
			//cout << "PAR.Br = "   << PAR.Br << endl;
			//cout << "PAR.Hr = " << PAR.Hr << endl;
			//cout << "PAR.Pi_e = " << PAR.Pi_e << endl;			
			//cout << "MPC.Bineq = " << MPC.Bineq << endl;
			//cout << "MPC.F = " << MPC.F << endl;	
			//cout << " \n " << endl;		
			last_print = Time::now();	
		}
		
		//Euler Angles & its setpoint publish
		euler_angles.x = roll/pi*180;
		euler_angles.y = pitch/pi*180;
		euler_angles.z = yaw/pi*180;
		
		euler_angles_sp.x = roll_sp/pi*180;
		euler_angles_sp.y = pitch_sp/pi*180;
		euler_angles_sp.z = yaw_sp/pi*180;
		
		attitude_euler_pub.publish(euler_angles);
		attitude_euler_sp_pub.publish(euler_angles_sp);
		      
        spinOnce();
        loop_rate.sleep();  
        i++; //Loop count      
    }
    return 0; 
}

/* 
Subscriber Callback Functions
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
	dt = (Time::now() - last_imu_cb).toSec();
	last_imu_cb = Time::now();
	node_run_time = node_run_time + dt;
	
	//cout << "[IMU_CB] dt = " << dt << endl;
	//cout << "[IMU_CB] node_run_time = " << node_run_time << endl;
	
	imu_msg_updated = 1;
}

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	z = msg->pose.position.z;
	//cout << " Read z ..." << endl;
}

void velocity_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
	z_vel = msg->twist.linear.z;
	//cout << " Read z velocity..." << endl;
}

void rc_in_cb(const mavros_msgs::RCIn::ConstPtr& msg){
	float channel_1 = msg->channels[0];
	float channel_2 = msg->channels[1];
	float channel_3 = msg->channels[2];
	float channel_4 = msg->channels[3];
	
	throttle_sp = map_torque(channel_1, 980, 2000, -1,1);
	roll_sp     = map_torque(channel_2, 980, 2000,-1,1);	
	pitch_sp    = map_torque(channel_3, 980, 2000,-1,1);
	yaw_sp      = map_torque(channel_4, 980, 2000,1,-1);
	
	if(roll_sp < 0.01 && roll_sp > -0.01) roll_sp = 0.0;
	if(pitch_sp < 0.01 && pitch_sp > -0.01) pitch_sp = 0.0;
    if(yaw_sp < 0.01 && yaw_sp > -0.01) yaw_sp = 0.0;
    if(throttle_sp < 0.01 && throttle_sp > -0.01) throttle_sp = 0.0;
	
	setpointVector(0,0) = roll_sp;
	setpointVector(2,0) = pitch_sp;
	setpointVector(5,0) = yaw_sp; // velocity setpoint!
	
}



float map_torque(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
