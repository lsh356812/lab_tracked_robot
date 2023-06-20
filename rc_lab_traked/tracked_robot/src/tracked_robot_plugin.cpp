/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   tracked_robot_plugin.cpp
 * Author: njy
 *
 * Created on July 13, 2020, 4:39 PM
 */

// Header file for C++



#include <stdio.h>
#include <functional>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <boost/bind.hpp>

// Header file for Gazebo and Ros

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include "sensor_msgs/Joy.h"
/*
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <std_msgs/Float64.h>
#include <ignition/math/Vector3.hh>

#include "geometry_msgs/Twist.h"
*/

//JEon 211115
#if GAZEBO_GPU_RAY
#define RaySensor GpuRaySensor
#define STR_Gpu  "Gpu"
#define STR_GPU_ "GPU "
#else
#define STR_Gpu  ""
#define STR_GPU_ ""
#endif

#if GAZEBO_GPU_RAY
#include <gazebo/sensors/GpuRaySensor.hh>
#else
#include <gazebo/sensors/RaySensor.hh>
#endif
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/transport/Node.hh>
#include <sensor_msgs/PointCloud2.h>
//JEon 211115

#include <cmath>
double pi1=M_PI;
//#define PI      3.141592
#define D2R     M_PI/180.
#define R2D     180./M_PI
#include <ros/console.h>

/*to get world pose (ola) */
#include <ignition/math/Vector3.hh>


#include <ctime>
#include <stdlib.h>
#include <vector>
#include <string>
#include <random>


/* gas_capture */
#include <gas_capture/set_gas.h>
#include <gas_capture/get_gas.h>

#include <gas_capture/set_capture.h>
#include <gas_capture/get_capture.h>


#define GRID_SIZE 0.5 //m

/*기체농도 최대값*/
#define DENSITY_MAX 100
#define DENSITY_MIN 0
#define DENSITY_MIN_THRESHOLD 3
#define DIFFUSE_RATIO 0.9

#define RANGE_MAX 101
#define X_ 0
#define Y_ 1

double SourcePoint[2] = {4.0,4.0}; //x, y

int x_begin = RANGE_MAX/2;
int x_end = RANGE_MAX/2;
int y_begin = RANGE_MAX/2;
int y_end = RANGE_MAX/2;
double density = DENSITY_MAX;

time_t timer;
struct tm* t; 
int tick_count = 0;
bool RtoG_flag = false;

std::default_random_engine generator;
std::normal_distribution<double> distribution(0.0,0.0);

int get_flow_intial;

double noise_param[32][2] = {{1.0,0.5},
                             {1.0,0.5},
                             {1.0,0.5},
                             {1.0,0.5},
                             {1.0,0.5},
                             {1.0,0.5},
                             {1.0,0.5},
                             {1.0,0.5},
                             {1.0,0.5},
                             {1.0,0.5},
                             {1.0,0.5},
                             {1.0,0.5},
                             {1.0,0.5},
                             {1.0,0.5},
                             {1.0,0.5},
                             {1.0,0.5},
                             {1.0,0.5},
                             {1.0,0.5},
                             {1.0,0.5},
                             {1.0,0.5},
                             {1.0,0.5},
                             {1.0,0.5},
                             {1.0,0.5},
                             {1.0,0.5},
                             {1.0,0.5},
                             {1.0,0.5},
                             {1.0,0.5},
                             {1.0,0.5},
                             {1.0,0.5},
                             {1.0,0.5},
                             {1.0,0.5},
                             {1.0,0.5}};

/* Capture data */

int16_t n_saved_data;
int16_t i_saved_data;
int16_t       get_line[100] = {0};
uint64_t get_MSMT_time[100] = {0};
int16_t       get_flow[100] = {0};
int16_t   get_cap_time[100] = {0};
float   get_integ_flow[100] = {0};

//


void update_time_(uint64_t &Date_, uint64_t &Time_now_){
    timer = time(NULL);
    t = localtime(&timer);

    int year = t->tm_year + 1900;
    int month = t->tm_mon+1;
    int day = t->tm_mday;
    int hour = t->tm_hour;
    int min = t->tm_min;
    int sec = t->tm_sec;
    Date_ = year*10000 + month*100 + hour;
    Time_now_ = hour*10000 + min*100 + sec;
}
void update_time_(uint64_t &Time_now_){
    timer = time(NULL);
    t = localtime(&timer);

    int hour = t->tm_hour;
    int min = t->tm_min;
    int sec = t->tm_sec;
    Time_now_ = hour*10000 + min*100 + sec;
}


namespace gazebo{
    class Tracked_Robot : public ModelPlugin{
        physics::LinkPtr Body;
        physics::LinkPtr Track_L_1;
        physics::LinkPtr Track_L_2;
        physics::LinkPtr Track_L_3;
        physics::LinkPtr Track_L_4;
        physics::LinkPtr Track_L_5;
        physics::LinkPtr Track_L_6;
        physics::LinkPtr Track_L_7;
        physics::LinkPtr Track_L_8;
        physics::LinkPtr Track_L_9;
        
        physics::LinkPtr Track_R_1;
        physics::LinkPtr Track_R_2;
        physics::LinkPtr Track_R_3;
        physics::LinkPtr Track_R_4;
        physics::LinkPtr Track_R_5;
        physics::LinkPtr Track_R_6;
        physics::LinkPtr Track_R_7;
        physics::LinkPtr Track_R_8;
        physics::LinkPtr Track_R_9;
        
        physics::LinkPtr Flipper_L;
        physics::LinkPtr Flipper_R;
        
        physics::LinkPtr Flipper_L_1;
        physics::LinkPtr Flipper_L_2;
        physics::LinkPtr Flipper_L_3;
        
        physics::LinkPtr Flipper_R_1;
        physics::LinkPtr Flipper_R_2;
        physics::LinkPtr Flipper_R_3;
        

        physics::JointPtr joint_Track_L_1;
        physics::JointPtr joint_Track_L_2;
        physics::JointPtr joint_Track_L_3;
        physics::JointPtr joint_Track_L_4;
        physics::JointPtr joint_Track_L_5;
        physics::JointPtr joint_Track_L_6;
        physics::JointPtr joint_Track_L_7;
        physics::JointPtr joint_Track_L_8;
        physics::JointPtr joint_Track_L_9;
        
        physics::JointPtr joint_Track_R_1;
        physics::JointPtr joint_Track_R_2;
        physics::JointPtr joint_Track_R_3;
        physics::JointPtr joint_Track_R_4;
        physics::JointPtr joint_Track_R_5;
        physics::JointPtr joint_Track_R_6;
        physics::JointPtr joint_Track_R_7;
        physics::JointPtr joint_Track_R_8;
        physics::JointPtr joint_Track_R_9;
        
        physics::JointPtr Joint_L;
        physics::JointPtr Joint_R;
        
        physics::JointPtr joint_Flipper_L_1;
        physics::JointPtr joint_Flipper_L_2;
        physics::JointPtr joint_Flipper_L_3;
        
        physics::JointPtr joint_Flipper_R_1;
        physics::JointPtr joint_Flipper_R_2;
        physics::JointPtr joint_Flipper_R_3;    
        
        physics::ModelPtr model;
        
        common::PID pidL;
        common::PID pidR;
        
        double trans = 0;
        //double delta_trans = 0;
        double yaw = 0;
        //double delta_yaw = 0;
        double vel_L = 0;
        double vel_R = 0;
        double flipper = 0;
        
        double angle_err_L = 0;
        double angle_err_R = 0;
        double tar_angle_L = 0;
        double tar_angle_R = 0;
        
        
        
        //ros topic 통신
        ros::NodeHandle nh;
        ros::Subscriber S_move;
        ros::Publisher density_pub;

        ros::Subscriber RtoG_Sub; //Remote controller to Gas sensor State
        ros::Publisher GtoR_Pub;  //Gas sensor to Remote controller State

        ros::Subscriber RtoC_sub;  //Remote controller to Capture device State
        ros::Publisher CtoR_Pub;   //Capture device to Remote controller State


        // ros message
        std_msgs::Float32 density_msg;

        gas_capture::set_gas S_Msg;
        gas_capture::get_gas G_Msg;

        gas_capture::set_capture setC_Msg;
        gas_capture::get_capture getC_Msg;

        
        //setting for getting <dt>(=derivative time) 
        common::Time last_update_time;
        event::ConnectionPtr update_connection;
        double dt;
        double time = 0;


        double x,y,z;
        ignition::math::Pose3d pose;


        double grid_area[RANGE_MAX][RANGE_MAX][1] = {0}; //x,y,z  1 = GRID_SIZE (m) X  GRID_SIZE (m) X  GRID_SIZE (m)

        
        int peak_point[1][3]={{1,1,1}};
        
        ///double density = DENSITY_MAX;

        
    public:
        void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/); //로드함수
        void UpdateAlgorithm(); //주기적 반복 (1ms)
        void movecallback(const sensor_msgs::Joy::ConstPtr& msg)
        {
            trans = msg->axes[1]*10;
//            yaw = msg->axes[0]*5 + msg->axes[3]*10;
            yaw = msg->axes[0]*5;
            flipper = msg->axes[4] * (M_PI/4);
            if (flipper >= M_PI/4)
            {
                flipper = M_PI/4;
            }
            if (flipper <= -M_PI/4 )
            {
                flipper = -M_PI/4;
            }
        }
        void RtoG_Cmd(const gas_capture::set_gas::ConstPtr& RMsg);
        void RtoC_Cmd(const gas_capture::set_capture::ConstPtr& RMsg);
        void Set_get_capture_msg_zero(gas_capture::get_capture &getC_msg);
        int random_int(int min, int max);
    };
    GZ_REGISTER_MODEL_PLUGIN(Tracked_Robot);
}


void gazebo::Tracked_Robot::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{

    ROS_INFO("tracked robot load function");
    this->model = _model;

    this->Body = this->model->GetLink("Body");
    this->Track_L_1 = this->model->GetLink("Track_L_1");
    this->Track_L_2 = this->model->GetLink("Track_L_2");
    this->Track_L_3 = this->model->GetLink("Track_L_3");
    this->Track_L_4 = this->model->GetLink("Track_L_4");
    this->Track_L_5 = this->model->GetLink("Track_L_5");
    this->Track_L_6 = this->model->GetLink("Track_L_6");
    this->Track_L_7 = this->model->GetLink("Track_L_7");
    this->Track_L_8 = this->model->GetLink("Track_L_8");
    this->Track_L_9 = this->model->GetLink("Track_L_9");
    this->Track_R_1 = this->model->GetLink("Track_R_1");
    this->Track_R_2 = this->model->GetLink("Track_R_2");
    this->Track_R_3 = this->model->GetLink("Track_R_3");
    this->Track_R_4 = this->model->GetLink("Track_R_4");
    this->Track_R_5 = this->model->GetLink("Track_R_5");
    this->Track_R_6 = this->model->GetLink("Track_R_6");
    this->Track_R_7 = this->model->GetLink("Track_R_7");
    this->Track_R_8 = this->model->GetLink("Track_R_8");
    this->Track_R_9 = this->model->GetLink("Track_R_9");
    
    this->Flipper_L = this->model->GetLink("Flipper_L");
    this->Flipper_R = this->model->GetLink("Flipper_R");
    this->Flipper_L_1 = this->model->GetLink("Flipper_L_1");
    this->Flipper_L_2 = this->model->GetLink("Flipper_L_2");
    this->Flipper_L_3 = this->model->GetLink("Flipper_L_3");
    this->Flipper_R_1 = this->model->GetLink("Flipper_R_1");
    this->Flipper_R_2 = this->model->GetLink("Flipper_R_2");
    this->Flipper_R_3 = this->model->GetLink("Flipper_R_3");
    
    this->joint_Track_L_1 = this->model->GetJoint("joint_Track_L_1");
    this->joint_Track_L_2 = this->model->GetJoint("joint_Track_L_2");
    this->joint_Track_L_3 = this->model->GetJoint("joint_Track_L_3");
    this->joint_Track_L_4 = this->model->GetJoint("joint_Track_L_4");
    this->joint_Track_L_5 = this->model->GetJoint("joint_Track_L_5");
    this->joint_Track_L_6 = this->model->GetJoint("joint_Track_L_6");
    this->joint_Track_L_7 = this->model->GetJoint("joint_Track_L_7");
    this->joint_Track_L_8 = this->model->GetJoint("joint_Track_L_8");
    this->joint_Track_L_9 = this->model->GetJoint("joint_Track_L_9");
    this->joint_Track_R_1 = this->model->GetJoint("joint_Track_R_1");
    this->joint_Track_R_2 = this->model->GetJoint("joint_Track_R_2");
    this->joint_Track_R_3 = this->model->GetJoint("joint_Track_R_3");
    this->joint_Track_R_4 = this->model->GetJoint("joint_Track_R_4");
    this->joint_Track_R_5 = this->model->GetJoint("joint_Track_R_5");
    this->joint_Track_R_6 = this->model->GetJoint("joint_Track_R_6");
    this->joint_Track_R_7 = this->model->GetJoint("joint_Track_R_7");
    this->joint_Track_R_8 = this->model->GetJoint("joint_Track_R_8");
    this->joint_Track_R_9 = this->model->GetJoint("joint_Track_R_9");
    
    this->Joint_L = this->model->GetJoint("Joint_L");
    this->Joint_R = this->model->GetJoint("Joint_R");
    
    this->joint_Flipper_L_1 = this->model->GetJoint("joint_Flipper_L_1");
    this->joint_Flipper_L_2 = this->model->GetJoint("joint_Flipper_L_2");
    this->joint_Flipper_L_3 = this->model->GetJoint("joint_Flipper_L_3");
    this->joint_Flipper_R_1 = this->model->GetJoint("joint_Flipper_R_1");
    this->joint_Flipper_R_2 = this->model->GetJoint("joint_Flipper_R_2");
    this->joint_Flipper_R_3 = this->model->GetJoint("joint_Flipper_R_3");
    
    this->pidL.Init(0.5, 0, 0.01, 200, -200, 1000, -1000);
    this->pidR.Init(0.5, 0, 0.01, 200, -200, 1000, -1000);

    density_pub = nh.advertise<std_msgs::Float32>("/density", 1000);
    GtoR_Pub = nh.advertise<gas_capture::get_gas>("GtoR_State", 1000);
    CtoR_Pub = nh.advertise<gas_capture::get_capture>("CtoR_State", 1000);
    RtoG_Sub = nh.subscribe("RtoG_Cmd", 1000, &gazebo::Tracked_Robot::RtoG_Cmd,this);

    Set_get_capture_msg_zero(getC_Msg);

    /*  Triangle
    for(int i=0;;i++){
        if(x_begin <= 1)
            break;
        std::cout<<"x_begin : "<<x_begin<<std::endl;
        for(int x = x_begin; x<=x_end; x++){
            grid_area[x][y_begin][0] = grid_area[x][y_end][0] = density;
        }
        for(int y = y_begin;y<=y_end; y++){
            grid_area[x_begin][y][0] = grid_area[x_end][y][0] = density;
        }
        x_begin--; y_begin--;
        x_end++; y_end++;
        density *= DIFFUSE_RATIO;
    }
    */

    //circle 1-cos
    for(int x=0;x<RANGE_MAX;x++){
      for(int y=0;y<RANGE_MAX;y++){
        double r = sqrt((50-abs(x))*(50-abs(x)) + (50-abs(y))*(50-abs(y)));
        if(r<0) r = 0;
        if(r>50) r = 50;
        density = 100.0 - 100.0*0.5*(1.0-cos(3.141593*(r/50)));
        grid_area[x][y][0] = density;
      }
    }


/*
    for(int x = 0; x<RANGE_MAX;x++){
        for(int y=0;y<RANGE_MAX;y++){
            std::cout<<grid_area[x][y][0]<<" ";
        }
        std::cout<<std::endl;
    }
*/
    
    //get_flow_initialize
    get_flow_intial = random_int(40 , 60);

    this->last_update_time = this->model->GetWorld()->SimTime();
    this->update_connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&Tracked_Robot::UpdateAlgorithm, this)); // Execute Gazebo Algorithm
    
    S_move = nh.subscribe<sensor_msgs::Joy>("joy", 100, &gazebo::Tracked_Robot::movecallback, this);
    //S_move = nh.subscribe<sensor_msgs::Joy>("joyFromController", 100, &gazebo::Tracked_Robot::movecallback, this);


}

void gazebo::Tracked_Robot::UpdateAlgorithm()
{
        //* Calculate time  
    //ROS_INFO("aaa");
    common::Time current_time = this->model->GetWorld()->SimTime();
    dt = current_time.Double() - this->last_update_time.Double();
    time = time + dt;
    printf("%lf\n",dt);
    
    vel_L = trans - yaw;
    vel_R = trans + yaw;
    
    this->joint_Track_L_1->SetVelocity(0,vel_L);
    this->joint_Track_L_2->SetVelocity(0,vel_L);
    this->joint_Track_L_3->SetVelocity(0,vel_L);
    this->joint_Track_L_4->SetVelocity(0,vel_L);
    this->joint_Track_L_5->SetVelocity(0,vel_L);
    this->joint_Track_L_6->SetVelocity(0,vel_L);
    this->joint_Track_L_7->SetVelocity(0,vel_L);
    this->joint_Track_L_8->SetVelocity(0,vel_L);
    this->joint_Track_L_9->SetVelocity(0,vel_L);
    this->joint_Flipper_L_1->SetVelocity(0,vel_L);
    this->joint_Flipper_L_2->SetVelocity(0,vel_L);
    this->joint_Flipper_L_3->SetVelocity(0,vel_L);
    
    this->joint_Track_R_1->SetVelocity(0,vel_R);
    this->joint_Track_R_2->SetVelocity(0,vel_R);
    this->joint_Track_R_3->SetVelocity(0,vel_R);
    this->joint_Track_R_4->SetVelocity(0,vel_R);
    this->joint_Track_R_5->SetVelocity(0,vel_R);
    this->joint_Track_R_6->SetVelocity(0,vel_R);
    this->joint_Track_R_7->SetVelocity(0,vel_R);
    this->joint_Track_R_8->SetVelocity(0,vel_R);
    this->joint_Track_R_9->SetVelocity(0,vel_R);
    this->joint_Flipper_R_1->SetVelocity(0,vel_R);
    this->joint_Flipper_R_2->SetVelocity(0,vel_R);
    this->joint_Flipper_R_3->SetVelocity(0,vel_R);
    
    tar_angle_L = -flipper;
    tar_angle_R = flipper;
    
    //angle error
    angle_err_L = this->Joint_L->Position(1) - tar_angle_L;
    angle_err_R = this->Joint_R->Position(1) - tar_angle_R;
    
    //std::cout << "err_L: " << std::endl << angle_err_L << std::endl;
    //std::cout << "err_R: " << std::endl << angle_err_R << std::endl;
    
    this->pidL.Update(angle_err_L, dt);
    this->pidR.Update(angle_err_R, dt);
    
    this->Joint_L->SetForce(1, this->pidL.GetCmd());
    this->Joint_R->SetForce(1, this->pidR.GetCmd());
    
    //this->joint_Track_L_3->SetForce(1,1);
    //this->joint_Track_R_3->SetForce(1,-1);



    /* world pose */
    this->pose = this->model->WorldPose();
    this->x = this->pose.Pos().X();
    this->y = this->pose.Pos().Y();
    this->z = this->pose.Pos().Z();

    std::cout<<"Source Point(X,Y) : ("<<SourcePoint[X_]<<", "<<SourcePoint[Y_]<<")"<<std::endl;
    std::cout<<"Position(X,Y) : ("<<x<<", "<<y<<")"<<std::endl;

    //std::cout<<"pose z:"<<this->z<<std::endl;
    std::cout<<std::endl;

    int Grid_X = (int)((RANGE_MAX/2) + (x - SourcePoint[X_])/GRID_SIZE);
    int Grid_Y = (int)((RANGE_MAX/2) + (y - SourcePoint[Y_])/GRID_SIZE);

    double density_ = grid_area[Grid_X][Grid_Y][0];

    density_msg.data = density_;
    density_pub.publish(density_msg);

    std::cout<<"density : "<<density_<<std::endl;

    /* get current time */
    uint64_t date, time_now;
    update_time_(date, time_now);
/*
    std::string year = std::to_string(t->tm_year + 1900);
    std::string month = std::to_string(t->tm_mon+1);
    std::string day = std::to_string(t->tm_mday);
    std::string hour = std::to_string(t->tm_hour);
    std::string minute = std::to_string(t->tm_min);
    std::string sec = std::to_string(t->tm_sec);


    if(month.length()<2) month = "0"+month;
    if(day.length()<2) day = "0"+day;
    if(hour.length()<2) hour = "0"+hour;
    if(minute.length()<2) minute = "0"+minute;
    if(sec.length()<2) sec = "0"+sec;

    std::string date = year+'-'+month+'-'+day;
    std::string time_now = hour+':'+minute+':'+sec;
    std::cout<<date<<std::endl;
    std::cout<<time_now<<std::endl;
*/


    /*get gas publish */
    G_Msg.HEADER.stamp = ros::Time::now();
    G_Msg.NAME = "Gas State";
    G_Msg.NODE_ID = 3;
    G_Msg.DATE = date;
    G_Msg.TIME = time_now;
    for(int i=0; i<32; i++){
        distribution.param(std::normal_distribution<double>::param_type(noise_param[i][0]+density_,noise_param[i][1]));
        double density_dist = distribution(generator);
        density_dist = (density_dist<0.0)?0.0:density_dist;
        density_dist = (density_dist>100.0)?100.0:density_dist;
        G_Msg.ADC_DATA[i] = density_dist;
    }

    getC_Msg.HEADER.stamp = ros::Time::now();
    getC_Msg.NAME = "Capture State";
    getC_Msg.GET_STATE = 1; //
    getC_Msg.N_SAVED_DATA = 1; //n_saved_data;
    getC_Msg.I_SAVED_DATA = 0;//i_saved_data;
    getC_Msg.GET_LINE[0] = 1;//get_line;
    update_time_(time_now);
    getC_Msg.GET_MSMT_TIME[0] = time_now;
    getC_Msg.GET_MSMT_TIME[1] = time_now-11;
    getC_Msg.GET_MSMT_TIME[2] = time_now-222;
    getC_Msg.GET_MSMT_TIME[3] = time_now-3333;

    getC_Msg.GET_FLOW[0] = get_flow_intial; //use random number
    getC_Msg.GET_CAP_TIME[0] = 10; //means nothing for now
    getC_Msg.GET_INTEG_FLOW[0] = 10; //means nothing for now
//
    if(tick_count>=1000 or RtoG_flag){
        GtoR_Pub.publish(G_Msg); 
        CtoR_Pub.publish(getC_Msg);

        int add_random_int;
        add_random_int = random_int(-5.0, 5.0);
        get_flow_intial = ((get_flow_intial + add_random_int)>100 or (get_flow_intial + add_random_int)<0) ? get_flow_intial-add_random_int : get_flow_intial+add_random_int;

        tick_count = 0;
    }

    
    this->last_update_time = current_time;

    tick_count++;
}

// Subscriber Callback Function //
void gazebo::Tracked_Robot::RtoG_Cmd(const gas_capture::set_gas::ConstPtr& RMsg)
{
    GtoR_Pub.publish(G_Msg); 
}

void gazebo::Tracked_Robot::RtoC_Cmd(const gas_capture::set_capture::ConstPtr& RMsg)
{
  /*
  S_Msg.SET_COMMAND = RMsg->SET_COMMAND;

  S_Msg.SET_LINE = RMsg->SET_LINE;
  S_Msg.SET_DISCHARGE = RMsg->SET_DISCHARGE;
  S_Msg.SET_CAP_TIME = RMsg->SET_CAP_TIME;

  */

  int command = 0;//RMsg.SET_COMMAND;

  if(command == 1){}

  getC_Msg.N_SAVED_DATA ++;
  getC_Msg.I_SAVED_DATA ++;

  int index = getC_Msg.I_SAVED_DATA;
  index = (index>=100)?99:index;

  uint64_t time_now;
  update_time_(time_now);

  getC_Msg.GET_LINE[index] = RMsg->SET_LINE;
  getC_Msg.GET_MSMT_TIME[index] = time_now;
  getC_Msg.GET_FLOW[index] = 0;
  getC_Msg.GET_CAP_TIME[index] = RMsg->SET_CAP_TIME;
  getC_Msg.GET_INTEG_FLOW[index] = 0;

  //CtoR_Pub.publish(getC_Msg);
}

void gazebo::Tracked_Robot::Set_get_capture_msg_zero(gas_capture::get_capture &getC_msg){

  getC_Msg.HEADER.stamp = ros::Time::now();
  getC_Msg.NAME = "Gas State";
  //getC_msg.NODE_ID = 3;

  getC_Msg.N_SAVED_DATA = 0;
  getC_Msg.I_SAVED_DATA = -1;

  for(int i=0;i<100;i++){
    getC_Msg.GET_LINE[i] = 0;
    getC_Msg.GET_MSMT_TIME[i] = 0;
    getC_Msg.GET_FLOW[i] = 0;
    getC_Msg.GET_CAP_TIME[i] = 0;
    getC_Msg.GET_INTEG_FLOW[i] = 0;
  }
}

int gazebo::Tracked_Robot::random_int(int min, int max){
  return min + (rand()%(max-min));
}
