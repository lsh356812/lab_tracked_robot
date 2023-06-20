/*

Kubot package simulation code

kudos edu.ver

23.03.16

*/

//* Header file for C++
#include <stdio.h>
#include <iostream>
#include <boost/bind.hpp>
#include <time.h>

//* Header file for Gazebo and Ros
#include <gazebo/gazebo.hh>
#include <ros/ros.h>

#include <gazebo/common/common.hh> 
#include <gazebo/common/Plugin.hh> //model plugin gazebo API에서 확인 
#include <gazebo/physics/physics.hh> //우리는 ODE physics 사용 
#include <gazebo/sensors/sensors.hh> //IMU sensor 사용 

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h> //topic을 어떤 형태로 보내는지 

#include <beginner_kubot_pkgs/angle_msgs.h>

#include <functional>
#include <ignition/math/Vector3.hh>

#include <Eigen/Dense>

// KUDOS CKubot Header 파일 선언
#include "kudos/CKubot.h"
// #include "kudos/kinematics.h"


//* Print color
#define C_BLACK   "\033[30m"
#define C_RED     "\x1b[91m"
#define C_GREEN   "\x1b[92m"
#define C_YELLOW  "\x1b[93m"
#define C_BLUE    "\x1b[94m"
#define C_MAGENTA "\x1b[95m"
#define C_CYAN    "\x1b[96m"
#define C_RESET   "\x1b[0m"


using namespace std;
//여기까지가 책으로 따지자면 목차. namespace까지 

//Eigen//
using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::MatrixXf;
using Eigen::VectorXf;

using Eigen::Matrix2f;
using Eigen::Matrix3f;
using Eigen::Matrix4f;

using Eigen::Vector2f;
using Eigen::Vector3f;
using Eigen::Vector4f;
//Eigen 파일 안에 있는 namespace를 사용하겠다.
VectorXd test_vector_1(12);

namespace gazebo {

    class B_kubot_plugin : public ModelPlugin
    {
        //*** Variables for Kubot Simulation in Gazebo ***//
        //* TIME variable
        common::Time last_update_time;
        event::ConnectionPtr update_connection;
        double dt;
        double time = 0;

        physics::ModelPtr model; // model = _model 관련

        physics::JointPtr L_Hip_yaw_joint;
        physics::JointPtr L_Hip_roll_joint;
        physics::JointPtr L_Hip_pitch_joint;
        physics::JointPtr L_Knee_joint;
        physics::JointPtr L_Ankle_pitch_joint;
        physics::JointPtr L_Ankle_roll_joint;

        physics::JointPtr R_Hip_yaw_joint;
        physics::JointPtr R_Hip_roll_joint;
        physics::JointPtr R_Hip_pitch_joint;
        physics::JointPtr R_Knee_joint;
        physics::JointPtr R_Ankle_pitch_joint;
        physics::JointPtr R_Ankle_roll_joint;
        //* Index setting for each joint
        // physics::JointPtr L_Foot_joint;
        // physics::JointPtr R_Foot_joint;


        //* ROS Subscribe
        ros::NodeHandle nh; // ros 통신을 위한 node handle 선언 
        ros::Subscriber KubotModesp;
        int ControlMode_by_ROS = 1;

        //ZMP
        std_msgs::Float64 m_ZMP_x;
        std_msgs::Float64 m_ZMP_y;
        std_msgs::Float64 m_ZMPcontrol_x;
        std_msgs::Float64 m_ZMPcontrol_y;
        std_msgs::Float64 m_ZMP_x_l_margin;
        std_msgs::Float64 m_ZMP_x_u_margin;
        std_msgs::Float64 m_ZMP_y_l_margin;
        std_msgs::Float64 m_ZMP_y_u_margin;

        std_msgs::Float64 m_Base_refpos_x;
        std_msgs::Float64 m_Base_refpos_y;
        std_msgs::Float64 m_Base_refpos_z;
        
        //Preview
        std_msgs::Float64 m_preview_ref_ZMP_x;
        std_msgs::Float64 m_preview_ref_ZMP_y;
        std_msgs::Float64 m_preview_COM_x;
        std_msgs::Float64 m_preview_COM_y;

        std_msgs::Float64 m_preview_FK_y;    /// 박사님 과제 FK

        
        //* ROS Publish
        ros::Publisher KubotModesp_pub;
        // ros::Publisher LHY_pub;
        // ros::Publisher LHR_pub;
        // ros::Publisher LHP_pub;
        // ros::Publisher LKN_pub;
        // ros::Publisher LAP_pub;
        // ros::Publisher LAR_pub;

        ros::Publisher COM_x;
        ros::Publisher COM_y;
        ros::Publisher COM_z;

        // rqt (Walking)
        ros::Publisher P_ZMP_x;
        ros::Publisher P_ZMP_y;
        ros::Publisher P_ZMPcontrol_x;
        ros::Publisher P_ZMPcontrol_y;
        ros::Publisher P_preview_ref_ZMP_x;
        ros::Publisher P_preview_ref_ZMP_y;
        ros::Publisher P_preview_COM_x;
        ros::Publisher P_preview_COM_y;

        ros::Publisher P_preview_FK_y;       /// 박사님 과제 FK

        ros::Publisher P_Base_refpos_x;
        ros::Publisher P_Base_refpos_y;
        ros::Publisher P_Base_refpos_z;
        ros::Publisher angle_pub;

        std_msgs::Float64 KubotModesp_msg;     
        // std_msgs::Float64 LHY_msg;
        // std_msgs::Float64 LHR_msg;
        // std_msgs::Float64 LHP_msg;
        // std_msgs::Float64 LKN_msg;
        // std_msgs::Float64 LAP_msg;
        // std_msgs::Float64 LAR_msg;



        enum
        { //wst는 torso joint 같음
            LHY = 0, LHR, LHP, LKN, LAP, LAR, RHY, RHR, RHP, RKN, RAP, RAR
        };

        //* Joint Variables
        int nDoF; // Total degrees of freedom, except position and orientation of the robot
//구조체는 변수 집합이라고 생각하면 편하다.

        // int Q0 = Kubot.Q0;
        // int Qi = Kubot.Qi;
        // int Qj = Kubot.Qj;
        // int Qk = Kubot.Qk;


        typedef struct RobotJoint //Joint variable struct for joint control 
        {
            double targetDegree; //The target deg, [deg]
            double targetRadian; //The target rad, [rad]
            double init_targetradian;

            double targetRadian_interpolation; //The interpolated target rad, [rad]

            double targetVelocity; //The target vel, [rad/s]
            double targetTorque; //The target torque, [N·m]

            double actualDegree; //The actual deg, [deg]
            double actualRadian; //The actual rad, [rad]
            double actualVelocity; //The actual vel, [rad/s]
            double actualRPM; //The actual rpm of input stage, [rpm]
            double actualTorque; //The actual torque, [N·m]

            double Kp;
            double Ki;
            double Kd;

        } ROBO_JOINT;
        ROBO_JOINT* joint;       

        bool joint_by_algorithm_update = false;
         

        //* Variables for IMU sensor
        sensors::SensorPtr Sensor;
        sensors::ImuSensorPtr IMU; //imu 센서 추가

        CKubot Kubot; // Kubot이라는 이름을 가진 CKubot class 선언함과 동시에 변수들을 다 갖고옴


        //* ROS Subscribe
        // ros::NodeHandle nh;
        // ros::Subscriber RoS_Mode;
        // ros::Subscriber Save_Mode;
        // int ControlMode_by_ROS = 0;
        // bool Balancing_Control_Mode_by_ROS = 0;

    public :
            //*** Functions for Kubot Simulation in Gazebo ***//
        void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/); // Loading model data and initializing the system before simulation 
        void UpdateAlgorithm(); // Algorithm update while simulation

        void jointUpdateByAlgorithm(); // joint ref update by control algorithm
        
        void setjoints(); // Get each joint data from [physics::ModelPtr _model]    
        void getjointdata(); // Get encoder data of each joint
        
        void setsensor();
        void getsensordata();

        void jointcontroller();


        void initializejoint(); // 추가한
        void setjointPIDgain(); // 함수

        //*ROS subscribe
        // void ros_MODE(const std_msgs::Int32 &msg);          
        void KubotMode(const std_msgs::Int32 &msg);
        
    };
    GZ_REGISTER_MODEL_PLUGIN(B_kubot_plugin);
    
}



//model.sdf파일에 대한 정보를 불러오는 함수
void gazebo::B_kubot_plugin::Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
{
    //Kubot.ControlMode = CTRLMODE_WALKREADY; // wave 함수 초기

    // Kubot.Move_current = false;
    printf("\nLoad_do\n");
    KubotModesp = nh.subscribe("/KubotMode",1,&gazebo::B_kubot_plugin::KubotMode,this);

    //* ROS Subscribe
    // Save_Mode = nh.subscribe("save_step", 1, &gazebo::B_kubot_plugin::data_save_Execution, this);
    // RoS_Mode = nh.subscribe("rok_mode", 1, &gazebo::B_kubot_plugin::ros_MODE, this);
    
    //* model.sdf file based model data input to [physics::ModelPtr model] for gazebo simulation
    model = _model;

    setjoints();
    //sdf변환하기 전에 urdf파일에서 조인트가 몇개인지 인식을 함.
    //인식이 끝나면 joint 갯수에 맞게 구조체를 여러개 생성
    //Robotjoint 구조체를 joint 갯수만큼 복제되는 거임
    setsensor();


    nDoF = 12; // Get degrees of freedom, except position and orientation of the robot
    //우리가 만든 joint는 12개로 지정
    joint = new ROBO_JOINT[nDoF]; // Generation joint variables struct    
    
    initializejoint();
    setjointPIDgain();


    Kubot.initializeBkubot();
    
    Kubot.setWalkingReadyPos(0,0,0.34);


        //ROS Publishers
    KubotModesp_pub = nh.advertise<std_msgs::Float64>("command/KubotMode", 1000);
    P_preview_COM_y = nh.advertise<std_msgs::Float64>("COM_y", 1000);
    P_preview_ref_ZMP_y = nh.advertise<std_msgs::Float64>("zmp_ref_y", 1000);
    P_ZMP_y = nh.advertise<std_msgs::Float64>("zmp_Y", 1000);
    P_preview_FK_y = nh.advertise<std_msgs::Float64>("FK_Y", 1000); 
    angle_pub = nh.advertise<beginner_kubot_pkgs::angle_msgs>("angle_pub_ku", 1000);
           
    // LHY_pub = nh.advertise<std_msgs::Float64>("command_joint/LHY", 1000);//왼발 x좌표 ref값
    // LHR_pub = nh.advertise<std_msgs::Float64>("command_joint/LHR", 1000);//왼발 y좌표 ref값
    // LHP_pub = nh.advertise<std_msgs::Float64>("command_joint/LHP", 1000);
    // LKN_pub = nh.advertise<std_msgs::Float64>("command_joint/LKN", 1000);
    // LAP_pub = nh.advertise<std_msgs::Float64>("command_joint/LAP", 1000);
    // LAR_pub = nh.advertise<std_msgs::Float64>("command_joint/LHR", 1000);
    //* setting for getting dt
    last_update_time = model->GetWorld()->SimTime();
    update_connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&B_kubot_plugin::UpdateAlgorithm, this));

}

void gazebo::B_kubot_plugin::KubotMode(const std_msgs::Int32 &msg)
{
    // ROS_INFO("I heard: [%d]",msg.data);

    ControlMode_by_ROS = msg.data;
    

    switch (ControlMode_by_ROS) {

    case 1:
        Kubot.ControlMode = CTRLMODE_HOMEPOSE;
        Kubot.CommandFlag = RETURN_HOMEPOSE;
        printf("Kubot home pose! \n");
        
        
        // Kubot.wave_cnt = 0;
        // Kubot.wave_run = 1;
        Kubot.Move_current = false;
        ControlMode_by_ROS = 0;
        

        break;
    

    case 2:
        Kubot.ControlMode = CTRLMODE_WALKREADY;
        Kubot.CommandFlag = GOTO_WALK_READY_POS;
        printf("WALK READY START! \n");
        


        // Kubot.wave_cnt = 0;
        // Kubot.wave_run = 1;
        Kubot.Move_current = false;
        ControlMode_by_ROS = 0;

        break;

    case 3:
        Kubot.ControlMode = CTRLMODE_UPDOWN;
        Kubot.CommandFlag = UPDOWN;
        printf("Up Down START! \n");


        Kubot.updown_cnt = 0;


        // Kubot.wave_cnt = 0;
        // Kubot.wave_run = 1;
        Kubot.Move_current = true;
        ControlMode_by_ROS = 0;

        break;

    case 4:
        Kubot.ControlMode = CTRLMODE_SWAY;
        Kubot.CommandFlag = SWAYMOTION_PATTERN;
        printf("SWAY START! \n");
        Kubot.zmp.previewControl.count = 0;
        Kubot.sway();
        

        Kubot.updown_cnt = 0;


        // Kubot.wave_cnt = 0;
        // Kubot.wave_run = 1;
        Kubot.Move_current = true;
        ControlMode_by_ROS = 0;

        Kubot.SWAY_READY = true;
        Kubot.SWAY_ING = false;

        break;
        
    case 5:
        Kubot.ControlMode = CTRLMODE_LRMOTION;
        Kubot.CommandFlag = LRMOTION;
        printf("LEGMOTION START! \n");
        Kubot.zmp.previewControl.count = 0;
        Kubot.legMotion();
        

        Kubot.updown_cnt = 0;


        Kubot.Move_current = true;
        ControlMode_by_ROS = 0;
        Kubot.kick_count = 0;
        Kubot.LRMOTION_READY = true;
        Kubot.LRMOTION_ING = false;

        break;

    default:
         break;
    }

}


void gazebo::B_kubot_plugin::UpdateAlgorithm()
{

    //* UPDATE TIME : 1ms
    //printf("update_do_time\n");
    beginner_kubot_pkgs::angle_msgs angle_msg;
    common::Time current_time = model->GetWorld()->SimTime();
    dt = current_time.Double() - last_update_time.Double();
    //        cout << "dt:" << dt << endl;
    time = time + dt;
    //    cout << "time:" << time << endl;

    //* setting for getting dt at next step
    last_update_time = current_time;
    
    getjointdata();
    getsensordata();

    static double steps    = 0;
    static double FBSize   = 0;
    static double LRSize   = 0;
    static double TurnSize = 0;

    double zmpFK_Y;

    //printf("update_do_getdata\n");
    // if(Kubot.Move_current == true){        
    //     VectorXd init_radian_po(12);
    //     // init_radian_po = Kubot.walkReadyAngle;
    //     for(int i=0;i<12;i++){

    //         joint[i].targetRadian = Kubot.walkReadyAngle[i];

    //     }

    // //     // VectorXd StandPose(12);
    // //     // StandPose=Kubot.IK_12(GB,LF,RF);
    // }
    //printf("update_do_init\n");


    static int con_count = 0;

    //* Real or simulated real-time thread time setting
    if (con_count % (int) (tasktime * 1000 + 0.001) == 0)
    {

        //* ControlMode
        switch (Kubot.ControlMode) 
        {

        case CTRLMODE_HOMEPOSE:

            break;

        case CTRLMODE_WALKREADY:

            break;

        case CTRLMODE_UPDOWN:

            break;
        
        case CTRLMODE_SWAY:

            break;               
        }


        //* CommandFlag
        switch (Kubot.CommandFlag) 
        {

        case GOTO_WALK_READY_POS: //down
            //WalkReady : Preparing the robot for walking
            if (Kubot.Move_current == false) {
                Kubot.walkingReady(5.0);
                for (int j = 0; j < nDoF; j++) {
                    joint[j].targetRadian = Kubot.refAngle[j];
                }
            }
            else {
                Kubot.CommandFlag = NONE_ACT;
                printf("WALK_READY COMPLETE !\n");
            }
            break;

        case RETURN_HOMEPOSE: // up

            if (Kubot.Move_current == false) {

                Kubot.HomePose(5.0);
                 for (int j = 0; j < nDoF; j++) {
                    joint[j].targetRadian = Kubot.refAngle[j];
                }
            }
            else {
                printf("RETURN_HOMEPOSE COMPLETE !\n");
                Kubot.ControlMode = CTRLMODE_HOMEPOSE;
                Kubot.CommandFlag = NONE_ACT;
            }
            break;

        case UPDOWN: // up and down

            if (Kubot.Move_current == true) {
                Kubot.ControlMode = CTRLMODE_UPDOWN;
                Kubot.CommandFlag = UPDOWN;
                printf("UPDOWN ING !\n");
            }
            
            break;       

        case SWAYMOTION_PATTERN: // sway motion pattern
            printf("pattern_start=====");
            if (Kubot.SWAY_ING == false) {
                Kubot.ControlMode = CTRLMODE_SWAY;
                printf("sway_start=====");

                printf("SWAY READY !\n");

            }
            else {
                Kubot.CommandFlag = NONE_ACT;
                printf("PATTERN READY COMPLETE !\n");
            }
            break;        

        case SWAYMOTION: // sway motion pattern

            if (Kubot.SWAY_READY = true) {
                Kubot.ControlMode = CTRLMODE_SWAY;
                Kubot.zmp.previewControl.count = Kubot.zmp.previewControl.count + 1;


                // printf("SWAY ING !\n");
            }
            else {
                Kubot.CommandFlag = NONE_ACT;
                printf("SWAY COMPLETE !\n");
            }

            break;     

        
        case NONE_ACT:

            break;
        }
        // printf("==walking generate start =====\n");
        Kubot.walkingPatternGenerator(Kubot.ControlMode,Kubot.CommandFlag,steps,FBSize,LRSize,TurnSize);


        VectorXd GB_Vector(6);

        double ref_B_x = 0;
        double ref_B_y = 0;
        double ref_B_z; //Base 위치

        VectorXd GFL_Vector(6);
        GFL_Vector << 0, 0.05, 0, 0, 0, 0;

        VectorXd GFR_Vector(6);
        GFR_Vector << 0, -0.05, 0, 0, 0, 0;

        double z_max = 0.42;
        double z_min = 0.34;
        double T = 1000;
        

        if (Kubot.ControlMode != CTRLMODE_HOMEPOSE) {
            if (Kubot.Move_current == true && Kubot.CommandFlag == UPDOWN) {
            
                //수정
                // VectorXd FK_xyz = VectorXd::Zero(3);
                // FK_xyz = Kubot.jointToPosition(test_vector_1);
                
                // // std::cout << "\nFK_xyz:\n" << FK_xyz << std::endl;

                // double FK_z = -FK_xyz(2);
                // std::cout << "\nFK_z:\n" << FK_z << std::endl;

// double CKubot::cosWave(amp,  period,  time,  int_pos)
// {
//    return (amp / 2)*(1 - cos(PI / period * time)) + int_pos;
// } amp 목표값-현재값, 전체 시간, dt같은 느낌, 현재값

                // ref_G_CoM_z = 0.34 + (FK_z)/2 * (1 - cos(PI/T * dt));
                ref_B_z = Kubot.cosWave(z_max-z_min, T, Kubot.updown_cnt, z_min);
                Kubot.updown_cnt = Kubot.updown_cnt + 1;

                std::cout << "\nref_B_z:\n" << ref_B_z << std::endl;

                GB_Vector << ref_B_x, ref_B_y, ref_B_z, 0, 0, 0;
                
                VectorXd joint_IK(12);
                
                joint_IK << Kubot.Geometric_IK_L(GB_Vector, GFL_Vector), Kubot.Geometric_IK_R(GB_Vector, GFR_Vector);
                
                joint[LHY].targetRadian = joint_IK[LHY];
                joint[LHR].targetRadian = joint_IK[LHR];
                joint[LHP].targetRadian = joint_IK[LHP];
                joint[LKN].targetRadian = joint_IK[LKN];
                joint[LAP].targetRadian = joint_IK[LAP];
                joint[LAR].targetRadian = joint_IK[LAR];
                
                joint[RHY].targetRadian = joint_IK[RHY];
                joint[RHR].targetRadian = joint_IK[RHR];
                joint[RHP].targetRadian = joint_IK[RHP];
                joint[RKN].targetRadian = joint_IK[RKN];
                joint[RAP].targetRadian = joint_IK[RAP];
                joint[RAR].targetRadian = joint_IK[RAR];

                // std::cout << "joint_IK:\n" << joint_IK << std::endl;

            }


        }

   // sway 실행 
        if(Kubot.CommandFlag == SWAYMOTION && Kubot.SWAY_READY == true) {
            VectorXd GBS_Vector(6);
            GBS_Vector << Kubot.zmp.previewControl.X.CoM, Kubot.zmp.previewControl.Y.CoM, 0.34, 0, 0, 0;
            
            VectorXd joint_IK(12);
                joint_IK << Kubot.Geometric_IK_L(GBS_Vector, GFL_Vector), Kubot.Geometric_IK_R(GBS_Vector, GFR_Vector);
                
                joint[LHY].targetRadian = joint_IK[LHY];
                joint[LHR].targetRadian = joint_IK[LHR];
                joint[LHP].targetRadian = joint_IK[LHP];
                joint[LKN].targetRadian = joint_IK[LKN];
                joint[LAP].targetRadian = joint_IK[LAP];
                joint[LAR].targetRadian = joint_IK[LAR];
                joint[RHY].targetRadian = joint_IK[RHY];
                joint[RHR].targetRadian = joint_IK[RHR];
                joint[RHP].targetRadian = joint_IK[RHP];
                joint[RKN].targetRadian = joint_IK[RKN];
                joint[RAP].targetRadian = joint_IK[RAP];
                joint[RAR].targetRadian = joint_IK[RAR];
                // m_preview_COM_y.data = Kubot.zmp.previewControl.Y.CoM;
                // P_preview_COM_y.publish(m_preview_COM_y);
                //printf("before_for");
                //for(int i=0;12;i++){
                //Kubot.Topic_pub_ref[i] = joint_IK[i];
                //}
                VectorXd zmpFKcheck(3);

                zmpFKcheck << Kubot.ZMPFK(test_vector_1);
                std::cout << "\nzmpFK:\n" << zmpFKcheck << std::endl;
                zmpFK_Y = zmpFKcheck(1);

                

    }
            
    }
    // if (Kubot.zmp.previewControl.count > 0 && Kubot.zmp.previewControl.count  < 3001) {
    //     printf("==============%d=============",Kubot.zmp.previewControl.count);
    //     printf("\n y_data : %lf\n",Kubot.zmp.previewControl.Y.CoM);
    //     printf("===========================");
    // }



    // Kubot.abc(); //Kubot class안에 만든 abc 실행
    // Kubot.FK();
    // Kubot.IK();
    // jointUpdateByAlgorithm();
    jointcontroller();

    m_preview_COM_y.data = Kubot.zmp.previewControl.Y.CoM;
    P_preview_COM_y.publish(m_preview_COM_y);

    m_preview_ref_ZMP_y.data = Kubot.zmp.previewControl.Y.m_ref;
    P_preview_ref_ZMP_y.publish(m_preview_ref_ZMP_y);

    m_ZMP_y.data = Kubot.zmp.previewControl.Y.old_zmp;
    P_ZMP_y.publish(m_ZMP_y);

    m_preview_FK_y.data = zmpFK_Y;
    P_preview_FK_y.publish(m_preview_FK_y);

    KubotModesp_pub.publish(KubotModesp_msg);    
     // angle_data_pub_start
     angle_msg.ref_angle_1 = joint[LHY].targetRadian; 
     angle_msg.ref_angle_2 = joint[LHR].targetRadian; 
     angle_msg.ref_angle_3 = joint[LHP].targetRadian; 
     angle_msg.ref_angle_4 = joint[LKN].targetRadian; 
     angle_msg.ref_angle_5 = joint[LAP].targetRadian; 
     angle_msg.ref_angle_6 = joint[LAR].targetRadian; 
     angle_msg.ref_angle_7 = joint[RHY].targetRadian; 
     angle_msg.ref_angle_8 = joint[RHR].targetRadian;
     angle_msg.ref_angle_9 = joint[RHP].targetRadian;
     angle_msg.ref_angle_10 = joint[RKN].targetRadian;
     angle_msg.ref_angle_11 = joint[RAP].targetRadian;
     angle_msg.ref_angle_12 = joint[RAR].targetRadian;

     angle_msg.present_angle_1 = joint[LHY].actualRadian;
     angle_msg.present_angle_2 = joint[LHR].actualRadian;
     angle_msg.present_angle_3 = joint[LHP].actualRadian;
     angle_msg.present_angle_4 = joint[LKN].actualRadian;
     angle_msg.present_angle_5 = joint[LAP].actualRadian;
     angle_msg.present_angle_6 = joint[LAR].actualRadian;
     angle_msg.present_angle_7 = joint[RHY].actualRadian;
     angle_msg.present_angle_8 = joint[RHR].actualRadian;
     angle_msg.present_angle_9 = joint[RHP].actualRadian;
     angle_msg.present_angle_10 = joint[RKN].actualRadian;
     angle_msg.present_angle_11 = joint[RAP].actualRadian;
     angle_msg.present_angle_12 = joint[RAR].actualRadian;  
     
     angle_msg.error_angle_1 = (joint[LHY].targetRadian - joint[LHY].actualRadian) * R2D;
     angle_msg.error_angle_2 = (joint[LHR].targetRadian - joint[LHR].actualRadian) * R2D;
     angle_msg.error_angle_3 = (joint[LHP].targetRadian - joint[LHP].actualRadian) * R2D;
     angle_msg.error_angle_4 = (joint[LKN].targetRadian - joint[LKN].actualRadian) * R2D;
     angle_msg.error_angle_5 = (joint[LAP].targetRadian - joint[LAP].actualRadian) * R2D;
     angle_msg.error_angle_6 = (joint[LAR].targetRadian - joint[LAR].actualRadian) * R2D;
     angle_msg.error_angle_7 = (joint[RHY].targetRadian - joint[RHY].actualRadian) * R2D;
     angle_msg.error_angle_8 = (joint[RHR].targetRadian - joint[RHR].actualRadian) * R2D;
     angle_msg.error_angle_9 = (joint[RHP].targetRadian - joint[RHP].actualRadian) * R2D;
     angle_msg.error_angle_10 = (joint[RKN].targetRadian -  joint[RKN].actualRadian) * R2D;
     angle_msg.error_angle_11 = (joint[RAP].targetRadian -  joint[RAP].actualRadian) * R2D;
     angle_msg.error_angle_12 = (joint[RAR].targetRadian -  joint[RAR].actualRadian) * R2D; 
    
    angle_pub.publish(angle_msg);
    //angle_data_pub_end
}
 



void gazebo::B_kubot_plugin::setjoints() //plugin에다가 joints name 설정 .sdf파일에서 설정한 이름이랑 확인하기
{
    /*
     * Get each joints data from [physics::ModelPtr _model]
     */

    //* Joint specified in model.sdf
    L_Hip_yaw_joint = this->model->GetJoint("L_Hip_yaw_joint");
    L_Hip_roll_joint = this->model->GetJoint("L_Hip_roll_joint");
    L_Hip_pitch_joint = this->model->GetJoint("L_Hip_pitch_joint");
    L_Knee_joint = this->model->GetJoint("L_Knee_joint");
    L_Ankle_pitch_joint = this->model->GetJoint("L_Ankle_pitch_joint");
    L_Ankle_roll_joint = this->model->GetJoint("L_Ankle_roll_joint");

    R_Hip_yaw_joint = this->model->GetJoint("R_Hip_yaw_joint");
    R_Hip_roll_joint = this->model->GetJoint("R_Hip_roll_joint");
    R_Hip_pitch_joint = this->model->GetJoint("R_Hip_pitch_joint");
    R_Knee_joint = this->model->GetJoint("R_Knee_joint");
    R_Ankle_pitch_joint = this->model->GetJoint("R_Ankle_pitch_joint");
    R_Ankle_roll_joint = this->model->GetJoint("R_Ankle_roll_joint");


    //L_Foot_joint = this->model->GetJoint("L_Foot_joint");
    //R_Foot_joint = this->model->GetJoint("R_Foot_joint");
}

void gazebo::B_kubot_plugin::getjointdata()
{
    /*
     * Get encoder and velocity data of each joint[j].targetRadian = joint_h[j];
     * encoder unit : [rad] and unit conversion to [deg]
     * velocity unit : [rad/s] and unit conversion to [rpm]
     */
    joint[LHY].actualRadian = L_Hip_yaw_joint->Position(0);
    joint[LHR].actualRadian = L_Hip_roll_joint->Position(0);
    joint[LHP].actualRadian = L_Hip_pitch_joint->Position(0);
    joint[LKN].actualRadian = L_Knee_joint->Position(0);
    joint[LAP].actualRadian = L_Ankle_pitch_joint->Position(0);
    joint[LAR].actualRadian = L_Ankle_roll_joint->Position(0);

    joint[RHY].actualRadian = R_Hip_yaw_joint->Position(0);
    joint[RHR].actualRadian = R_Hip_roll_joint->Position(0);
    joint[RHP].actualRadian = R_Hip_pitch_joint->Position(0);
    joint[RKN].actualRadian = R_Knee_joint->Position(0);
    joint[RAP].actualRadian = R_Ankle_pitch_joint->Position(0);
    joint[RAR].actualRadian = R_Ankle_roll_joint->Position(0);
    //printf("before for \n");
    
    for (int j = 0; j < 12; j++) { //ndof -> 12
        joint[j].actualDegree = joint[j].actualRadian*R2D;
        
        test_vector_1(j) = joint[j].actualRadian;
        //printf("update_encorder\n");
        //Kubot.actual_radian_vector(j) = joint[j].actualRadian;
        //printf("vector_data : %lf \n", Kubot.actual_radian_vector(j));
        // printf(C_BLUE "joint[%d].actualRadian = %f\n" C_RESET, j, joint[j].actualRadian);
    }

    //printf("complete for\n");
    joint[LHY].actualVelocity = L_Hip_yaw_joint->GetVelocity(0);
    joint[LHR].actualVelocity = L_Hip_roll_joint->GetVelocity(0);
    joint[LHP].actualVelocity = L_Hip_pitch_joint->GetVelocity(0);
    joint[LKN].actualVelocity = L_Knee_joint->GetVelocity(0);
    joint[LAP].actualVelocity = L_Ankle_pitch_joint->GetVelocity(0);
    joint[LAR].actualVelocity = L_Ankle_roll_joint->GetVelocity(0);

    joint[RHY].actualVelocity = R_Hip_yaw_joint->GetVelocity(0);
    joint[RHR].actualVelocity = R_Hip_roll_joint->GetVelocity(0);
    joint[RHP].actualVelocity = R_Hip_pitch_joint->GetVelocity(0);
    joint[RKN].actualVelocity = R_Knee_joint->GetVelocity(0);
    joint[RAP].actualVelocity = R_Ankle_pitch_joint->GetVelocity(0);
    joint[RAR].actualVelocity = R_Ankle_roll_joint->GetVelocity(0);

    for (int j = 0; j < nDoF; j++) {

        // printf(C_BLUE "joint[%d].targetVelocity = %f\n" C_RESET, j, joint[j].targetVelocity);
    }

    joint[LHY].actualTorque = L_Hip_yaw_joint->GetForce(0);
    joint[LHR].actualTorque = L_Hip_roll_joint->GetForce(0);
    joint[LHP].actualTorque = L_Hip_pitch_joint->GetForce(0);
    joint[LKN].actualTorque = L_Knee_joint->GetForce(0);
    joint[LAP].actualTorque = L_Ankle_pitch_joint->GetForce(0);
    joint[LAR].actualTorque = L_Ankle_roll_joint->GetForce(0);

    joint[RHY].actualTorque = R_Hip_yaw_joint->GetForce(0);
    joint[RHR].actualTorque = R_Hip_roll_joint->GetForce(0);
    joint[RHP].actualTorque = R_Hip_pitch_joint->GetForce(0);
    joint[RKN].actualTorque = R_Knee_joint->GetForce(0);
    joint[RAP].actualTorque = R_Ankle_pitch_joint->GetForce(0);
    joint[RAR].actualTorque = R_Ankle_roll_joint->GetForce(0);

    for (int j = 0; j < nDoF; j++) {

        joint[j].actualRPM = joint[j].actualVelocity * 60. / (2 * PI);
    }  

}


void gazebo::B_kubot_plugin::setsensor() //sdf파일에 있는 sensor를 설정하는 함수
{
        Sensor = sensors::get_sensor("IMU");
        IMU = std::dynamic_pointer_cast<sensors::ImuSensor>(Sensor);   
}

void gazebo::B_kubot_plugin::getsensordata()
{
     // IMU sensor data
        double IMUdata[3];

        IMUdata[0] = IMU->Orientation().Euler()[0];
        IMUdata[1] = IMU->Orientation().Euler()[1];
        IMUdata[2] = IMU->Orientation().Euler()[2];

        // printf(C_BLUE "IMU roll = %f\n" C_RESET, IMUdata[0]*R2D );
        // printf(C_RED "IMU pitch = %f\n" C_RESET, IMUdata[1]*R2D );
        // printf(C_YELLOW "IMU yaw = %f\n" C_RESET, IMUdata[2]*R2D );
        

        // IMU_dtheta[Roll] = IMU->AngularVelocity(false)[Roll];
        // IMU_dtheta[Pitch] = IMU->AngularVelocity(false)[Pitch];
        // IMU_dtheta[Yaw] = IMU->AngularVelocity(false)[Yaw];

}



void gazebo::B_kubot_plugin::jointcontroller()
{
    static double pre_rad[12];

    // double amp = 1;
    // double period = 3.0; // 이동하는 데 걸리는 시간 (주기)
    // double int_pos = 0.0;

    // double target_angle = cosWave(amp, period, time, int_pos);

    
    //     for (int j = 0; j < nDoF; j++) {

    //     joint[j].targetRadian = target_angle*joint[j].init_targetradian;
    //     joint[j].targetDegree = joint[j].targetRadian * R2D;
    //     joint[j].targetVelocity = (joint[j].targetRadian - pre_rad[j]) / dt;
    //     pre_rad[j] = joint[j].targetRadian;

    //     joint[j].targetTorque = (joint[j].Kp * (joint[j].targetRadian - joint[j].actualRadian)) \
    //                           + (joint[j].Kd * (joint[j].targetVelocity - joint[j].actualVelocity));
    //     }
    

    
        for (int j = 0; j < nDoF; j++) {
        joint[j].targetTorque = (joint[j].Kp * (joint[j].targetRadian - joint[j].actualRadian)) \
                              + (joint[j].Kd * (joint[j].targetVelocity - joint[j].actualVelocity));
        }
    
     
          

        //* Update target torque in gazebo simulation
        L_Hip_yaw_joint->SetForce(0, joint[LHY].targetTorque);
        L_Hip_roll_joint->SetForce(0, joint[LHR].targetTorque);
        L_Hip_pitch_joint->SetForce(0, joint[LHP].targetTorque);
        L_Knee_joint->SetForce(0, joint[LKN].targetTorque);
        L_Ankle_pitch_joint->SetForce(0, joint[LAP].targetTorque);        
        L_Ankle_roll_joint->SetForce(0, joint[LAR].targetTorque);         

        R_Hip_yaw_joint->SetForce(0, joint[RHY].targetTorque);
        R_Hip_roll_joint->SetForce(0, joint[RHR].targetTorque);
        R_Hip_pitch_joint->SetForce(0, joint[RHP].targetTorque);
        R_Knee_joint->SetForce(0, joint[RKN].targetTorque); 
        R_Ankle_pitch_joint->SetForce(0, joint[RAP].targetTorque);        
        R_Ankle_roll_joint->SetForce(0, joint[RAR].targetTorque); 

   
        //함수만 선언해놓고 함수를 실행안해서 아직은 실행불가
        //여기서 안돌아가는데 sdf파일 lower / upper 값 수정 필요 주석처리로 제한해제
        //괄호안을 0->2로 수정 x,y,z 순서라 그런가...

        
    


}




void gazebo::B_kubot_plugin::initializejoint()
{
    /*
     * Initialize joint variables for joint control
     */


        joint[0].init_targetradian  = (0  *D2R);     // L_Hip_yaw_joint
        joint[1].init_targetradian  = (0  *D2R);     // L_Hip_roll_joint
        joint[2].init_targetradian  = (-45*D2R);     // L_Hip_pitch_joint
        joint[3].init_targetradian  = (90 *D2R);     // L_Knee_joint
        joint[4].init_targetradian  = (-45*D2R);     // L_Ankle_pitch_joint
        joint[5].init_targetradian  = (0  *D2R);     // L_Ankle_roll_joint
        joint[6].init_targetradian  = (0  *D2R);     // R_Hip_yaw_joint
        joint[7].init_targetradian  = (0  *D2R);     // R_Hip_roll_joint
        joint[8].init_targetradian  = (-45*D2R);     // R_Hip_pitch_joint
        joint[9].init_targetradian  = (90 *D2R);     // R_Knee_joint
        joint[10].init_targetradian = (-45*D2R);     // R_Ankle_pitch_joint
        joint[11].init_targetradian = (0  *D2R);     // R_Ankle_roll_joint
    // for (int j = 0; j < nDoF; j++) {
    //     joint[j].targetDegree = 0;
    //     joint[j].targetRadian = 0;
    //     joint[j].targetVelocity = 0;
    //     joint[j].targetTorque = 0;
        
    //     joint[j].actualDegree = 0;
    //     joint[j].actualRadian = 0;
    //     joint[j].actualVelocity = 0;
    //     joint[j].actualRPM = 0;
    //     joint[j].actualTorque = 0;
    // }
}


void gazebo::B_kubot_plugin::setjointPIDgain()
{
    /*
     * Set each joint PID gain for joint control
     */
        joint[LHY].Kp = 100;
        joint[LHR].Kp = 110;
        joint[LHP].Kp = 140;
        joint[LKN].Kp = 200;
        joint[LAP].Kp = 100;
        joint[LAR].Kp = 100;

        joint[RHY].Kp = 100;
        joint[RHR].Kp = 110;
        joint[RHP].Kp = 140;
        joint[RKN].Kp = 200;
        joint[RAP].Kp = 100;
        joint[RAR].Kp = 100;

        joint[LHY].Kd = 0.03;
        joint[LHR].Kd = 0.05;
        joint[LHP].Kd = 0.08;
        joint[LKN].Kd = 0.07;
        joint[LAP].Kd = 0.06;
        joint[LAR].Kd = 0.03;

        joint[RHY].Kd = 0.03;
        joint[RHR].Kd = 0.05;
        joint[RHP].Kd = 0.08;
        joint[RKN].Kd = 0.07;
        joint[RAP].Kd = 0.06;
        joint[RAR].Kd = 0.03;

}

