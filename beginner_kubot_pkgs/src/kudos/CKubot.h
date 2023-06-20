#ifndef CKUBOT_H_
#define CKUBOT_H_


//* Basic header, Library
#include <stdio.h>
#include <iostream>
#include <string>


#include <Eigen/Dense>
#include <Eigen/Core>


using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::MatrixXf;
using Eigen::VectorXf;
using Eigen::Matrix3d;
using Eigen::Vector3d;

using Eigen::Vector2f;
using Eigen::Vector3f;
using Eigen::Vector4f;


#define Qp_N 30


#define PI      3.141592
#define D2R     PI/180.
#define R2D     180./PI 

#define onesecSize  100
#define tasktime    0.001

#define PatternPlan 4
#define PatternElement 6

#define MPC_GRF_N 5 // Control Horizon N for Model Predictive Control based Ground Reaction Force Control 


//* Command flag mode

typedef enum {
    NONE_ACT,
    RETURN_HOMEPOSE,
    GOTO_WALK_READY_POS,

    UPDOWN,
    SWAYMOTION,
    SWAYMOTION_PATTERN,
    LRMOTION,
    LRMOTION_PATTERN,

    // Walking
    ACT_DSP,
    ACT_SSP,
    ACT_SSP_Mode,
    ACT_TEST_WALK,
    ACT_TESTING_WALK,
    ACT_STOPPING_WALK,
    ACT_STOP_WALK,
    ACT_START_WALK,
    ACT_CHANGE_WALK,
    ACT_INF_WALK,
    ACT_PATTERN_TEST,
    ACT_PATTERN_TESTING

} _COMMAND_FLAG;

//* Control Mode

typedef enum {
    CTRLMODE_NONE,
    CTRLMODE_HOMEPOSE,
    CTRLMODE_WALKREADY,
    CTRLMODE_WALKING,
    CTRLMODE_WALKING_TEST,

    CTRLMODE_UPDOWN,
    CTRLMODE_SWAY,
    CTRLMODE_LRMOTION,

} _CONTROL_MODE;

// bool initposition; //* InitPosition


//* struct for Joints and End Points variables

typedef struct Joint {
    //* Current information

    float currentAngle; // [rad]
    float currentVel; // [rad/s]
    float currentAcc; // [rad/s^2]
    float currentTorque; // [N·m]

    //* Reference information

    // float refAngle; //[rad]
    float refVel; // [rad/s]
    float refAcc; // [rad/s^2]
    float refTorque; // [N·m]
    float ctcTorque; // [N·m]

    float compensate_refAngle; // [rad]

    float walkReadyAngle; // [rad]

    float gain_scheme; // changed by pattern scheme
} JOINT;

typedef struct Base //coordinate of Base
{
    //* Current information
    VectorXd current; //* EndPoint's Current Position
    VectorXd vel;     //* EndPoint's Current Velocity
    VectorXd acc;     //* EndPoint's Current Acceleration
    VectorXd pre;     //* EndPoint's Pre Position
    VectorXd prevel;  //* EndPoint's Pre Position
    VectorXd preacc;  //* EndPoint's Pre Position

//* Reference information
    VectorXd refpos;     //* EndPoint's ref Position
    VectorXd refvel;     //* EndPoint's ref Velocity
    VectorXd refacc;     //* EndPoint's ref acceleration
    VectorXd refquat;    //* EndPoint's ref quat                                                                              quat
    VectorXd prerefpos;  //* EndPoint's ref Position
    VectorXd prerefvel;  //* EndPoint's ref Velocity
    VectorXd prerefacc;  //* EndPoint's ref Acceleration

    int ID;

} BASE;

typedef struct Bodykiematics //coordinate of Body
{
    //*B_kubot Body Kinematics 단위는 mm
    double BASE_TO_LEG_Y;// = 50; 
    double BASE_TO_LEG_Z;// = 104.64; 
    double THIGH_LENGTH;// = 138.0; 
    double CALF_LENGTH;// = 143.0; 
    double ANKLE_LENGTH;// = 40; 
    double LEG_LENGTH;// = THIGH_LENGTH + CALF_LENGTH + ANKLE_LENGTH; //mm (THIGH_LENGTH + CALF_LENGTH + ANKLE_LENGTH)
    double LEG_SIDE_OFFSET;// = BASE_TO_LEG_Y; // BASE_TO_LEG_Y
    // End point ~ base : 425.64mm

    //*B_kubot Body Kinematics

    double foot_x_length ;//= 150.0; //*Foot x length
    double foot_y_length ;//= 100.0; //*Foot y length

} BK;

typedef struct XYSystemID // systemID for x,y-axis
{
    MatrixXd T1;     //SystemID for ZMPobserver
    MatrixXd T2;     //SystemID for ZMPobserver
    MatrixXd T1_DSP; //SystemID for ZMPobserver
    MatrixXd T2_DSP; //SystemID for ZMPobserver
    MatrixXd T1_SSP; //SystemID for ZMPobserver
    MatrixXd T2_SSP; //SystemID for ZMPobserver
    double T;        //SystemID for ZMPobserver

    double Spring_K;  //SystemID K for ZMPobserver
    double Damping_C; //SystemID C for ZMPobserver

    double Spring_K_DSP;  //SystemID K for ZMPobserver
    double Damping_C_DSP; //SystemID C for ZMPobserver

    double Spring_K_SSP;  //SystemID K for ZMPobserver
    double Damping_C_SSP; //SystemID C for ZMPobserver

    MatrixXd A; //SystemID for ZMPobserver
    MatrixXd B; //SystemID for ZMPobserver
    MatrixXd C; //SystemID for ZMPobserver
    double   D; //SystemID for ZMPobserver

    MatrixXd A_DSP; //SystemID for ZMPobserver
    MatrixXd B_DSP; //SystemID for ZMPobserver
    MatrixXd C_DSP; //SystemID for ZMPobserver
    double   D_DSP; //SystemID for ZMPobserver

    MatrixXd A_SSP; //SystemID for ZMPobserver
    MatrixXd B_SSP; //SystemID for ZMPobserver
    MatrixXd C_SSP; //SystemID for ZMPobserver
    double   D_SSP; //SystemID for ZMPobserver


} XYID;

typedef struct SystemID //SystemID
{
    double ComHeight; //mm /
    double robotM;    //kg //
    double robotI;    //Moment of Inertia
    double robotH;    //now Robot CoM height
    double robotw;    //now Robot natural frequency

    XYID X, Y;

} ID;

typedef struct XYPreviewControl // PreviewControl for xy-axis
{
    MatrixXd ref ;
    double m_ref ;

    double old_zmp;
    MatrixXd state;     //3X1 matrix
    MatrixXd new_state; //3X1 matrix
    double E;
    double sum_E;
    double sum_P;
    double U;
    double CoM, dCoM;

} XYPRE;





// typedef struct ZPreviewControl // PreviewControl for z-axis
// {
//     MatrixXd ref ;
//     double m_ref ;

//     double old_zmp;
//     MatrixXd state;     //3X1 matrix
//     MatrixXd new_state; //3X1 matrix
//     double E;
//     double sum_E;
//     double sum_P;
//     double U;
//     double CoM, dCoM;

// } ZPRE;






typedef struct PreviewControl //
{
    double RefTotalTrajSize = 19500;
    double RefTotalTime;
    double PreviewTimeSize;
    double PeriodTimeSize;
    double PreviewTime;
    double PreviewReadyTime;

    MatrixXd A;

    MatrixXd B;

    MatrixXd C;

    MatrixXd B_Tilde;

    MatrixXd I_Tilde;

    MatrixXd F_Tilde;

    MatrixXd Q_Tilde;

    MatrixXd R;

    MatrixXd A_Tilde;

    MatrixXd K_Tilde;

    MatrixXd G_I;

    MatrixXd G_X;

    MatrixXd G_P;

    MatrixXd A_Tilde_c;

    MatrixXd X_Tilde;

    XYPRE X, Y;

    // ZPRE Z;

    int count;
    int setcount;

} PREVIEWCONTROL;


typedef struct XYObserver // Observer for y-axis
{
    double m_hat_state;         //* ZMPobserver
    MatrixXd hat_state;         //* ZMPobserver hat state
    MatrixXd hat_new_state;     //* ZMPobserver new hat state
    MatrixXd hat_d_state;       //* ZMPobserver hat d state

    MatrixXd dsp_hat_state;     //* ZMPobserver dsp hat state
    MatrixXd dsp_hat_new_state; //* ZMPobserver dsp new hat state
    MatrixXd dsp_hat_d_state;   //* ZMPobserver dsp hat d state

    MatrixXd ssp_hat_state;     //* ZMPobserver ssp hat state
    MatrixXd ssp_hat_new_state; //* ZMPobserver ssp new hat state
    MatrixXd ssp_hat_d_state;   //* ZMPobserver ssp hat d state

    double hat_ZMP;             //* ZMPobserver hay zmp
    double m_hat_ZMP;           //* ZMPobserver
    double dsp_hat_ZMP;         //* ZMPobserver dsp hat zmp
    double ssp_hat_ZMP;         //* ZMPobserver ssp hat zmp

    double m_disturbance;       //* ZMPobserver

    MatrixXd L;   //ZMPobserver gain L

    MatrixXd L_DSP; //ZMPobserver gain L
    MatrixXd L_SSP; //ZMPobserver gain L

} XYOBSERVER;


typedef struct Observer //
{
    XYOBSERVER X, Y;

    enum {
        NOdisturbance = 0,
        Disturbance = 1,
        Slope = 2 //start foot L
    };

    int disturbanceExist;

    Eigen::Vector3d groundAngle;
    Eigen::Vector3d groundVel;
    Eigen::Vector3d groundpreAngle;

} OBSERVER;

typedef struct XControl // Observer for x-axis
{
    double U; //ZMPControl Input U
    double dsp_U; //ZMPControl Input U
    double ssp_U; //ZMPControl Input U
    double m_U; //ZMPControl Input U
    double disturbanceRejection_U; //ZMPControl Input disturbance Rejection
    double dsp_disturbanceRejection_U; //ZMPControl Input disturbance Rejection
    double ssp_disturbanceRejection_U; //ZMPControl Input disturbance Rejection
    double slope_compensate; //Input Compensate slope
    double U_Limit; //ZMPControl Input U

    MatrixXd K; //ZMPControl gain K
    MatrixXd K_DSP; //ZMPControl gain K
    MatrixXd K_SSP; //ZMPControl gain K

    //JUMPREADY by YSJ
    double jump_U;
    double jump_Error;

    double jump_U_dot;
    double jump_U_old;

} XCONTROL;

typedef struct YControl // Observer for y-axis
{
    double U; //ZMPControl Input U
    double dsp_U; //ZMPControl Input U
    double ssp_U; //ZMPControl Input U
    double m_U; //ZMPControl Input U
    double disturbanceRejection_U; //ZMPControl Input disturbance Rejection
    double dsp_disturbanceRejection_U; //ZMPControl Input disturbance Rejection
    double ssp_disturbanceRejection_U; //ZMPControl Input disturbance Rejection
    double slope_compensate; //Input Compensate slope
    double U_Limit; //ZMPControl Input U

    MatrixXd K; //ZMPControl gain K
    MatrixXd K_DSP; //ZMPControl gain K
    MatrixXd K_SSP; //ZMPControl gain K
} YCONTROL;

typedef struct Controller //
{
    XCONTROL X;
    YCONTROL Y;
    bool dsp2ssp;
    bool ssp2dsp;
    bool gainchangeflag;
    double gainChangeTime;
    double gainChange_t;

    double dsp_gain;
    double ssp_gain;

    Eigen::Vector3d U;

    bool ON;
    bool OFF;

} CONTROL;

typedef struct zmpx //
{
    double sensor, old, init;
    double Fmargin;
    double Bmargin;

} XZMP;

typedef struct zmpy //
{
    double sensor, old, init;
    double Lmargin;
    double Rmargin;

} YZMP;

 typedef struct ZeroMomentPoint //ZMP
{
    PREVIEWCONTROL previewControl;
    OBSERVER observer;
    CONTROL controller;

    Eigen::Vector3d global_ref;
    Eigen::Vector3d global;
    Eigen::Vector3d local;

    XZMP X;
    YZMP Y;
    

} ZMP;

//  typedef struct FootZeroMomentPoint //ZMP
// {
//     PREVIEWCONTROL previewControl;
//     OBSERVER observer;
//     CONTROL controller;

//     Eigen::Vector3d global_ref;
//     Eigen::Vector3d global;
//     Eigen::Vector3d local;

//     XZMP X;
//     YZMP Y;
//     // ZZMP Z;

// } FZMP;

typedef struct Step //Step
{
    int current, current_footprints;
    double total, footprints;

    double FBstepSize,   oldFBstepSize;
    double LRstepSize,   oldLRstepSize;
    double TurnSize,     oldTurnSize;
    double footHeight;
    double turningPoint;

    enum {
        S_L_F = 1,  //start foot L
        S_R_F = -1, //start foot R
        L_L_F = 1,  //last foot L
        L_R_F = -1  //last foot R
    };

    int start_foot; //start foot
    int last_foot;

    enum {
        forwardWalking = 1, //walk forward
        backwardWalking = -1, // back forward
        leftWalking = 1, //left
        rightWalking = -1, // right
        leftTurning = 1,
        rightTurning = -1,
        TurnModeOn =1,
        TurnModeOff = 0,
        WalkinginPlace = 0 //
    };
    //    static int None = 0;
    int walking_f_or_b; //walkig f or b?
    int walking_l_or_r; //walkig l or r?
    int walking_lT_or_rT; //walkig l or r?
    int walking_Turn;

    MatrixXd Lfoot, Rfoot;
    VectorXd preLfoot, preRfoot, prefootCenter;
    VectorXd LastLfoot, LastRfoot, LastfootCenter;

} STEP;

typedef struct WalkingTime //Step
{
    double sec; //*time while robot is walking

    double periodTime; //*walking periodTime
    double DSP_ratio; //*DSP ratio of walking periodTime
    double SSP_ratio; //*SSP ratio of walking periodTime
    double SSP_start_time;
    double SSP_end_time;
    double SSP_time;

    double XYDSP_ratio; //*DSP ratio of walking XY pattern periodTime
    double XYSSP_ratio; //*SSP ratio of walking XY pattern periodTime
    double XYSSP_start_time;
    double XYSSP_end_time;
    double XYSSP_time;
    double readytime;

} WALKINGTIME;

typedef struct Walking //
{

    STEP step;
    WALKINGTIME time;
    // SWINGLEGVIBRATION SwingLegVibrationCon;
    // LANDINGCON Landing;

    enum \
     {
        LSSP = 1,
        RSSP = -1,
        DSP = 0,
        SSP = 2,
        UP = 3
    };

    int ft_SP; //* SP by FTsensor
    int old_ft_SP; //* SP by FTsensor

    int walk_SP; //* SP by WalkingPattern
    int walk_SSP;
    int old_walk_SP; //* SP by WalkingPattern
    int old_walk_SSP; //* SSP by WalkingPattern

    int SP; //* Checking SP by FTsensor, WalkingPattern
    int old_SP; //* Checking SP by FTsensor, WalkingPattern

    int SP_check; //* SP for Checking
    int SP_oldcheck; //* SP for Checking
    int SP_checking; //* check number for Checking
    int Mode;

    bool change_FB_walking_flag;
    bool change_LR_walking_flag;
    bool change_Turn_flag;
    bool change_LTurn_flag;
    bool change_RTurn_flag;


    bool Ready;
    bool walk;
    bool ssp_mode;

    bool NewfootPlan;

    bool initialize;
} WALK;

typedef struct Quadratic_programming//
{
//    Quadratic Programming
//    Based on 'Trajectory Free Linear Model Predictive Control for Stable Walking in the Presence of Strong Perturbations
//    - Pierre-Brice Wieber -
//    1/2 xHx+xf

    MatrixXd A;
    MatrixXd B;
    MatrixXd C;
    
    MatrixXd Px;
    MatrixXd Pu;
    
    MatrixXd H; //H=Q*Pu'*Pu+R*I
    MatrixXd G; //[Pu];
    
    MatrixXd f;
    MatrixXd ref_ZMP_traj;
    MatrixXd zmp_by_com;
 
    VectorXd max_zmpx;
    VectorXd min_zmpx;
    VectorXd max_zmpy;
    VectorXd min_zmpy;

    
    Eigen::Vector3d m_max_zmp_margin;
    Eigen::Vector3d m_min_zmp_margin;
    
    Eigen::Vector3d m_ref_zmp;
    Eigen::Vector3d m_ref_com;
    Eigen::Vector3d m_zmp_by_com;

    
    double Q;
    double R;
    
    int N; //the number N of pieces of the trajectory
    int total_N; //the total N of pieces of the every trajectory
    double T_ref_traj; //sec, time of reference trajectory
    double Test_time_traj; //sec, time of All reference trajectory
    double T; //sampling time
    int count;

    float H_x[(Qp_N+1)*Qp_N/2];
    int   H_nnz = (Qp_N+1)*Qp_N/2;
    int   H_i[(Qp_N+1)*Qp_N/2];
    int   H_p[(Qp_N+1)];
    
    float q_x[Qp_N];
    float q_y[Qp_N];


    float A_x[(Qp_N+1)*Qp_N/2];
    int   A_nnz = (Qp_N+1)*Qp_N/2;
    int   A_i[(Qp_N+1)*Qp_N/2];
    int   A_p[(Qp_N+1)];
    
    float zmpxl[Qp_N];
    float zmpxu[Qp_N];
    float zmpyl[Qp_N];
    float zmpyu[Qp_N];
    
    int n = Qp_N;
    int m = Qp_N;
    
    int exitflag_for_x;
    int exitflag_for_y;
    
    MatrixXd state; //3X3 Matrix, col(1) = x, dx, ddx, col(2) = y, dy, ddy, col(3) = z, dz, ddz
    MatrixXd CoM_traj; //QP N X 3 Matrix, col(1) = com X, col(2) = com Y, col(3) = com Z
    
} QP;


class CKubot {
public:

    CKubot();
    CKubot(const CKubot& orig);
    virtual ~CKubot();

    //***Variables***//

    //* Control Mode and Command Flag

    unsigned int ControlMode;
    unsigned int CommandFlag;

    int joint_DoF; //Joint Degrees of Freedom


    //* enum

    enum JointNumber { // Kubot
        LHY = 0,
        LHR, LHP, LKN, LAP, LAR,
        RHY, RHR, RHP, RKN, RAP, RAR
    };

    enum EndPointAxis { // RoK-3's End point pos, ori axis
        X = 0, Y, Z, Roll, Pitch, Yaw
    };


    VectorXd GFLL_Vector;
    VectorXd GFRR_Vector;




    // int test = 10;
    int sign(double a);
    Eigen::Matrix3d transposeMat(Matrix3d mat);

    MatrixXd rotMatX(double q);
    MatrixXd rotMatY(double q);
    MatrixXd rotMatZ(double q);

    MatrixXd getTransformI0();
    MatrixXd getTransform6E();
    MatrixXd jointToTransform01(VectorXd q);
    MatrixXd jointToTransform01_R(VectorXd q);
    MatrixXd jointToTransform12(VectorXd q);
    MatrixXd jointToTransform23(VectorXd q);
    MatrixXd jointToTransform34(VectorXd q);
    MatrixXd jointToTransform45(VectorXd q);
    MatrixXd jointToTransform56(VectorXd q);

    MatrixXd jointToPosition(VectorXd q);
    MatrixXd jointToPosition_R(VectorXd q);
    Matrix3d jointToRotMat(VectorXd q);
    Matrix3d jointToRotMat_R(VectorXd q);
    VectorXd rotToEuler(MatrixXd rotMat);

    MatrixXd jointToPosJac(VectorXd q);
    MatrixXd jointToPosJac_R(VectorXd q);
    MatrixXd jointToRotJac(VectorXd q);
    MatrixXd jointToRotJac_R(VectorXd q);

    MatrixXd pseudoInverseMat(MatrixXd A, double lambda);
    VectorXd rotMatToRotVec(MatrixXd C);
    MatrixXd angleAxisToRotMat(VectorXd rotVec);
    MatrixXd jointToGeoJac(VectorXd q);
    MatrixXd jointToGeoJac_R(VectorXd q);
    VectorXd inverseKinematics(Vector3d r_des, Matrix3d C_des, VectorXd q0, double tol);
    VectorXd inverseKinematics_R(Vector3d r_des, Matrix3d C_des, VectorXd q0, double tol);
    VectorXd Geometric_IK_L(VectorXd GB_cfg, VectorXd GF_cfg);
    VectorXd Geometric_IK_R(VectorXd GB_cfg, VectorXd GF_cfg);

    //***Functions***//
    void abc();
    void FK(VectorXd zmp_fk);
    void IK();
    MatrixXd ZMPFK(VectorXd q);
    MatrixXd jointToTransform01_zmp(VectorXd q);



    // VectorXd init_pose(int param_1, VectorXd present, VectorXd real);

    bool Move_current;

    bool SWAY_READY;
    bool SWAY_ING;

    bool LRMOTION_READY;
    bool LRMOTION_ING;

    VectorXd actual_radian_vector;
    float refAngle[12];
    float walkReadyAngle[12];
    int updown_cnt = 0;


    void HomePose(float return_time);
    void walkingReady(float readytime);
    void UpDown(float readytime);
    double cosWave(double amp, double period, double time, double int_pos);

    void setWalkingReadyPos(double init_x, double init_y, double init_z);
    // bool wave_flag;
    
    void sway();
    void legMotion();
    void CStand();
    // zmp function and variable
    void initializeBkubot();
    void setZmpPreview(double previewTime); //* set about ZMP preview control
    void setZmpPreviewTime(double previewTime); //* set about ZMP preview time

    void initializeQP(void);

    void walkingPatternGenerator(int controlMODE, int CommandFLAG, double steps, double FBsize, double LRsize, double Turnsize); //* Planning Foot X,Y,Z, roll, pitch, yaw Reference for Walking
    void zmpPreviewControl(void); //* ZMP preview Control


    MatrixXd ZMP_DARE(MatrixXd& A, MatrixXd& B, MatrixXd& Q, MatrixXd& R);
    Eigen::EigenSolver<MatrixXd> eZMPSolver;


    BASE Base; //* coordinate of Body
    ZMP zmp; //* About ZMP, ZMP preview control, ZMP Observer and Controller
    BK body; //* Body Kinematics
    WALK walking; //* Variable of Walking
    QP qp;


    ID systemID; //* System ID about the robot
    int kick_count;
    // FZMP fzmp;


    //* Functions related to Walking, are in CRobot_Walking.cpp

private:

    //***Variables***//    
    double init_t; //* Time for Initialize

    //***Functions***//

};
#endif