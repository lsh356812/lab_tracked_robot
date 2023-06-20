#include "CKubot.h"
// #include "kinematics.h"


using namespace std;


CKubot::CKubot()
{

}

CKubot::CKubot(const CKubot& orig)
{
}

CKubot::~CKubot()
{
}

void CKubot::abc()
{
    printf("@@@@@@@@@@과연 결과는?@@@@@@@@@@\n");
    
}

int CKubot::sign(double a){
    if(a>=0) return 1;
    return -1;
}



Matrix3d CKubot::transposeMat(Matrix3d mat) {
    Matrix3d tmp_m;

    tmp_m << mat(0, 0), mat(1, 0), mat(2, 0), \
             mat(0, 1), mat(1, 1), mat(2, 1), \
             mat(0, 2), mat(1, 2), mat(2, 2);
 
    return tmp_m;
}

MatrixXd CKubot::rotMatX(double q) {
    MatrixXd tmp_m(3,3);
    tmp_m << 1, 0, 0,\
             0, cos(q), -sin(q),\
             0, sin(q), cos(q);
    return tmp_m;
}

MatrixXd CKubot::rotMatY(double q) {
    MatrixXd tmp_m(3,3);
    tmp_m << cos(q), 0, sin(q),\
             0 , 1, 0 ,\
             -sin(q), 0, cos(q);
    return tmp_m;
}

MatrixXd CKubot::rotMatZ(double q) {
    MatrixXd tmp_m(3,3);
    tmp_m << cos(q), -sin(q), 0,\
             sin(q), cos(q), 0, \
             0, 0, 1;
    return tmp_m;
}

MatrixXd CKubot::getTransformI0()
{
//  Frame 0 to Frame I

    MatrixXd tmp_m(4,4);
  
// Alternative representations 
// 1. tmp_m = MatrixXd::Identity(4,4);
// 2. tmp_m(m-1,n-1) = m,n element where m,n>=1;

    tmp_m << 1, 0, 0, 0, \
             0, 1, 0, 0, \
             0, 0, 1, 0, \
             0, 0, 0, 1;
    
    return tmp_m;
}

MatrixXd CKubot::getTransform6E()
{
//  Frame E to Frame 6
    
    MatrixXd tmp_m(4,4);
            
    tmp_m << 1, 0, 0,  0, \
             0, 1, 0,  0, \
             0, 0, 1, -0.04, \
             0, 0, 0,  1;
    
    return tmp_m;
}

MatrixXd CKubot::jointToTransform01(VectorXd q)
{
//  Frame 1 to Frame 0
// q: generalized coordinates, q = [q1;q2;q3;q4;q5;q6]    

    MatrixXd tmp_m(4,4);
    double qq = q(0);
   
    tmp_m << cos(qq), -sin(qq), 0, 0, \
             sin(qq),  cos(qq), 0, 0.05, \
             0,        0,       1, -0.10464, \
             0,        0,       0, 1;
    
    return tmp_m;    
}

MatrixXd CKubot::jointToTransform01_R(VectorXd q)
{
//  Frame 1 to Frame 0
// q: generalized coordinates, q = [q1;q2;q3;q4;q5;q6]    

    MatrixXd tmp_m(4,4);
    double qq = q(0);
   
    tmp_m << cos(qq), -sin(qq), 0, 0, \
             sin(qq),  cos(qq), 0, -0.05, \
             0,        0,       1, -0.10464, \
             0,        0,       0, 1;
    
    return tmp_m;    
}

MatrixXd CKubot::jointToTransform12(VectorXd q)
{
//  Frame 2 to Frame 1

    MatrixXd tmp_m(4,4);
    double qq = q(1);

    tmp_m << 1, 0,        0,       0, \
             0, cos(qq), -sin(qq), 0, \
             0, sin(qq),  cos(qq), 0, \
             0      , 0,  0      , 1;
    
    return tmp_m;
}

MatrixXd CKubot::jointToTransform23(VectorXd q)
{
//  Frame 3 to Frame 2
    MatrixXd tmp_m(4,4);
    double qq = q(2);

    tmp_m << cos(qq), 0, sin(qq), 0, \
             0      , 1, 0      , 0, \
            -sin(qq), 0, cos(qq), 0, \
             0      , 0, 0      , 1;
    
    return tmp_m;
}

MatrixXd CKubot::jointToTransform34(VectorXd q)
{
//  Frame 4 to Frame 3
    MatrixXd tmp_m(4,4);
    double qq = q(3);
    
    tmp_m << cos(qq), 0, sin(qq), 0, \
             0      , 1, 0      , 0, \
            -sin(qq), 0, cos(qq), -0.138, \
             0      , 0, 0      , 1;
    
    return tmp_m;
}

MatrixXd CKubot::jointToTransform45(VectorXd q)
{
//  Frame 5 to Frame 4
    MatrixXd tmp_m(4,4);
    double qq = q(4);

    tmp_m << cos(qq), 0, sin(qq), 0, \
             0      , 1, 0      , 0, \
            -sin(qq), 0, cos(qq), -0.143, \
             0      , 0, 0      , 1;
    
    return tmp_m;
}

MatrixXd CKubot::jointToTransform56(VectorXd q)
{
//  Frame 5 to Frame 4
    MatrixXd tmp_m(4,4);
    double qq = q(5);

    tmp_m << 1, 0,        0,       0, \
             0, cos(qq), -sin(qq), 0, \
             0, sin(qq),  cos(qq), 0, \
             0      , 0,  0      , 1;

    return tmp_m;
}

MatrixXd CKubot::jointToPosition(VectorXd q)
{
//  Extract position vector(3x1) 
    
    VectorXd tmp_vL = VectorXd::Zero(3);
    VectorXd tmp_vR = VectorXd::Zero(3);

    VectorXd tmp_v = VectorXd::Zero(6);
    
    MatrixXd tmp_m(4,4);
    
    int left = 0;
    int right = 1;
    for (int leg = 0; leg < 2; leg++){
 
        if (leg == 0) {
            tmp_m = getTransformI0()*
                    jointToTransform01(q)* 
                    jointToTransform12(q)* 
                    jointToTransform23(q)* 
                    jointToTransform34(q)* 
                    jointToTransform45(q)* 
                    jointToTransform56(q)* 
                    getTransform6E();
        
//    tmp_v = tmp_m.block(0,3,3,1);

                tmp_vL(0) = tmp_m(0,3);
                tmp_vL(1) = tmp_m(1,3);
                tmp_vL(2) = tmp_m(2,3);

                // std::cout << "\nFK_jointToPosition_L:\n" << tmp_vL << std::endl;
        }
    
        else if (leg == 1) {
            tmp_m = getTransformI0()*
                    jointToTransform01_R(q)* 
                    jointToTransform12(q)* 
                    jointToTransform23(q)* 
                    jointToTransform34(q)* 
                    jointToTransform45(q)* 
                    jointToTransform56(q)* 
                    getTransform6E();
        
//    tmp_v = tmp_m.block(0,3,3,1);

                tmp_vR(0) = tmp_m(0,3);
                tmp_vR(1) = tmp_m(1,3);
                tmp_vR(2) = tmp_m(2,3);

                // std::cout << "FK_jointToPosition_R:\n" << tmp_vR << std::endl;

        }
       

    }
        tmp_v << tmp_vL,tmp_vR;

        // std::cout << "FK_jointToPosition_tmp_vL+tmp_vR:\n" << tmp_vL << "\n \n" << tmp_vR << std::endl; 

        // std::cout << "FK_jointToPosition_tmp_v:\n" << tmp_v  << std::endl; 

    return tmp_v; // std::cout << "FK_jointToPosition: " << tmp_m << std::endl;


}




MatrixXd CKubot::jointToPosition_R(VectorXd q)
{
//  Extract position vector(3x1) 
    
    VectorXd tmp_v = VectorXd::Zero(3);
    MatrixXd tmp_m(4,4);
    
    tmp_m = getTransformI0()*
            jointToTransform01_R(q)* 
            jointToTransform12(q)* 
            jointToTransform23(q)* 
            jointToTransform34(q)* 
            jointToTransform45(q)* 
            jointToTransform56(q)* 
            getTransform6E();
        
//    tmp_v = tmp_m.block(0,3,3,1);

    tmp_v(0) = tmp_m(0,3);
    tmp_v(1) = tmp_m(1,3);
    tmp_v(2) = tmp_m(2,3);
    
    return tmp_v;
    // std::cout << "FK_jointToPosition: " << tmp_m << std::endl;

}


Matrix3d CKubot::jointToRotMat(VectorXd q)
{
    Matrix3d tmp_m;
    MatrixXd T_IE(4,4);
    
    T_IE =  getTransformI0()*
            jointToTransform01(q)* 
            jointToTransform12(q)* 
            jointToTransform23(q)* 
            jointToTransform34(q)* 
            jointToTransform45(q)* 
            jointToTransform56(q)* 
            getTransform6E();
    
    tmp_m = T_IE.block(0,0,3,3);
    
    return tmp_m;
}

Matrix3d CKubot::jointToRotMat_R(VectorXd q)
{
    Matrix3d tmp_m;
    MatrixXd T_IE(4,4);
    
    T_IE =  getTransformI0()*
            jointToTransform01_R(q)* 
            jointToTransform12(q)* 
            jointToTransform23(q)* 
            jointToTransform34(q)* 
            jointToTransform45(q)* 
            jointToTransform56(q)* 
            getTransform6E();
    
    tmp_m = T_IE.block(0,0,3,3);
    
    return tmp_m;
}

VectorXd CKubot::rotToEuler(MatrixXd rotMat)
{
    // ZYX Euler Angle - yaw-pitch-roll
    Vector3d tmp_v;
    
    tmp_v(0) = atan2(rotMat(1,0),rotMat(0,0));
    tmp_v(1) = atan2(-rotMat(2,0),sqrt(pow(rotMat(2,1),2)+pow(rotMat(2,2),2)));
    tmp_v(2) = atan2(rotMat(2,1),rotMat(2,2));
            
    // std::cout << tmp_v << endl;
    
    return tmp_v;
}


VectorXd CKubot::Geometric_IK_L(VectorXd GB_PosOri, VectorXd GF_PosOri) { // Global to Base,Foot position and orientaion
    double A, B, C; // A = thigh, B = calf
    double alpha, c5;
    VectorXd BH(3), p1(3), p2(3), p7(3), pf(3), FA(3), r(3);
    VectorXd q(6);
    MatrixXd R1(3,3), R7(3,3), RH(3,3);

    MatrixXd GB = MatrixXd::Identity(4,4);
    MatrixXd GF = MatrixXd::Identity(4,4);

    GB.block(0,0,3,3) = rotMatZ(GB_PosOri(5))*rotMatY(GB_PosOri(4))*rotMatX(GB_PosOri(3));
    GB.block(0,3,3,1) << GB_PosOri(0), GB_PosOri(1), GB_PosOri(2);


    GF.block(0,0,3,3) = rotMatZ(GF_PosOri(5))*rotMatY(GF_PosOri(4))*rotMatX(GF_PosOri(3));
    GF.block(0,3,3,1) << GF_PosOri(0), GF_PosOri(1), GF_PosOri(2);

    BH << 0, 0.05, -0.105; // Base to Hip

    R1 = GB.block(0,0,3,3);
    R7 = GF.block(0,0,3,3);
    pf = GF.block(0,3,3,1); // Foot position
    FA << 0, 0, 0.04 ; // Foot to Ankle
    p7 = pf+FA ;

    A = 0.138;
    B = 0.143;

    p1 << GB(0,3), GB(1,3), GB(2,3);
    p2 = p1 + R1 * BH;
   
    r = R7.transpose() * (p2-p7);
    C = r.norm();
    c5 = cos((A*A + B*B - C*C) / (2 * A * B));

    if(c5 >= 1) q(3) = 0.0;
    else if(c5 <= -1) q(3) = PI;
    else q(3)=-acos((A*A + B*B - C*C) / (2 * A * B)) + PI;

    alpha = asin((A*sin(PI - q(3)))/C);

    q(5) = atan2(r(1), r(2));
    q(4) = -atan2(r(0), sign(r(2))*sqrt(r(1)*r(1)+r(2)*r(2))) - alpha;

    RH = R1.transpose() * R7 * rotMatX(-q(5)) * rotMatY(-q(3)-q(4)); // Rotation Hip

    q(0) = atan2(-RH(0,1), RH(1,1));
    q(1) = atan2(RH(2,1), -sin(q(0)) * RH(0,1) + RH(1,1) * cos(q(0)));
    q(2) = atan2(-RH(2,0), RH(2,2));

    // std::cout << "\nIK_joint_L:\n" << q << std::endl;
    
    return q;

}

VectorXd CKubot::Geometric_IK_R(VectorXd GB_cfg, VectorXd GF_cfg) {
    double A, B, C; // thigh, calf
    double alpha, c5;
    VectorXd PH(3) ;
    VectorXd p1(3), p2(3), p7(3), pf(3), fa(3), r(3);
    VectorXd q(6);
    MatrixXd R1(3,3), R7(3,3), RH(3,3);
   
    MatrixXd GB = MatrixXd::Identity(4,4);
    MatrixXd GF = MatrixXd::Identity(4,4);

    GB.block(0,0,3,3) = rotMatZ(GB_cfg(5))*rotMatY(GB_cfg(4))*rotMatX(GB_cfg(3));
    GB.block(0,3,3,1) << GB_cfg(0), GB_cfg(1), GB_cfg(2);


    GF.block(0,0,3,3) = rotMatZ(GF_cfg(5))*rotMatY(GF_cfg(4))*rotMatX(GF_cfg(3));
    GF.block(0,3,3,1) << GF_cfg(0), GF_cfg(1), GF_cfg(2);

    PH << 0, -0.05, -0.105;
   
    R1 = GB.block(0,0,3,3);
    R7 = GF.block(0,0,3,3);
    pf = GF.block(0,3,3,1);
    fa << 0, 0, 0.04 ;
    p7 = pf+fa ;
   
    A = 0.138;
    B = 0.143;
   
    p1 << GB(0,3), GB(1,3), GB(2,3);
    p2 = p1 + R1 * PH;
   
    r= R7.transpose() * (p2-p7);
    C= r.norm();
    c5 = cos((A*A + B*B - C*C) / (2 * A * B));
   
    if(c5 >= 1) q(3) = 0.0;
   
    else if(c5 <= -1) q(3) = PI;
   
    else q(3)=-acos((A*A + B*B - C*C) / (2 * A * B)) + PI;
   
    alpha = asin((A*sin(PI - q(3)))/C);
   
    q(5) = atan2(r(1), r(2));
    q(4) = -atan2(r(0), sign(r(2))*sqrt(r(1)*r(1)+r(2)*r(2))) - alpha;
   
    RH = R1.transpose() * R7 * rotMatX(-q(5)) * rotMatY(-q(3)-q(4));
   
    q(0) = atan2(-RH(0,1), RH(1,1));
    q(1) = atan2(RH(2,1), -sin(q(0)) * RH(0,1) + RH(1,1) * cos(q(0)));
    q(2) = atan2(-RH(2,0), RH(2,2));

    // std::cout << "IK_joint_R:\n" << q << std::endl;

    return q;
}

void CKubot::FK(VectorXd zmp_fk)
{
    VectorXd inputRadian(12);
    //Matrix3d FK_ori;

    inputRadian << zmp_fk;


    jointToPosition(inputRadian);
    //FK_ori = jointToRotMat(inputRadian);

    //std::cout << "\nFK_ori:\n" << rotToEuler(FK_ori) <<std::endl;

}

void CKubot::IK()
{
    VectorXd GB_PosOri(6);
    VectorXd GF_PosOri_L(6), GF_PosOri_R(6);
    VectorXd GF_cfg_R(6);


    GB_PosOri << 0,0,0.5,0,0,0;
    GF_PosOri_L << -0.00353553,0.05,0.156664,0,0,0;
    GF_PosOri_R << -0.00353553,-0.05,0.156664,0,0,0;

    Geometric_IK_L(GB_PosOri, GF_PosOri_L);
    Geometric_IK_R(GB_PosOri, GF_PosOri_R);
    
    
    // GB_cfg << 0,0,0.5,0,0,0;
    // GF_cfg_L << 0,0.05,0.2,0,0,0;
    // GF_cfg_R << 0,-0.05,0.2,0,0,0;
    // Geometric_IK_L(GB_cfg, GF_cfg_L);
    // Geometric_IK_R(GB_cfg, GF_cfg_R);
        
//    tmp_v = tmp_m.block(0,3,3,1);
}

double CKubot::cosWave(double amp, double period, double time, double int_pos)
{
   return (amp / 2)*(1 - cos(PI / period * time)) + int_pos;
}


void CKubot::HomePose(float return_time)
{
    /* HomePose, Return to HomePose(Humanoid Robots)
     * input : time while the robot acts Homepose posture,
     * 
     * main processing:  
     * 1. Homepose
     */
    static float re_time = 0;
    static float init_joint_angle[12];


    if (re_time == 0 && Move_current == false) {
        for (int j = 0; j < 12; j++) {
            init_joint_angle[j] = refAngle[j];
        }
    }

    if (re_time <= return_time && Move_current == false) {
        for (int j = 0; j < 12; j++) {
           refAngle[j] = cosWave(-init_joint_angle[j], return_time, re_time, init_joint_angle[j]); // 여기서 값을 다시 부여함 ㅇㅇ
                        // -> init_joint_angle -> 0 까지(-init_joint_angle 만큼 이동하세요 명령)
        }
        re_time += tasktime;
    }
    else {
        re_time = 0;
        Move_current = true;
    }
}//base와 관절의 궤적제어가 동시에 들어가고있다.
// 


void CKubot::walkingReady(float readytime)
{
    /* walkingReady
     * input : time while the robot acts ready posture,
     *
     * main processing:  
     * 1. Walking Ready
     */

    static float time = 0;
    static float currentAngle[12]; // Now nDoF = 13

    if (time == 0 && Move_current == false) {
        for (int j = 0; j < 12; j++) {
            currentAngle[j] = refAngle[j];
        }
    }

    if (time < readytime && Move_current == false) {
        for (int j = 0; j < 12; j++) {
            refAngle[j] = cosWave(walkReadyAngle[j] - currentAngle[j], readytime, time, currentAngle[j]);
        }
        time += tasktime;
    }
    else {
        time = 0;
        Move_current = true;
    }

}


void CKubot::setWalkingReadyPos(double init_x, double init_y, double init_z)
{
    /* setWalkingReadyPos, Set CoM pos for Walking Ready(Humanoid Robots)
     * input : Init Positions [X, Y, Z][unit:m] of CoM for walking ready,
     * 
     * main processing:  
     * 1. Setting CoM Position when walking Ready
     */
    VectorXd Base_pos(6);


    Base_pos(0) = init_x;
    Base_pos(1) = init_y;
    Base_pos(2) = init_z;
    Base_pos(3) = 0;
    Base_pos(4) = 0;
    Base_pos(5) = 0;

    VectorXd init_LFoot(6), init_RFoot(6);
    init_LFoot << 0,0.05,0,0,0,0;
    init_RFoot << 0,-0.05,0,0,0,0;

    VectorXd IK_walkreadyAngle(12);

    IK_walkreadyAngle << Geometric_IK_L(Base_pos, init_LFoot),Geometric_IK_R(Base_pos, init_RFoot);

    // std::cout << "IK_walkreadyAngle:\n" << IK_walkreadyAngle << std::endl;

        for (int j = 0; j < 12; j++) {

            walkReadyAngle[j] = IK_walkreadyAngle(j);
            // printf(C_CYAN "The Robot's %d Joint for Walking Ready = %f\n" C_RESET, j, joint[j].walkReadyAngle * R2D);
        }
    
}

void CKubot::sway(){

    printf("==== ref_setting_start ====\n");

        zmp.previewControl.X.ref.resize(3, zmp.previewControl.RefTotalTrajSize);
        zmp.previewControl.Y.ref.resize(3, zmp.previewControl.RefTotalTrajSize);

        for (int i = 0; i < zmp.previewControl.RefTotalTrajSize; i++) {
        // double time = i * tasktime;  // time은 0초에서 (15001-1)*0.001 = 15초 까지
        if (i < 2500) {
            zmp.previewControl.X.ref.col(i) << 0,\
                                        0,\
                                        0;
            zmp.previewControl.Y.ref.col(i) << 0,\
                                        0,\
                                        0;                           
        }
        else if (i >=2500 && i < 4500) {
            zmp.previewControl.X.ref.col(i) << 0,\
                                        0,\
                                        0;
            zmp.previewControl.Y.ref.col(i) << 0.04,\
                                        0,\
                                        0;

        }
        else if (i >= 4500 && i < 6500) {
            zmp.previewControl.X.ref.col(i) << 0,\
                                        0,\
                                        0;
            zmp.previewControl.Y.ref.col(i) << -0.04,\
                                        0,\
                                        0;

        }
        else if (i >= 6500 && i < 8500) {
            zmp.previewControl.X.ref.col(i) << 0,\
                                        0,\
                                        0;
            zmp.previewControl.Y.ref.col(i) << 0.04,\
                                        0,\
                                        0;

        }
        else if (i >= 8500 && i < 10500) {
            zmp.previewControl.X.ref.col(i) << 0,\
                                        0,\
                                        0;
            zmp.previewControl.Y.ref.col(i) << -0.04,\
                                        0,\
                                        0;

        }
        else if (i >= 10500 && i < 12500) {
            zmp.previewControl.X.ref.col(i) << 0,\
                                        0,\
                                        0;
            zmp.previewControl.Y.ref.col(i) << 0.04,\
                                        0,\
                                        0;

        }
        else if (i >= 12500 && i < 14500) {
            zmp.previewControl.X.ref.col(i) << 0,\
                                        0,\
                                        0;
            zmp.previewControl.Y.ref.col(i) << -0.04,\
                                        0,\
                                        0;

        }
        else if (i >= 14500 && i < 16500) {
            zmp.previewControl.X.ref.col(i) << 0,\
                                        0,\
                                        0;
            zmp.previewControl.Y.ref.col(i) << 0.04,\
                                        0,\
                                        0;

        }
        else {
            zmp.previewControl.X.ref.col(i) << 0,\
                                        0,\
                                        0;
            zmp.previewControl.Y.ref.col(i) << 0,\
                                        0,\
                                        0;

        }

    }
        printf("==== ref_setting_end ====");
}

void CKubot::initializeBkubot() {

    /* initialize CRobot class's variable
     * initializeFTSensorValue();
     * initializeIMUSensorValue();
     * initializeSystemID(38.9410,555,zmp.observer.disturbanceExist); // kg, mm
     * initializeZmpControl(zmp.observer.disturbanceExist);
     */


    Base.current = VectorXd::Zero(6);
    Base.vel     = VectorXd::Zero(6);
    Base.acc     = VectorXd::Zero(6);
    Base.pre     = VectorXd::Zero(6);
    Base.prevel  = VectorXd::Zero(6);
    Base.preacc  = VectorXd::Zero(6);

    Base.refpos    = VectorXd::Zero(6);
    Base.refvel    = VectorXd::Zero(6);
    Base.refacc    = VectorXd::Zero(6);
    Base.refquat   = VectorXd::Zero(4);
    Base.refquat(0)= 1;

    Base.prerefpos = VectorXd::Zero(6);
    Base.prerefvel = VectorXd::Zero(6);
    Base.prerefacc = VectorXd::Zero(6);


    // walking.step.Lfoot = MatrixXd::Zero(PatternElement,PatternPlan/2);
    // walking.step.Rfoot = MatrixXd::Zero(PatternElement,PatternPlan/2);
    // walking.step.preLfoot = VectorXd::Zero(PatternElement);
    // walking.step.preRfoot = VectorXd::Zero(PatternElement);
    // walking.step.preLfoot = VectorXd::Zero(PatternElement);
    walking.step.prefootCenter = VectorXd::Zero(PatternElement);
    // walking.step.LastLfoot = VectorXd::Zero(PatternElement);
    // walking.step.LastRfoot = VectorXd::Zero(PatternElement);
    walking.step.LastfootCenter = VectorXd::Zero(PatternElement);
    // // variable for Pattern plan



    walking.Ready = false;
    walking.walk  = false;
    walking.SP = walking.DSP;
    walking.old_SP = walking.DSP;
    walking.SP_check = walking.DSP;
    walking.SP_oldcheck = walking.DSP;
    walking.SP_checking = 0;

    walking.ft_SP = walking.DSP;
    walking.old_ft_SP = walking.DSP;
    walking.walk_SP = walking.DSP;
    walking.old_walk_SP = walking.DSP;

    walking.NewfootPlan = false;

    walking.time.readytime = 0;
    walking.time.sec = 0;


    zmp.previewControl.setcount = 0;
    // // Walking init

    init_t = 0;

    // mode = POSITIONCONTROLMODE;
    // Mode

    zmp.controller.ON = true;
    zmp.controller.OFF = false;
    zmp.controller.gainchangeflag = false;
    //ZMP control, observer


   
    setZmpPreview(15);
    // initializeQP();

    body.BASE_TO_LEG_Y = 50.0; //mm
    body.BASE_TO_LEG_Z = 104.64; // mm
    body.THIGH_LENGTH = 138.0; //
    body.CALF_LENGTH = 143.0; //mm
    body.ANKLE_LENGTH = 40; //mm
    body.LEG_LENGTH = body.THIGH_LENGTH + body.CALF_LENGTH + body.ANKLE_LENGTH; //mm (THIGH_LENGTH + CALF_LENGTH + ANKLE_LENGTH)
    body.LEG_SIDE_OFFSET = body.BASE_TO_LEG_Y; // BASE_TO_LEG_Y

    body.foot_x_length = 150; // mm
    body.foot_y_length = 100; // mm


    walking.initialize = false;


    // checkCRobot();
}

void CKubot::setZmpPreview(double previewTime){

    /* setZmpPreview
     * input : preview time for ZMP preview control
     * output : previewTime --> setZmpPreviewTime [Crobot class's Function]
     *          Setting variable for ZMP preview control
     *
     * Before run function
     * [First, Run once Crobot class's Function initializeSystemID when starting program]
     */

    std::cout<<"===== Setting ZMP preview control ====="<<std::endl;


    setZmpPreviewTime(previewTime);

    double T = (double)tasktime;

    zmp.previewControl.A = MatrixXd::Zero(3, 3);
    zmp.previewControl.A << 1, T, T*T/2.0\
                           ,0, 1, T\
                           ,0, 0, 1;


    zmp.previewControl.B = MatrixXd::Zero(3, 1);
    zmp.previewControl.B << T*T*T/6.0\
                           ,T*T/2.0\
                           ,T;

    zmp.previewControl.C = MatrixXd::Zero(1, 3);
    // zmp.previewControl.C << 1 ,0 ,-(systemID.robotH*0.34)/9.81;
    zmp.previewControl.C << 1 ,0 ,-(0.34*1.1)/9.81;

    zmp.previewControl.B_Tilde = MatrixXd::Zero(4, 1);
    zmp.previewControl.B_Tilde.block(0, 0, 1, 1) = zmp.previewControl.C* zmp.previewControl.B;
    zmp.previewControl.B_Tilde.block(1, 0, 3, 1) = zmp.previewControl.B;

    zmp.previewControl.I_Tilde = MatrixXd::Zero(4, 1);
    zmp.previewControl.I_Tilde << 1, 0, 0, 0;

    zmp.previewControl.F_Tilde = MatrixXd::Zero(4, 3);
    zmp.previewControl.F_Tilde.block(0, 0, 1, 3) = zmp.previewControl.C * zmp.previewControl.A;
    zmp.previewControl.F_Tilde.block(1, 0, 3, 3) = zmp.previewControl.A;


    zmp.previewControl.Q_Tilde = MatrixXd::Zero(4, 4);

    zmp.previewControl.Q_Tilde.block(0, 0, 1, 1) = MatrixXd::Identity(1, 1);
    zmp.previewControl.Q_Tilde.block(1, 1, 3, 3) = MatrixXd::Zero(3, 3);

    zmp.previewControl.R = MatrixXd::Identity(1, 1)*0.000001;

    zmp.previewControl.A_Tilde = MatrixXd::Zero(4, 4);
    zmp.previewControl.A_Tilde.block(0, 0, 4, 1) = zmp.previewControl.I_Tilde;
    zmp.previewControl.A_Tilde.block(0, 1, 4, 3) = zmp.previewControl.F_Tilde;

    zmp.previewControl.K_Tilde = MatrixXd::Zero(4, 4);
    zmp.previewControl.K_Tilde = ZMP_DARE(zmp.previewControl.A_Tilde, zmp.previewControl.B_Tilde, zmp.previewControl.Q_Tilde, zmp.previewControl.R);

    zmp.previewControl.G_I = MatrixXd::Zero(1, 1);
    zmp.previewControl.G_I = (zmp.previewControl.R + zmp.previewControl.B_Tilde.transpose()*zmp.previewControl.K_Tilde*zmp.previewControl.B_Tilde).inverse()*zmp.previewControl.B_Tilde.transpose()*zmp.previewControl.K_Tilde*zmp.previewControl.I_Tilde;

    zmp.previewControl.G_P = MatrixXd::Zero(1, zmp.previewControl.PreviewTimeSize);

    zmp.previewControl.G_X = MatrixXd::Zero(1, 3);
    zmp.previewControl.G_X = (zmp.previewControl.R + zmp.previewControl.B_Tilde.transpose()*zmp.previewControl.K_Tilde*zmp.previewControl.B_Tilde).inverse()*zmp.previewControl.B_Tilde.transpose()*zmp.previewControl.K_Tilde*zmp.previewControl.F_Tilde;

    zmp.previewControl.A_Tilde_c = MatrixXd::Zero(4, 4);
    zmp.previewControl.A_Tilde_c = zmp.previewControl.A_Tilde - zmp.previewControl.B_Tilde*(zmp.previewControl.R + zmp.previewControl.B_Tilde.transpose()*zmp.previewControl.K_Tilde*zmp.previewControl.B_Tilde).inverse()*zmp.previewControl.B_Tilde.transpose()*zmp.previewControl.K_Tilde*zmp.previewControl.A_Tilde;

    zmp.previewControl.X_Tilde = MatrixXd::Zero(4, 1);

    for (int pre_cnt = 0; pre_cnt < zmp.previewControl.PreviewTimeSize; pre_cnt++) {
        if (pre_cnt == 0) {
            zmp.previewControl.X_Tilde = -zmp.previewControl.A_Tilde_c.transpose() * zmp.previewControl.K_Tilde * zmp.previewControl.I_Tilde;
            zmp.previewControl.G_P(0, pre_cnt) = -zmp.previewControl.G_I(0, 0);
        }
        else {
            zmp.previewControl.X_Tilde = zmp.previewControl.A_Tilde_c.transpose() * zmp.previewControl.X_Tilde;
            zmp.previewControl.G_P(0, pre_cnt) = ((zmp.previewControl.R + zmp.previewControl.B_Tilde.transpose() * zmp.previewControl.K_Tilde * zmp.previewControl.B_Tilde).inverse() * zmp.previewControl.B_Tilde.transpose() * zmp.previewControl.X_Tilde)(0,0);
        }
    }

/////////////////////////////////////////////////////////////////////////////////////////////////////////
    zmp.previewControl.Y.state = MatrixXd::Zero(3, 1);	//3X1 matrix
    zmp.previewControl.Y.E = 0;

    if(CommandFlag == SWAYMOTION)
    {
        zmp.previewControl.Y.sum_E = zmp.previewControl.Y.sum_E;

    }
    else
    {
        zmp.previewControl.Y.sum_E = 0;
        zmp.previewControl.Y.sum_P = 0;
        zmp.previewControl.Y.U = 0;
    }


    // zmp.previewControl.X.ref = MatrixXd::Zero(1, (int)(zmp.previewControl.RefTotalTrajSize+0.001));
    // zmp.previewControl.X.m_ref = walking.step.LastfootCenter(X);
    zmp.previewControl.X.state = MatrixXd::Zero(3, 1);	//3X1 matrix
    zmp.previewControl.X.E = 0;

    if(CommandFlag == SWAYMOTION)
    {
        zmp.previewControl.X.sum_E = zmp.previewControl.X.sum_E;

    }
    else
    {   
        zmp.previewControl.X.sum_E = 0;
        zmp.previewControl.X.sum_P = 0;
        zmp.previewControl.X.U = 0;
    }


    zmp.previewControl.Y.new_state = MatrixXd::Zero(3, 1);
    zmp.previewControl.Y.old_zmp = walking.step.LastfootCenter(Y);
    zmp.previewControl.Y.CoM = walking.step.LastfootCenter(Y);
    zmp.previewControl.Y.new_state(0) = zmp.previewControl.Y.CoM;
    zmp.previewControl.Y.dCoM = 0;

    zmp.previewControl.X.new_state = MatrixXd::Zero(3, 1);
    zmp.previewControl.X.old_zmp = walking.step.LastfootCenter(X);
    zmp.previewControl.X.CoM = walking.step.LastfootCenter(X);
    zmp.previewControl.X.new_state(0) = zmp.previewControl.X.CoM;
    zmp.previewControl.X.dCoM = 0;

    std::cout<<"===== Setting End ZMP preview control ====="<<std::endl;

    // zmp.previewControl.count = 0;

}

void CKubot::setZmpPreviewTime(double previewTime){

    /* setZmpPreviewTime
     * input : preview time for ZMP preview control
     * output : preview time, array about preview time for ZMP preview control,
     *          total ZMP trajectory array and time of the trajectory considering zmp preview time
     *
     * Before run function
     * [First, Run Crobot class's Function setWalkingTime before running this function]
     */

    zmp.previewControl.PreviewTime = previewTime;
    zmp.previewControl.PreviewReadyTime = 2.0;

    zmp.previewControl.PreviewTimeSize = zmp.previewControl.PreviewTime*(double)onesecSize;
    zmp.previewControl.PeriodTimeSize = walking.time.periodTime*(double)onesecSize;  //한 발자국 걸을 때 걸리는 시간

    // zmp.previewControl.RefTotalTrajSize = (zmp.previewControl.PreviewReadyTime\
    //                                        + walking.step.total * walking.time.periodTime\
    //                                        + 3*zmp.previewControl.PreviewTime)*(double)onesecSize;
    //2+total*periodTime + 3*1.5*onesecSize(100)

    zmp.previewControl.RefTotalTime = zmp.previewControl.RefTotalTrajSize/(double)onesecSize;

    std::cout<<"RefTotalTrajSize :"<<zmp.previewControl.RefTotalTrajSize<<std::endl;
    std::cout<<"RefTotalTrajTime :"<<zmp.previewControl.RefTotalTime<<std::endl;

   printf("%lf\t%lf\t%lf\t%lf\n",zmp.previewControl.PreviewTime,zmp.previewControl.PreviewTimeSize,zmp.previewControl.RefTotalTrajSize,zmp.previewControl.RefTotalTime);
}

MatrixXd CKubot::ZMP_DARE(MatrixXd& A, MatrixXd& B, MatrixXd& Q, MatrixXd& R)//Kookmin.Univ Preview
{
    unsigned int nSize = A.rows();
    MatrixXd Z(nSize* 2, nSize* 2);

    Z.block(0, 0, nSize, nSize) = A+ B* R.inverse()* B.transpose()* (A.inverse()).transpose() * Q;
    Z.block(0, nSize, nSize, nSize) = -B* R.inverse()* B.transpose()* (A.inverse()).transpose();
    Z.block(nSize, 0, nSize, nSize) = -(A.inverse()).transpose()* Q;
    Z.block(nSize, nSize, nSize, nSize) = (A.inverse()).transpose();

     eZMPSolver.compute(Z, true);

    Eigen::MatrixXcd U(nSize* 2, nSize);
    unsigned int j=0;
    for (unsigned int i=0; i<nSize* 2; i++)
    {
        std::complex<double> eigenvalue = eZMPSolver.eigenvalues()[i];
        double dReal = eigenvalue.real();
        double dImag = eigenvalue.imag();

        if( std::sqrt((dReal* dReal) + (dImag* dImag)) < 1.0)
        {
            U.block(0, j, nSize* 2, 1) = eZMPSolver.eigenvectors().col(i);
            j++;
        }
    }
    if(j != nSize)
    {
        printf("Warning! ******* Pelvis Planning *******\n");
    }

    Eigen::MatrixXcd U1 = U.block(0, 0, nSize, nSize);
    Eigen::MatrixXcd U2 = U.block(nSize, 0, nSize, nSize);

    Eigen::MatrixXcd X = U2 * U1.inverse();

    return X.real();
}

void CKubot::walkingPatternGenerator(int controlMODE, int CommandFLAG, double steps, double FBsize, double LRsize, double Turnsize)
{
    
    if(controlMODE == CTRLMODE_LRMOTION)
    {// printf("==walking generate start =====\n");
        if(CommandFLAG == LRMOTION_PATTERN)
        {

            walking.initialize = true;

            setZmpPreview(1.5);

            walking.walk  = true;
            CommandFlag = LRMOTION;
        }
        else if(CommandFLAG == LRMOTION)
        {
            walking.initialize = false;

            zmpPreviewControl();

            // generateFootTraj_ver2(walking.step.Lfoot,walking.step.Rfoot);

            // if(walking.NewfootPlan == true)
            // {
            //     // changeWalkingStep(FBsize,LRsize,Turnsize);
            //     // FootPlannerWindowMove();
            //     // generateZmpTraj(walking.step.Lfoot,walking.step.Rfoot);
            //     walking.NewfootPlan = false;
            // }

            if(zmp.previewControl.count > zmp.previewControl.RefTotalTrajSize)
            {
                LRMOTION_READY              = false;
                CommandFlag             = NONE_ACT;

                printf("=========legMotion End========\n");
                printf("=========legMotion Ready========\n");
            }

        }

    }

    
    if(controlMODE == CTRLMODE_SWAY)
    {// printf("==walking generate start =====\n");
        if(CommandFLAG == SWAYMOTION_PATTERN)
        {
            // printf("=========walkingPatternGenerator check========\n");

            // initializeGlobalPosition();
            walking.initialize = true;
            // setWalkingStep(7,walking.step.S_L_F,0,0,0,50);
            // setWalkingTime(2.0,0.2);
            setZmpPreview(1.5);
            // FootPlanner();
            // generateZmpTraj(walking.step.Lfoot,walking.step.Rfoot);
            walking.walk  = true;
            CommandFlag = SWAYMOTION;
        }
        else if(CommandFLAG == SWAYMOTION)
        {
            walking.initialize = false;

            zmpPreviewControl();

            // generateFootTraj_ver2(walking.step.Lfoot,walking.step.Rfoot);

            // if(walking.NewfootPlan == true)
            // {
            //     // changeWalkingStep(FBsize,LRsize,Turnsize);
            //     // FootPlannerWindowMove();
            //     // generateZmpTraj(walking.step.Lfoot,walking.step.Rfoot);
            //     walking.NewfootPlan = false;
            // }

            if(zmp.previewControl.count > zmp.previewControl.RefTotalTrajSize)
            {
                SWAY_READY              = false;
                CommandFlag             = NONE_ACT;

                printf("=========SWAY End========\n");
                printf("=========SWAY Ready========\n");
            }

//            printf("=========Walking========\n");
        }

    }


}


void CKubot::zmpPreviewControl()
{

    /* zmpPreviewControl
     * output : trajectory by zmp preview control --> zmp.previewControl.X.CoM, zmp.previewControl.Y.CoM [Crobot class's Variable]
     *
     * Before run function
     * [First, Run once Crobot class's Function initializeSystemID when starting program]
     * [Second, Run Crobot class's Function setZmpPreview before running this function]
     * [Third, Run Crobot class's Function zmp plan(ex setZmpSwapPlan) before running this function]
     */

    
            // printf("=========zmpPreviewControl check========\n");

    if(zmp.previewControl.count < zmp.previewControl.RefTotalTrajSize - zmp.previewControl.PreviewTimeSize) {
            //printf("\n if 1 ====== \n");
        zmp.previewControl.Y.state = zmp.previewControl.Y.new_state;
        zmp.previewControl.X.state = zmp.previewControl.X.new_state;

        zmp.previewControl.Y.E = zmp.previewControl.Y.old_zmp - zmp.previewControl.Y.ref(0, zmp.previewControl.count);
        zmp.previewControl.X.E = zmp.previewControl.X.old_zmp - zmp.previewControl.X.ref(0, zmp.previewControl.count);

//        std::cout<<"zmp.previewControl.Y.E :"<<zmp.previewControl.Y.E<<std::endl;
//        std::cout<<"zmp.previewControl.X.E :"<<zmp.previewControl.X.E<<std::endl;

        zmp.previewControl.Y.m_ref = zmp.previewControl.Y.ref(0, zmp.previewControl.count);
        zmp.previewControl.X.m_ref = zmp.previewControl.X.ref(0, zmp.previewControl.count);

        zmp.previewControl.Y.sum_E = zmp.previewControl.Y.sum_E + zmp.previewControl.Y.E;
        zmp.previewControl.X.sum_E = zmp.previewControl.X.sum_E + zmp.previewControl.X.E;
        //printf("\n if 2222 ====== \n");

        for (int pre_cnt = 0; pre_cnt < zmp.previewControl.PreviewTimeSize; pre_cnt++) {
            //printf("\n pre_cnt : %d\n", pre_cnt);
            zmp.previewControl.Y.sum_P = zmp.previewControl.Y.sum_P + zmp.previewControl.G_P(0, pre_cnt) * zmp.previewControl.Y.ref(0, zmp.previewControl.count + pre_cnt);
            zmp.previewControl.X.sum_P = zmp.previewControl.X.sum_P + zmp.previewControl.G_P(0, pre_cnt) * zmp.previewControl.X.ref(0, zmp.previewControl.count + pre_cnt);
        }
            //printf("\n if 33333333333 ====== \n");
        zmp.previewControl.X.U = -zmp.previewControl.G_I(0, 0) * zmp.previewControl.X.sum_E - (zmp.previewControl.G_X * zmp.previewControl.X.state)(0, 0) - zmp.previewControl.X.sum_P;
        zmp.previewControl.Y.U = -zmp.previewControl.G_I(0, 0) * zmp.previewControl.Y.sum_E - (zmp.previewControl.G_X * zmp.previewControl.Y.state)(0, 0) - zmp.previewControl.Y.sum_P;

        zmp.previewControl.Y.new_state = zmp.previewControl.A * zmp.previewControl.Y.state + zmp.previewControl.B * zmp.previewControl.Y.U;
        zmp.previewControl.X.new_state = zmp.previewControl.A * zmp.previewControl.X.state + zmp.previewControl.B * zmp.previewControl.X.U;

        zmp.previewControl.Y.old_zmp = (zmp.previewControl.C * zmp.previewControl.Y.state)(0, 0);
        zmp.previewControl.X.old_zmp = (zmp.previewControl.C * zmp.previewControl.X.state)(0, 0);

        zmp.previewControl.Y.CoM = zmp.previewControl.Y.new_state(0, 0);
        zmp.previewControl.Y.dCoM = zmp.previewControl.Y.new_state(1, 0);
        zmp.previewControl.X.CoM = zmp.previewControl.X.new_state(0, 0);
        zmp.previewControl.X.dCoM = zmp.previewControl.X.new_state(1, 0);
        // printf("\n XY_com : %lf \n", zmp.previewControl.Y.CoM);

        zmp.previewControl.Y.sum_P = 0;
        zmp.previewControl.X.sum_P = 0;

        // cout << "y_com_testprint: \n" << zmp.previewControl.Y.CoM << endl;

    }
    else{

    }

}

MatrixXd CKubot::ZMPFK(VectorXd q)
{
//  Extract position vector(3x1) 
    
    VectorXd tmp_v = VectorXd::Zero(3);
    MatrixXd tmp_m(4,4);
    // MatrixXd tmp_fk(4,4);

    

    tmp_m = getTransform6E()*
            jointToTransform56(q).inverse()* 
            jointToTransform45(q).inverse()* 
            jointToTransform34(q).inverse()* 
            jointToTransform23(q).inverse()* 
            jointToTransform12(q).inverse()* 
            jointToTransform01_zmp(q).inverse()* 
            getTransformI0().inverse();
        
//    tmp_v = tmp_m.block(0,3,3,1);
    //tmp_fk = tmp_m.inverse();

    tmp_v(0) = tmp_m(0,3);
    tmp_v(1) = tmp_m(1,3);
    tmp_v(2) = tmp_m(2,3);
    
    return tmp_v;
    // std::cout << "FK_jointToPosition: " << tmp_m << std::endl;


}

MatrixXd CKubot::jointToTransform01_zmp(VectorXd q)
{
//  Frame 1 to Frame 0
// q: generalized coordinates, q = [q1;q2;q3;q4;q5;q6]    

    MatrixXd tmp_m(4,4);
    double qq = q(0);
   
    tmp_m << cos(qq), -sin(qq), 0, 0, \
             sin(qq),  cos(qq), 0, 0, \
             0,        0,       1, -0.10464, \
             0,        0,       0, 1;
    
    return tmp_m;    
}


void CKubot::legMotion(){

        printf("==== RF ref_setting_start ====");
        VectorXd GB_Vector(6);

        double ref_F_x =0;
        double ref_F_y =0;
        double ref_F_z =0;


        // VectorXd GFL_Vector(6);
        // GFL_Vector << 0, 0.05, 0, 0, 0, 0;

        // VectorXd GFR_Vector(6);
        // GFR_Vector << 0, -0.05, 0, 0, 0, 0;

        double z_height= 0.05;

        double T = 5000;

        zmp.previewControl.X.ref.resize(3, zmp.previewControl.RefTotalTrajSize);
        zmp.previewControl.Y.ref.resize(3, zmp.previewControl.RefTotalTrajSize);


        for (int i = 0; i < zmp.previewControl.RefTotalTrajSize; i++) {
        // double time = i * tasktime;  // time은 0초에서 (19451-1)*0.001 = 15초 까지
        if (i < 2500) {
            zmp.previewControl.X.ref.col(i) << 0,\
                                        0,\
                                        0;
            zmp.previewControl.Y.ref.col(i) << 0,\
                                        0,\
                                        0;

        }
        else if (i >=2500 && i <15000) {
            zmp.previewControl.X.ref.col(i) << 0,\
                                        0,\
                                        0;
            zmp.previewControl.Y.ref.col(i) << 0.05,\
                                        0,\
                                        0;

        }
        // else if(i >= 7000){
        //     zmp.previewControl.X.ref.col(i) << 0,\
        //                                 0,\
        //                                 0;
        //     zmp.previewControl.Y.ref.col(i) << 0.05,\
        //                                 0,\
        //                                 0.0;
        //     ref_F_z = cosWave(z_height, T, updown_cnt, 0.0);
        //     updown_cnt++;
        //     printf("\n%d\n",updown_cnt);
        //     GFRR_Vector.resize(6);
        //     GFRR_Vector << 0,-0.05,ref_F_z,0,0,0;

        //     // printf("==== 승훈이가 시킨 init 확인 ====");

        // }

        else if (i >= 15000) {
            zmp.previewControl.X.ref.col(i) << 0,\
                                        0,\
                                        0;
            zmp.previewControl.Y.ref.col(i) << 0,\
                                        0,\
                                        0;

        }


        // else {
        //     zmp.previewControl.X.ref.col(i) << 0,\
        //                                 0,\
        //                                 0;
        //     zmp.previewControl.Y.ref.col(i) << 0.05,\
        //                                 0,\
        //                                 0;
            // ref_F_z = cosWave(z_height, T, updown_cnt, 0.0);
            // updown_cnt++;
            // printf("\n%d\n",updown_cnt);
            // GFRR_Vector.resize(6);
            // GFRR_Vector << 0,-0.05,ref_F_z,0,0,0;

            // printf("==== 승훈이가 시킨 init 확인 ====");

        // }


    }
        printf("==== RF ref_setting_end ====");
}


// void CKubot::CStand(){

//         printf("==== center_stand_start ====");


//         zmp.previewControl.X.ref.resize(3, zmp.previewControl.RefTotalTrajSize);
//         zmp.previewControl.Y.ref.resize(3, zmp.previewControl.RefTotalTrajSize);


//         for (int i = 0; i < 5000; i++) {
//         // double time = i * tasktime;  // time은 0초에서 (19451-1)*0.001 = 15초 까지

//             zmp.previewControl.X.ref.col(i) << 0,\
//                                         0,\
//                                         0;
//             zmp.previewControl.Y.ref.col(i) << 0,\
//                                         0,\
//                                         0;

//         }

//         printf("==== center_stand_end ====");
// }