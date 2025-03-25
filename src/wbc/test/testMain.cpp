
#include <iostream> 
#include <functional> 
#include "../include/mujoco/mujoco.h"
#include <GLFW/glfw3.h>
#include <cstdio>
#include "../mjsim/GLFW_callbacks.h"
#include "../mjsim/MJ_interface.h"
#include "../robotData/robotData.h"
#include "../robotData/robotStructs.h"
#include "../algorithm/qp.h"
// #include <OsqpEigen/OsqpEigen.h>
#include "../include/OsqpEigen/OsqpEigen.h"
#include <iostream>

#include <fstream>
using namespace std;
class tempTest
{
private:
    /* data */
public:
     tempTest(/* args */);
    ~tempTest();
    Eigen::Matrix3d skew(Eigen::VectorXd& v);
    // Eigen::VectorXd wqp();
    Eigen::VectorXd wqp_new();
    Eigen::Matrix<double, 3, 3> eul2Rot(double roll, double pitch, double yaw);
    Eigen::Matrix<double, 3, 1> diffRot( Eigen::Matrix3d &Rcur, Eigen::Matrix3d &Rdes);
    int aaa = 0;
    ofstream fileCom;
    ofstream fileHand;
    ofstream fileWaist;
    RobotData *robot;

    // 获取机器人状态
    RobotStructs state;

};
Eigen::Matrix<double, 3, 3> tempTest::eul2Rot(double roll, double pitch, double yaw) {
    Eigen::Matrix<double,3,3> Rx,Ry,Rz;
    Rz<<cos(yaw),-sin(yaw),0,
            sin(yaw),cos(yaw),0,
            0,0,1;
    Ry<<cos(pitch),0,sin(pitch),
            0,1,0,
            -sin(pitch),0,cos(pitch);
    Rx<<1,0,0,
            0,cos(roll),-sin(roll),
            0,sin(roll),cos(roll);
    return Rz*Ry*Rx;
}

Eigen::Matrix<double, 3, 1> tempTest::diffRot( Eigen::Matrix3d &Rcur, Eigen::Matrix3d &Rdes) {
    Eigen::Matrix3d R = Rcur.transpose() * Rdes;
    Eigen::Vector3d w;

    if (R.isDiagonal(1e-5) && fabs(R(0, 0)) + fabs(R(1, 1)) + fabs(R(2, 2)) - 3 < 1e-3) {
        w.setZero();
    } else if (R.isDiagonal(1e-5)) {
        w << R(0, 0) + 1, R(1, 1) + 1, R(2, 2) + 1;
        w = w * 3.1415 / 2.0;
    } else {
        Eigen::Vector3d l;
        l << R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1);
        double sita = atan2(l.norm(), R(0, 0) + R(1, 1) + R(2, 2) - 1);
        w = sita * l / l.norm();
    }
    w = Rcur * w;
    return w;
}

Eigen::VectorXd tempTest::wqp_new()
{
    //ax =b  x =  tau(20)  qdd(27)  Fr 3d * 8
    //ax =b  x =  tau(20)  qdd(27)  Fr 6d * 2

    double M = 70.7394 ;
    int nq = 27;
    state = robot->getRobotState();
    // std::cout<<"===state=="<<state.qd(10)<<std::endl;

    Eigen::MatrixXd S  = Eigen:: MatrixXd::Zero(21,27);
    S.block<21,21>(0,6) = Eigen:: MatrixXd::Identity(21,21);

    Eigen::MatrixXd S1  = Eigen:: MatrixXd::Identity(27,27);
    S1.block<21,21>(6,6) = Eigen:: MatrixXd::Zero(21,21);

    Eigen::MatrixXd Mq = Eigen::MatrixXd::Identity(27,27);
    Eigen::MatrixXd Cq = Eigen::MatrixXd::Identity(27,27);
    Eigen::MatrixXd Gq = Eigen::VectorXd::Zero(27);
    Mq = S1*state.DynM;
    Cq = S1*state.DynC;
    Gq = S1*state.DynG;
    // std::cout<<"===wqp002=="<<std::endl;
    //52-6+12 = 58
    Eigen::MatrixXd A  = Eigen:: MatrixXd::Zero(nq,60);
    A.block<27,21>(0,0) = -S.transpose();
    A.block<27,27>(0,21) = Mq;
    A.block<27, 6>(0,48) = - S1 * state.JLFoot1.block<6,27>(0,0).transpose();
    A.block<27, 6>(0,54) = - S1 * state.JRFoot1.block<6,27>(0,0).transpose();
    Eigen::VectorXd b = Eigen::VectorXd::Zero(nq);
    b =(Cq * state.qd + Gq);
    Eigen::MatrixXd H1 = Eigen:: MatrixXd::Zero(12,60);
    H1.block<3+3,27>(0,21) = state.JLFoot1;
    H1.block<3+3,27>(6,21) = state.JRFoot1;
    Eigen::VectorXd c1 = Eigen:: VectorXd::Zero(12);
    c1.head(6) =  -state.dJLFoot1*state.qd;
    c1.tail(6) =  -state.dJRFoot1*state.qd;
    
    //  w3  质心xy跟
    Eigen::MatrixXd H3 = Eigen:: MatrixXd::Zero(3,60);
    auto j3 = state.Jcom.block<3,27>(0,0);
    H3.block<3,27>(0,21) = j3;
    auto tempAg_m = state.Ag;
    auto tempdAg_m = state.dAg;
    tempAg_m.block<3,27>(0,0) =  tempAg_m.block<3,27>(0,0)/M;
    tempdAg_m.block<3,27>(0,0) =  tempdAg_m.block<3,27>(0,0)/M;
    Eigen::VectorXd c3 = Eigen:: VectorXd::Zero(3);
    Eigen::VectorXd b0 = Eigen:: VectorXd::Zero(3);
    Eigen::VectorXd bv0 = Eigen:: VectorXd::Zero(3);
    bv0<<0.0,0.0,0;//+0.051*0.15*sin(0.051*aaa);
    b0<<0.000,0.00,-0.2;//+0.15*sin(0.051*aaa);
    auto tempJcom = state.Jcom.block<3,27>(0,0);
    c3(0) = 750*(b0(0) - state.Xcom(0)) + 35 * ( bv0(0)- tempJcom.block<1,27>(0,0)*state.qd) - tempdAg_m.block<1,27>(0,0)*state.qd;
    c3(1) = 750*(b0(1) - state.Xcom(1)) + 35 * ( bv0(1)- tempJcom.block<1,27>(1,0)*state.qd) - tempdAg_m.block<1,27>(1,0)*state.qd;
    c3(2) = 750*(b0(2) - state.Xcom(2)) + 35 * ( bv0(2)- tempJcom.block<1,27>(2,0)*state.qd) - tempdAg_m.block<1,27>(2,0)*state.qd;
    std::cout<<"===eCom=="<<(b0-state.Xcom.segment(0,3)).transpose()<<std::endl;
    auto ecom = b0-state.Xcom.segment(0,3);
    fileCom<<ecom(0)<<";"<<ecom(1)<<";"<<ecom(2)<<";"<<std::endl;
    Eigen::MatrixXd H7 = Eigen:: MatrixXd::Zero(3,60);

    //w7
    H7.block<3,27>(0,21) = tempAg_m.block<3,27>(3,0);
    Eigen::VectorXd c7 = Eigen::VectorXd::Zero(3);
    c7 = -tempdAg_m.block<3,27>(3,0) * state.qd;

    //  w4 
    Eigen::MatrixXd H4 = Eigen:: MatrixXd::Zero(3,60);
    auto tempJ4 = state.Jbase.block<3,27>(0,0);
    H4.block<3,27>(0,21) = tempJ4;
    Eigen::VectorXd c4 = Eigen:: VectorXd::Zero(3);
    Eigen::VectorXd e4 = Eigen:: VectorXd::Zero(3);
    Eigen::VectorXd ev4 = Eigen:: VectorXd::Zero(3);

    ev4 = -state.qd.head(3);
    e4.head(3) = -state.q.head(3);
    e4(0) = e4(0) ;
    // e4(2) = e4(2) -0.15+0.08*sin(0.051*aaa);
    // e4(2) = e4(2) -0.12;
    // ev4(2) = ev4(2) + 0.051*0.08*(cos(0.051*aaa));
    aaa++; 
    c4(0) =  (250*e4(0) +5*(ev4(0)) - state.dJbase.block<1,27>(0,0) * state.qd);
    c4(1) =  (250*e4(1) +5*(ev4(1)) - state.dJbase.block<1,27>(1,0) * state.qd);
    // c4(2) =  650*e4(2) + 1*(ev4(2)) - state.dJbase.block<1,27>(2,0) * state.qd;


    //w2
    Eigen::MatrixXd H2 = Eigen:: MatrixXd::Zero(3,60);
    auto tempJ2 = state.Jbase.block<3,27>(3,0);
    std::cout<<"==state.Jbase.=="<<std::endl;
    std::cout<<state.Jbase<<std::endl;
    H2.block<3,27>(0,21) = tempJ2;
    Eigen::VectorXd c2 = Eigen:: VectorXd::Zero(3);
    Eigen::VectorXd e = Eigen:: VectorXd::Zero(3);
    Eigen::VectorXd ev = Eigen:: VectorXd::Zero(3);
    auto Rcur= eul2Rot(state.rpy(0), state.rpy(1), state.rpy(2));
    Eigen::Matrix3d Rdes = Eigen::Matrix3d::Identity();
    auto ew = diffRot(Rcur, Rdes);
    e = ew;
    e(1) = e(1)+0.21 ;//0.21
    // e() = e(1)+0.25*cos(0.051*aaa);;//0.21

    ev = -state.qd.segment(3,3);
    // ev(1)+= 0.051*0.25*cos(0.051*aaa);;
    c2 =  280*e + 3*ev - state.dJbase.block<3,27>(3,0) * state.qd;


    Eigen::MatrixXd H8 = Eigen:: MatrixXd::Zero(27,60);
    Eigen::MatrixXd J8 = Eigen:: MatrixXd::Zero(27,27);
    J8(18,18) = 1;
    J8(21,21) = -1;
    J8(25,25) = 1;


    H8.block<27,27>(0,21) = J8;
    Eigen::VectorXd c8 = Eigen:: VectorXd::Zero(27);
    // c8(18) =  250*(sin(0.05*aaa) - state.q(19)) - 5 *  state.qd(18) ;
   
    fileWaist<<0.09*sin(0.05*aaa) - state.q(19)<<";"<<std::endl;

    //  w5  手臂位置 
    Eigen::MatrixXd H5 = Eigen:: MatrixXd::Zero(3,60);
    Eigen::MatrixXd handw = Eigen:: MatrixXd::Identity(27,27);
    H5.block<3,27>(0,21) = state.JLHand.block<3,27>(0,0);
    Eigen::VectorXd c5 = Eigen:: VectorXd::Zero(3);
    Eigen::VectorXd e5 = Eigen:: VectorXd::Zero(3);
    Eigen::VectorXd ev5 = Eigen:: VectorXd::Zero(3);
    // ==handLpose=
    // ==handRpose=
    std::cout<<"=================="<<state.q.segment(3,4)<<std::endl;
    e5(0) = 0.20;
    e5(1) = 0.426967;
    e5(2) = -0.15;
    // <<0.20,  0.426967,-0.15;//0.15, -0.379501,  0.88;
    if(aaa<500)
    {
        e5(0) = -0.55;//0.15, -0.379501,  0.88;
    }
    else
    {
        e5(0) = 0.55;//0.15, -0.379501,  0.88;
    }


    if(aaa>1500)
    {
        e5(0) = -0.15;//0.15, -0.379501,  0.88;
    }

    e5.head(3) = e5.head(3) -state.LHandPos;
    std::cout<<"==handLpose="<< e5.transpose()<<std::endl;
    ev5.head(3) = -state.JLHand.block<3,27>(0,0) * state.qd;
    Eigen::VectorXd temp5dj = Eigen:: VectorXd::Zero(3);
    temp5dj.head(3) = -state.dJLHand.block<3,27>(0,0) * state.qd;
    c5 = 500*e5 + 5*ev5 + temp5dj;   // 
    fileHand<<e5(0)<<";"<<e5(1)<<";"<<e5(2)<<";"<<std::endl;

    //  w6  手臂位置 
    Eigen::MatrixXd H6 = Eigen:: MatrixXd::Zero(3,60);

    H6.block<3,27>(0,21) = state.JRHand.block<3,27>(0,0) * handw;
    // H5.block<3,27>(3,20) = state.JRHand.block<3,27>(0,0);


    Eigen::VectorXd c6 = Eigen:: VectorXd::Zero(3);
    Eigen::VectorXd e6 = Eigen:: VectorXd::Zero(3);
    Eigen::VectorXd ev6 = Eigen:: VectorXd::Zero(3);
    // ==handLpose=
    // ==handRpose=
    e6<<0.20,  -0.426967,  -0.15;//0.15, -0.379501,  0.88;

    e6.head(3) = e6.head(3) -state.RHandPos;
    std::cout<<"==handRpose="<< e6.transpose()<<std::endl;

    // e5.tail(3) = e5.tail(3) -state.RHandPos;
    ev6.head(3) = -state.JRHand.block<3,27>(0,0) * state.qd;
    // ev5.tail(3) = -state.JRHand.block<3,27>(0,0) * state.qd;
    Eigen::VectorXd temp6dj = Eigen:: VectorXd::Zero(3);
    temp6dj.head(3) = -state.dJRHand.block<3,27>(0,0) * state.qd;
    // temp5dj.tail(3) = -state.dJRHand.block<3,27>(0,0) * state.qd;
    c6 = 500*e6 + 5*ev6 + temp6dj;   // 
    // file2<<e6(0)<<";"<<e6(1)<<";"<<e6(2)<<";"<< "\n";

    //===============整理===============
    Eigen::MatrixXd w1 = Eigen::MatrixXd::Identity(3,3);
    Eigen::MatrixXd w2 = Eigen::MatrixXd::Identity(12,12);
    Eigen::MatrixXd w3 = Eigen::MatrixXd::Identity(3,3);
    Eigen::MatrixXd w4 = Eigen::MatrixXd::Identity(3,3);
    Eigen::MatrixXd w5 = Eigen::MatrixXd::Identity(3,3);
    Eigen::MatrixXd w6 = Eigen::MatrixXd::Identity(3,3);
    Eigen::MatrixXd w7 = Eigen::MatrixXd::Identity(3,3);
    Eigen::MatrixXd w8 = Eigen::MatrixXd::Identity(27,27);
    w1 = 1*w1;
    w4 = 0*w4;
    w5 = 1*w5;
    w6 = 1*w6;
    w2 = 0*w2;
    w3 = 1*w3;
    w7 = 1*w7;
    w8 = 5*w8;
    Eigen::MatrixXd wI = Eigen::MatrixXd::Identity(60, 60);

    Eigen::MatrixXd qp_H =    H1.transpose()*w2 * H1 
                            + H2.transpose()*w1 * H2 
                            + H3.transpose()*w3 * H3
                            + H4.transpose()*w4 * H4 
                            + H5.transpose()*w5 * H5 
                            + H6.transpose()*w6 * H6 
                            + H7.transpose()*w7 * H7 
                            + H8.transpose()*w8 * H8 
                            +0.0005*wI;
    Eigen::VectorXd qp_c = Eigen::VectorXd::Zero(60);
    qp_c = -H1.transpose() *w2* c1 
          - H2.transpose() *w1* c2  
          - H3.transpose() *w3* c3
          - H4.transpose() *w4* c4 
          - H5.transpose() *w5* c5
          - H6.transpose() *w6* c6
          - H7.transpose() *w7* c7
          - H8.transpose() *w8* c8;



    Eigen::MatrixXd qp_A = Eigen::MatrixXd::Zero(99+12,60);
    qp_A.block<27,60>(0,0) = A;
    qp_A.block<12,60>(99,0) = H1;
    // qp_A.block<3,58>(96+12,0) = accCMw;
    Eigen::VectorXd up = Eigen::VectorXd::Zero(99+12);
    Eigen::VectorXd lp = Eigen::VectorXd::Zero(99+12);
    Eigen::VectorXd theta = Eigen::VectorXd::Zero(27);
    theta.setConstant(0.0000001);
    up.head(27) = -b + theta;
    lp.head(27) = -b - theta;
    up.segment(99,12) = c1 + theta.head(12);
    lp.segment(99,12) = c1 - theta.head(12);

    // up.tail(3) = accCMwBound + 5000000*theta.head(3);
    // lp.tail(3) = accCMwBound - 5000000*theta.head(3);
    
    double dx =0.1;
    double dy =0.05;
    double u = 0.8/sqrt(2);
    double tempd = u * sqrt(dx*dx +dy*dy)/16;
    Eigen::MatrixXd uF = Eigen::MatrixXd::Zero(12,6);
    uF<< 1 , 0 , -u , 0 ,0 ,0 ,
         1 , 0 ,  u , 0 ,0 ,0 ,
         0 , 1 , -u , 0 ,0 ,0 ,
         0 , 1 ,  u , 0 ,0 ,0 ,
         0 , 0 ,  1 , 0 ,0 ,0 ,
         0 , 0 , -1*dy , 1 ,0 ,0 ,
         0 , 0 ,  1*dy , 1 ,0 ,0 ,
         0 , 0 , -1*dx , 0 ,1 ,0 ,
         0 , 0 ,  1*dx , 0 ,1 ,0 ,
         0 , 0 ,  0 , 1 ,0 ,0 ,
         0 , 0 ,  0 , 0 ,1 ,0 ,
         0 , 0 ,  0 , 0 ,0 ,1 ;
    Eigen::MatrixXd temprotL = Eigen::MatrixXd::Identity(6,6);
    Eigen::MatrixXd temprotR = Eigen::MatrixXd::Identity(6,6);
    Eigen::VectorXd tempFrLp = Eigen::VectorXd::Zero(12);
    Eigen::VectorXd tempFrUp = Eigen::VectorXd::Zero(12);
    tempFrUp<< 0    ,1e30,  0   ,1e30,1200,      0,1e30,  0   ,1e30,  100,100,20;
    tempFrLp<<-1e30 ,0   ,-1e30 ,  0 ,0 ,  -1e30 ,0   ,-1e30 ,   0, -100,-100,-20;
    qp_A.block<12,6>(27,60-6-6) = uF*temprotL;
    qp_A.block<12,6>(27+12,60-6) = uF*temprotR;
    // std::cout<<"===wqp1.2=="<<std::endl;
    qp_A.block<48,48>(27+12+12,0) = Eigen::MatrixXd::Identity(48,48);
    Eigen::VectorXd limtua = Eigen::VectorXd::Zero(21);
    limtua<< 500.0, 500.0, 500.0, 500.0, 180.0, 180.0,
    500.0, 500.0, 500.0, 500.0, 180.0, 180.0, 250.0,
        20,20,20,20,20,20,20,20;
    Eigen::VectorXd limqdd = Eigen::VectorXd::Zero(27);
    limqdd.setConstant(1);
    limqdd = 100 * limqdd;
    up.segment(51,21) =  1*limtua;
    lp.segment(51,21) = -1*limtua;

    up.segment(72,27) =  limqdd;
    lp.segment(72,27) = -limqdd;


    up.segment(27,12) = tempFrUp;
    up.segment(39,12) = tempFrUp;

    lp.segment(27,12) = tempFrLp;
    lp.segment(39,12) = tempFrLp;

    OsqpEigen::Solver solver;
    std::cout<<"==test="<<std::endl;
    int n = 60;
    solver.settings()->setWarmStart(false);
    solver.settings()->setVerbosity(false);
    solver.data()->setNumberOfVariables(60);
    solver.data()->setNumberOfConstraints(99+12);
    Eigen::SparseMatrix<double> H_sparse = qp_H.sparseView();
    if (!solver.data()->setHessianMatrix(H_sparse)) 
    {
        std::cerr << "Error setting Hessian matrix" << std::endl;
        return Eigen::VectorXd::Zero(n);
    }
    if (!solver.data()->setGradient(qp_c)) {
        std::cerr << "Error setting gradient vector" << std::endl;
        return Eigen::VectorXd::Zero(n);
    }
    Eigen::SparseMatrix<double> A_sparse = qp_A.sparseView();
    if (!solver.data()->setLinearConstraintsMatrix(A_sparse)) {
        std::cerr << "Error setting constraint matrix" << std::endl;
        return Eigen::VectorXd::Zero(n);
    }
    if (!solver.data()->setLowerBound(lp)) {
        std::cerr << "Error setting lower bounds" << std::endl;
        return Eigen::VectorXd::Zero(n);
    }
    if (!solver.data()->setUpperBound(up)) {
        std::cerr << "Error setting upper bounds" << std::endl;
        return Eigen::VectorXd::Zero(n);
    }
    if (!solver.initSolver()) {
        std::cerr << "Solver initialization failed" << std::endl;
        return Eigen::VectorXd::Zero(n);
    }
    auto result = solver.solve();
    if (!solver.solve())
    {
        std::cout<<"========="<<std::endl;
        return Eigen::VectorXd::Zero(n);
    }
    auto tempRes = solver.getSolution();
    // std::cout <<"==tau=="<<tempRes.head(21).transpose()<<std::endl;
    // std::cout <<"==qdd=="<<tempRes(31)<<std::endl;
    // std::cout <<"==6df=="<<tempRes.segment(48,12).transpose()<<std::endl;
    solver.clearSolver();
    //ax =b  x =  tau(27)  qdd(27)  Fr 6d*2
    Eigen::VectorXd result111 = Eigen::VectorXd::Zero(42);

    result111.head(21) = tempRes.segment(0,21);
    result111.tail(21) = tempRes.segment(21+6,21);
    // auto nlv =  (state.DynC * state.qd + state.DynG).tail(21);
    // auto qaac =  tempRes.segment(21+6,21);

    // // Mq = S1*state.DynM;
    // // Cq = S1*state.DynC;
    // // Gq = S1*state.DynG;
    // result111.head(21) = state.DynM.block<21,21>(6,6) *qaac + nlv - 
    // -state.dJLFoot1.block<6,21>(0,6).transpose()  *  tempRes.segment(48,6)
    // -state.dJRFoot1.block<6,21>(0,6).transpose()  *  tempRes.segment(54,6);
    return result111;

}
tempTest::tempTest(/* args */)
{
    std::cout<<"===tempTest=="<<std::endl;
    string filePath1 = "comError.txt";
    string filePath2 = "handError.txt";
    string filePath3 = "waistError.txt";
    fileCom.open(filePath1);
    fileHand.open(filePath2);
    fileWaist.open(filePath3);

    // ofstream file1;
    // ofstream file2;
    // ofstream file3;
    robot = &RobotData::getInstance();
    state = robot->getRobotState();
}
Eigen::Matrix3d tempTest::skew(Eigen::VectorXd& v)
{
    // 确保输入是三维向量
    if (v.size() != 3)
    {
        std::cout << "Input vector must be of size 3." << std::endl;
    }

    Eigen::Matrix3d skew_matrix;
    skew_matrix << 0,    -v(2),  v(1),
                   v(2),  0,    -v(0),
                  -v(1),  v(0),  0;

    return skew_matrix;
}

tempTest::~tempTest()
{
}

int main(int argc, char* argv[]) 
{

    char error[1000] = "Could not load binary model";
    mjModel* mj_model = mj_loadXML("/home/nikoo/workWS/armWorkCS/src/wbc/models/tiangong2_EVT_V2/urdf/evt2.xml", 0, error, 5000);
    mjData* mj_data = mj_makeData(mj_model);
    UIctr uiController(mj_model,mj_data);   // UI control for Mujoco
    MJ_Interface mj_interface(mj_model, mj_data); // data interface for Mujoco
    std::cout << "mj ok" << std::endl;
    tempTest Test;
    std::cout<<"===tempTest Test;=="<<std::endl;

    double simEndTime=20;
    mjtNum simstart = mj_data->time;
    double simTime = mj_data->time;
    // init UI: GLFW
    uiController.iniGLFW();
    uiController.enableTracking(); // enable viewpoint tracking of the body 1 of the robot
    uiController.createWindow("Demo",false);
    // mj_interface.joint_state_sub_ = mj_interface. nh_.subscribe("/joint_states", 10, &MJ_Interface::jointStateCallback, this);
    RobotData &robot = RobotData::getInstance();

    while(1)
    {
        simstart=mj_data->time;
        while( mj_data->time - simstart < 1.0/400)
        {
            // std::cout<<"===wqp=="<<std::endl;
            Eigen::VectorXd t = Eigen::VectorXd::Zero(42);
            // mj_interface.setMotorsTorque(t);
            mj_interface.updateSensorValues();
            mj_interface.robotStateUpData();

            simTime=mj_data->time;
            // auto a = robot.getRobotState();
            t = Test.wqp_new();
            // std::cout<<b.transpose()<<std::endl;
            mj_interface.setMotorsTorque(t);
            mj_forward(mj_model, mj_data);
            mj_step(mj_model, mj_data);
            // std::cout<<"==单例输出="<<(m_state.q).transpose()<<std::endl;
            // printf("-------------%.3d s------------\n",simTime);
            
        }
        uiController.updateScene();

    }
    //    // free visualization storage
    uiController.Close();

    // free MuJoCo model and data, deactivate
    mj_deleteData(mj_data);
    mj_deleteModel(mj_model);
    return 0;
}