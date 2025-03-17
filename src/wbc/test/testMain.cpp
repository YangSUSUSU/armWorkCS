
// #include <iostream> 
// #include <functional> 
// // #include <mujoco/mujoco.h>
// #include "../include/mujoco/mujoco.h"
// #include <GLFW/glfw3.h>
// #include <cstdio>
// #include "../mjsim/GLFW_callbacks.h"
// #include "../mjsim/MJ_interface.h"
// #include "../robotData/robotData.h"
// #include "../robotData/robotStructs.h"
// int main(int argc, char* argv[]) {

// //     // RobotData& robotData = RobotData::getInstance();
// //     // std::cout<<"==build ok=="<<std::endl;
// //     // return 0;
// //     // 获取单例实例
// //     RobotData& robot = RobotData::getInstance();

// //     // 获取机器人状态
// //     RobotStructs state = robot.getRobotState();
    
// //     // std::cout << "Xcom: " << state.Xcom.transpose() << std::endl;

// //     // 设置机器人状态
// //     RobotStructs newState;
// //     newState.Xcom << 1.0, 2.0, 3.0;
// //     robot.setRobotState(newState);

// //     // 再次获取状态
// //     state = robot.getRobotState();
// //     // std::cout << "Updated Xcom: " << state.Xcom.transpose() << std::endl;


// //     char error[1000] = "Could not load binary model";
// //     mjModel* mj_model = mj_loadXML("/home/nikoo/workWS/armWorkCS/src/wbc/models/x_humanoid_ultra_noeyes/mjcf/mjmodel.xml", 0, error, 5000);
// //     mjData* mj_data = mj_makeData(mj_model);
// //     UIctr uiController(mj_model,mj_data);   // UI control for Mujoco
// //     MJ_Interface mj_interface(mj_model, mj_data); // data interface for Mujoco
// //     std::cout << "mj ok" << std::endl;

// //     double simEndTime=20;
// //     mjtNum simstart = mj_data->time;
// //     double simTime = mj_data->time;
// //     // init UI: GLFW
// //     uiController.iniGLFW();
// //     uiController.enableTracking(); // enable viewpoint tracking of the body 1 of the robot
// //     uiController.createWindow("Demo",false);
// //     // mj_interface.joint_state_sub_ = mj_interface. nh_.subscribe("/joint_states", 10, &MJ_Interface::jointStateCallback, this);

// //     while(1)
// //     {
// //         simstart=mj_data->time;
// //         while( mj_data->time - simstart < 1.0/500)
// //         {
// //             Eigen::VectorXd t;
// //             mj_interface.setMotorsTorque(t);
// //             mj_interface.updateSensorValues();
// //             mj_interface.robotStateUpData();
// //             mj_forward(mj_model, mj_data);
// //             mj_step(mj_model, mj_data);
// //             simTime=mj_data->time;
// //             RobotStructs m_state = robot.getRobotState();
// //             // std::cout<<"==单例输出="<<(m_state.q).transpose()<<std::endl;
// //             // printf("-------------%.3d s------------\n",simTime);
            
// //         }
// //         uiController.updateScene();

// //     }
// //     //    // free visualization storage
// //     uiController.Close();

// //     // free MuJoCo model and data, deactivate
// //     mj_deleteData(mj_data);
// //     mj_deleteModel(mj_model);

//     return 0;
// }


#include <iostream> 
#include <functional> 
// #include <mujoco/mujoco.h>
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
    Eigen::VectorXd wqp();
    Eigen::VectorXd wqp_new();
    Eigen::Matrix<double, 3, 3> eul2Rot(double roll, double pitch, double yaw);
    Eigen::Matrix<double, 3, 1> diffRot( Eigen::Matrix3d &Rcur, Eigen::Matrix3d &Rdes);
    int aaa = 0;
    ofstream file1;
    ofstream file2;
    ofstream file3;
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
// Eigen::VectorXd tempTest::wqp_new()
// {
//     //ax =b  x =  tau(20)  qdd(27)  Fr 6d*2
//     // std::cout<<"===wqp001=="<<std::endl;

//     double M = 34.1093;
//     int nq = 27;
//     state = robot->getRobotState();
//     Eigen::MatrixXd S  = Eigen:: MatrixXd::Zero(20,27);
//     S.block<20,20>(0,6) = Eigen:: MatrixXd::Identity(20,20);
//     Eigen::MatrixXd Mq = Eigen::MatrixXd::Identity(27,27);
//     Eigen::MatrixXd Cq = Eigen::MatrixXd::Identity(27,27);
//     Eigen::MatrixXd Gq = Eigen::VectorXd::Zero(27);
//     Mq = state.DynM;
//     Cq = state.DynC;
//     Gq = state.DynG;
//     std::cout<<"===wqp002=="<<std::endl;

//     Eigen::MatrixXd A  = Eigen:: MatrixXd::Zero(nq,2*nq + 2*6-6);
//     A.block<27,20>(0,0) = -S.transpose();
//     A.block<27,27>(0,20) = Mq;
//     A.block<27, 6>(0,46) = - state.JLFoot2.transpose();
//     A.block<27, 6>(0,52) = - state.JRFoot2.transpose();
//     Eigen::VectorXd b = Eigen::VectorXd::Zero(nq);
//     b =(Cq * state.qd + Gq);
//     std::cout<<"===wqp0.1=="<<std::endl;

//     //  w1  足底接触不打滑
//     Eigen::MatrixXd H1 = Eigen:: MatrixXd::Zero(6,2*nq+12-6);
//     std::cout<<"===wqp0.2=="<<std::endl;

//     H1.block<3,27>(0,20) = state.JLFoot2.block<3,27>(0,0);
//     H1.block<3,27>(3,20) = state.JRFoot2.block<3,27>(0,0);
//     std::cout<<"===wqp0.3=="<<std::endl;


//     Eigen::VectorXd c1 = Eigen:: VectorXd::Zero(6);
//     c1.head(3) = - state.dJLFoot2.block<3,27>(0,0)*state.qd;
//     c1.tail(3) = - state.dJRFoot2.block<3,27>(0,0)*state.qd;
//     // std::cout<<"===wqp0.4=="<<std::endl;
    
//     //  w3  质心xy跟踪
//     Eigen::MatrixXd H3 = Eigen:: MatrixXd::Zero(3,2*nq+12-6);
//     // std::cout<<"===wqp0.2=="<<std::endl;

//     H3.block<3,27>(0,20) = state.Jcom;
//     // std::cout<<"===wqp0.3=="<<std::endl;
//     auto tempAg_m = state.Ag;
//     auto tempdAg_m = state.dAg;
//     tempAg_m.block<3,27>(0,0) =  tempAg_m.block<3,27>(0,0)/M;
//     tempdAg_m.block<3,27>(0,0) =  tempdAg_m.block<3,27>(0,0)/M;

//     Eigen::VectorXd c3 = Eigen:: VectorXd::Zero(3);
//     Eigen::VectorXd b0 = Eigen:: VectorXd::Zero(3);
//     b0<<0.0801,0.0,0.7;
//     c3 = (b0-state.Xcom)*1+5000*(b0 -state.Xcom) - 1.5* state.Jcom*state.qd - tempdAg_m.block<3,27>(0,0)*state.qd;
//     // c3(2) = 0;
//     // c3.head(6) = - state.dJLFoot2*state.qd;
//     std::cout<<"===e=="<<std::endl;







//     //  w2  基座跟踪
//     Eigen::MatrixXd H2 = Eigen:: MatrixXd::Zero(6,2*nq+12-6);

//     // std::cout<<"===wqp0.6=="<<std::endl;

//     H2.block<6,27>(0,20) = state.Jbase;
//     // Eigen::VectorXd pcom0 = Eigen::VectorXd::Zero(6);
//     // pcom0<< -0.01000,-0,0.87,0,0;
//     Eigen::VectorXd c2 = Eigen:: VectorXd::Zero(6);
//     Eigen::VectorXd e = Eigen:: VectorXd::Zero(6);
//     Eigen::VectorXd ev = Eigen:: VectorXd::Zero(6);
//     e.head(3) = -state.q.head(3);
//     e(0) = e(0) + 0.001;
//     e(2) = e(2) + 0.1;

//     auto Rcur= eul2Rot(state.rpy(0), state.rpy(1), state.rpy(2));
//     Eigen::AngleAxisd angleAxis(Rcur);
//     double angle = angleAxis.angle();       // 旋转角度
//     Eigen::Vector3d axis = angle * angleAxis.axis(); 
//     e(3)  = -state.rpy(0);
//     e(4)  =-state.rpy(1);
//     e(5)  =-state.rpy(2);
//     ev.head(6) = -state.qd.head(6);


//     // ev(5) = 0;
//     // e(5) = 0;
//     ev(5) = 1* ev(5);
//     e(5) = 1*e(5);
//     ev(3) = 1* ev(3);
//     e(3) = 1*e(3);
//     ev(4) = ev(4);
//     e(4) = 1*e(4);

//     c2 =  e/0.1+10.1*e + 0.1*ev - state.dJbase * state.qd;
//     std::cout<<"===wqp0.7=="<<std::endl;

//     Eigen::MatrixXd qp_H =  100*H1.transpose() * H1 + 0.1*H2.transpose() * H2 + 500000* H3.transpose() * H3 ;
//     Eigen::VectorXd qp_c = Eigen::VectorXd::Zero(64);
//     qp_c = -100*H1.transpose() * c1 - 0.1*H2.transpose() * c2 -500000* H3.transpose() * c3;
//     Eigen::MatrixXd qp_A = Eigen::MatrixXd::Zero(27+22,58);
//     qp_A.block<27,58>(0,0) = A;
//     Eigen::VectorXd up = Eigen::VectorXd::Zero(48);
//     Eigen::VectorXd lp = Eigen::VectorXd::Zero(48);
//     Eigen::VectorXd theta = Eigen::VectorXd::Zero(27);
//     theta.setConstant(0.000001);
//     up.head(27) = -b + theta;
//     lp.head(27) = -b - theta;
//     double dx =0.05155;
//     double dy =0.01755;
//     double u = 0.2135;
//     double tempd = u * sqrt(dx*dx +dy*dy)/16;
//     Eigen::MatrixXd uF = Eigen::MatrixXd::Zero(11,6);
//     uF<< 1 , 0 , -u , 0 , 0 , 0,
//          1 , 0 ,  u , 0 , 0 , 0,
//          0 , 1 , -u , 0 , 0 , 0,
//          0 , 1 ,  u , 0 , 0 , 0,
//          0 , 0 ,  1 , 0 , 0 , 0,
//          0 , 0 , -dy, 1 , 0 , 0,
//          0 , 0 ,  dy, 1 , 0 , 0,
//          0 , 0 , -dx, 0 , 1 , 0,
//          0 , 0 ,  dx, 0 , 1 , 0,
//          0 , 0 , -tempd , 0 , 0 , 1,
//          0 , 0 ,  tempd , 0 , 0 , 1;

//     std::cout<<"===wqp1=="<<std::endl;

//     Eigen::VectorXd tempFrLp = Eigen::VectorXd::Zero(11);
//     Eigen::VectorXd tempFrUp = Eigen::VectorXd::Zero(11);
//     tempFrUp<< 0    ,1e10,  0   ,1e10,500,     0,1e10,  0   ,1e10,0,1e10;
//     tempFrLp<<-1e10 ,0   ,-1e10 ,  0 ,0  ,-1e10 ,0   ,-1e10 ,   0,-1e10 ,0;
//     qp_A.block<11,6>(27,64-12-6) = uF;
//     qp_A.block<11,6>(27+11,64 - 2*6) = uF;
//     up.segment(27,11) = tempFrUp;
//     up.tail(11) = tempFrUp;
//     lp.segment(27,11) = tempFrLp;
//     lp.tail(11) = tempFrLp;
//     // std::cout<<"===wqp2=="<<std::endl;

//     OsqpEigen::Solver solver;

//     int n = 64;
//     solver.settings()->setWarmStart(false);
//     solver.settings()->setVerbosity(false);
//     solver.data()->setNumberOfVariables(58);
//     solver.data()->setNumberOfConstraints(48);
//     Eigen::SparseMatrix<double> H_sparse = qp_H.sparseView();
//     if (!solver.data()->setHessianMatrix(H_sparse)) 
//     {
//         std::cerr << "Error setting Hessian matrix" << std::endl;
//         return Eigen::VectorXd::Zero(n);
//     }
//     if (!solver.data()->setGradient(qp_c)) {
//         std::cerr << "Error setting gradient vector" << std::endl;
//         return Eigen::VectorXd::Zero(n);
//     }
//     Eigen::SparseMatrix<double> A_sparse = qp_A.sparseView();
//     if (!solver.data()->setLinearConstraintsMatrix(A_sparse)) {
//         std::cerr << "Error setting constraint matrix" << std::endl;
//         return Eigen::VectorXd::Zero(n);
//     }
//     if (!solver.data()->setLowerBound(lp)) {
//         std::cerr << "Error setting lower bounds" << std::endl;
//         return Eigen::VectorXd::Zero(n);
//     }
//     if (!solver.data()->setUpperBound(up)) {
//         std::cerr << "Error setting upper bounds" << std::endl;
//         return Eigen::VectorXd::Zero(n);
//     }
//     if (!solver.initSolver()) {
//         std::cerr << "Solver initialization failed" << std::endl;
//         return Eigen::VectorXd::Zero(n);
//     }
//     auto result = solver.solve();
//     if (!solver.solve())
//     {
//         std::cout<<"========="<<std::endl;
//         return Eigen::VectorXd::Zero(n);
//     }
//     auto tempRes = solver.getSolution();

//     // std::cout <<tempRes.tail(12).transpose()<<std::endl;
//     solver.clearSolver();
//     //ax =b  x =  tau(27)  qdd(27)  Fr 6d*2
//     Eigen::VectorXd result111 = Eigen::VectorXd::Zero(40);

//     result111.head(20) = tempRes.segment(0,20);
//     result111.tail(20) = tempRes.segment(27,20);
//     return result111;

// }

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
// Eigen::VectorXd tempTest::wqp_new()
// {
//     //ax =b  x =  tau(20)  qdd(27)  Fr 3d * 8
//     // std::cout<<"===wqp001=="<<std::endl;
//     //ax =b  x =  tau(20)  qdd(27)  Fr 6d * 8

//     double M = 34.1093;
//     int nq = 27;
//     state = robot->getRobotState();
//     Eigen::MatrixXd S  = Eigen:: MatrixXd::Zero(20,27);
//     S.block<20,20>(0,6) = Eigen:: MatrixXd::Identity(20,20);
//     Eigen::MatrixXd Mq = Eigen::MatrixXd::Identity(27,27);
//     Eigen::MatrixXd Cq = Eigen::MatrixXd::Identity(27,27);
//     Eigen::MatrixXd Gq = Eigen::VectorXd::Zero(27);
//     Mq = state.DynM;
//     Cq = state.DynC;
//     Gq = state.DynG;
//     // std::cout<<"===wqp002=="<<std::endl;

//     Eigen::MatrixXd A  = Eigen:: MatrixXd::Zero(nq,2*nq + 3*8-6);
//     A.block<27,20>(0,0) = -S.transpose();
//     A.block<27,27>(0,20) = Mq;
//     A.block<27, 3>(0,46) = - state.JLFoot1.block<3,27>(0,0).transpose();
//     A.block<27, 3>(0,49) = - state.JLFoot2.block<3,27>(0,0).transpose();
//     A.block<27, 3>(0,49+3) = - state.JLFoot3.block<3,27>(0,0).transpose();
//     A.block<27, 3>(0,49+6) = - state.JLFoot4.block<3,27>(0,0).transpose();
//     A.block<27, 3>(0,49+9) = - state.JRFoot1.block<3,27>(0,0).transpose();
//     A.block<27, 3>(0,49+12) = - state.JRFoot2.block<3,27>(0,0).transpose();
//     A.block<27, 3>(0,46+15) = - state.JRFoot3.block<3,27>(0,0).transpose();
//     A.block<27, 3>(0,49+18) = - state.JRFoot4.block<3,27>(0,0).transpose();
//     Eigen::VectorXd b = Eigen::VectorXd::Zero(nq);
//     b =(Cq * state.qd + Gq);
//     // std::cout<<"===wqp0.1=="<<std::endl;

//     //  w1  足底接触不打滑
//     Eigen::MatrixXd H1 = Eigen:: MatrixXd::Zero(24*2,2*nq+24-6);
//     // std::cout<<"===wqp0.2=="<<std::endl;

//     H1.block<3+3,27>(0,20) = state.JLFoot1;
//     H1.block<3+3,27>(6,20) = state.JLFoot2;
//     H1.block<3+3,27>(6+6,20) = state.JLFoot3;
//     H1.block<3+3,27>(6+12,20) = state.JLFoot4;
//     H1.block<3+3,27>(6+18,20) = state.JRFoot1;
//     H1.block<3+3,27>(6+24,20) = state.JRFoot2;
//     H1.block<3+3,27>(6+30,20) = state.JRFoot3;
//     H1.block<3+3,27>(42,20) = state.JRFoot4;
//     // std::cout<<"===wqp0.3=="<<std::endl;


//     Eigen::VectorXd c1 = Eigen:: VectorXd::Zero(48);
//     c1.head(6) =      - state.dJLFoot1*state.qd;
//     c1.segment(6,6) = - state.dJLFoot2*state.qd;
//     c1.segment(6+6,6) = - state.dJLFoot3*state.qd;
//     c1.segment(6+12,6) = - state.dJLFoot4*state.qd;
//     c1.segment(6+18,6) = - state.dJRFoot1*state.qd;
//     c1.segment(6+24,6) = - state.dJRFoot2*state.qd;
//     c1.segment(6+30,6) = - state.dJRFoot3*state.qd;
//     c1.tail(6) = - state.dJRFoot4*state.qd;
//     // Eigen::MatrixXd H1 = Eigen:: MatrixXd::Zero(24,2*nq+24-6);
//     // // std::cout<<"===wqp0.2=="<<std::endl;

//     // H1.block<3 ,27>(0,20) = state.JLFoot1.block<3,27>(0,0);
//     // H1.block<3 ,27>(3,20) = state.JLFoot2.block<3,27>(0,0);;
//     // H1.block<3 ,27>(3+3,20) = state.JLFoot3.block<3,27>(0,0);;
//     // H1.block<3 ,27>(3+6,20) = state.JLFoot4.block<3,27>(0,0);;
//     // H1.block<3 ,27>(3+9,20) = state.JRFoot1.block<3,27>(0,0);;
//     // H1.block<3 ,27>(3+12,20) = state.JRFoot2.block<3,27>(0,0);;
//     // H1.block<3,27>(3+15,20) = state.JRFoot3.block<3,27>(0,0);;
//     // H1.block<3,27>(3+18,20) = state.JRFoot4.block<3,27>(0,0);;
//     // // std::cout<<"===wqp0.3=="<<std::endl;


//     // Eigen::VectorXd c1 = Eigen:: VectorXd::Zero(24);
//     // c1.head(3) =      - state.dJLFoot1.block<3,27>(0,0)*state.qd;
//     // c1.segment(3,3) = - state.dJLFoot2.block<3,27>(0,0)*state.qd;
//     // c1.segment(6,3) = - state.dJLFoot3.block<3,27>(0,0)*state.qd;
//     // c1.segment(9,3) = - state.dJLFoot4.block<3,27>(0,0)*state.qd;
//     // c1.segment(12,3) = - state.dJRFoot1.block<3,27>(0,0)*state.qd;
//     // c1.segment(15,3) = - state.dJRFoot2.block<3,27>(0,0)*state.qd;
//     // c1.segment(18,3) = - state.dJRFoot3.block<3,27>(0,0)*state.qd;
//     // c1.tail(3) = - state.dJRFoot4.block<3,27>(0,0)*state.qd;
//     // std::cout<<"===wqp0.4=="<<std::endl;
    
//     //  w3  质心xy跟踪
//     Eigen::MatrixXd H3 = Eigen:: MatrixXd::Zero(2,2*nq+24-6);
//     // std::cout<<"===wqp0.2=="<<std::endl;

//     H3.block<2,27>(0,20) = state.Jcom.block<2,27>(0,0);
//     // std::cout<<"===wqp0.3=="<<std::endl;
//     auto tempAg_m = state.Ag;
//     auto tempdAg_m = state.dAg;
//     tempAg_m.block<6,6>(0,0).setZero();
//     tempdAg_m.block<6,6>(0,0).setZero();

//     tempAg_m.block<2,27>(0,0) =  tempAg_m.block<2,27>(0,0)/M;
//     tempdAg_m.block<2,27>(0,0) =  tempdAg_m.block<2,27>(0,0)/M;
//     // std::cout<<"===wqp0.4=="<<std::endl;

//     Eigen::VectorXd c3 = Eigen:: VectorXd::Zero(2);
//     Eigen::VectorXd b0 = Eigen:: VectorXd::Zero(2);
//     b0<<-0.005,0.0;
//     // std::cout<<"===wqp0.5=="<<std::endl;
//     auto tempJcom = state.Jcom.block<2,27>(0,0);
//     tempJcom.block<2,6>(0,0).setZero();
//     // c3 = 60500000.0*(b0 -state.Xcom) - 0* state.Jcom*state.qd - tempdAg_m.block<3,27>(0,0)*state.qd;
//     c3 = 2000*(b0 - state.Xcom.segment(0,2)) - 200* tempJcom*state.qd - tempdAg_m.block<2,27>(0,0)*state.qd;
//     // std::cout<<"===wqp0.6=="<<std::endl;

//     // c3(2) = 0;
//     // c3(1) = 0;


  


//     // c3.head(6) = - state.dJLFoot2*state.qd;
//     std::cout<<"===eCom=="<<(b0-state.Xcom.segment(0,2)).transpose()<<std::endl;






//     //  w4  基座跟踪 xyz
//     Eigen::MatrixXd H4 = Eigen:: MatrixXd::Zero(3,2*nq+24-6);

//     // std::cout<<"===wqp0.6=="<<std::endl;
//     // double time = ros::Time::now().toSec();

//     // int aaa = 0;
//     H4.block<3,27>(0,20) = state.Jbase.block<3,27>(0,0);
//     // Eigen::VectorXd pcom0 = Eigen::VectorXd::Zero(6);
//     // pcom0<< -0.01000,-0,0.87,0,0;
//     Eigen::VectorXd c4 = Eigen:: VectorXd::Zero(3);
//     Eigen::VectorXd e4 = Eigen:: VectorXd::Zero(3);
//     Eigen::VectorXd ev4 = Eigen:: VectorXd::Zero(3);
//     ev4 = state.Jbase.block<3,27>(0,0) * state.qd;
//     e4.head(3) = -state.q.head(3);
//     e4(0) = e4(0) + 0.01;
//     e4(2) = e4(2) -0.1515+0.04*sin(0.021*aaa);
//     // e4(2) = e4(2) -0.1515;
//     ev4(2) = ev4(2) + 0.021*0.04*(cos(0.021*aaa));
//     aaa++;
//     // e4(0) =0;
//     // e4(1) =0;
//     // ev4(0) =0;
//     // ev4(1) =0;

//     c4(0) =  5000*e4(0) -0.15*(ev4(0)) - state.dJbase.block<1,27>(0,0) * state.qd;
//     c4(1) =  5000*e4(1) -0.15*(ev4(1)) - state.dJbase.block<1,27>(1,0) * state.qd;
//     c4(2) =  1500*e4(2) -5*(ev4(2)) - state.dJbase.block<1,27>(2,0) * state.qd;


//     //  w2  基座跟踪 rpy
//     Eigen::MatrixXd H2 = Eigen:: MatrixXd::Zero(3,2*nq+24-6);
//     H2.block<3,27>(0,20) = state.Jbase.block<3,27>(3,0);
//     // Eigen::VectorXd pcom0 = Eigen::VectorXd::Zero(6);
//     // pcom0<< -0.01000,-0,0.87,0,0;
//     Eigen::VectorXd c2 = Eigen:: VectorXd::Zero(3);
//     Eigen::VectorXd e = Eigen:: VectorXd::Zero(3);
//     Eigen::VectorXd ev = Eigen:: VectorXd::Zero(3);
//     auto Rcur= eul2Rot(state.rpy(0), state.rpy(1), state.rpy(2));
//     Eigen::Matrix3d Rdes = Eigen::Matrix3d::Identity();
//     auto ew = diffRot(Rcur, Rdes);
//     e = ew;
//     ev = -state.qd.segment(3,3);
//     c2 =  800*e + 1.5*ev - state.dJbase.block<3,27>(3,0) * state.qd;


//     //  w5  手臂位置 
//     Eigen::MatrixXd H5 = Eigen:: MatrixXd::Zero(3,2*nq+24-6);
//     Eigen::MatrixXd www = Eigen::MatrixXd::Identity(27,27);
//     for (int i = 0; i < 18; i++) 
//     {
//         www(i,i) = 1000;
//     }
//     // H5 = H5 *www;
//     H5.block<3,27>(0,20) = state.JLHand.block<3,27>(0,0);
//     // H5.block<3,27>(3,20) = state.JRHand.block<3,27>(0,0);

//     Eigen::VectorXd c5 = Eigen:: VectorXd::Zero(3);
//     Eigen::VectorXd e5 = Eigen:: VectorXd::Zero(3);
//     Eigen::VectorXd ev5 = Eigen:: VectorXd::Zero(3);
//     // ==handLpose=
//     // ==handRpose=
//     e5<<0.15,  0.376967,  0.88;//0.15, -0.379501,  0.88;
//     e5.head(3) = e5.head(3) -state.LHandPos;
//     // e5.tail(3) = e5.tail(3) -state.RHandPos;
//     ev5.head(3) = -state.JLHand.block<3,27>(0,0) * state.qd;
//     // ev5.tail(3) = -state.JRHand.block<3,27>(0,0) * state.qd;
//     Eigen::VectorXd temp5dj = Eigen:: VectorXd::Zero(3);
//     temp5dj.head(3) = -state.dJLHand.block<3,27>(0,0) * state.qd;
//     // temp5dj.tail(3) = -state.dJRHand.block<3,27>(0,0) * state.qd;
//     c5 = 100*e5 + 5*ev5 + temp5dj;   // 
//     std::cout<<"===e5 =="<<e5.transpose()<<std::endl;

//     //  w6  手臂位置 
//     Eigen::MatrixXd H6 = Eigen:: MatrixXd::Zero(3,2*nq+24-6);
//     H6.block<3,27>(0,20) = state.JRHand.block<3,27>(0,0);
//     // H5.block<3,27>(3,20) = state.JRHand.block<3,27>(0,0);

//     Eigen::VectorXd c6 = Eigen:: VectorXd::Zero(3);
//     Eigen::VectorXd e6 = Eigen:: VectorXd::Zero(3);
//     Eigen::VectorXd ev6 = Eigen:: VectorXd::Zero(3);
//     // ==handLpose=
//     // ==handRpose=
//     e6<<0.15,  -0.376967,  0.88;//0.15, -0.379501,  0.88;

//     e6.head(3) = e6.head(3) -state.RHandPos;
//     // e5.tail(3) = e5.tail(3) -state.RHandPos;
//     ev6.head(3) = -state.JRHand.block<3,27>(0,0) * state.qd;
//     // ev5.tail(3) = -state.JRHand.block<3,27>(0,0) * state.qd;
//     Eigen::VectorXd temp6dj = Eigen:: VectorXd::Zero(3);
//     temp6dj.head(3) = -state.dJRHand.block<3,27>(0,0) * state.qd;
//     // temp5dj.tail(3) = -state.dJRHand.block<3,27>(0,0) * state.qd;
//     c6 = 100*e6 + 5*ev6 + temp6dj;   // 
//     //===============整理===============
//     Eigen::MatrixXd w1 = Eigen::MatrixXd::Identity(3,3);
//     Eigen::MatrixXd w2 = Eigen::MatrixXd::Identity(48,48);
//     Eigen::MatrixXd w3 = Eigen::MatrixXd::Identity(2,2);
//     Eigen::MatrixXd w4 = Eigen::MatrixXd::Identity(3,3);
//     Eigen::MatrixXd w5 = Eigen::MatrixXd::Identity(3,3);
//     Eigen::MatrixXd w6 = Eigen::MatrixXd::Identity(3,3);


//     Eigen::MatrixXd wI = Eigen::MatrixXd::Zero(2*nq+24-6, 2*nq+24-6);
//     for (int i = 27; i < 46; i++) 
//     {
//         wI(i,i) = 1000;
//     }
//     w1 = 2000*w1;
//     w4 = 2000*w4;
//     w5 = 0*w5;
//     w6 = 0*w6;

//     w2 = 100*w2;
//     w3 = 0*w3;
//     // w3(2,2) = 0;

//     // w1 = 10000*w1;
//     // w4 = 0*w4;
//     // w5 = 100*w5;
//     // w2 = 100000*w2;
//     // w3 = 0*w3;
//     // std::cout<<"===w5  手臂位置6 =="<<std::endl;

//     Eigen::MatrixXd qp_H =  H1.transpose()*w2 * H1 
//                             + H2.transpose()*w1 * H2 
//                             +  H3.transpose()*w3 * H3
//                             + H4.transpose()*w4 * H4 
//                             + H5.transpose()*w5 * H5 
//                             + H6.transpose()*w6 * H6 
//                             + 0*wI;
//     Eigen::VectorXd qp_c = Eigen::VectorXd::Zero(64);
//     qp_c = -H1.transpose() *w2* c1 
//         - H2.transpose() *w1* c2  
//         - H3.transpose() *w3* c3
//         - H4.transpose() *w4* c4 
//         - H5.transpose() *w5* c5
//         - H6.transpose() *w6* c6;


//     Eigen::MatrixXd qp_A = Eigen::MatrixXd::Zero(66+70,58+12);
//     qp_A.block<27,70>(0,0) = A;
//     Eigen::VectorXd up = Eigen::VectorXd::Zero(66+70);
//     Eigen::VectorXd lp = Eigen::VectorXd::Zero(66+70);
//     Eigen::VectorXd theta = Eigen::VectorXd::Zero(27);
//     theta.setConstant(0.0000001);
//     up.head(27) = -b + theta;
//     lp.head(27) = -b - theta;
//     double dx =0.05155;
//     double dy =0.01755;
//     double u = 0.5/sqrt(2);
//     double tempd = u * sqrt(dx*dx +dy*dy)/16;
//     Eigen::MatrixXd uF = Eigen::MatrixXd::Zero(5,3);
//     uF<< 1 , 0 , -u ,
//          1 , 0 ,  u ,
//          0 , 1 , -u ,
//          0 , 1 ,  u ,
//          0 , 0 ,  1 ;

//     // std::cout<<"===wqp1.1=="<<std::endl;

//     Eigen::VectorXd tempFrLp = Eigen::VectorXd::Zero(5);
//     Eigen::VectorXd tempFrUp = Eigen::VectorXd::Zero(5);
//     tempFrUp<< 0    ,1e30,  0   ,1e30,1000; //    0,1e10,  0   ,1e10,0,1e10;
//     tempFrLp<<-1e30 ,0   ,-1e30 ,  0 ,10  ;//-1e10 ,0   ,-1e10 ,   0,-1e10 ,0;
//     qp_A.block<5,3>(27,64-12-6) = uF;
//     qp_A.block<5,3>(27+5,64-12-6+3) = uF;
//     qp_A.block<5,3>(27+10,64-12-6+6) = uF;
//     qp_A.block<5,3>(27+15,64-12-6+9) = uF;
//     // std::cout<<"===wqp1.1.1=="<<std::endl;

//     qp_A.block<5,3>(27+20,64-12-6+12) = uF;
//     qp_A.block<5,3>(27+25,64-12-6+15) = uF;
//     qp_A.block<5,3>(27+30,64-12-6+18) = uF;
//     qp_A.block<5,3>(27+35,64-12-6+21) = uF;
//     // std::cout<<"===wqp1.2=="<<std::endl;
//     qp_A.block<46,46>(66,0) = Eigen::MatrixXd::Identity(46,46);
//     Eigen::VectorXd limtua = Eigen::VectorXd::Zero(20);
//     limtua<< 160.0, 50.0, 200.0, 200.0, 50.0, 30.0,
//     160.0, 50.0, 200.0, 200.0, 50.0, 30.0,
//         13,13,13,13,13,13,13,13;
//     Eigen::VectorXd limqdd = Eigen::VectorXd::Zero(27);
//     limqdd.setConstant(1);
//     limqdd.tail(8) = 0.1*limqdd.tail(8);
//     limqdd = 50 * limqdd;
//     // limtua.setConstant(1);

//     // limtua = 900000000000000*limtua;
//     // // limtua(0) = 50;
//     // // limtua(6) = 50;
//     // // limtua(5) = 40;
//     // // limtua(5+6) = 40;
//     up.segment(66,20) = 1*limtua;
//     lp.segment(66,20) = -1*limtua;

//     up.segment(86,27) = limqdd;
//     lp.segment(86,27) = -limqdd;


//     up.segment(27,5) = tempFrUp;
//     up.segment(31,5) = tempFrUp;
//     up.segment(36,5) = tempFrUp;
//     up.segment(41,5) = tempFrUp;
//     // std::cout<<"===wqp1.3=="<<std::endl;

//     up.segment(46,5) = tempFrUp;
//     up.segment(51,5) = tempFrUp;
//     up.segment(56,5) = tempFrUp;
//     up.segment(61,5) = tempFrUp;
//     // std::cout<<"===wqp1.4=="<<std::endl;

    
//     lp.segment(27,5) = tempFrLp;
//     lp.segment(31,5) = tempFrLp;
//     lp.segment(36,5) = tempFrLp;
//     lp.segment(41,5) = tempFrLp;
//     // std::cout<<"===wqp1.5=="<<std::endl;

//     lp.segment(46,5) = tempFrLp;
//     lp.segment(51,5) = tempFrLp;
//     lp.segment(56,5) = tempFrLp;
//     lp.segment(61,5) = tempFrLp;
//     // std::cout<<"===wqp2=="<<std::endl;

//     OsqpEigen::Solver solver;

//     int n = 64;
//     solver.settings()->setWarmStart(false);
//     solver.settings()->setVerbosity(false);
//     solver.data()->setNumberOfVariables(70);
//     solver.data()->setNumberOfConstraints(66+70);
//     Eigen::SparseMatrix<double> H_sparse = qp_H.sparseView();
//     if (!solver.data()->setHessianMatrix(H_sparse)) 
//     {
//         std::cerr << "Error setting Hessian matrix" << std::endl;
//         return Eigen::VectorXd::Zero(n);
//     }
//     if (!solver.data()->setGradient(qp_c)) {
//         std::cerr << "Error setting gradient vector" << std::endl;
//         return Eigen::VectorXd::Zero(n);
//     }
//     Eigen::SparseMatrix<double> A_sparse = qp_A.sparseView();
//     if (!solver.data()->setLinearConstraintsMatrix(A_sparse)) {
//         std::cerr << "Error setting constraint matrix" << std::endl;
//         return Eigen::VectorXd::Zero(n);
//     }
//     if (!solver.data()->setLowerBound(lp)) {
//         std::cerr << "Error setting lower bounds" << std::endl;
//         return Eigen::VectorXd::Zero(n);
//     }
//     if (!solver.data()->setUpperBound(up)) {
//         std::cerr << "Error setting upper bounds" << std::endl;
//         return Eigen::VectorXd::Zero(n);
//     }
//     if (!solver.initSolver()) {
//         std::cerr << "Solver initialization failed" << std::endl;
//         return Eigen::VectorXd::Zero(n);
//     }
//     auto result = solver.solve();
//     if (!solver.solve())
//     {
//         std::cout<<"========="<<std::endl;
//         return Eigen::VectorXd::Zero(n);
//     }
//     auto tempRes = solver.getSolution();

//     std::cout <<tempRes.segment(20,27).transpose()<<std::endl;
//     solver.clearSolver();
//     //ax =b  x =  tau(27)  qdd(27)  Fr 6d*2
//     Eigen::VectorXd result111 = Eigen::VectorXd::Zero(40);

//     result111.head(20) = tempRes.segment(0,20);
//     result111.tail(20) = tempRes.segment(27,20);
//     return result111;

// }
Eigen::VectorXd tempTest::wqp_new()
{
    //ax =b  x =  tau(20)  qdd(27)  Fr 3d * 8
    //ax =b  x =  tau(20)  qdd(27)  Fr 6d * 2

    double M = 70.7394 ;
    int nq = 27;
    state = robot->getRobotState();
    std::cout<<"===state=="<<state.qd(10)<<std::endl;

    Eigen::MatrixXd S  = Eigen:: MatrixXd::Zero(21,27);
    S.block<21,21>(0,6) = Eigen:: MatrixXd::Identity(21,21);

    Eigen::MatrixXd S1  = Eigen:: MatrixXd::Identity(27,27);
    S1.block<21,21>(6,6) = Eigen:: MatrixXd::Identity(21,21);

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
    
    //  w3  质心xy跟踪


    Eigen::MatrixXd H3 = Eigen:: MatrixXd::Zero(3,60);
    // std::cout<<"===wqp0.2=="<<std::endl;
    // tempJcom.block<3,3>(0,7).setZero();
    // tempJcom.block<3,3>(0,13).setZero();
    auto j3 = state.Jcom.block<3,27>(0,0);
    // j3.block<3,6>(0,0).setZero();
    H3.block<3,27>(0,21) = j3;
    // H3.block<2,3>(0,7).setZero();
    // H3.block<2,3>(0,13).setZero();


    // std::cout<<"===wqp0.3=="<<std::endl;
    auto tempAg_m = state.Ag;
    auto tempdAg_m = state.dAg;
    // tempAg_m.block<6,6>(0,0).setZero();
    // tempdAg_m.block<6,6>(0,0).setZero();

    tempAg_m.block<3,27>(0,0) =  tempAg_m.block<3,27>(0,0)/M;
    tempdAg_m.block<3,27>(0,0) =  tempdAg_m.block<3,27>(0,0)/M;
    // std::cout<<"===wqp0.4=="<<std::endl;

    Eigen::VectorXd c3 = Eigen:: VectorXd::Zero(3);
    Eigen::VectorXd b0 = Eigen:: VectorXd::Zero(3);
    Eigen::VectorXd bv0 = Eigen:: VectorXd::Zero(3);
    bv0<<0.0,0.00,0.15*0.08*(cos(0.1*aaa));
    // +0.08*sin(0.051*aaa)
    b0<<0.000,0.00,-0.20+0.15*sin(0.1*aaa);
    // std::cout<<"===wqp0.5=="<<std::endl;
    auto tempJcom = state.Jcom.block<3,27>(0,0);

    // tempJcom.block<2,6>(0,0).setZero();
    // c3 = 60500000.0*(b0 -state.Xcom) - 0* state.Jcom*state.qd - tempdAg_m.block<3,27>(0,0)*state.qd;
    c3(0) = 500*(b0(0) - state.Xcom(0)) + 20 * ( bv0(0)- tempJcom.block<1,27>(0,0)*state.qd) - tempdAg_m.block<1,27>(0,0)*state.qd;
    c3(1) = 500*(b0(1) - state.Xcom(1)) + 20 * ( bv0(1)- tempJcom.block<1,27>(1,0)*state.qd) - tempdAg_m.block<1,27>(1,0)*state.qd;
    c3(2) = 900*(b0(2) - state.Xcom(2)) + 20 * ( bv0(2)- tempJcom.block<1,27>(2,0)*state.qd) - tempdAg_m.block<1,27>(2,0)*state.qd;
    std::cout<<"===eCom=="<<(b0-state.Xcom.segment(0,3)).transpose()<<std::endl;
    auto ecom = b0-state.Xcom.segment(0,3);
    // file1<<ecom(0)<<";"<<ecom(1)<<";"<<ecom(2)<<";"<< "\n";

    Eigen::MatrixXd H7 = Eigen:: MatrixXd::Zero(3,60);
    H7.block<3,27>(0,21) = tempAg_m.block<3,27>(3,0);
    Eigen::VectorXd c7 = Eigen::VectorXd::Zero(3);
    c7 = -tempdAg_m.block<3,27>(3,0) * state.qd;



    //  w4  基座跟踪 xyz
    Eigen::MatrixXd H4 = Eigen:: MatrixXd::Zero(3,60);
    auto tempJ4 = state.Jbase.block<3,27>(0,0);
    H4.block<3,27>(0,21) = tempJ4;
    // Eigen::VectorXd pcom0 = Eigen::VectorXd::Zero(6);
    // pcom0<< -0.01000,-0,0.87,0,0;
    Eigen::VectorXd c4 = Eigen:: VectorXd::Zero(3);
    Eigen::VectorXd e4 = Eigen:: VectorXd::Zero(3);
    Eigen::VectorXd ev4 = Eigen:: VectorXd::Zero(3);

    ev4 = -state.qd.head(3);
    e4.head(3) = -state.q.head(3);
    e4(0) = e4(0) - 0.01;
    e4(2) = e4(2) -0.15+0.08*sin(0.051*aaa);
    // e4(2) = e4(2) -0.12;
    ev4(2) = ev4(2) + 0.051*0.08*(cos(0.051*aaa));
    aaa++;
    e4(0) =0;
    e4(1) =0;
    ev4(0) =0;
    ev4(1) =0;   
    // std::cout<<"==aa=="<<e4(2)<<std::endl;
    c4(0) =  (250*e4(0) +5*(ev4(0)) - state.dJbase.block<1,27>(0,0) * state.qd);
    c4(1) =  (250*e4(1) +5*(ev4(1)) - state.dJbase.block<1,27>(1,0) * state.qd);
    // c4(0) = 0;
    // c4(1) =  0;
    c4(2) =  650*e4(2) + 1*(ev4(2)) - state.dJbase.block<1,27>(2,0) * state.qd;


    //  w2  基座跟踪 rpy
    Eigen::MatrixXd H2 = Eigen:: MatrixXd::Zero(3,60);
    auto tempJ2 = state.Jbase.block<3,27>(3,0);
    // tempJ2.block<3,6>(0,0).setZero();
    // tempJ2.block<3,3>(0,10).setZero();
    // tempJ2.block<3,3>(0,16).setZero();


    H2.block<3,27>(0,21) = tempJ2;
    // Eigen::VectorXd pcom0 = Eigen::VectorXd::Zero(6);
    // pcom0<< -0.01000,-0,0.87,0,0;
    Eigen::VectorXd c2 = Eigen:: VectorXd::Zero(3);
    Eigen::VectorXd e = Eigen:: VectorXd::Zero(3);
    Eigen::VectorXd ev = Eigen:: VectorXd::Zero(3);
    auto Rcur= eul2Rot(state.rpy(0), state.rpy(1), state.rpy(2));
    Eigen::Matrix3d Rdes = Eigen::Matrix3d::Identity();
    auto ew = diffRot(Rcur, Rdes);
    e = ew;
    e(1) = e(1)+0.21;
    ev = -state.qd.segment(3,3);
    // ev(1)+= 0.051*0.25*cos(0.051*aaa);;
    c2 =  280*e + 1*ev - state.dJbase.block<3,27>(3,0) * state.qd;


    //  w5  手臂位置 
    Eigen::MatrixXd H5 = Eigen:: MatrixXd::Zero(3,60);
    Eigen::MatrixXd handw = Eigen:: MatrixXd::Identity(27,27);
    // H5 = H5 *www;
    H5.block<3,27>(0,21) = state.JLHand.block<3,27>(0,0)*handw;
    Eigen::VectorXd c5 = Eigen:: VectorXd::Zero(3);
    Eigen::VectorXd e5 = Eigen:: VectorXd::Zero(3);
    Eigen::VectorXd ev5 = Eigen:: VectorXd::Zero(3);
    // ==handLpose=
    // ==handRpose=
    e5<<0.20,  0.426967,  -0.25;//0.15, -0.379501,  0.88;
    e5.head(3) = e5.head(3) -state.LHandPos;
    // e5.tail(3) = e5.tail(3) -state.RHandPos;
    ev5.head(3) = -state.JLHand.block<3,27>(0,0) * state.qd;
    // ev5.tail(3) = -state.JRHand.block<3,27>(0,0) * state.qd;
    Eigen::VectorXd temp5dj = Eigen:: VectorXd::Zero(3);
    temp5dj.head(3) = -state.dJLHand.block<3,27>(0,0) * state.qd;
    // temp5dj.tail(3) = -state.dJRHand.block<3,27>(0,0) * state.qd;
    c5 = 400*e5 + 2*ev5 + temp5dj;   // 
    // std::cout<<"===e5 =="<<e5.transpose()<<std::endl;

    //  w6  手臂位置 
    Eigen::MatrixXd H6 = Eigen:: MatrixXd::Zero(3,60);

    H6.block<3,27>(0,21) = state.JRHand.block<3,27>(0,0) * handw;
    // H5.block<3,27>(3,20) = state.JRHand.block<3,27>(0,0);


    Eigen::VectorXd c6 = Eigen:: VectorXd::Zero(3);
    Eigen::VectorXd e6 = Eigen:: VectorXd::Zero(3);
    Eigen::VectorXd ev6 = Eigen:: VectorXd::Zero(3);
    // ==handLpose=
    // ==handRpose=
    e6<<0.20,  -0.426967,  -0.25;//0.15, -0.379501,  0.88;

    e6.head(3) = e6.head(3) -state.RHandPos;
    // e5.tail(3) = e5.tail(3) -state.RHandPos;
    ev6.head(3) = -state.JRHand.block<3,27>(0,0) * state.qd;
    // ev5.tail(3) = -state.JRHand.block<3,27>(0,0) * state.qd;
    Eigen::VectorXd temp6dj = Eigen:: VectorXd::Zero(3);
    temp6dj.head(3) = -state.dJRHand.block<3,27>(0,0) * state.qd;
    // temp5dj.tail(3) = -state.dJRHand.block<3,27>(0,0) * state.qd;
    c6 = 400*e6 + 2*ev6 + temp6dj;   // 
    // file2<<e6(0)<<";"<<e6(1)<<";"<<e6(2)<<";"<< "\n";

    //===============整理===============
    Eigen::MatrixXd w1 = Eigen::MatrixXd::Identity(3,3);
    Eigen::MatrixXd w2 = Eigen::MatrixXd::Identity(12,12);
    Eigen::MatrixXd w3 = Eigen::MatrixXd::Identity(3,3);
    Eigen::MatrixXd w4 = Eigen::MatrixXd::Identity(3,3);
    Eigen::MatrixXd w5 = Eigen::MatrixXd::Identity(3,3);
    Eigen::MatrixXd w6 = Eigen::MatrixXd::Identity(3,3);
    Eigen::MatrixXd w7 = Eigen::MatrixXd::Identity(3,3);



    // Eigen::MatrixXd wI = Eigen::MatrixXd::Zero(2*nq+24-6, 2*nq+24-6);
    // for (int i = 27; i < 46; i++) 
    // {
    //     wI(i,i) = 1000;
    // }
    w1 = 1*w1;
    w4 = 0 * w4;
    w5 = 0.5*w5;
    w6 = 0.5*w6;

    w2 = 0*w2;
    // w3(0)=2*w3(0);
    w3 = 1*w3;

    w7 = 1*w7;

    // w3(2,2) = 0;

    // w1 = 10000*w1;
    // w4 = 0*w4;
    // w5 = 100*w5;
    // w2 = 100000*w2;
    // w3 = 0*w3;
    // std::cout<<"===w5  手臂位置6 =="<<std::endl;
    Eigen::MatrixXd wI = Eigen::MatrixXd::Identity(60, 60);

    Eigen::MatrixXd qp_H =    H1.transpose()*w2 * H1 
                            + H2.transpose()*w1 * H2 
                            + H3.transpose()*w3 * H3
                            + H4.transpose()*w4 * H4 
                            + H5.transpose()*w5 * H5 
                            + H6.transpose()*w6 * H6 
                            + H7.transpose()*w7 * H7 +0.0005*wI;
    Eigen::VectorXd qp_c = Eigen::VectorXd::Zero(60);
    qp_c = -H1.transpose() *w2* c1 
          - H2.transpose() *w1* c2  
          - H3.transpose() *w3* c3
          - H4.transpose() *w4* c4 
          - H5.transpose() *w5* c5
          - H6.transpose() *w6* c6
          - H7.transpose() *w7* c7;


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
    double u = 0.9/sqrt(2);
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
    tempFrUp<< 0    ,1e30,  0   ,1e30,1000,      0,1e30,  0   ,1e30,  100,100,50;
    tempFrLp<<-1e30 ,0   ,-1e30 ,  0 ,0 ,  -1e30 ,0   ,-1e30 ,   0, -100,-100,-50;
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
    limqdd.tail(8) = 1.02*limqdd.tail(8);
    limqdd = 100 * limqdd;
    // limtua.setConstant(1);

    // limtua = 900000000000000*limtua;
    // // limtua(0) = 50;
    // // limtua(6) = 50;
    // // limtua(5) = 40;
    // // limtua(5+6) = 40;
    up.segment(51,21) =  1*limtua;
    lp.segment(51,21) = -1*limtua;

    up.segment(72,27) =  limqdd;
    lp.segment(72,27) = -limqdd;


    up.segment(27,12) = tempFrUp;
    up.segment(39,12) = tempFrUp;

    lp.segment(27,12) = tempFrLp;
    lp.segment(39,12) = tempFrLp;

    OsqpEigen::Solver solver;

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
    std::cout <<"==qdd=="<<tempRes(31)<<std::endl;
    // std::cout <<"==6df=="<<tempRes.segment(48,12).transpose()<<std::endl;
    solver.clearSolver();
    //ax =b  x =  tau(27)  qdd(27)  Fr 6d*2
    Eigen::VectorXd result111 = Eigen::VectorXd::Zero(42);

    result111.head(21) = tempRes.segment(0,21);
    result111.tail(21) = tempRes.segment(21+6,21);
    return result111;

}
tempTest::tempTest(/* args */)
{
    std::cout<<"===tempTest=="<<std::endl;
    string filePath1 = "logger1.txt";
    string filePath2 = "logger2.txt";
    string filePath3 = "logger3.txt";
    file1.open(filePath1);
    file2.open(filePath2);
    file3.open(filePath3);

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
Eigen::VectorXd tempTest::wqp()
{
    // state = robot->getRobotState();
    // // std::cout<<"===wqp=="<<std::endl;
    // Eigen::MatrixXd H = Eigen::MatrixXd::Zero(12,38);
    // // std::cout<<"===wqp1=="<<std::endl;

    // double M = 34.1093;
    // Eigen::Matrix3d M33 = Eigen::Matrix3d::Identity();
    // // std::cout<<"===wqp2=="<<std::endl;

    // Eigen::Matrix3d skewFC1;
    // Eigen::Matrix3d skewFC2;
    // Eigen::Matrix3d skewFC3;
    // Eigen::Matrix3d skewFC4;
    // // std::cout<<"===wqp3=="<<std::endl;

    // skewFC1 = skew(state.Com2LFootPoint1);
    // skewFC2 = skew(state.Com2LFootPoint2);
    // skewFC3 = skew(state.Com2RFootPoint1);
    // skewFC4 = skew(state.Com2RFootPoint2);
    // // std::cout<<"===wqp4=="<<std::endl;

    // // 填充H矩阵
    // H.block<3, 3>(0, 0) = 1*M33;  // 左上角
    // H.block<3, 3>(0, 3) = 1*M33;
    // H.block<3, 3>(0, 6) = 1*M33;
    // H.block<3, 3>(0, 9) = 1*M33;
    // // std::cout<<"===wqp5=="<<std::endl;

    // H.block<3, 27>(0, 12) = - state.baseRot * state.Ag.block<3,27>(0,0);  // 右上角
    // // std::cout<<"===wqp6=="<<std::endl;

    // H.block<3, 3>(3, 0) = skewFC1;  // 左下角
    // H.block<3, 3>(3, 3) = skewFC2;
    // H.block<3, 3>(3, 6) = skewFC3;
    // H.block<3, 3>(3, 9) = skewFC4;
    // // std::cout<<"===wqp7=="<<std::endl;
    // auto tempI = ( state.baseRot.transpose() * state.inertia * state.baseRot);
    // // auto tempI = ( state.baseRot.transpose()* state.inertia);

    // auto tempAg = state.Ag;
    // tempAg.block<3, 27>(0, 0) = state.baseRot * tempAg.block<3, 27>(0, 0)/M;
    // auto tempdAg = state.Ag;
    // tempdAg.block<3, 27>(0, 0) = state.baseRot * tempdAg.block<3, 27>(0, 0)/M;
    // // H.block<6, 27>(6, 12) = state.Jbase;

    // H.block<3, 27>(3, 12) = -tempI.inverse()*(state.baseRot * state.Ag.block<3,27>(3,0)) ; // 右下角
    // // std::cout<<"===wqp8=="<<std::endl;
    // // H.block<3, 27>(6, 12) =  state.Jcom *1; // 右下角
    // Eigen::VectorXd c = Eigen::VectorXd::Zero(12);
    // Eigen::VectorXd pcom = Eigen::VectorXd::Zero(3);
    // Eigen::VectorXd pcom0 = Eigen::VectorXd::Zero(3);
    // // std::cout<<"===wqp12=="<<std::endl;
    // pcom0<< -0.01000,-0,0.87;
    // pcom = state.Xcom;
    // Eigen::VectorXd tempComw = Eigen::VectorXd::Zero(6);
    // Eigen::Quaterniond quat; // 这个四元数代表绕 X 轴 90° 旋转
    // quat.x() = state.q(3);
    // quat.y() = state.q(4);
    // quat.z() = state.q(5);
    // quat.w() = state.q(6);

    // Eigen::AngleAxisd angleAxis(quat);
    // Eigen::Vector3d axis = angleAxis.axis();
    // double angle = angleAxis.angle();
    // axis = angle * axis;
    // tempComw << 0,0,0,axis;
    // // c.tail(6) = -0.000001 *state.Jbase * state.qd + 0.000010*tempComw;
    // // std::cout<<"===wqp9=="<<std::endl;
    // Eigen::VectorXd gg = Eigen::VectorXd::Zero(3);
    // gg<< 0.0,0.0,-9.8;
    // c.head(3) =  state.baseRot * state.dAg.block<3,27>(0,0) * state.qd -  M *gg;//
    // // std::cout<<"===wqp10=="<<std::endl;
    // Eigen::VectorXd w =Eigen::VectorXd::Zero(3);
    // w(0) = state.qd(3);
    // w(1) = state.qd(4);
    // w(2) = state.qd(5);
    // auto w33 = skew(w);

    // // std::cout<<"==pcom=="<<pcom.transpose()<<std::endl;
    // // c.segment(3,3) = state.Ag.block<3,27>(3,0)  * state.qd;
    // c.segment(3,3) =  tempI.inverse()* (state.baseRot *state.dAg.block<3,27>(3,0) * state.qd) ;//+  tempI *w ;
    // Eigen::MatrixXd I = Eigen::MatrixXd::Identity(38,38);
    // I.block<12,12>(0,0)= I.block<12,12>(0,0);
    // I.block<27,27>(12,12)= 5*I.block<27,27>(12,12);
    // Eigen::MatrixXd qp_H = H.transpose() * H;
    // Eigen::VectorXd qp_c = Eigen::VectorXd::Zero(38);
    // qp_c = -H.transpose() * c;

    // double u = 0.05;
    // Eigen::MatrixXd A0 = Eigen::MatrixXd::Zero(5,3);
    // A0<<1, 0,-u,
    //     1, 0,u,
    //     0, 1,-u,
    //     0, 1, u,
    //     0, 0, 1;
    // Eigen::VectorXd tempL = Eigen::VectorXd::Zero(5);
    // tempL<<-100000,0,-100000,0,-500;
    // Eigen::VectorXd tempU= Eigen::VectorXd::Zero(5);
    // tempU<<0,1000000,0,100000,500;


    // Eigen::MatrixXd A = Eigen::MatrixXd::Zero(52-3,38);
    // A.block<5,3>(0,0) = A0;
    // A.block<5,3>(5,3) = A0;
    // A.block<5,3>(5*2,3*2) = A0;
    // A.block<5,3>(5*3,3*3) = A0;
    // // A.block<3,3>(5*4,0) = 0.5*0.0001*M33;
    // // A.block<3,3>(5*4,3) = 0.5*0.0001*M33;
    // // A.block<3,3>(5*4,6) = 0.5*0.0001*M33;
    // // A.block<3,3>(5*4,9) = 0.5*0.0001*M33;

    // // A.block<3,27>(5*4,3*4) = 0.5*0.0001*state.Jcom;
    // A.block<27,27>(5*4,3*4) = Eigen::MatrixXd::Identity(27,27);
    // // std::cout<<"===wqp11=="<<std::endl;

    // Eigen::VectorXd fc_min = Eigen::VectorXd::Zero(20);
    // fc_min.setConstant(-150000);
    // Eigen::VectorXd up = Eigen::VectorXd::Zero(49);
    // Eigen::VectorXd lp = Eigen::VectorXd::Zero(49);
    // up.head(20) =  Eigen::VectorXd::Zero(20);
    // lp.head(20) =  fc_min;
    // up.segment(0,5) = tempU;
    // up.segment(5,5) = tempU;
    // up.segment(10,5) = tempU;
    // up.segment(15,5) = tempU;
    // lp.segment(0,5) = tempL;
    // lp.segment(5,5) = tempL;
    // lp.segment(10,5) = tempL;
    // lp.segment(15,5) = tempL;

    // // up(4) = 250;
    // // up(9) = 250;
    // // up(14)= 250;
    // // up(19)= 250;

    // // lp(4) = -150;
    // // lp(9) = -150;
    // // lp(14)= -150;
    // // lp(19)= -150;
    // Eigen::Vector3d vcom;
    // Eigen::Vector3d dJdqcom;


    // vcom = state.baseRot * state.Ag.block<3,27>(0,0)/M  * state.qd;
    // dJdqcom = 0.5*0.0001 * state.baseRot * state.dAg.block<3,27>(0,0)/M *  state.qd - 0.01*vcom + 0.5*0.0001*gg;
    // // up(20) = pcom0(0) - pcom(0) + 0.015 +dJdqcom(0);
    // // lp(20) = pcom0(0) - pcom(0) - 0.015 +dJdqcom(0);

    // // up(21) = pcom0(1) - pcom(1) + 0.015 +dJdqcom(1);
    // // lp(21) = pcom0(1) - pcom(1) - 0.015 + dJdqcom(1);
    // // up(22) = pcom0(2) - pcom(2) + 0.015 + dJdqcom(2);
    // // lp(22) = pcom0(2) - pcom(2) - 0.015 + dJdqcom(2);
    // // Eigen::VectorXd pcom = Eigen::VectorXd::Zero(3);
    // // up(20) = 0.01*(pcom0(0) - pcom(0) - 0.05*vcom(0)+0.0125/0.01);
    // // lp(20) = 0.01*(pcom0(0) - pcom(0) - 0.05*vcom(0)-0.00125/0.01);

    // // up(21) = 0.01*(pcom0(1) - pcom(1) - 0.05*vcom(1)+0.0125/0.01);
    // // lp(21) = 0.01*(pcom0(1) - pcom(1) - 0.05*vcom(1)-0.0125/0.01);

    // // up(22) = 0.01*(pcom0(2) - pcom(2) - 0.05*vcom(1)+0.05251/0.01);
    // // lp(22) = 0.01*(pcom0(2) - pcom(2) - 0.05*vcom(1)-0.05251/0.01);


    // Eigen::VectorXd accBond = Eigen::VectorXd::Zero(27);
    // accBond.setConstant(10);

    // lp.tail(27) = -accBond;
    // up.tail(27) =  accBond;
    // OsqpEigen::Solver solver;

    //     int n = 38;
    //     solver.settings()->setWarmStart(false);
    //     solver.settings()->setVerbosity(false);
    //     solver.data()->setNumberOfVariables(38);
    //     solver.data()->setNumberOfConstraints(49);
    //     Eigen::SparseMatrix<double> H_sparse = qp_H.sparseView();
    //     if (!solver.data()->setHessianMatrix(H_sparse)) 
    //     {
    //         std::cerr << "Error setting Hessian matrix" << std::endl;
    //         return Eigen::VectorXd::Zero(n);
    //     }
    //     if (!solver.data()->setGradient(qp_c)) {
    //         std::cerr << "Error setting gradient vector" << std::endl;
    //         return Eigen::VectorXd::Zero(n);
    //     }
    //     Eigen::SparseMatrix<double> A_sparse = A.sparseView();
    //     if (!solver.data()->setLinearConstraintsMatrix(A_sparse)) {
    //         std::cerr << "Error setting constraint matrix" << std::endl;
    //         return Eigen::VectorXd::Zero(n);
    //     }
    //     if (!solver.data()->setLowerBound(lp)) {
    //         std::cerr << "Error setting lower bounds" << std::endl;
    //         return Eigen::VectorXd::Zero(n);
    //     }
    //     if (!solver.data()->setUpperBound(up)) {
    //         std::cerr << "Error setting upper bounds" << std::endl;
    //         return Eigen::VectorXd::Zero(n);
    //     }
    //     if (!solver.initSolver()) {
    //         std::cerr << "Solver initialization failed" << std::endl;
    //         return Eigen::VectorXd::Zero(n);
    //     }
    //     auto result = solver.solve();
    //     if (!solver.solve())
    //     {
    //         std::cout<<"========="<<std::endl;
    //         return Eigen::VectorXd::Zero(n);
    //     }
    //     auto tempRes = solver.getSolution();

    //     // std::cout <<tempRes.head(12).transpose()<<std::endl;
    //     solver.clearSolver();
    //     Eigen::VectorXd f1 = Eigen::VectorXd::Zero(6);  // 第 1 个接触点的力
    //     Eigen::VectorXd f2 = Eigen::VectorXd::Zero(6);  // 第 2 个接触点的力
    //     Eigen::VectorXd f3 = Eigen::VectorXd::Zero(6);  // 第 3 个接触点的力
    //     Eigen::VectorXd f4 = Eigen::VectorXd::Zero(6);  // 第 4 个接触点的力
    //     f1.head(3) = tempRes.head(3);
    //     f2.head(3) = tempRes.segment(3,3);
    //     f3.head(3) = tempRes.segment(6,3);
    //     f4.head(3) = tempRes.segment(9,3);
    //     Eigen::VectorXd JcT_f = state.JLFoot1.transpose() * f1 +
    //                             state.JLFoot2.transpose() * f2 +
    //                             state.JRFoot1.transpose() * f3 +
    //                             state.JRFoot2.transpose() * f4;
    //     auto cmdtau = state.DynM * tempRes.tail(27) + state.DynC * state.qd +state.DynG - JcT_f;
    //     Eigen::VectorXd result111 = Eigen::VectorXd::Zero(40);

    //     result111.head(20) = cmdtau.segment(6,20);
    //     result111.tail(20) = tempRes.tail(20);
    //     // std::cout<<"=-================"<<result111.size()<<std::endl;
    //     return result111;

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