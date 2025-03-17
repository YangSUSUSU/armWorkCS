
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
#include <Eigen/core.hpp>
class tempTest
{
private:
    /* data */
public:
     tempTest(/* args */);
    ~tempTest();
    skew(Eigen::VectorXd& v);
    Eigen::VectorXdwqp();
    RobotData *robot;

    // 获取机器人状态
    RobotStructs state;

};

test1::tempTest(/* args */)
{
        std::cout<<"===tempTest=="<<std::endl;

    robot = &RobotData::getInstance();
    state = robot.getRobotState();
}
Eigen::Matrix3d test1::skew(Eigen::VectorXd& v)
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
Eigen::VectorXd test1::wqp()
{
    std::cout<<"===wqp=="<<std::endl;
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(6,38);
    double M = 44.9956;
    Eigen::Matrix3d M33 = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d skewFC1;
    Eigen::Matrix3d skewFC2;
    Eigen::Matrix3d skewFC3;
    Eigen::Matrix3d skewFC4;
    skewFC1 = skew(state.Com2LFootPoint1);
    skewFC2 = skew(state.Com2LFootPoint2);
    skewFC3 = skew(state.Com2RFootPoint1);
    skewFC4 = skew(state.Com2RFootPoint2);
    // 填充H矩阵
    H.block<3, 3>(0, 0) = M33;  // 左上角
    H.block<3, 3>(0, 3) = M33;
    H.block<3, 3>(0, 6) = M33;
    H.block<3, 3>(0, 9) = M33;

    H.block<3, 26>(0, 12) = -state.Ag.block<3,26>(0,0);  // 右上角

    H.block<3, 3>(3, 0) = skewFC1;  // 左下角
    H.block<3, 3>(3, 3) = skewFC2;
    H.block<3, 3>(3, 6) = skewFC3;
    H.block<3, 3>(3, 9) = skewFC4;

    H.block<3, 26>(3, 12) = -state.inertia*state.Ag.block<3,26>(3,0); // 右下角
    Eigen::VectorXd c = Eigen::VecotrXd::Zero(6);
    c.head(3) = state.dAg.block<3,26>(0,0) * state.qd;
    c.tail(3) = state.inertia * state.dAg.block<3,26>(3,0) * state.qd;
    Eigen::MatrixXd qp_H = H.transpose() * H
    Eigen::VectorXd qp_c = Eigen::VecotrXd::Zero(38);
    qp_c = H.transpose() * c;
    double u = 0.05;
    Eigen::MatrixXd A0 = Eigen::MatrixXd::Zero(4,3);
    A0<<1, 0,u,
       -1, 0,u,
        0, 1,u,
        0,-1,u;


    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(48,38);
    A.block<4,3>(0,0) = A0;
    A.block<4,3>(4,3) = A0;
    A.block<4,3>(4*2,3*2) = A0;
    A.block<4,3>(4*3,3*3) = A0;
    A.block<3,26>(4*4,3*4) = state.Jcom;
    A.block<26,26>(4*4+3,3*4) = Eigen::MatrixXd::Identity(26,26);

    Eigen::VectorXd fc_min = Eigen::VectorXd::Zero(16);
    fc_min.setConstant(-1000);
    Eigen::VectorXd up = Eigen::VectorXd::Zero(48);
    Eigen::VectorXd lp = Eigen::VectorXd::Zero(48);
    up.head(16) =  Eigen::VectorXd::Zero(15);
    lp.head(16) =  fc_min;
    Eigen::VectorXd pcom = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd pcom0 = Eigen::VectorXd::Zero(3);
    pcom0<< 0.0186,-0.00001,0.894707;
    pcom = robotStructs.Xcom;
    up(16) = pcom0 - pcom +0.05;
    lp(16) = pcom0 - pcom -0.05;

    up(17) = pcom0 - pcom +0.05;
    lp(17) = pcom0 - pcom -0.05;

    up(18) = pcom0 - pcom +0.1;
    lp(18) = pcom0 - pcom -0.1;


    Eigen::VectorXd accBond = Eigen::VectorXd::Zero(26);
    accBond.setConstant(5);

    lp.tail(26) = -accBond;
    up.tail(26) =  accBond;
    OsqpEigen::Solver solver;

        int n = 38;
        solver.settings()->setWarmStart(false);
        solver.settings()->setVerbosity(false);
        solver.data()->setNumberOfVariables(38);
        solver.data()->setNumberOfConstraints(48);
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
        Eigen::SparseMatrix<double> A_sparse = A.sparseView();
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

        std::cout <<tempRes.head(12).transpose()<<std::endl;
        solver.clearSolver();


}
test1::~tempTest()
{
}

int main(int argc, char* argv[]) 
{

    // char error[1000] = "Could not load binary model";
    // mjModel* mj_model = mj_loadXML("/home/nikoo/workWS/armWorkCS/src/wbc/models/x_humanoid_ultra_noeyes/mjcf/mjmodel.xml", 0, error, 5000);
    // mjData* mj_data = mj_makeData(mj_model);
    // UIctr uiController(mj_model,mj_data);   // UI control for Mujoco
    // MJ_Interface mj_interface(mj_model, mj_data); // data interface for Mujoco
    // std::cout << "mj ok" << std::endl;
    // tempTest Test;
    // // std::cout<<"===tempTest Test;=="<<std::endl;

    // double simEndTime=20;
    // mjtNum simstart = mj_data->time;
    // double simTime = mj_data->time;
    // // init UI: GLFW
    // uiController.iniGLFW();
    // uiController.enableTracking(); // enable viewpoint tracking of the body 1 of the robot
    // uiController.createWindow("Demo",false);
    // // mj_interface.joint_state_sub_ = mj_interface. nh_.subscribe("/joint_states", 10, &MJ_Interface::jointStateCallback, this);

    // while(1)
    // {
    //     simstart=mj_data->time;
    //     while( mj_data->time - simstart < 1.0/500)
    //     {
    //         std::cout<<"===wqp=="<<std::endl;
    //         Eigen::VectorXd t;
    //         mj_interface.setMotorsTorque(t);
    //         mj_interface.updateSensorValues();
    //         mj_interface.robotStateUpData();
    //         mj_forward(mj_model, mj_data);
    //         mj_step(mj_model, mj_data);
    //         simTime=mj_data->time;
    //         robot.getRobotState();
    //         Test.wqp();
    //         // std::cout<<"==单例输出="<<(m_state.q).transpose()<<std::endl;
    //         // printf("-------------%.3d s------------\n",simTime);
            
    //     }
    //     uiController.updateScene();

    // }
    // //    // free visualization storage
    // uiController.Close();

    // // free MuJoCo model and data, deactivate
    // mj_deleteData(mj_data);
    // mj_deleteModel(mj_model);
    return 0;
}