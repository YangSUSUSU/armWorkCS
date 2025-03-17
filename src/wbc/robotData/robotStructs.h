#ifndef ROBOT_STRUCTS_H
#define ROBOT_STRUCTS_H

#include <Eigen/Dense>

struct RobotStructs 
{
    Eigen::VectorXd Xcom;       
    Eigen::VectorXd Xdcom;     
    Eigen::VectorXd Xddcom; 
    Eigen::VectorXd rpy; 

    //  浮动基 7 + n   
    Eigen::VectorXd q;     
    //  浮动基 6 + n   
    Eigen::VectorXd qd;     
    Eigen::VectorXd qdd;   
    Eigen::VectorXd LFootPoint1;
    Eigen::VectorXd LFootPoint2;

    Eigen::VectorXd LHandPos;
    Eigen::VectorXd RHandPos;



    Eigen::VectorXd RFootPoint1;
    Eigen::VectorXd RFootPoint2;


    Eigen::VectorXd Com2RFootPoint1;
    Eigen::VectorXd Com2RFootPoint2;
    Eigen::VectorXd Com2LFootPoint1;
    Eigen::VectorXd Com2LFootPoint2;

    Eigen::MatrixXd JLFoot1;
    Eigen::MatrixXd JLFoot2;
    Eigen::MatrixXd JLFoot3;
    Eigen::MatrixXd JLFoot4;
    Eigen::MatrixXd JRFoot3;
    Eigen::MatrixXd JRFoot4;
    Eigen::MatrixXd JRFoot1;
    Eigen::MatrixXd JRFoot2;
    Eigen::MatrixXd Jbase;
    Eigen::MatrixXd dJbase;

    Eigen::MatrixXd RotFoot1;
    Eigen::MatrixXd RotFoot2;

    Eigen::MatrixXd dJLFoot1;
    Eigen::MatrixXd dJLFoot2;

    Eigen::MatrixXd dJLFoot3;
    Eigen::MatrixXd dJLFoot4;
    Eigen::MatrixXd dJRFoot1;
    Eigen::MatrixXd dJRFoot2;
    Eigen::MatrixXd dJRFoot3;
    Eigen::MatrixXd dJRFoot4;

    Eigen::MatrixXd JLHand;
    Eigen::MatrixXd JRHand;
    Eigen::MatrixXd dJLHand;
    Eigen::MatrixXd dJRHand;
    Eigen::MatrixXd inertia;
    Eigen::MatrixXd Jcom;
    Eigen::MatrixXd Jd;

    Eigen::MatrixXd Jworld;
    Eigen::MatrixXd Jbody;

    Eigen::MatrixXd baseRot;

    Eigen::MatrixXd Ag;
    Eigen::MatrixXd dAg;
    Eigen::MatrixXd DynM;
    Eigen::MatrixXd DynMinv;
    Eigen::MatrixXd DynC;
    Eigen::VectorXd DynG;


    // 构造函数：初始化所有向量和矩阵
    RobotStructs()
        : baseRot(Eigen::MatrixXd::Zero(3, 3)),
          q(Eigen::VectorXd::Zero(28)), 
          rpy(Eigen::VectorXd::Zero(3)), 

          RHandPos(Eigen::VectorXd::Zero(3)), 
          LHandPos(Eigen::VectorXd::Zero(3)), 

          qd(Eigen::VectorXd::Zero(27)), 
          qdd(Eigen::VectorXd::Zero(27)),
          Xcom(Eigen::VectorXd::Zero(3)), 
          Xdcom(Eigen::VectorXd::Zero(3)), 
          Xddcom(Eigen::VectorXd::Zero(3)),
          LFootPoint1(Eigen::VectorXd::Zero(3)), 
          LFootPoint2(Eigen::VectorXd::Zero(3)), 
          RFootPoint1(Eigen::VectorXd::Zero(3)), 
          RFootPoint2(Eigen::VectorXd::Zero(3)), 
          Com2RFootPoint1(Eigen::VectorXd::Zero(3)), 
          Com2LFootPoint1(Eigen::VectorXd::Zero(3)),
          Com2RFootPoint2(Eigen::VectorXd::Zero(3)), 
          Com2LFootPoint2(Eigen::VectorXd::Zero(3)),
          RotFoot1(Eigen::MatrixXd::Zero(3, 3)),
          RotFoot2(Eigen::MatrixXd::Zero(3, 3)),
          inertia(Eigen::MatrixXd::Zero(3, 3)),
          dJRHand(Eigen::MatrixXd::Zero(6, 27)),
          dJLHand(Eigen::MatrixXd::Zero(6, 27)),
          JRHand(Eigen::MatrixXd::Zero(6, 27)),
          JLHand(Eigen::MatrixXd::Zero(6, 27)),
          JLFoot1(Eigen::MatrixXd::Zero(6, 27)),
          JLFoot2(Eigen::MatrixXd::Zero(6, 27)), 
          JLFoot3(Eigen::MatrixXd::Zero(6, 27)),
          JLFoot4(Eigen::MatrixXd::Zero(6, 27)), 
          JRFoot3(Eigen::MatrixXd::Zero(6, 27)),
          JRFoot4(Eigen::MatrixXd::Zero(6, 27)),
          JRFoot1(Eigen::MatrixXd::Zero(6, 27)),
          JRFoot2(Eigen::MatrixXd::Zero(6, 27)),
          Jbase(Eigen::MatrixXd::Zero(6, 27)),
          dJbase(Eigen::MatrixXd::Zero(6, 27)),
          dJLFoot1(Eigen::MatrixXd::Zero(6, 27)),
          dJLFoot2(Eigen::MatrixXd::Zero(6, 27)), 
          dJLFoot3(Eigen::MatrixXd::Zero(6, 27)),
          dJLFoot4(Eigen::MatrixXd::Zero(6, 27)), 
          dJRFoot1(Eigen::MatrixXd::Zero(6, 27)),
          dJRFoot2(Eigen::MatrixXd::Zero(6, 27)),
          dJRFoot3(Eigen::MatrixXd::Zero(6, 27)),
          dJRFoot4(Eigen::MatrixXd::Zero(6, 27)),
          Jcom(Eigen::MatrixXd::Zero(3, 27)), 
          Jd(Eigen::MatrixXd::Zero(6, 27)),
          Jworld(Eigen::MatrixXd::Zero(6, 27)), 
          Jbody(Eigen::MatrixXd::Zero(6, 27)),
          Ag(Eigen::MatrixXd::Zero(6, 27)),
          dAg(Eigen::MatrixXd::Zero(6, 27)),
          DynM(Eigen::MatrixXd::Zero(27, 27)),
          DynMinv(Eigen::MatrixXd::Zero(27, 27)),
          DynC(Eigen::MatrixXd::Zero(27, 27)),
          DynG(Eigen::VectorXd::Zero(27)){}
};

#endif // ROBOT_STRUCTS_H
