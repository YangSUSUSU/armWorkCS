#ifndef ROBOT_PARA_HPP
#define ROBOT_PARA_HPP

#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>

#include <Eigen/Dense>
#include <string>
#include <memory>
#include <mutex>

class RobotPara
{
public:
    RobotPara(const RobotPara&) = delete;
    RobotPara& operator=(const RobotPara&) = delete;

    static RobotPara* instance() 
    {
        static std::mutex mutex;  
        if (!m_instance) {
            std::lock_guard<std::mutex> lock(mutex); 
            if (!m_instance) {
                m_instance = new RobotPara(); 
            }
        }
        return m_instance;
    }


    static void destroy() 
    {
        if (m_instance) 
        {
            delete m_instance;
            m_instance = nullptr;
        }
    }


    void setQ(const Eigen::VectorXd& q_);
    void setQd(const Eigen::VectorXd& qd_);
    Eigen::MatrixXd getM();
    Eigen::MatrixXd getC();
    Eigen::VectorXd getG();
    Eigen::VectorXd getArmTcp();
    Eigen::MatrixXd getJacL();
    Eigen::MatrixXd getJacR();

private:

    pinocchio::Model model_;        
    pinocchio::Data data_;
    Eigen::VectorXd q;
    Eigen::VectorXd tcp;
    Eigen::VectorXd qd;
    Eigen::VectorXd G;
    Eigen::MatrixXd M;
    Eigen::MatrixXd C;
    Eigen::MatrixXd jac_l;
    Eigen::MatrixXd jac_r;

    RobotPara() {}  // 私有化构造函数，防止外部直接创建实例

    static RobotPara* m_instance;  // 单例实例指针
};

#endif // ROBOT_PARA_HPP
