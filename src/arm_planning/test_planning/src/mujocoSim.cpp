
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <cstdio>
#include <iostream>
#include "GLFW_callbacks.h"
#include "MJ_interface.h"
#include <functional>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "mujocoSimNode");
    char error[1000] = "Could not load binary model";
    mjModel* mj_model = mj_loadXML("/home/nikoo/workWS/armWorkCS/src/arm_planning/test_planning/model3/max3new.xml", 0, error, 5000);
    mjData* mj_data = mj_makeData(mj_model);
    UIctr uiController(mj_model,mj_data);   // UI control for Mujoco
    MJ_Interface mj_interface(mj_model, mj_data); // data interface for Mujoco
    double simEndTime=20;
    mjtNum simstart = mj_data->time;
    double simTime = mj_data->time;
    // init UI: GLFW
    uiController.iniGLFW();
    uiController.enableTracking(); // enable viewpoint tracking of the body 1 of the robot
    uiController.createWindow("Demo",false);
    // mj_interface.joint_state_sub_ = mj_interface. nh_.subscribe("/joint_states", 10, &MJ_Interface::jointStateCallback, this);

    while(1)
    {
        simstart=mj_data->time;
        while( mj_data->time - simstart < 1.0/500)
        {
            ros::spinOnce();

    // mj_step(mj_model, mj_data);
            Eigen::VectorXd t;
            mj_interface.setMotorsTorque(t);

            mj_interface.updateSensorValues();
            // mj_forward(mj_model, mj_data);
            mj_step(mj_model, mj_data);
            simTime=mj_data->time;
            printf("-------------%.3d s------------\n",simTime);
            
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