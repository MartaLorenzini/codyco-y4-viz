#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <geometry_msgs/WrenchStamped.h>
#include "iDynTree/Model/Traversal.h"
#include <iDynTree/Core/TestUtils.h>
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/ModelIO/ModelLoader.h>
#include <matio.h>
#include <URDFdir.h>

struct wrench {
    double Fx;
    double Fy;
    double Fz;
    double Mx;
    double My;
    double Mz;
  };

int main(int argc, char **argv){

    ros::init(argc, argv, "wrench_stamped_publisher");
    ros::NodeHandle n;

    ros::Publisher wrench_pub = n.advertise<geometry_msgs::WrenchStamped>("wrench_stamped", 1);
    ros::Rate loop_rate(30);

    //LOAD OF robot.mat
    mat_t *pRobot;

    std::cerr<<"Load robot.mat at "<<getAbsModelPath("robot.mat")<< std::endl;
    pRobot = Mat_Open(getAbsModelPath("robot.mat").c_str(),MAT_ACC_RDONLY);
    iDynTree::assertTrue(pRobot!=NULL);
    std::cerr << "Robot loaded" << std::endl;

    matvar_t *robotVar;
    robotVar = Mat_VarRead(pRobot,"robot");
    iDynTree::assertTrue(robotVar != NULL);
    std::cerr<<"Found robot variable"<<std::endl;

    matvar_t *robotDataVar;
    robotDataVar = Mat_VarGetStructFieldByName(robotVar,"processedData",0);
    iDynTree::assertTrue(robotDataVar != NULL);
    std::cerr<<"Found 'data' variable with dimension "<< robotDataVar->dims[0] << std::endl;

    matvar_t *robotStateRightArm;
    robotStateRightArm = Mat_VarGetStructFieldByName(robotDataVar, "humanRightHandWrench", 0);
    iDynTree::assertTrue(robotStateRightArm != NULL);
    std::cerr << "Dimensions: "<< robotStateRightArm->dims[0] << "x" << robotStateRightArm->dims[1] << std::endl;
    Eigen::Map< Eigen::MatrixXd > mapRobotStateRightArm((double*)robotStateRightArm->data, robotStateRightArm->dims[0], robotStateRightArm->dims[1]);
    iDynTree::MatrixDynSize robotStateRightArm_i(robotStateRightArm->dims[0], robotStateRightArm->dims[1]);
    iDynTree::toEigen(robotStateRightArm_i) = mapRobotStateRightArm;

    matvar_t *robotStateLeftArm;
    robotStateLeftArm = Mat_VarGetStructFieldByName(robotDataVar, "humanLeftHandWrench", 0);
    iDynTree::assertTrue(robotStateLeftArm != NULL);
    std::cerr << "Dimensions: "<< robotStateLeftArm->dims[0] << "x" << robotStateLeftArm->dims[1] << std::endl;
    Eigen::Map< Eigen::MatrixXd > mapRobotStateLeftArm((double*)robotStateLeftArm->data, robotStateLeftArm->dims[0], robotStateLeftArm->dims[1]);
    iDynTree::MatrixDynSize robotStateLeftArm_i(robotStateLeftArm->dims[0], robotStateLeftArm->dims[1]);
    iDynTree::toEigen(robotStateLeftArm_i) = mapRobotStateLeftArm;

    // message declarations
    geometry_msgs::WrenchStamped wrenchMsg;
    ros::param::param<std::string>("~frame_id", wrenchMsg.header.frame_id, "l_forearm");

    int j;
    double null_F = 0, null_M = 0;
    wrench msgStream;
    msgStream.Fx = 0.5;
    msgStream.Fy = 0.5;
    msgStream.Fz = 1;
    msgStream.Mx = 0.5;
    msgStream.My = 0.5;
    msgStream.Mz = -1.1;

    //msg.stream = getdata();

    while (ros::ok()){

        j++;
        if(j > robotStateRightArm->dims[1]) j = 0;

        //compose WrenchStamped Msg
        wrenchMsg.header.stamp = ros::Time::now();

        wrenchMsg.wrench.force.x = 0.05*robotStateLeftArm_i.getVal(0,j);
        wrenchMsg.wrench.force.y= 0.05*robotStateLeftArm_i.getVal(1,j);
        wrenchMsg.wrench.force.z = 0.05*robotStateLeftArm_i.getVal(2,j);
        wrenchMsg.wrench.torque.x = 0.05*robotStateLeftArm_i.getVal(3,j);
        wrenchMsg.wrench.torque.y = 0.05*robotStateLeftArm_i.getVal(4,j);
        wrenchMsg.wrench.torque.z = 0.05*robotStateLeftArm_i.getVal(5,j);

        wrench_pub.publish(wrenchMsg);

        // This will adjust as needed per iteration
        loop_rate.sleep();
      }
    return 0;
  }