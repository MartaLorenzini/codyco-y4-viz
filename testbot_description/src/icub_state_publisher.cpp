#include <ros/ros.h>
#include <string>
#include <std_msgs/Header.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include "iDynTree/Model/Traversal.h"
#include <iDynTree/Core/TestUtils.h>
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/ModelIO/ModelLoader.h>
#include <matio.h>
#include <URDFdir.h>


int main(int argc, char **argv){

    ros::init(argc, argv, "icub_state_publisher");
    ros::NodeHandle n;

    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    tf::TransformBroadcaster broadcaster;
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
    robotDataVar = Mat_VarGetStructFieldByName(robotVar,"data",0);
    iDynTree::assertTrue(robotDataVar != NULL);
    std::cerr<<"Found 'data' variable with dimension "<< robotDataVar->dims[0] << std::endl;

    matvar_t *robotState;
    robotState = Mat_VarGetStructFieldByName(robotDataVar,"q",0);
    iDynTree::assertTrue(robotState != NULL);
    std::cerr<<"Found 'q' variable with dimension "<< robotState->dims[0] << std::endl;

    matvar_t *robotStateRightArm;
    robotStateRightArm = Mat_VarGetStructFieldByName(robotState, "rightArm", 0);
    iDynTree::assertTrue(robotStateRightArm != NULL);
    std::cerr << "Dimensions: "<< robotStateRightArm->dims[0] << "x" << robotStateRightArm->dims[1] << std::endl;
    Eigen::Map< Eigen::MatrixXd > mapRobotStateRightArm((double*)robotStateRightArm->data, robotStateRightArm->dims[0], robotStateRightArm->dims[1]);
    iDynTree::MatrixDynSize robotStateRightArm_i(robotStateRightArm->dims[0], robotStateRightArm->dims[1]);
    iDynTree::toEigen(robotStateRightArm_i) = mapRobotStateRightArm;

    matvar_t *robotStateRightLeg;
    robotStateRightLeg = Mat_VarGetStructFieldByName(robotState, "rightLeg", 0);
    iDynTree::assertTrue(robotStateRightLeg != NULL);
    std::cerr << "Dimensions: "<< robotStateRightLeg->dims[0] << "x" << robotStateRightLeg->dims[1] << std::endl;
    Eigen::Map< Eigen::MatrixXd > mapRobotStateRightLeg((double*)robotStateRightLeg->data, robotStateRightLeg->dims[0], robotStateRightLeg->dims[1]);
    iDynTree::MatrixDynSize robotStateRightLeg_i(robotStateRightLeg->dims[0], robotStateRightLeg->dims[1]);
    iDynTree::toEigen(robotStateRightLeg_i) = mapRobotStateRightLeg;

    matvar_t *robotStateLeftArm;
    robotStateLeftArm = Mat_VarGetStructFieldByName(robotState, "leftArm", 0);
    iDynTree::assertTrue(robotStateLeftArm != NULL);
    std::cerr << "Dimensions: "<< robotStateLeftArm->dims[0] << "x" << robotStateLeftArm->dims[1] << std::endl;
    Eigen::Map< Eigen::MatrixXd > mapRobotStateLeftArm((double*)robotStateLeftArm->data, robotStateLeftArm->dims[0], robotStateLeftArm->dims[1]);
    iDynTree::MatrixDynSize robotStateLeftArm_i(robotStateLeftArm->dims[0], robotStateLeftArm->dims[1]);
    iDynTree::toEigen(robotStateLeftArm_i) = mapRobotStateLeftArm;

    matvar_t *robotStateLeftLeg;
    robotStateLeftLeg = Mat_VarGetStructFieldByName(robotState, "leftLeg", 0);
    iDynTree::assertTrue(robotStateLeftLeg != NULL);
    std::cerr << "Dimensions: "<< robotStateLeftLeg->dims[0] << "x" << robotStateLeftLeg->dims[1] << std::endl;
    Eigen::Map< Eigen::MatrixXd > mapRobotStateLeftLeg((double*)robotStateLeftLeg->data, robotStateLeftLeg->dims[0], robotStateLeftLeg->dims[1]);
    iDynTree::MatrixDynSize robotStateLeftLeg_i(robotStateLeftLeg->dims[0], robotStateLeftLeg->dims[1]);
    iDynTree::toEigen(robotStateLeftLeg_i) = mapRobotStateLeftLeg;

    matvar_t *robotStateTorso;
    robotStateTorso = Mat_VarGetStructFieldByName(robotState, "torso", 0);
    iDynTree::assertTrue(robotStateTorso != NULL);
    std::cerr << "Dimensions: "<< robotStateTorso->dims[0] << "x" << robotStateTorso->dims[1] << std::endl;
    Eigen::Map< Eigen::MatrixXd > mapRobotStateTorso((double*)robotStateTorso->data, robotStateTorso->dims[0], robotStateTorso->dims[1]);
    iDynTree::MatrixDynSize robotStateTorso_i(robotStateTorso->dims[0], robotStateTorso->dims[1]);
    iDynTree::toEigen(robotStateTorso_i) = mapRobotStateTorso;

    // declarations
    int i = 0, j = 0;
    const double degree = M_PI/180;
    double angle = 0;

    //robot state
    double zero_pos = 0, zero_vel = 0, zero_effort = 0,
    inc = degree, var = 0,
    speed = 1, wrench = 0, wrench_inc = 1;

    // message declarations
    sensor_msgs::JointState joint_state;

    geometry_msgs::TransformStamped ground_to_base;
    ground_to_base.header.frame_id = "ground";
    ground_to_base.child_frame_id = "base_link";

    geometry_msgs::TransformStamped going_around;
    going_around.header.frame_id = "ground";
    going_around.child_frame_id = "base_link";

    // joint_state setting
    joint_state.name.resize(55);
    joint_state.name[0] ="torso_yaw";
    joint_state.name[1] ="chest_skin_frame_fixed_joint";
    joint_state.name[2] ="codyco_balancing_world_fixed_joint";
    joint_state.name[3] ="neck_yaw";
    joint_state.name[4] ="imu_frame_fixed_joint";
    joint_state.name[5] ="l_ankle_pitch";
    joint_state.name[6] ="l_ankle_roll";
    joint_state.name[7] ="l_elbow";
    joint_state.name[8] ="l_foot_ft_sensor";
    joint_state.name[9] ="l_foot_dh_frame_fixed_joint";
    joint_state.name[10] ="l_wrist_prosup";
    joint_state.name[11] ="l_forearm_dh_frame_fixed_joint";
    joint_state.name[12] ="l_gripper_joint";
    joint_state.name[13] ="l_wrist_yaw";
    joint_state.name[14] ="l_hand_dh_frame_fixed_joint";
    joint_state.name[15] ="l_hip_pitch";
    joint_state.name[16] ="l_hip_roll";
    joint_state.name[17] ="l_leg_ft_sensor";
    joint_state.name[18] ="l_knee";
    joint_state.name[19] ="l_shoulder_pitch";
    joint_state.name[20] ="l_shoulder_roll";
    joint_state.name[21] ="l_shoulder_yaw";
    joint_state.name[22] ="l_sole_fixed_joint";
    joint_state.name[23] ="l_arm_ft_sensor";
    joint_state.name[24] ="l_upper_arm_dh_frame_fixed_joint";
    joint_state.name[25] ="l_hip_yaw";
    joint_state.name[26] ="l_wrist_pitch";
    joint_state.name[27] ="neck_pitch";
    joint_state.name[28] ="neck_roll";
    joint_state.name[29] ="r_ankle_pitch";
    joint_state.name[30] ="r_ankle_roll";
    joint_state.name[31] ="r_elbow";
    joint_state.name[32] ="r_foot_ft_sensor";
    joint_state.name[33] ="r_foot_dh_frame_fixed_joint";
    joint_state.name[34] ="r_wrist_prosup";
    joint_state.name[35] ="r_forearm_dh_frame_fixed_joint";
    joint_state.name[36] ="r_gripper_joint";
    joint_state.name[37] ="r_wrist_yaw";
    joint_state.name[38] ="r_hand_dh_frame_fixed_joint";
    joint_state.name[39] ="r_hip_pitch";
    joint_state.name[40] ="r_hip_roll";
    joint_state.name[41] ="r_leg_ft_sensor";
    joint_state.name[42] ="r_knee";
    joint_state.name[43] ="r_shoulder_pitch";
    joint_state.name[44] ="r_shoulder_roll";
    joint_state.name[45] ="r_shoulder_yaw";
    joint_state.name[46] ="r_sole_fixed_joint";
    joint_state.name[47] ="r_arm_ft_sensor";
    joint_state.name[48] ="r_upper_arm_dh_frame_fixed_joint";
    joint_state.name[49] ="r_hip_yaw";
    joint_state.name[50] ="r_wrist_pitch";
    joint_state.name[51] ="base_fixed_joint";
    joint_state.name[52] ="torso_joint";
    joint_state.name[53] ="torso_pitch";
    joint_state.name[54] ="torso_roll";

    joint_state.position.resize(55);
    joint_state.velocity.resize(55);
    joint_state.effort.resize(55);

    while (ros::ok()) {

        //update joint_state
        joint_state.header.stamp = ros::Time::now();

        j++;
        if(j > robotStateRightArm->dims[1]) j = 0;
        joint_state.position[0] =robotStateTorso_i.getVal(0,j);
        joint_state.position[5] =robotStateLeftLeg_i.getVal(4,j);
        joint_state.position[6] =robotStateLeftLeg_i.getVal(5,j);
        joint_state.position[7] =robotStateLeftArm_i.getVal(3,j);
        joint_state.position[10] =robotStateLeftArm_i.getVal(4,j);
        joint_state.position[13] =robotStateLeftArm_i.getVal(6,j);
        joint_state.position[15] =robotStateLeftLeg_i.getVal(0,j);
        joint_state.position[16] =robotStateLeftLeg_i.getVal(1,j);
        joint_state.position[18] =robotStateLeftLeg_i.getVal(3,j);
        joint_state.position[19] =robotStateLeftArm_i.getVal(0,j);
        joint_state.position[20] =robotStateLeftArm_i.getVal(1,j);
        joint_state.position[21] =robotStateLeftArm_i.getVal(2,j);
        joint_state.position[25] =robotStateLeftLeg_i.getVal(2,j);
        joint_state.position[26] =robotStateLeftArm_i.getVal(5,j);
        joint_state.position[29] =robotStateRightLeg_i.getVal(4,j);
        joint_state.position[30] =robotStateRightLeg_i.getVal(5,j);
        joint_state.position[31] =robotStateRightArm_i.getVal(3,j);
        joint_state.position[34] =robotStateRightArm_i.getVal(4,j);
        joint_state.position[37] =robotStateRightArm_i.getVal(6,j);
        joint_state.position[39] =robotStateRightLeg_i.getVal(0,j);
        joint_state.position[40] =robotStateRightLeg_i.getVal(1,j);
        joint_state.position[42] =robotStateRightLeg_i.getVal(3,j);
        joint_state.position[43] =robotStateRightArm_i.getVal(0,j);
        joint_state.position[44] =robotStateRightArm_i.getVal(1,j);
        joint_state.position[45] =robotStateRightArm_i.getVal(2,j);
        joint_state.position[49] =robotStateRightLeg_i.getVal(2,j);
        joint_state.position[50] =robotStateRightArm_i.getVal(5,j);
        joint_state.position[53] =robotStateTorso_i.getVal(2,j);
        joint_state.position[54] =robotStateTorso_i.getVal(1,j);


        // update transform

        // ground to Pelvis transform
        ground_to_base.header.stamp = ros::Time::now();
        ground_to_base.transform.translation.x = 0.68;
        ground_to_base.transform.translation.y = 0.08;
        ground_to_base.transform.translation.z = 0.6;
        ground_to_base.transform.rotation.x = 0;
        ground_to_base.transform.rotation.y = 0;
        ground_to_base.transform.rotation.z = 0;
        ground_to_base.transform.rotation.w = 1;

        //ground_to_base.transform.rotation = tf::createQuaternionMsgFromYaw();

        // (moving in a circle with radius=2)
        going_around.header.stamp = ros::Time::now();
        going_around.transform.translation.x = sin(angle)*2;
        going_around.transform.translation.y = cos(angle)*2;
        going_around.transform.translation.z = 0.6;
        going_around.transform.rotation = tf::createQuaternionMsgFromYaw(angle+M_PI/2);
        angle += degree/4;

        // Send the joint state
        joint_pub.publish(joint_state);

        // send the transform
        broadcaster.sendTransform(ground_to_base);

        // This will adjust as needed per iteration
        loop_rate.sleep();
      }

      std::cerr<<"Closing robot.mat file."<<std::endl;
      Mat_Close(pRobot);

    return 0;

  }