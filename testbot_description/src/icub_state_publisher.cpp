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
#include <stdarg.h>
#include <string>
#include <sstream>
#include <vector>

template<typename Out>
void split(const std::string &s, char delim, Out result);
std::vector<std::string> split(const std::string &s, char delim);

inline iDynTree::Transform transformFromMatlabExtractor_byString(std::string matVarName, std::string matVarString);
inline iDynTree::Transform transformFromMatlabExtractor(std::string matName, int n, ...);

inline iDynTree::MatrixDynSize matrixFromMatlabExtractor_byString(std::string matVarName, std::string matVarString);
inline iDynTree::MatrixDynSize matrixFromMatlabExtractor(std::string matName, int n, ...);

inline std::vector< iDynTree::MatrixDynSize > suitLinksFromMatlabExtractor_byString(std::string matVarName, std::string matVarString);
inline std::vector< iDynTree::MatrixDynSize > suitQuaternionsFromMatlabExtractor_byString(std::string matVarName, std::string matVarString);

int main(int argc, char **argv){

    ros::init(argc, argv, "icub_state_publisher");
    ros::NodeHandle n;

    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate(100);

    //LOAD OF robot.mat
    iDynTree::MatrixDynSize robotStateRightArm_i = matrixFromMatlabExtractor_byString("robot.mat","robot/data/q/rightArm");
    iDynTree::MatrixDynSize robotStateRightLeg_i = matrixFromMatlabExtractor_byString("robot.mat","robot/data/q/rightLeg");
    iDynTree::MatrixDynSize robotStateLeftArm_i = matrixFromMatlabExtractor_byString("robot.mat","robot/data/q/leftArm");
    iDynTree::MatrixDynSize robotStateLeftLeg_i = matrixFromMatlabExtractor_byString("robot.mat","robot/data/q/leftLeg");
    iDynTree::MatrixDynSize robotStateTorso_i = matrixFromMatlabExtractor_byString("robot.mat","robot/data/q/torso");

    iDynTree::Transform LeftFoot_to_base_link_i = transformFromMatlabExtractor_byString("robot.mat","robot/transform");
    iDynTree::Rotation LeftFoot_to_base_link_rot = LeftFoot_to_base_link_i.getRotation();
    iDynTree::Position LeftFoot_to_base_link_pos = LeftFoot_to_base_link_i.getPosition();

    //LOAD OF suit.mat
    std::vector< iDynTree::MatrixDynSize > linksPositions = suitLinksFromMatlabExtractor_byString("suit.mat","suit/links");
    std::vector< iDynTree::MatrixDynSize > linksQuaternions = suitQuaternionsFromMatlabExtractor_byString("suit.mat","suit/links");

    // declarations
    int j = 0;
    const double degree = M_PI/180;
    double angle = 0;

    //robot state
    double zero_pos = 0, zero_vel = 0, zero_effort = 0,
    inc = degree, var = 0,
    speed = 1, wrench = 0, wrench_inc = 1;

    // message declarations
    sensor_msgs::JointState joint_state;

    geometry_msgs::TransformStamped ground_to_LeftFoot;
    ground_to_LeftFoot.header.frame_id = "ground";
    ground_to_LeftFoot.child_frame_id = "LeftFoot";

    geometry_msgs::TransformStamped LeftFoot_to_base_link;
    LeftFoot_to_base_link.header.frame_id = "LeftFoot";
    LeftFoot_to_base_link.child_frame_id = "base_link";

    LeftFoot_to_base_link.transform.translation.x = LeftFoot_to_base_link_pos.getVal(0);
    LeftFoot_to_base_link.transform.translation.y = LeftFoot_to_base_link_pos.getVal(1);
    LeftFoot_to_base_link.transform.translation.z = LeftFoot_to_base_link_pos.getVal(2);
    iDynTree::Vector4 LeftFoot_to_base_link_quat;
    LeftFoot_to_base_link_quat = LeftFoot_to_base_link_rot.asQuaternion();
    LeftFoot_to_base_link.transform.rotation.x = LeftFoot_to_base_link_quat.getVal(1);
    LeftFoot_to_base_link.transform.rotation.y = LeftFoot_to_base_link_quat.getVal(2);
    LeftFoot_to_base_link.transform.rotation.z = LeftFoot_to_base_link_quat.getVal(3);
    LeftFoot_to_base_link.transform.rotation.w = LeftFoot_to_base_link_quat.getVal(0);

    geometry_msgs::TransformStamped ground_to_base;
    ground_to_base.header.frame_id = "ground";
    ground_to_base.child_frame_id = "base_link";

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
        int colNum = robotStateTorso_i.cols();
        if(j > (colNum)) j = 0;
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
        // ground to LeftFoot transform
        ground_to_LeftFoot.header.stamp = ros::Time::now();
        ground_to_LeftFoot.transform.translation.x = linksPositions[22].getVal(0,j);
        ground_to_LeftFoot.transform.translation.y = linksPositions[22].getVal(1,j);
        ground_to_LeftFoot.transform.translation.z = linksPositions[22].getVal(2,j);
        iDynTree::Position ground_to_LeftFoot_pos  (ground_to_LeftFoot.transform.translation.x,
                                                    ground_to_LeftFoot.transform.translation.y,
                                                    ground_to_LeftFoot.transform.translation.z);
        ground_to_LeftFoot.transform.rotation.x = linksQuaternions[22].getVal(1,j);
        ground_to_LeftFoot.transform.rotation.y = linksQuaternions[22].getVal(2,j);
        ground_to_LeftFoot.transform.rotation.z = linksQuaternions[22].getVal(3,j);
        ground_to_LeftFoot.transform.rotation.w = linksQuaternions[22].getVal(0,j);
        iDynTree::Vector4 ground_to_LeftFoot_quat;
        ground_to_LeftFoot_quat.setVal(0,ground_to_LeftFoot.transform.rotation.w);
        ground_to_LeftFoot_quat.setVal(1,ground_to_LeftFoot.transform.rotation.x);
        ground_to_LeftFoot_quat.setVal(2,ground_to_LeftFoot.transform.rotation.y);
        ground_to_LeftFoot_quat.setVal(3,ground_to_LeftFoot.transform.rotation.z);
        iDynTree::Rotation ground_to_LeftFoot_rot;
        ground_to_LeftFoot_rot.fromQuaternion(ground_to_LeftFoot_quat);
        iDynTree::Transform ground_to_LeftFoot_i(ground_to_LeftFoot_rot,ground_to_LeftFoot_pos);

        // ground to base_link transform
        iDynTree::Transform ground_to_base_i;
        ground_to_base_i = ground_to_LeftFoot_i*(LeftFoot_to_base_link_i);
        iDynTree::Matrix4x4 ground_to_base_H;
        ground_to_base_H = ground_to_base_i.asHomogeneousTransform();

        ground_to_base.header.stamp = ros::Time::now();
        ground_to_base.transform.translation.x = (ground_to_base_H.getVal(0,3))-0.16;
        ground_to_base.transform.translation.y = (ground_to_base_H.getVal(1,3))-0.04;
        ground_to_base.transform.translation.z = (ground_to_base_H.getVal(2,3))+0.03;
        iDynTree::Rotation ground_to_base_rot   (ground_to_base_H.getVal(0,0),
                                                 ground_to_base_H.getVal(0,1),
                                                 ground_to_base_H.getVal(0,2),
                                                 ground_to_base_H.getVal(1,0),
                                                 ground_to_base_H.getVal(1,1),
                                                 ground_to_base_H.getVal(1,2),
                                                 ground_to_base_H.getVal(2,0),
                                                 ground_to_base_H.getVal(2,1),
                                                 ground_to_base_H.getVal(2,2));
        iDynTree::Vector4 ground_to_base_quat = ground_to_base_rot.asQuaternion();
        ground_to_base.transform.rotation.x = ground_to_base_quat.getVal(1);
        ground_to_base.transform.rotation.y = ground_to_base_quat.getVal(2);
        ground_to_base.transform.rotation.z = ground_to_base_quat.getVal(3);
        ground_to_base.transform.rotation.w = ground_to_base_quat.getVal(0);

        // Send the joint state
        joint_pub.publish(joint_state);

        // send the transform
        broadcaster.sendTransform(ground_to_base);

        // This will adjust as needed per iteration
        loop_rate.sleep();
      }

    return 0;

}

template<typename Out>
void split(const std::string &s, char delim, Out result) {
    std::stringstream ss;
    ss.str(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        *(result++) = item;
    }
}

std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    split(s, delim, std::back_inserter(elems));
    return elems;
}

inline iDynTree::Transform transformFromMatlabExtractor_byString(std::string matVarName, std::string matVarString){

    std::vector<std::string> matFields = split(matVarString, '/');
    int n = matFields.size();
    const char *varName = matFields[0].c_str();

    const char *fieldName;
    const char *subFieldName;
    const char *subSubFieldName;
    const char *subSubSubFieldName;

    iDynTree::Transform mat_i;

    mat_t *matPointer;
    matPointer = Mat_Open(getAbsModelPath(matVarName).c_str(),MAT_ACC_RDONLY);
    iDynTree::assertTrue(matPointer !=NULL);

    matvar_t *matVar;
    matVar = Mat_VarRead(matPointer, varName);
    iDynTree::assertTrue(matVar != NULL);

    switch(n){

        case (2):
        {
            fieldName = matFields[1].c_str();
            matvar_t *matField;
            matField = Mat_VarGetStructFieldByName(matVar, fieldName, 0);
            iDynTree::assertTrue(matField != NULL);

            Eigen::Map< Eigen::Matrix4d > mapMatField((double*)matField->data, matField->dims[0], matField->dims[1]);
            iDynTree::Transform matField_i;
            iDynTree::fromEigen(matField_i,mapMatField);
            mat_i = matField_i;
        }
        break;

        case (3):
        {
            fieldName = matFields[1].c_str();
            matvar_t *matField;
            matField = Mat_VarGetStructFieldByName(matVar, fieldName, 0);
            iDynTree::assertTrue(matField != NULL);

            subFieldName = matFields[2].c_str();
            matvar_t *matSubField;
            matSubField = Mat_VarGetStructFieldByName(matField, subFieldName, 0);
            iDynTree::assertTrue(matSubField != NULL);

            Eigen::Map< Eigen::Matrix4d > mapMatSubField((double*)matSubField->data, matSubField->dims[0], matSubField->dims[1]);
            iDynTree::Transform matSubField_i;
            iDynTree::fromEigen(matSubField_i,mapMatSubField);
            mat_i = matSubField_i;
        }
        break;

        case(4):
        {
            fieldName = matFields[1].c_str();
            matvar_t *matField;
            matField = Mat_VarGetStructFieldByName(matVar, fieldName, 0);
            iDynTree::assertTrue(matField != NULL);

            subFieldName = matFields[2].c_str();
            matvar_t *matSubField;
            matSubField = Mat_VarGetStructFieldByName(matField, subFieldName, 0);
            iDynTree::assertTrue(matSubField != NULL);

            subSubFieldName = matFields[3].c_str();
            matvar_t *matSubSubField;
            matSubSubField = Mat_VarGetStructFieldByName(matSubField, subSubFieldName, 0);
            iDynTree::assertTrue(matSubSubField != NULL);

            Eigen::Map< Eigen::Matrix4d > mapMatSubSubField((double*)matSubSubField->data, matSubSubField->dims[0], matSubSubField->dims[1]);
            iDynTree::Transform matSubSubField_i;
            iDynTree::fromEigen(matSubSubField_i,mapMatSubSubField);
            mat_i = matSubSubField_i;
        }
        break;

        case(5):
        {
            fieldName = matFields[1].c_str();
            matvar_t *matField;
            matField = Mat_VarGetStructFieldByName(matVar, fieldName, 0);
            iDynTree::assertTrue(matField != NULL);

            subFieldName = matFields[2].c_str();
            matvar_t *matSubField;
            matSubField = Mat_VarGetStructFieldByName(matField, subFieldName, 0);
            iDynTree::assertTrue(matSubField != NULL);

            subSubFieldName = matFields[3].c_str();
            matvar_t *matSubSubField;
            matSubSubField = Mat_VarGetStructFieldByName(matSubField, subSubFieldName, 0);
            iDynTree::assertTrue(matSubSubField != NULL);

            subSubSubFieldName = matFields[4].c_str();
            matvar_t *matSubSubSubField;
            matSubSubSubField = Mat_VarGetStructFieldByName(matSubSubField, subSubSubFieldName, 0);
            iDynTree::assertTrue(matSubSubSubField != NULL);

            Eigen::Map< Eigen::Matrix4d > mapMatSubSubSubField((double*)matSubSubSubField->data, matSubSubSubField->dims[0], matSubSubSubField->dims[1]);
            iDynTree::Transform matSubSubSubField_i;
            iDynTree::fromEigen(matSubSubSubField_i,mapMatSubSubSubField);
            mat_i = matSubSubSubField_i;
        }
        break;
    }

    Mat_Close(matPointer);
    return mat_i;
}
inline iDynTree::Transform transformFromMatlabExtractor(std::string matName, int n, ...){

    iDynTree::Transform mat_i;
    const char *name;
    va_list vl;
    va_start(vl,n);

    mat_t *matPointer;
    matPointer = Mat_Open(getAbsModelPath(matName).c_str(),MAT_ACC_RDONLY);
    iDynTree::assertTrue(matPointer !=NULL);

    name = va_arg(vl,const char*);
    matvar_t *matVar;
    matVar = Mat_VarRead(matPointer, name);
    iDynTree::assertTrue(matVar != NULL);

    switch(n){

        case (2):
        {
            name = va_arg(vl,const char*);
            matvar_t *matField;
            matField = Mat_VarGetStructFieldByName(matVar, name, 0);
            iDynTree::assertTrue(matField != NULL);

            Eigen::Map< Eigen::Matrix4d > mapMatField((double*)matField->data, matField->dims[0], matField->dims[1]);
            iDynTree::Transform matField_i;
            iDynTree::fromEigen(matField_i,mapMatField);
            mat_i = matField_i;
            va_end(vl);
        }
        break;

        case (3):
        {
            name = va_arg(vl,const char*);
            matvar_t *matField;
            matField = Mat_VarGetStructFieldByName(matVar, name, 0);
            iDynTree::assertTrue(matField != NULL);

            name = va_arg(vl,const char*);
            matvar_t *matSubField;
            matSubField = Mat_VarGetStructFieldByName(matField, name, 0);
            iDynTree::assertTrue(matSubField != NULL);

            Eigen::Map< Eigen::Matrix4d > mapMatSubField((double*)matSubField->data, matSubField->dims[0], matSubField->dims[1]);
            iDynTree::Transform matSubField_i;
            iDynTree::fromEigen(matSubField_i,mapMatSubField);
            va_end(vl);
            mat_i = matSubField_i;
        }
            break;

        case(4):
        {
            name = va_arg(vl,const char*);
            matvar_t *matField;
            matField = Mat_VarGetStructFieldByName(matVar, name, 0);
            iDynTree::assertTrue(matField != NULL);

            name = va_arg(vl,const char*);
            matvar_t *matSubField;
            matSubField = Mat_VarGetStructFieldByName(matField, name, 0);
            iDynTree::assertTrue(matSubField != NULL);

            name = va_arg(vl,const char*);
            matvar_t *matSubSubField;
            matSubSubField = Mat_VarGetStructFieldByName(matSubField, name, 0);
            iDynTree::assertTrue(matSubSubField != NULL);

            Eigen::Map< Eigen::Matrix4d > mapMatSubSubField((double*)matSubSubField->data, matSubSubField->dims[0], matSubSubField->dims[1]);
            iDynTree::Transform matSubSubField_i;
            iDynTree::fromEigen(matSubSubField_i,mapMatSubSubField);
            va_end(vl);
            mat_i = matSubSubField_i;
        }
            break;

        case(5):
        {
            name = va_arg(vl,const char*);
            matvar_t *matField;
            matField = Mat_VarGetStructFieldByName(matVar, name, 0);
            iDynTree::assertTrue(matField != NULL);

            name = va_arg(vl,const char*);
            matvar_t *matSubField;
            matSubField = Mat_VarGetStructFieldByName(matField, name, 0);
            iDynTree::assertTrue(matSubField != NULL);

            name = va_arg(vl,const char*);
            matvar_t *matSubSubField;
            matSubSubField = Mat_VarGetStructFieldByName(matSubField, name, 0);
            iDynTree::assertTrue(matSubSubField != NULL);

            name = va_arg(vl,const char*);
            matvar_t *matSubSubSubField;
            matSubSubSubField = Mat_VarGetStructFieldByName(matSubSubField, name, 0);
            iDynTree::assertTrue(matSubSubSubField != NULL);

            Eigen::Map< Eigen::Matrix4d > mapMatSubSubSubField((double*)matSubSubSubField->data, matSubSubSubField->dims[0], matSubSubSubField->dims[1]);
            iDynTree::Transform matSubSubSubField_i;
            iDynTree::fromEigen(matSubSubSubField_i,mapMatSubSubSubField);
            va_end(vl);
            mat_i = matSubSubSubField_i;
        }
        break;
    }

    Mat_Close(matPointer);
    return mat_i;
}

inline iDynTree::MatrixDynSize matrixFromMatlabExtractor_byString(std::string matVarName, std::string matVarString){

    std::vector<std::string> matFields = split(matVarString, '/');
    int n = matFields.size();
    const char *varName = matFields[0].c_str();

    const char *fieldName;
    const char *subFieldName;
    const char *subSubFieldName;
    const char *subSubSubFieldName;

    iDynTree::MatrixDynSize mat_i;

    mat_t *matPointer;
    matPointer = Mat_Open(getAbsModelPath(matVarName).c_str(),MAT_ACC_RDONLY);
    iDynTree::assertTrue(matPointer !=NULL);

    matvar_t *matVar;
    matVar = Mat_VarRead(matPointer, varName);
    iDynTree::assertTrue(matVar != NULL);

    switch(n){

        case (2):
        {
            fieldName = matFields[1].c_str();
            matvar_t *matField;
            matField = Mat_VarGetStructFieldByName(matVar, fieldName, 0);
            iDynTree::assertTrue(matField != NULL);

            Eigen::Map< Eigen::MatrixXd > mapMatField((double*)matField->data, matField->dims[0], matField->dims[1]);
            iDynTree::MatrixDynSize matField_i(matField->dims[0], matField->dims[1]);
            iDynTree::toEigen(matField_i) = mapMatField;
            mat_i = matField_i;
        }
        break;

        case (3):
        {
            fieldName = matFields[1].c_str();
            matvar_t *matField;
            matField = Mat_VarGetStructFieldByName(matVar, fieldName, 0);
            iDynTree::assertTrue(matField != NULL);

            subFieldName = matFields[2].c_str();
            matvar_t *matSubField;
            matSubField = Mat_VarGetStructFieldByName(matField, subFieldName, 0);
            iDynTree::assertTrue(matSubField != NULL);

            Eigen::Map< Eigen::MatrixXd > mapMatSubField((double*)matSubField->data, matSubField->dims[0], matSubField->dims[1]);
            iDynTree::MatrixDynSize matSubField_i(matSubField->dims[0], matSubField->dims[1]);
            iDynTree::toEigen(matSubField_i) = mapMatSubField;
            mat_i = matSubField_i;
        }
        break;

        case(4):
        {
            fieldName = matFields[1].c_str();
            matvar_t *matField;
            matField = Mat_VarGetStructFieldByName(matVar, fieldName, 0);
            iDynTree::assertTrue(matField != NULL);

            subFieldName = matFields[2].c_str();
            matvar_t *matSubField;
            matSubField = Mat_VarGetStructFieldByName(matField, subFieldName, 0);
            iDynTree::assertTrue(matSubField != NULL);

            subSubFieldName = matFields[3].c_str();
            matvar_t *matSubSubField;
            matSubSubField = Mat_VarGetStructFieldByName(matSubField, subSubFieldName, 0);
            iDynTree::assertTrue(matSubSubField != NULL);

            Eigen::Map< Eigen::MatrixXd > mapMatSubSubField((double*)matSubSubField->data, matSubSubField->dims[0], matSubSubField->dims[1]);
            iDynTree::MatrixDynSize matSubSubField_i(matSubSubField->dims[0], matSubSubField->dims[1]);
            iDynTree::toEigen(matSubSubField_i) = mapMatSubSubField;
            mat_i = matSubSubField_i;
        }
        break;

        case(5):
        {
            fieldName = matFields[1].c_str();
            matvar_t *matField;
            matField = Mat_VarGetStructFieldByName(matVar, fieldName, 0);
            iDynTree::assertTrue(matField != NULL);

            subFieldName = matFields[2].c_str();
            matvar_t *matSubField;
            matSubField = Mat_VarGetStructFieldByName(matField, subFieldName, 0);
            iDynTree::assertTrue(matSubField != NULL);

            subSubFieldName = matFields[3].c_str();
            matvar_t *matSubSubField;
            matSubSubField = Mat_VarGetStructFieldByName(matSubField, subSubFieldName, 0);
            iDynTree::assertTrue(matSubSubField != NULL);

            subSubSubFieldName = matFields[4].c_str();
            matvar_t *matSubSubSubField;
            matSubSubSubField = Mat_VarGetStructFieldByName(matSubSubField, subSubSubFieldName, 0);
            iDynTree::assertTrue(matSubSubSubField != NULL);

            Eigen::Map< Eigen::MatrixXd > mapMatSubSubSubField((double*)matSubSubSubField->data, matSubSubSubField->dims[0], matSubSubSubField->dims[1]);
            iDynTree::MatrixDynSize matSubSubSubField_i(matSubSubSubField->dims[0], matSubSubSubField->dims[1]);
            iDynTree::toEigen(matSubSubSubField_i) = mapMatSubSubSubField;
            mat_i = matSubSubSubField_i;
        }
        break;
    }

    Mat_Close(matPointer);
    return mat_i;
}
inline iDynTree::MatrixDynSize matrixFromMatlabExtractor(std::string matName, int n, ...){

    // Call examples
    //     iDynTree::MatrixDynSize robotStateRightArm_i = matrixFromMatlabExtractor("robot.mat", 4,"robot","data","q","rightArm");
    //     iDynTree::MatrixDynSize robotStateRightLeg_i = matrixFromMatlabExtractor("robot.mat", 4,"robot","data","q","rightLeg");
    //     iDynTree::MatrixDynSize robotStateLeftArm_i = matrixFromMatlabExtractor("robot.mat", 4,"robot","data","q","leftArm");
    //     iDynTree::MatrixDynSize robotStateLeftLeg_i = matrixFromMatlabExtractor("robot.mat", 4,"robot","data","q","leftLeg");
    //     iDynTree::MatrixDynSize robotStateTorso_i = matrixFromMatlabExtractor("robot.mat", 4,"robot","data","q","torso");

    iDynTree::MatrixDynSize mat_i;
    const char *name;
    va_list vl;
    va_start(vl,n);

    mat_t *matPointer;
    matPointer = Mat_Open(getAbsModelPath(matName).c_str(),MAT_ACC_RDONLY);
    iDynTree::assertTrue(matPointer !=NULL);

    name = va_arg(vl,const char*);
    matvar_t *matVar;
    matVar = Mat_VarRead(matPointer, name);
    iDynTree::assertTrue(matVar != NULL);

    switch(n){

        case (2):
        {
            name = va_arg(vl,const char*);
            matvar_t *matField;
            matField = Mat_VarGetStructFieldByName(matVar, name, 0);
            iDynTree::assertTrue(matField != NULL);

            Eigen::Map< Eigen::MatrixXd > mapMatField((double*)matField->data, matField->dims[0], matField->dims[1]);
            iDynTree::MatrixDynSize matField_i(matField->dims[0], matField->dims[1]);
            iDynTree::toEigen(matField_i) = mapMatField;
            mat_i = matField_i;
            va_end(vl);
        }
        break;

        case (3):
        {
            name = va_arg(vl,const char*);
            matvar_t *matField;
            matField = Mat_VarGetStructFieldByName(matVar, name, 0);
            iDynTree::assertTrue(matField != NULL);

            name = va_arg(vl,const char*);
            matvar_t *matSubField;
            matSubField = Mat_VarGetStructFieldByName(matField, name, 0);
            iDynTree::assertTrue(matSubField != NULL);

            Eigen::Map< Eigen::MatrixXd > mapMatSubField((double*)matSubField->data, matSubField->dims[0], matSubField->dims[1]);
            iDynTree::MatrixDynSize matSubField_i(matSubField->dims[0], matSubField->dims[1]);
            iDynTree::toEigen(matSubField_i) = mapMatSubField;
            va_end(vl);
            mat_i = matSubField_i;
        }
            break;

        case(4):
        {
            name = va_arg(vl,const char*);
            matvar_t *matField;
            matField = Mat_VarGetStructFieldByName(matVar, name, 0);
            iDynTree::assertTrue(matField != NULL);

            name = va_arg(vl,const char*);
            matvar_t *matSubField;
            matSubField = Mat_VarGetStructFieldByName(matField, name, 0);
            iDynTree::assertTrue(matSubField != NULL);

            name = va_arg(vl,const char*);
            matvar_t *matSubSubField;
            matSubSubField = Mat_VarGetStructFieldByName(matSubField, name, 0);
            iDynTree::assertTrue(matSubSubField != NULL);

            Eigen::Map< Eigen::MatrixXd > mapMatSubSubField((double*)matSubSubField->data, matSubSubField->dims[0], matSubSubField->dims[1]);
            iDynTree::MatrixDynSize matSubSubField_i(matSubSubField->dims[0], matSubSubField->dims[1]);
            iDynTree::toEigen(matSubSubField_i) = mapMatSubSubField;
            va_end(vl);
            mat_i = matSubSubField_i;
        }
            break;

        case(5):
        {
            name = va_arg(vl,const char*);
            matvar_t *matField;
            matField = Mat_VarGetStructFieldByName(matVar, name, 0);
            iDynTree::assertTrue(matField != NULL);

            name = va_arg(vl,const char*);
            matvar_t *matSubField;
            matSubField = Mat_VarGetStructFieldByName(matField, name, 0);
            iDynTree::assertTrue(matSubField != NULL);

            name = va_arg(vl,const char*);
            matvar_t *matSubSubField;
            matSubSubField = Mat_VarGetStructFieldByName(matSubField, name, 0);
            iDynTree::assertTrue(matSubSubField != NULL);

            name = va_arg(vl,const char*);
            matvar_t *matSubSubSubField;
            matSubSubSubField = Mat_VarGetStructFieldByName(matSubSubField, name, 0);
            iDynTree::assertTrue(matSubSubSubField != NULL);

            Eigen::Map< Eigen::MatrixXd > mapMatSubSubSubField((double*)matSubSubSubField->data, matSubSubSubField->dims[0], matSubSubSubField->dims[1]);
            iDynTree::MatrixDynSize matSubSubSubField_i(matSubSubSubField->dims[0], matSubSubSubField->dims[1]);
            iDynTree::toEigen(matSubSubSubField_i) = mapMatSubSubSubField;
            va_end(vl);
            mat_i = matSubSubSubField_i;
        }
        break;
    }

    Mat_Close(matPointer);
    return mat_i;
}

inline std::vector< iDynTree::MatrixDynSize > suitLinksFromMatlabExtractor_byString(std::string matVarName, std::string matVarString){

    std::vector<std::string> matFields = split(matVarString, '/');
    int n = matFields.size();
    const char *varName = matFields[0].c_str();
    const char *fieldName = matFields[1].c_str();

    mat_t *pSuit;
    pSuit = Mat_Open(getAbsModelPath(matVarName).c_str(),MAT_ACC_RDONLY);
    iDynTree::assertTrue(pSuit!=NULL);

    matvar_t *suitVar;
    suitVar = Mat_VarRead(pSuit,varName);
    iDynTree::assertTrue(suitVar != NULL);


    matvar_t *linksVar;
    linksVar = Mat_VarGetStructFieldByName(suitVar,fieldName,0);
    iDynTree::assertTrue(linksVar != NULL);

    matvar_t *linkCell;
    std::vector< std::string > linksName;
    linksName.resize(linksVar->dims[0]);

    std::vector< iDynTree::MatrixDynSize > linksPositions;
    linksPositions.resize(linksVar->dims[0]);

    matvar_t *temp;
    matvar_t *tempMeas;

    for(int i=0; i < linksVar->dims[0] ;++i){

	linkCell = Mat_VarGetCell(linksVar, i);

	//Get Name
	temp = Mat_VarGetStructFieldByName(linkCell,"label", 0);
	iDynTree::assertTrue(temp != NULL);
	linksName[i] =(char*) temp->data;

	//Get meas
	tempMeas = Mat_VarGetStructFieldByName(linkCell,"meas", 0);
	iDynTree::assertTrue(tempMeas != NULL);

	//Get positions
	temp = Mat_VarGetStructFieldByName(tempMeas,"position", 0);
	iDynTree::assertTrue(temp != NULL);
	Eigen::Map< Eigen::MatrixXd > tempMapPosition((double*)temp->data, temp->dims[0], temp->dims[1]);
	linksPositions[i].resize(temp->dims[0], temp->dims[1]);
	toEigen(linksPositions[i]) = tempMapPosition;

    }

    return linksPositions;

}
inline std::vector< iDynTree::MatrixDynSize > suitQuaternionsFromMatlabExtractor_byString(std::string matVarName, std::string matVarString){

    std::vector<std::string> matFields = split(matVarString, '/');
    int n = matFields.size();
    const char *varName = matFields[0].c_str();
    const char *fieldName = matFields[1].c_str();

    mat_t *pSuit;
    pSuit = Mat_Open(getAbsModelPath(matVarName).c_str(),MAT_ACC_RDONLY);
    iDynTree::assertTrue(pSuit!=NULL);

    matvar_t *suitVar;
    suitVar = Mat_VarRead(pSuit,varName);
    iDynTree::assertTrue(suitVar != NULL);


    matvar_t *linksVar;
    linksVar = Mat_VarGetStructFieldByName(suitVar,fieldName,0);
    iDynTree::assertTrue(linksVar != NULL);

    matvar_t *linkCell;
    std::vector< std::string > linksName;
    linksName.resize(linksVar->dims[0]);

    std::vector< iDynTree::MatrixDynSize > linksQuaternions;
    linksQuaternions.resize(linksVar->dims[0]);

    matvar_t *temp;
    matvar_t *tempMeas;

    for(int i=0; i < linksVar->dims[0] ;++i){

	linkCell = Mat_VarGetCell(linksVar, i);

	//Get Name
	temp = Mat_VarGetStructFieldByName(linkCell,"label", 0);
	iDynTree::assertTrue(temp != NULL);
	linksName[i] =(char*) temp->data;

	//Get meas
	tempMeas = Mat_VarGetStructFieldByName(linkCell,"meas", 0);
	iDynTree::assertTrue(tempMeas != NULL);

	//Get orientations
	temp = Mat_VarGetStructFieldByName(tempMeas,"orientation", 0);
	iDynTree::assertTrue(temp != NULL);
	Eigen::Map< Eigen::MatrixXd > tempMapOrientation((double*)temp->data, temp->dims[0], temp->dims[1]);
	linksQuaternions[i].resize(temp->dims[0], temp->dims[1]);
	toEigen(linksQuaternions[i]) = tempMapOrientation;
    }

    return linksQuaternions;

}
