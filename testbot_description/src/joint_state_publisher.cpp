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

    // Load model
    iDynTree::ModelLoader modelLoader;
    iDynTree::Model model;
    bool ok=modelLoader.loadModelFromFile(getAbsModelPath("XSensURDF_subj1.urdf"));
    model = modelLoader.model();
    iDynTree::assertTrue(ok);

    //LOAD OF human_state.mat
    iDynTree::MatrixDynSize humanStateQi = matrixFromMatlabExtractor_byString("human_state.mat","human_state/q");

    //LOAD OF mu_dgiveny.mat
    iDynTree::MatrixDynSize mu_dgiveny_i = matrixFromMatlabExtractor_byString("mu_dgiveny.mat","mu_dgiveny/mu_dgiveny_ALLsens");

    //LOAD OF suit.mat
    std::vector< iDynTree::MatrixDynSize > linksPositions = suitLinksFromMatlabExtractor_byString("suit.mat","suit/links");
    std::vector< iDynTree::MatrixDynSize > linksQuaternions = suitQuaternionsFromMatlabExtractor_byString("suit.mat","suit/links");

    // Publish data
    ros::init(argc, argv, "joint_state_pub");
    ros::NodeHandle n;

    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate(100);

    // declarations
    int i =0, j = 0;
    // Pelvis to LeftFoot transform data
    double tx_1 = 0.000037, ty_1 = 0.079215, tz_1 = 0.000367;
    double tx_2 = -0.000002, tz_2 = -0.40926;
    double tx_3 = -0.000373, tz_3 = -0.42811;

    // LeftFoot to pointFP transform data
    double tz_4 = -0.091847;

    // message declarations
    sensor_msgs::JointState joint_state;

    geometry_msgs::TransformStamped ground_to_Pelvis;
    ground_to_Pelvis.header.frame_id = "ground";
    ground_to_Pelvis.child_frame_id = "Pelvis";

    geometry_msgs::TransformStamped ground_to_LeftFoot;
    ground_to_LeftFoot.header.frame_id = "ground";
    ground_to_LeftFoot.child_frame_id = "LeftFoot";

    geometry_msgs::TransformStamped LeftFoot_to_pointFP;
    LeftFoot_to_pointFP.header.frame_id = "LeftFoot";
    LeftFoot_to_pointFP.child_frame_id = "pointFP";

    geometry_msgs::TransformStamped Pelvis_to_LeftFoot;
    Pelvis_to_LeftFoot.header.frame_id = "Pelvis";
    Pelvis_to_LeftFoot.child_frame_id = "LeftFoot";

    geometry_msgs::TransformStamped ground_to_base;
    ground_to_base.header.frame_id = "ground";
    ground_to_base.child_frame_id = "Pelvis";

    // joint_state setting
    joint_state.name.resize(48);
    joint_state.name[0] ="jL5S1_rotx";
    joint_state.name[1] ="jL5S1_roty";
    joint_state.name[2] ="jL4L3_rotx";
    joint_state.name[3] ="jL4L3_roty";
    joint_state.name[4] ="jL1T12_rotx";
    joint_state.name[5] ="jL1T12_roty";
    joint_state.name[6] ="jT9T8_rotx";
    joint_state.name[7] ="jT9T8_roty";
    joint_state.name[8] ="jT9T8_rotz";
    joint_state.name[9] ="jT1C7_rotx";
    joint_state.name[10] ="jT1C7_roty";
    joint_state.name[11] ="jT1C7_rotz";
    joint_state.name[12] ="jC1Head_rotx";
    joint_state.name[13] ="jC1Head_roty";
    joint_state.name[14] ="jRightC7Shoulder_rotx";
    joint_state.name[15] ="jRightShoulder_rotx";
    joint_state.name[16] ="jRightShoulder_roty";
    joint_state.name[17] ="jRightShoulder_rotz";
    joint_state.name[18] ="jRightElbow_roty";
    joint_state.name[19] ="jRightElbow_rotz";
    joint_state.name[20] ="jRightWrist_rotx";
    joint_state.name[21] ="jRightWrist_rotz";
    joint_state.name[22] ="jLeftC7Shoulder_rotx";
    joint_state.name[23] ="jLeftShoulder_rotx";
    joint_state.name[24] ="jLeftShoulder_roty";
    joint_state.name[25] ="jLeftShoulder_rotz";
    joint_state.name[26] ="jLeftElbow_roty";
    joint_state.name[27] ="jLeftElbow_rotz";
    joint_state.name[28] ="jLeftWrist_rotx";
    joint_state.name[29] ="jLeftWrist_rotz";
    joint_state.name[30] ="jRightHip_rotx";
    joint_state.name[31] ="jRightHip_roty";
    joint_state.name[32] ="jRightHip_rotz";
    joint_state.name[33] ="jRightKnee_roty";
    joint_state.name[34] ="jRightKnee_rotz";
    joint_state.name[35] ="jRightAnkle_rotx";
    joint_state.name[36] ="jRightAnkle_roty";
    joint_state.name[37] ="jRightAnkle_rotz";
    joint_state.name[38] ="jRightBallFoot_roty";
    joint_state.name[39] ="jLeftHip_rotx";
    joint_state.name[40] ="jLeftHip_roty";
    joint_state.name[41] ="jLeftHip_rotz";
    joint_state.name[42] ="jLeftKnee_roty";
    joint_state.name[43] ="jLeftKnee_rotz";
    joint_state.name[44] ="jLeftAnkle_rotx";
    joint_state.name[45] ="jLeftAnkle_roty";
    joint_state.name[46] ="jLeftAnkle_rotz";
    joint_state.name[47] ="jLeftBallFoot_roty";

    joint_state.position.resize(48);
    joint_state.velocity.resize(48);
    joint_state.effort.resize(48);

    while (ros::ok()){

        //update joint_state
        joint_state.header.stamp = ros::Time::now();

        j++;
        int colNum = humanStateQi.cols();
        if(j > (colNum)) j = 0;
        for(i=0; i<=47; i++){
            joint_state.position[i] = humanStateQi.getVal(i,j);
        }

        //RightHip_roty
        joint_state.effort[31] = mu_dgiveny_i.getVal(304,j);
        //LeftHip_roty
        joint_state.effort[40] = mu_dgiveny_i.getVal(200,j);

        // Send the joint state
        joint_pub.publish(joint_state);

        // update transform
        // ground to Pelvis transform
        ground_to_Pelvis.header.stamp = ros::Time::now();
        ground_to_Pelvis.transform.translation.x = linksPositions[0].getVal(0,j);
        ground_to_Pelvis.transform.translation.y = linksPositions[0].getVal(1,j);
        ground_to_Pelvis.transform.translation.z = linksPositions[0].getVal(2,j);
        iDynTree::Position ground_to_Pelvis_pos  (ground_to_Pelvis.transform.translation.x,
                                                  ground_to_Pelvis.transform.translation.y,
                                                  ground_to_Pelvis.transform.translation.z);
        ground_to_Pelvis.transform.rotation.x = linksQuaternions[0].getVal(1,j);
        ground_to_Pelvis.transform.rotation.y = linksQuaternions[0].getVal(2,j);
        ground_to_Pelvis.transform.rotation.z = linksQuaternions[0].getVal(3,j);
        ground_to_Pelvis.transform.rotation.w = linksQuaternions[0].getVal(0,j);
        iDynTree::Vector4 ground_to_Pelvis_quat;
        ground_to_Pelvis_quat.setVal(0,ground_to_Pelvis.transform.rotation.w);
        ground_to_Pelvis_quat.setVal(1,ground_to_Pelvis.transform.rotation.x);
        ground_to_Pelvis_quat.setVal(2,ground_to_Pelvis.transform.rotation.y);
        ground_to_Pelvis_quat.setVal(3,ground_to_Pelvis.transform.rotation.z);
        iDynTree::Rotation ground_to_Pelvis_rot;
        ground_to_Pelvis_rot.fromQuaternion(ground_to_Pelvis_quat);
        iDynTree::Transform ground_to_Pelvis_i(ground_to_Pelvis_rot,ground_to_Pelvis_pos);

        // ground to LeftFoot transform
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

        // Pelvis to LeftFoot transform
        Pelvis_to_LeftFoot.header.stamp = ros::Time::now();
        Pelvis_to_LeftFoot.transform.translation.x = tx_1+tx_2+tx_3;
        Pelvis_to_LeftFoot.transform.translation.y = ty_1;
        Pelvis_to_LeftFoot.transform.translation.z = tz_1+tz_2+tz_3;
        iDynTree::Position Pelvis_to_LeftFoot_pos  (Pelvis_to_LeftFoot.transform.translation.x,
                                                    Pelvis_to_LeftFoot.transform.translation.y,
                                                    Pelvis_to_LeftFoot.transform.translation.z);
        Pelvis_to_LeftFoot.transform.rotation.x = 0;
        Pelvis_to_LeftFoot.transform.rotation.y = 0;
        Pelvis_to_LeftFoot.transform.rotation.z = 0;
        Pelvis_to_LeftFoot.transform.rotation.w = 1;
        iDynTree::Vector4 Pelvis_to_LeftFoot_quat;
        Pelvis_to_LeftFoot_quat.setVal(0,Pelvis_to_LeftFoot.transform.rotation.w);
        Pelvis_to_LeftFoot_quat.setVal(1,Pelvis_to_LeftFoot.transform.rotation.x);
        Pelvis_to_LeftFoot_quat.setVal(2,Pelvis_to_LeftFoot.transform.rotation.y);
        Pelvis_to_LeftFoot_quat.setVal(3,Pelvis_to_LeftFoot.transform.rotation.z);
        iDynTree::Rotation Pelvis_to_LeftFoot_rot;
        Pelvis_to_LeftFoot_rot.fromQuaternion(Pelvis_to_LeftFoot_quat);
        iDynTree::Transform Pelvis_to_LeftFoot_i(Pelvis_to_LeftFoot_rot,Pelvis_to_LeftFoot_pos);
        iDynTree::Transform LeftFoot_to_Pelvis_i;
        LeftFoot_to_Pelvis_i = Pelvis_to_LeftFoot_i.inverse();

        // LeftFoot to pointFP
        iDynTree::Position LeftFoot_to_pointFP_pos (0, 0, tz_4);
        iDynTree::Rotation LeftFoot_to_pointFP_rot (1, 0, 0,
                                                   0, 1, 0,
                                                   0, 0, 1);
        iDynTree::Transform LeftFoot_to_pointFP_i(LeftFoot_to_pointFP_rot,LeftFoot_to_pointFP_pos);

        // ground to pointFP
        iDynTree::Transform ground_to_pointFP_i;
        ground_to_pointFP_i = ground_to_LeftFoot_i*(LeftFoot_to_pointFP_i);
        iDynTree::Matrix4x4 ground_to_pointFP_H;
        ground_to_pointFP_H = ground_to_pointFP_i.asHomogeneousTransform();
        //std::cerr << "ground_to_pointFP:" << ground_to_pointFP_H.getVal(1,3) << std::endl;
        iDynTree::Matrix4x4 ground_to_LeftFoot_H;
        ground_to_LeftFoot_H = ground_to_LeftFoot_i.asHomogeneousTransform();
        //std::cerr << "ground_to_LeftFoot:" << ground_to_LeftFoot_H.getVal(1,3) << std::endl;

        // ground to Pelvis alternative transform
        iDynTree::Transform ground_to_base_i;
        ground_to_base_i = ground_to_LeftFoot_i*(LeftFoot_to_Pelvis_i);
        iDynTree::Matrix4x4 ground_to_base_H;
        ground_to_base_H = ground_to_base_i.asHomogeneousTransform();

        ground_to_base.header.stamp = ros::Time::now();
        ground_to_base.transform.translation.x = ground_to_base_H.getVal(0,3);
        ground_to_base.transform.translation.y = ground_to_base_H.getVal(1,3);
        ground_to_base.transform.translation.z = ground_to_base_H.getVal(2,3);
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

        // send the transform
        broadcaster.sendTransform(ground_to_Pelvis);

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





