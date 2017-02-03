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

    // Load data
    iDynTree::ModelLoader modelLoader;
    iDynTree::Model model;

    std::cerr<<"Load model from "<<getAbsModelPath("XSensURDF_subj1.urdf")<< std::endl;
    bool ok=modelLoader.loadModelFromFile(getAbsModelPath("XSensURDF_subj1.urdf"));
    model = modelLoader.model();

    iDynTree::assertTrue(ok);

    std::cerr <<"Model Loaded"<< std::endl;

    mat_t *pHumanState;

    std::cerr<<"Load human_state "<<getAbsModelPath("human_state.mat")<< std::endl;

    pHumanState = Mat_Open(getAbsModelPath("human_state.mat").c_str(),MAT_ACC_RDONLY);
    iDynTree::assertTrue(pHumanState!=NULL);

    //LOADING OF human_state.mat

    matvar_t *humanStateVar;
    humanStateVar = Mat_VarRead(pHumanState,"human_state");
    iDynTree::assertTrue(humanStateVar != NULL);
    std::cerr<<"Found human_state variable"<<std::endl;

    matvar_t *humanStateQ;

    humanStateQ = Mat_VarGetStructFieldByName(humanStateVar, "q", 0);
    iDynTree::assertTrue(humanStateQ != NULL);

    std::cerr << "Dimensions: "<< humanStateQ->dims[0] << "x" << humanStateQ->dims[1] << std::endl;

    Eigen::Map< Eigen::MatrixXd > mapHumanState((double*)humanStateQ->data, humanStateQ->dims[0], humanStateQ->dims[1]);

    iDynTree::MatrixDynSize humanStateQi(humanStateQ->dims[0], humanStateQ->dims[1]);

    iDynTree::toEigen(humanStateQi) = mapHumanState;

    // Publish data
    ros::init(argc, argv, "joint_state_pub");
    ros::NodeHandle n;

    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate(30);

    //LOAD OF suit.mat
    mat_t *pSuit;

    std::cerr<<"Load suit.mat at "<<getAbsModelPath("suit.mat")<< std::endl;
    pSuit = Mat_Open(getAbsModelPath("suit.mat").c_str(),MAT_ACC_RDONLY);
    iDynTree::assertTrue(pSuit!=NULL);
    std::cerr << "suit loaded" << std::endl;

    matvar_t *suitVar;
    suitVar = Mat_VarRead(pSuit,"suit");
    iDynTree::assertTrue(humanStateVar != NULL);
    std::cerr<<"Found suit variable"<<std::endl;

    matvar_t *linksVar;
    linksVar = Mat_VarGetStructFieldByName(suitVar,"links",0);
    iDynTree::assertTrue(linksVar != NULL);
    std::cerr<<"Found 'links' variable with dimension "<< linksVar->dims[0] << std::endl;

    matvar_t *linkCell;

    std::vector< std::string > linksName;
    linksName.resize(linksVar->dims[0]);

    std::vector< iDynTree::MatrixDynSize > linksPositions;
    linksPositions.resize(linksVar->dims[0]);

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

        //Get positions
        temp = Mat_VarGetStructFieldByName(tempMeas,"position", 0);
        iDynTree::assertTrue(temp != NULL);
        Eigen::Map< Eigen::MatrixXd > tempMapPosition((double*)temp->data, temp->dims[0], temp->dims[1]);
        linksPositions[i].resize(temp->dims[0], temp->dims[1]);
        toEigen(linksPositions[i]) = tempMapPosition;

        //Get orientations
        temp = Mat_VarGetStructFieldByName(tempMeas,"orientation", 0);
        iDynTree::assertTrue(temp != NULL);
        Eigen::Map< Eigen::MatrixXd > tempMapOrientation((double*)temp->data, temp->dims[0], temp->dims[1]);
        linksQuaternions[i].resize(temp->dims[0], temp->dims[1]);
        toEigen(linksQuaternions[i]) = tempMapOrientation;
    }

    // declarations
    const double degree = M_PI/180;
    double angle = 0;

    //robot state
    int i = 0, j = 0;
    double zero_pos = 0, zero_vel = 0, zero_effort = 0,
    inc = degree, var = 0,
    speed = 1, wrench = 0, wrench_inc = 1;
    double tx_g = 0.000037, ty_g = 0.079215, tz_g = -0.000367;
    double tx_h = -0.000002, tz_h = -0.40926;
    double tx_i = -0.000373, tz_i = -0.42811;

    // message declarations
    sensor_msgs::JointState joint_state;

    geometry_msgs::TransformStamped ground_to_Pelvis;
    ground_to_Pelvis.header.frame_id = "ground";
    ground_to_Pelvis.child_frame_id = "Pelvis";

    geometry_msgs::TransformStamped going_around;
    going_around.header.frame_id = "ground";
    going_around.child_frame_id = "Pelvis";

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

        // position
        //for(int i=0; i<=47; i++){
        //	joint_state.position[i] = zero_pos;
        //}

        // Create new robot state
        // joint_state.position[31] = var;
        //var += inc;
        //if (var<-0.3 || var>0.3) inc *= -1;

        j++;
        if(j > humanStateQ->dims[1]) j = 0;
        for(i=0; i<=47; i++){
            joint_state.position[i] = humanStateQi.getVal(i,j);
        }

        // velocity
        //             for(int i=0; i<=47; i++){
        //                 joint_state.velocity[i] = zero_vel;
        //             }

        //effort
        //             for(int i=0; i<=47; i++){
        //                 joint_state.effort[i] = zero_effort;
        //             }
        //joint_state.effort[31] = wrench;
        wrench += wrench_inc;
        if (wrench<-50 || wrench>50) wrench_inc *= -1;

        // Send the joint state
        joint_pub.publish(joint_state);

        // update transform

        // ground to Pelvis transform
        ground_to_Pelvis.header.stamp = ros::Time::now();
        ground_to_Pelvis.transform.translation.x = linksPositions[0].getVal(0,j);
        ground_to_Pelvis.transform.translation.y = linksPositions[0].getVal(1,j);
        ground_to_Pelvis.transform.translation.z = linksPositions[0].getVal(2,j);
        ground_to_Pelvis.transform.rotation.x = linksQuaternions[0].getVal(1,j);
        ground_to_Pelvis.transform.rotation.y = linksQuaternions[0].getVal(2,j);
        ground_to_Pelvis.transform.rotation.z = linksQuaternions[0].getVal(3,j);
        ground_to_Pelvis.transform.rotation.w = linksQuaternions[0].getVal(0,j);

        // (moving in a circle with radius=2)
        going_around.header.stamp = ros::Time::now();
        going_around.transform.translation.x = cos(angle)*3;
        going_around.transform.translation.y = sin(angle)*3;
        going_around.transform.translation.z = 1;
        going_around.transform.rotation = tf::createQuaternionMsgFromYaw(angle+M_PI/2);
        angle += degree/4;

        // send the transform
        broadcaster.sendTransform(ground_to_Pelvis);
        //broadcaster.sendTransform(going_around);

        // This will adjust as needed per iteration
        loop_rate.sleep();
      }

      //Deleting stuff
      std::cerr<< "Deleting human_state variable" << std::endl;
      Mat_VarFree(humanStateVar);

      std::cerr<<"Closing human_state.mat file."<<std::endl;
      Mat_Close(pHumanState);

      std::cerr<<"Closing suit.mat file."<<std::endl;
      Mat_Close(pSuit);

    return 0;
  }








