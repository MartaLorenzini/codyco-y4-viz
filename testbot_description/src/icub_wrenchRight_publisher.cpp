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

template<typename Out>
void split(const std::string &s, char delim, Out result);
std::vector<std::string> split(const std::string &s, char delim);

inline iDynTree::Transform transformFromMatlabExtractor_byString(std::string matVarName, std::string matVarString);
inline iDynTree::Transform transformFromMatlabExtractor(std::string matName, int n, ...);

inline iDynTree::MatrixDynSize matrixFromMatlabExtractor_byString(std::string matVarName, std::string matVarString);
inline iDynTree::MatrixDynSize matrixFromMatlabExtractor(std::string matName, int n, ...);

int main(int argc, char **argv){

    ros::init(argc, argv, "icub_wrenchRight_publisher");
    ros::NodeHandle n;

    ros::Publisher wrench_pub = n.advertise<geometry_msgs::WrenchStamped>("wrench_stamped", 1);
    ros::Rate loop_rate(100);

    //LOAD OF robot.mat
    iDynTree::MatrixDynSize robotWrenchRightArmForces_i = matrixFromMatlabExtractor_byString("robot.mat","robot/data/links/rightarm/forces");
    iDynTree::MatrixDynSize robotWrenchRightArmMoments_i = matrixFromMatlabExtractor_byString("robot.mat","robot/data/links/rightarm/moments");

    int j = 0;
    geometry_msgs::WrenchStamped wrenchMsgRight;
    ros::param::param<std::string>("~frame_id", wrenchMsgRight.header.frame_id, "r_forearm");
    
    while (ros::ok()){

        j++;
        if(j > (robotWrenchRightArmForces_i.cols())) j = 0;

        //compose WrenchStamped Msg
        wrenchMsgRight.header.stamp = ros::Time::now();
        wrenchMsgRight.wrench.force.x = robotWrenchRightArmForces_i.getVal(0,j);
        wrenchMsgRight.wrench.force.y= robotWrenchRightArmForces_i.getVal(1,j);
        wrenchMsgRight.wrench.force.z = robotWrenchRightArmForces_i.getVal(2,j);
        wrenchMsgRight.wrench.torque.x = robotWrenchRightArmMoments_i.getVal(0,j);
        wrenchMsgRight.wrench.torque.y = robotWrenchRightArmMoments_i.getVal(1,j);
        wrenchMsgRight.wrench.torque.z = robotWrenchRightArmMoments_i.getVal(2,j);

        wrench_pub.publish(wrenchMsgRight);

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
