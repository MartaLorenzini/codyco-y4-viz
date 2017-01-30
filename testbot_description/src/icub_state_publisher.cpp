#include <ros/ros.h>
#include <string>
#include <std_msgs/Header.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>


int main(int argc, char **argv){

    ros::init(argc, argv, "icub_state_publisher");
    ros::NodeHandle n;

    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate(30);

    // declarations
    const double degree = M_PI/180;
    double angle = 0;

    //robot state
    double zero_pos = 0, zero_vel = 0, zero_effort = 0,
    inc = degree, var = 0,
    speed = 1, wrench = 0, wrench_inc = 1;

    // message declarations
    sensor_msgs::JointState joint_state;
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

        // update transform
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
        broadcaster.sendTransform(going_around);

        // This will adjust as needed per iteration
        loop_rate.sleep();
      }

    return 0;

  }