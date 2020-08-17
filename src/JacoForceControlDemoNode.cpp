#include <ros/ros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <trac_ik/trac_ik.hpp>
#include "JacoForceControlCPP/test_trac_ik.hpp"
#include <assert.h>
#include "math.h"
#include <iostream>


int main(int argc, char** argv){
    ros::init(argc, argv, "JacoForceControlDemo_node");
    ros::NodeHandle nh;

    //std::cout << "started" << std::endl;
    test_trac_ik obj(nh);
    // std::cout << obj.getHandle("Jaco_joint1") << std::endl;
    // obj.getJointValue(obj.getHandle("Jaco_joint1"));
    // obj.setJointValue(obj.getHandle("Jaco_joint1"), -1.0);
    // obj.setJointValue(obj.getHandle("Jaco_joint1"), -0.5);

    obj.getJntID();
    obj.printJntID();

    // std::vector<float> target {-M_PI/2, M_PI/2, 3/2*M_PI, 3/2*M_PI, 3/2*M_PI, M_PI};
    // assert(obj.moveToTargetJntAngle(target));
    KDL::JntArray curPos;
    if (!obj.getJntValue(curPos)){
        ROS_ERROR("Failed to get current values");
    }

    KDL::Vector Pos(0.0, 0.5, 0.4);
    KDL::Rotation Rot = KDL::Rotation::Identity();
    KDL::Frame targetPos(Pos);
    assert(obj.moveToTargetPos(targetPos));
    return 0;
}