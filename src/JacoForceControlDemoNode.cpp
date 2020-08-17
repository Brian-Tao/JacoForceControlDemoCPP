#include <ros/ros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <trac_ik/trac_ik.hpp>
#include "JacoForceControlCPP/test_trac_ik.hpp"
#include <assert.h>

#include <iostream>


int main(int argc, char** argv){
    ros::init(argc, argv, "JacoForceControlDemo_node");
    ros::NodeHandle nh;

    //std::cout << "started" << std::endl;
    test_trac_ik obj(nh);
    std::cout << obj.getHandle("Jaco_joint1") << std::endl;
    obj.getJointValue(obj.getHandle("Jaco_joint1"));
    obj.setJointValue(obj.getHandle("Jaco_joint1"), -1.0);
    obj.setJointValue(obj.getHandle("Jaco_joint1"), -0.5);
    return 0;
}