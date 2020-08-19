#include <ros/ros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <trac_ik/trac_ik.hpp>
#include "JacoForceControlCPP/base_and_manipulator.hpp"
#include <assert.h>
#include "math.h"
#include <iostream>

void pause(double duration);

int main(int argc, char** argv){
    ros::init(argc, argv, "JacoForceControlDemo_node");
    ros::NodeHandle nh;

    //std::cout << "started" << std::endl;
    base_and_manipulator obj(nh);
    // std::cout << obj.getHandle("Jaco_joint1") << std::endl;
    // obj.getJointValue(obj.getHandle("Jaco_joint1"));
    // obj.setJointValue(obj.getHandle("Jaco_joint1"), -1.0);
    // obj.setJointValue(obj.getHandle("Jaco_joint1"), 0.5);

    obj.getJntID();
    obj.printJntID();
    obj.getBaseID();
    obj.printBaseID();
    obj.getGripperID();
    obj.printGripperID();

    // std::vector<float> target {-M_PI/2, M_PI/2, 3/2*M_PI, 3/2*M_PI, 3/2*M_PI, M_PI};
    // assert(obj.moveToTargetJntAngle(target));
    KDL::JntArray curPos;
    if (!obj.getJntValue(curPos)){
        ROS_ERROR("Failed to get current values");
    }

    // Manipulator test
    // KDL::Vector Pos(0.1, 0.4, 0.2);
    // KDL::Rotation Rot = KDL::Rotation::RPY( M_PI / 2.0, -M_PI / 2, 0);
    // KDL::Frame targetPos(Rot, Pos);
    // assert(obj.moveToTargetPos(targetPos));

    // KDL::Vector Pos2(0.0, 0.5, 0.1);
    // KDL::Frame targetPos2(Pos2);
    // assert(obj.moveToTargetPos(targetPos2));

    // Base Test
    // obj.getBaseID();
    // obj.printBaseID();
    // pause(1);

    // assert(obj.moveToTargetXPos(0.5));
    // pause(2);
    // assert(obj.moveToTargetXPos(-0.5));

    // Gripper Test
    // obj.closeGripper();
    // pause(1);
    
    obj.pick();
    obj.place();
    return 0;
}


void pause(double duration){
    /*
    @breif pause for certain time, in seconds
    */
    boost::posix_time::ptime start_time = boost::posix_time::microsec_clock::local_time();;
    boost::posix_time::time_duration diff = boost::posix_time::microsec_clock::local_time() - start_time;
    while (diff.total_nanoseconds() / 1e9 < duration){
        diff = boost::posix_time::microsec_clock::local_time() - start_time;
    }
}