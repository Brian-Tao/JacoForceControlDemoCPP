#include <ros/ros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <trac_ik/trac_ik.hpp>
#include "JacoForceControlCPP/test_trac_ik.hpp"


int main(int argc, char** argv){
    ros::init(argc, argv, "JacoForceControlDemo_node");
    ros::NodeHandle nh;

    test_trac_ik obj(nh);
    return 0;
}