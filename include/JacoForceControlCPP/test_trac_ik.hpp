#ifndef TEST_TRAC_IK_HPP
#define TEST_TRAC_IK_HPP

#include <ros/ros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <trac_ik/trac_ik.hpp>
#include "JacoForceControlCPP/b0RemoteApi.h"
#include <string>

class test_trac_ik{
    ros::NodeHandle nh_; 
    KDL::Tree robot_tree_;
    KDL::Chain robot_chain_;
    TRAC_IK::TRAC_IK* trac_ik_solver;
    b0RemoteApi* client_;

public:
    test_trac_ik(ros::NodeHandle& nh);
    ~test_trac_ik();

    int getHandle(std::string objName );
    double getJointValue(int objHandle );
    void setJointValue(int objHandle, float jntValue);
};

#endif







