#ifndef TEST_TRAC_IK_HPP
#define TEST_TRAC_IK_HPP

#include <ros/ros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <trac_ik/trac_ik.hpp>
class test_trac_ik{
    ros::NodeHandle nh_; 
    KDL::Tree robot_tree_;
    KDL::Chain robot_chain_;
    TRAC_IK::TRAC_IK* trac_ik_solver;

public:
    test_trac_ik(ros::NodeHandle& nh);
    ~test_trac_ik();
};

#endif







