#include "JacoForceControlCPP/test_trac_ik.hpp"

test_trac_ik::test_trac_ik(ros::NodeHandle& nh){
    this->nh_ = nh;

    std::string robot_description_string;
    this->nh_.param("robot_description", robot_description_string, std::string());

    if (kdl_parser::treeFromString(robot_description_string, this->robot_tree_)){
        
        if (this->robot_tree_.getChain("world", "Jaco_link7", this->robot_chain_)){
            //pass
        }else{
            ROS_ERROR("Fail to get chain");
        }
    }else{
        ROS_ERROR("Fail to get tree");
    }
}

test_trac_ik::~test_trac_ik(){}