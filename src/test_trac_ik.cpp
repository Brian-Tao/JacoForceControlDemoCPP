#include "JacoForceControlCPP/test_trac_ik.hpp"
#include "JacoForceControlCPP/b0RemoteApi.h"
#include <iostream>


test_trac_ik::test_trac_ik(ros::NodeHandle& nh){
    this->nh_ = nh;
    this->client_ = new b0RemoteApi("b0RemoteApi_c++Client", "b0RemoteApi");

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

int test_trac_ik::getHandle(std::string objName){
    int res = this->client_->readInt(this->client_->simxGetObjectHandle(objName.c_str(),this->client_->simxServiceCall()), 1);
    return res;
}

double test_trac_ik::getJointValue(int objHandle){
    auto jntValue = this->client_->simxGetJointPosition(objHandle, this->client_->simxServiceCall());

    for (auto msgBox : *jntValue){
        std::cout << msgBox << std::endl;
    }
    
    return 0;
}

void test_trac_ik::setJointValue(int objHandle, float jntValue){
    auto res = this->client_->simxSetJointPosition(objHandle, jntValue, this->client_->simxServiceCall());
    std::cout << (*res)[0] << std::endl;
};