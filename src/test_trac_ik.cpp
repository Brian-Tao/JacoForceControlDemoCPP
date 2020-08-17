#include "JacoForceControlCPP/test_trac_ik.hpp"
#include "JacoForceControlCPP/b0RemoteApi.h"
#include <iostream>
#include <iomanip>

test_trac_ik::test_trac_ik(ros::NodeHandle& nh){
    this->isInitialized = false;
    this->isIDSet = false;

    // initialize nodehandle and remote api client
    this->nh_ = nh;
    this->client_ = new b0RemoteApi("b0RemoteApi_c++Client", "b0RemoteApi");

    //initialize KDL solver
    std::string robot_description_string;
    this->nh_.param("robot_description", robot_description_string, std::string());
    
    if (kdl_parser::treeFromString(robot_description_string, this->robot_tree_)){
        if (this->robot_tree_.getChain("world", "j2n6s300_link_6", this->robot_chain_)){
            // successfully get chain
            
        }else{
            ROS_ERROR("Fail to get chain");
        }
    }else{
        ROS_ERROR("Fail to get tree");
    }

    // initialize trac ik solver
    this->trac_ik_solver_ = new TRAC_IK::TRAC_IK("world", "j2n6s300_link_6", "/robot_description", 0.05, 1e-5);
    KDL::Chain chain;

    bool valid = this->trac_ik_solver_->getKDLChain(chain);

    if (!valid)
    {
        ROS_ERROR("There was no valid KDL chain found");
        return;
    }
    valid = this->trac_ik_solver_->getKDLLimits(this->jntLowerLimit, this->jntUpperLimit);
    if (!valid)
    {
        ROS_ERROR("There were no valid KDL joint limits found");
        return;
    }

    assert(chain.getNrOfJoints() == this->jntLowerLimit.data.size());
    assert(chain.getNrOfJoints() == this->jntUpperLimit.data.size());

    this->isInitialized = true;
    this->NrOfJnts = chain.getNrOfJoints();

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


bool test_trac_ik::getJntID(){
    this->jnt1ID_ = this->client_->readInt(this->client_->simxGetObjectHandle("Jaco_joint1",this->client_->simxServiceCall()), 1);
    this->jnt2ID_ = this->client_->readInt(this->client_->simxGetObjectHandle("Jaco_joint2",this->client_->simxServiceCall()), 1);
    this->jnt3ID_ = this->client_->readInt(this->client_->simxGetObjectHandle("Jaco_joint3",this->client_->simxServiceCall()), 1);
    this->jnt4ID_ = this->client_->readInt(this->client_->simxGetObjectHandle("Jaco_joint4",this->client_->simxServiceCall()), 1);
    this->jnt5ID_ = this->client_->readInt(this->client_->simxGetObjectHandle("Jaco_joint5",this->client_->simxServiceCall()), 1);
    this->jnt6ID_ = this->client_->readInt(this->client_->simxGetObjectHandle("Jaco_joint6",this->client_->simxServiceCall()), 1);

    this->isIDSet = true;

    return true;
}

void test_trac_ik::printJntID(){
    if (this->isIDSet){
        std::cout << std::setw(4) << this->jnt1ID_
                    << std::setw(4) << this->jnt2ID_
                    << std::setw(4) << this->jnt3ID_
                    << std::setw(4) << this->jnt4ID_
                    << std::setw(4) << this->jnt5ID_
                    << std::setw(4) << this->jnt6ID_ << std::endl;
    }
}

bool test_trac_ik::moveToTargetJntAngle(std::vector<float> jntValue){
    if (this->isInitialized and jntValue.size() == this->NrOfJnts){
        this->client_->simxSetJointPosition(this->jnt1ID_, jntValue[0], this->client_->simxServiceCall());
        this->client_->simxSetJointPosition(this->jnt2ID_, jntValue[1], this->client_->simxServiceCall());
        this->client_->simxSetJointPosition(this->jnt3ID_, jntValue[2], this->client_->simxServiceCall());
        this->client_->simxSetJointPosition(this->jnt4ID_, jntValue[3], this->client_->simxServiceCall());
        this->client_->simxSetJointPosition(this->jnt5ID_, jntValue[4], this->client_->simxServiceCall());
        this->client_->simxSetJointPosition(this->jnt6ID_, jntValue[5], this->client_->simxServiceCall());
    }else{
        return false;
    }
}

bool test_trac_ik::moveToTargetPos(KDL::Frame target_pos){
    //TBD
}

std::vector<float> test_trac_ik::getNominalJntAngle(){
    if (this->isInitialized) {
        std::vector<float> returner;
        for (int i = 0; i < this->NrOfJnts; ++i){
            returner.push_back((this->jntLowerLimit.data[i] + this->jntUpperLimit.data[i]) / 2);
        }

        return returner;

    }else{
        return std::vector<float>(); 
    }
}




