#include "JacoForceControlCPP/test_trac_ik.hpp"
#include "JacoForceControlCPP/b0RemoteApi.h"
#include <iostream>
#include <iomanip>

test_trac_ik::test_trac_ik(ros::NodeHandle& nh, std::string chainStart, std::string chainEnd, std::string roboType){
    this->isInitialized = false;
    this->isIDSet = false;

    // initialize nodehandle and remote api client
    this->nh_ = nh;
    this->client_ = new b0RemoteApi("b0RemoteApi_c++Client", "b0RemoteApi");

    // initialize trac ik solver
    this->trac_ik_solver_ = new TRAC_IK::TRAC_IK(chainStart, chainEnd, "/robot_description", 0.005, 1e-5, TRAC_IK::Speed);
    
    bool valid = this->trac_ik_solver_->getKDLChain(this->robot_chain_);

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

    assert(this->robot_chain_.getNrOfJoints() == this->jntLowerLimit.data.size());
    assert(this->robot_chain_.getNrOfJoints() == this->jntUpperLimit.data.size());

    this->isInitialized = true;
    this->NrOfJnts = this->robot_chain_.getNrOfJoints();

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


bool test_trac_ik::setSingleJntValueForSure(int objHandle, float targetJntValue, double timeout, double tolerance){
    boost::posix_time::ptime start_time = boost::posix_time::microsec_clock::local_time();;
    boost::posix_time::time_duration diff = boost::posix_time::microsec_clock::local_time() - start_time;
    auto res = this->client_->simxSetJointPosition(objHandle, targetJntValue, this->client_->simxServiceCall());
    double jntFeedback;
    while (diff.total_nanoseconds() / 1e9 < timeout) {
        res = this->client_->simxSetJointPosition(objHandle, targetJntValue, this->client_->simxServiceCall());
        auto data = this->client_->simxGetJointPosition(objHandle, this->client_->simxServiceCall());
        (*data)[1].convert(jntFeedback);

        if (abs(jntFeedback - targetJntValue) <= tolerance){
            return true;
        }

        diff = boost::posix_time::microsec_clock::local_time() - start_time;
    }

    ROS_ERROR("Joint actuation time out");
    return false;
}


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

bool test_trac_ik::getJntValue(KDL::JntArray& jntArray){
    if (isIDSet){
        jntArray.resize(this->NrOfJnts);
        float temp;
        auto data = this->client_->simxGetJointPosition(this->jnt1ID_, this->client_->simxServiceCall());
        (*data)[1].convert(jntArray.data[0]);
        data = this->client_->simxGetJointPosition(this->jnt2ID_, this->client_->simxServiceCall());
        (*data)[1].convert(jntArray.data[1]);
        data = this->client_->simxGetJointPosition(this->jnt3ID_, this->client_->simxServiceCall());
        (*data)[1].convert(jntArray.data[2]);
        data = this->client_->simxGetJointPosition(this->jnt4ID_, this->client_->simxServiceCall());
        (*data)[1].convert(jntArray.data[3]);
        data = this->client_->simxGetJointPosition(this->jnt5ID_, this->client_->simxServiceCall());
        (*data)[1].convert(jntArray.data[4]);
        data = this->client_->simxGetJointPosition(this->jnt6ID_, this->client_->simxServiceCall());
        (*data)[1].convert(jntArray.data[5]);

        return true;

    }else{
        return false;
    }

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

bool test_trac_ik::moveToTargetJntAngle(std::vector<float> jntValue, double duration){
    if (this->isIDSet and jntValue.size() == this->NrOfJnts){
        this->client_->simxSetJointPosition(this->jnt1ID_, jntValue[0], this->client_->simxServiceCall());
        pause(duration);
        this->client_->simxSetJointPosition(this->jnt2ID_, jntValue[1], this->client_->simxServiceCall());
        pause(duration);
        this->client_->simxSetJointPosition(this->jnt3ID_, jntValue[2], this->client_->simxServiceCall());
        pause(duration);
        this->client_->simxSetJointPosition(this->jnt4ID_, jntValue[3], this->client_->simxServiceCall());
        pause(duration);
        this->client_->simxSetJointPosition(this->jnt5ID_, jntValue[4], this->client_->simxServiceCall());
        pause(duration);
        this->client_->simxSetJointPosition(this->jnt6ID_, jntValue[5], this->client_->simxServiceCall());
        pause(duration);
        
        return true;
    }else{
        return false;
    }
}

bool test_trac_ik::moveToTargetJntAngle(KDL::JntArray jntValue, double duration){
    if (this->isIDSet and jntValue.data.size() == this->NrOfJnts){
        bool res = true;
        res = this->setSingleJntValueForSure(this->jnt1ID_, jntValue.data[0]);
        res = res and this->setSingleJntValueForSure(this->jnt2ID_, jntValue.data[1]);
        res = res and this->setSingleJntValueForSure(this->jnt3ID_, jntValue.data[2]);
        res = res and this->setSingleJntValueForSure(this->jnt4ID_, jntValue.data[3]);
        res = res and this->setSingleJntValueForSure(this->jnt5ID_, jntValue.data[4]);
        res = res and this->setSingleJntValueForSure(this->jnt6ID_, jntValue.data[5]);

        if (!res){
            ROS_ERROR("At least one joint was not fully actuated");
        }
        return res;
    }else{
        return false;
    }
}

bool test_trac_ik::moveToTargetPos(KDL::Frame targetPos){
    if (this->isInitialized)
    {   
        KDL::JntArray curPos;
        KDL::JntArray result;

        if (!this->getJntValue(curPos)){
            ROS_ERROR("Failed to get current values");
            return false;
        }

        int res = this->trac_ik_solver_->CartToJnt(curPos, targetPos, result);

        if (res < 0){
            ROS_ERROR("Trac ik can't solve given frame");
            return false;
        }

        this->getJntValue(curPos);
        for (int i = 0; i < 6; ++i){
            std::cout << std::setw(8) << result.data[i] << std::setw(8) << curPos.data[i] << std::endl;
        }

        this->moveToTargetJntAngle(result);
        
        pause(2);
        return true;
    }else {
        return false;
    }
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


void test_trac_ik::pause(double duration){
    /*
    @breif pause for certain time, in seconds
    */
    boost::posix_time::ptime start_time = boost::posix_time::microsec_clock::local_time();;
    boost::posix_time::time_duration diff = boost::posix_time::microsec_clock::local_time() - start_time;
    while (diff.total_nanoseconds() / 1e9 < duration){
        diff = boost::posix_time::microsec_clock::local_time() - start_time;
    }
}



