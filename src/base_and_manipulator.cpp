#include "JacoForceControlCPP/base_and_manipulator.hpp"
#include "JacoForceControlCPP/b0RemoteApi.h"
#include <iostream>
#include <iomanip>

base_and_manipulator::base_and_manipulator(ros::NodeHandle& nh, std::string chainStart, std::string chainEnd, std::string roboType){
    this->isInitialized = false;
    this->isIDSet = false;
    this->isBaseIDSet = false;

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

base_and_manipulator::~base_and_manipulator(){}

int base_and_manipulator::getHandle(std::string objName){
    int res = this->client_->readInt(this->client_->simxGetObjectHandle(objName.c_str(),this->client_->simxServiceCall()), 1);
    return res;
}

double base_and_manipulator::getJointValue(int objHandle){
    auto jntValue = this->client_->simxGetJointPosition(objHandle, this->client_->simxServiceCall());

    for (auto msgBox : *jntValue){
        std::cout << msgBox << std::endl;
    }
    
    return 0;
}


bool base_and_manipulator::setSingleJntValueForSure(int objHandle, float targetJntValue, double timeout, double tolerance){
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


bool base_and_manipulator::getJntID(){
    this->jnt1ID_ = this->client_->readInt(this->client_->simxGetObjectHandle("Jaco_joint1",this->client_->simxServiceCall()), 1);
    this->jnt2ID_ = this->client_->readInt(this->client_->simxGetObjectHandle("Jaco_joint2",this->client_->simxServiceCall()), 1);
    this->jnt3ID_ = this->client_->readInt(this->client_->simxGetObjectHandle("Jaco_joint3",this->client_->simxServiceCall()), 1);
    this->jnt4ID_ = this->client_->readInt(this->client_->simxGetObjectHandle("Jaco_joint4",this->client_->simxServiceCall()), 1);
    this->jnt5ID_ = this->client_->readInt(this->client_->simxGetObjectHandle("Jaco_joint5",this->client_->simxServiceCall()), 1);
    this->jnt6ID_ = this->client_->readInt(this->client_->simxGetObjectHandle("Jaco_joint6",this->client_->simxServiceCall()), 1);

    this->isIDSet = true;

    return true;
}

bool base_and_manipulator::getJntValue(KDL::JntArray& jntArray){
    if (this->isIDSet){
        jntArray.resize(this->NrOfJnts);
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

void base_and_manipulator::printJntID(){
    if (this->isIDSet){
        std::cout << std::setw(4) << this->jnt1ID_
                    << std::setw(4) << this->jnt2ID_
                    << std::setw(4) << this->jnt3ID_
                    << std::setw(4) << this->jnt4ID_
                    << std::setw(4) << this->jnt5ID_
                    << std::setw(4) << this->jnt6ID_ << std::endl;
    }
}

bool base_and_manipulator::moveToTargetJntAngle(std::vector<float> jntValue, double duration){
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

bool base_and_manipulator::moveToTargetJntAngle(KDL::JntArray jntValue, double duration, double timeout, double tolerance){
    if (this->isIDSet and jntValue.data.size() == this->NrOfJnts){
        boost::posix_time::ptime start_time = boost::posix_time::microsec_clock::local_time();;
        boost::posix_time::time_duration diff = boost::posix_time::microsec_clock::local_time() - start_time;
        KDL::JntArray curPos;
        std::vector<bool> progress(this->NrOfJnts, false);
        int actIteration = 20;
        
        while (diff.total_nanoseconds() / 1e9 < timeout) {
            bool flag = true;
            if (!this->getJntValue(curPos)){
                ROS_ERROR("Failed to get current values");
                return false;
            }

            for (int i = 0; i < this->NrOfJnts; ++i){
                if (abs(jntValue.data[i] - curPos.data[i]) < tolerance){
                    progress[i] = true;
                }
                flag = flag and progress[i];
            }
            
            if (flag){
                return true;
            }else {
                // run #actIteration loop
                for (int i = 0; i < actIteration; ++i){
                    if (!progress[0])
                        this->client_->simxSetJointPosition(this->jnt1ID_, jntValue.data[0], this->client_->simxServiceCall());
                    if (!progress[1])
                        this->client_->simxSetJointPosition(this->jnt2ID_, jntValue.data[1], this->client_->simxServiceCall());
                    if (!progress[2])
                        this->client_->simxSetJointPosition(this->jnt3ID_, jntValue.data[2], this->client_->simxServiceCall());
                    if (!progress[3])
                        this->client_->simxSetJointPosition(this->jnt4ID_, jntValue.data[3], this->client_->simxServiceCall());
                    if (!progress[4])
                        this->client_->simxSetJointPosition(this->jnt5ID_, jntValue.data[4], this->client_->simxServiceCall());
                    if (!progress[5])
                        this->client_->simxSetJointPosition(this->jnt6ID_, jntValue.data[5], this->client_->simxServiceCall());
                }
            }
            diff = boost::posix_time::microsec_clock::local_time() - start_time;
        }
        ROS_ERROR("Actuation timed out");
        return false;
    }else{
        ROS_ERROR("Jnt ID is not set yet or wrong input size");
        return false;
    }
}

bool base_and_manipulator::moveToTargetPos(KDL::Frame targetPos){
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
        std::cout << std::setw(20) << "Expected Jnt values" << std::setw(20) << "Current Jnt Values" << std::endl;
        for (int i = 0; i < 6; ++i){
            std::cout << std::setw(20) << result.data[i] << std::setw(20) << curPos.data[i] << std::endl;
        }

        this->moveToTargetJntAngle(result);

        pause(2);
        return true;
    }else {
        return false;
    }
}

std::vector<float> base_and_manipulator::getNominalJntAngle(){
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


void base_and_manipulator::pause(double duration){
    /*
    @breif pause for certain time, in seconds
    */
    boost::posix_time::ptime start_time = boost::posix_time::microsec_clock::local_time();;
    boost::posix_time::time_duration diff = boost::posix_time::microsec_clock::local_time() - start_time;
    while (diff.total_nanoseconds() / 1e9 < duration){
        diff = boost::posix_time::microsec_clock::local_time() - start_time;
    }
}


bool base_and_manipulator::getBaseID(){
    this->baseWorldConnectorID_ = this->client_->readInt(this->client_->simxGetObjectHandle("Pioneer_p3dx_connection7",this->client_->simxServiceCall()), 1);
    this->baseLeftMotorID_ = this->client_->readInt(this->client_->simxGetObjectHandle("Pioneer_p3dx_leftMotor",this->client_->simxServiceCall()), 1);
    this->baseRightMotorID_ = this->client_->readInt(this->client_->simxGetObjectHandle("Pioneer_p3dx_rightMotor",this->client_->simxServiceCall()), 1);

    this->isBaseIDSet = true;

    return true;
}


void base_and_manipulator::printBaseID(){
    if (this->isBaseIDSet){
        std::cout << std::setw(4) << this->baseWorldConnectorID_
                    << std::setw(4) << this->baseLeftMotorID_
                    << std::setw(4) << this->baseRightMotorID_ << std::endl;
    }
}


bool base_and_manipulator::moveToTargetBasePos(double x, double y, double theta){
    //pass
}

bool base_and_manipulator::moveBaseForward(double vel){
    if (this->isBaseIDSet){
        if (vel < 0){
            ROS_ERROR("Expect positive input");
            return false;
        }
        this->client_->simxSetJointTargetVelocity(this->baseLeftMotorID_, vel, this->client_->simxServiceCall());
        this->client_->simxSetJointTargetVelocity(this->baseRightMotorID_, vel, this->client_->simxServiceCall());
        
        return true;
    }else {
        return false;
    }
}

 bool base_and_manipulator::moveBaseBackward(double vel){
     if (this->isBaseIDSet){
        if (vel > 0){
            ROS_ERROR("Expect negative input");
            return false;
        }
        this->client_->simxSetJointTargetVelocity(this->baseLeftMotorID_, vel, this->client_->simxServiceCall());
        this->client_->simxSetJointTargetVelocity(this->baseRightMotorID_, vel, this->client_->simxServiceCall());
        
        return true;
    }else {
        ROS_ERROR("Base ID is not set yet");
        return false;
    }
 }

 
bool base_and_manipulator::getManipRootPos(KDL::Frame& rootPos){
    if (this->isBaseIDSet){
        auto data = this->client_->simxGetObjectPose(this->baseWorldConnectorID_, -1, this->client_->simxServiceCall());
        std::vector<double> Pos;
        (*data)[1].convert(Pos);

        rootPos.p.data[0] = Pos[0];
        rootPos.p.data[1] = Pos[1];
        rootPos.p.data[2] = Pos[2];

        return true;
    }else{
        ROS_ERROR("Base ID is not set yet");
        return false;
    }
}

bool base_and_manipulator::moveToTargetXPos(double xPos, double tolerance){
    KDL::Frame rootPos;

    bool success = this->getManipRootPos(rootPos);
    if (!success) {
        ROS_ERROR("Can't obtain current base pos");
        return false;
    }

    while (abs(xPos - rootPos.p.x()) >= tolerance){
        if (xPos > rootPos.p.x()){
            this->moveBaseForward(0.8);
        }else{
            this->moveBaseBackward();
        }

        success = this->getManipRootPos(rootPos);
        if (!success) {
            ROS_ERROR("Can't obtain current base pos");
            return false;
        }
    }

    this->stopBase();
    return true;
    
}

bool base_and_manipulator::stopBase(){
    if (this->isBaseIDSet){
        this->client_->simxSetJointTargetVelocity(this->baseLeftMotorID_, 0, this->client_->simxServiceCall());
        this->client_->simxSetJointTargetVelocity(this->baseRightMotorID_, 0, this->client_->simxServiceCall());
        
        return true;
    }else {
        ROS_ERROR("Base ID is not set yet");
        return false;
    }
}


void base_and_manipulator::pick(){
    assert(this->moveToTargetXPos(0.8));

    std::cout << "Target pos 1" << std::endl;
    KDL::Vector Pos(0, 0.4, 0.2);
    KDL::Rotation Rot = KDL::Rotation::RPY( M_PI / 2.0, -M_PI / 2, 0);
    KDL::Frame targetPos(Rot, Pos);
    assert(this->moveToTargetPos(targetPos));
    pause(1);

    std::cout << "Target pos 2" << std::endl;
    KDL::Vector Pos2(0, 0.5, 0.2);
    KDL::Frame targetPos2(Rot, Pos2);
    assert(this->moveToTargetPos(targetPos2));
    pause(1);

    std::cout << "Closing gripper" << std::endl;
    this->closeGripper();
    pause(2);

    std::cout << "Target pos 3" << std::endl;
    KDL::Vector Pos3(0, 0.4, 0.3);
    KDL::Frame targetPos3(Rot, Pos3);
    assert(this->moveToTargetPos(targetPos3));

}

void base_and_manipulator::place(){
    assert(this->moveToTargetXPos(-0.8));

    std::cout << "Target pos 4" << std::endl;
    KDL::Vector Pos(0, 0.5, 0.2);
    KDL::Rotation Rot = KDL::Rotation::RPY( M_PI / 2.0, -M_PI / 2, 0);
    KDL::Frame targetPos(Rot, Pos);
    assert(this->moveToTargetPos(targetPos));
    pause(1);

    std::cout << "Opening gripper" << std::endl;
    this->openGripper();
    pause(2);

    std::cout << "Target pos 5" << std::endl;
    KDL::Vector Pos2(0, 0.4, 0.2);
    KDL::Frame targetPos2(Rot, Pos2);
    assert(this->moveToTargetPos(targetPos2));

}

bool base_and_manipulator::getGripperID(){
    this->finger12Motor1ID_ = this->client_->readInt(this->client_->simxGetObjectHandle("JacoHand_fingers12_motor1",this->client_->simxServiceCall()), 1);
    this->finger12Motor2ID_ = this->client_->readInt(this->client_->simxGetObjectHandle("JacoHand_fingers12_motor2",this->client_->simxServiceCall()), 1);
    this->finger3Motor1ID_ = this->client_->readInt(this->client_->simxGetObjectHandle("JacoHand_finger3_motor1",this->client_->simxServiceCall()), 1);
    this->finger3Motor2ID_ = this->client_->readInt(this->client_->simxGetObjectHandle("JacoHand_finger3_motor2",this->client_->simxServiceCall()), 1);

    this->isGripperIDSet = true;

    return true;
}

void base_and_manipulator::printGripperID(){
    if (this->isGripperIDSet){
        std::cout << std::setw(4) << this->finger12Motor1ID_
                    << std::setw(4) << this->finger12Motor2ID_
                    << std::setw(4) << this->finger3Motor1ID_
                    << std::setw(4) << this->finger3Motor2ID_ << std::endl;
    }
}

void base_and_manipulator::openGripper(double vel){
    if (this->isGripperIDSet){
        if (vel < 0){
            ROS_ERROR("Expect positive input");
            return;
        }
        this->client_->simxSetJointTargetVelocity(this->finger12Motor1ID_, vel, this->client_->simxServiceCall());
        this->client_->simxSetJointTargetVelocity(this->finger12Motor2ID_, vel, this->client_->simxServiceCall());
        this->client_->simxSetJointTargetVelocity(this->finger3Motor1ID_, vel, this->client_->simxServiceCall());
        this->client_->simxSetJointTargetVelocity(this->finger3Motor2ID_, vel, this->client_->simxServiceCall());
    }else{
        ROS_ERROR("Gripper ID is not set yet");
        return;
    }
}

void base_and_manipulator::closeGripper(double vel){
    if (this->isGripperIDSet){
        if (vel > 0){
            ROS_ERROR("Expect negative input");
            return;
        }
        this->client_->simxSetJointTargetVelocity(this->finger12Motor1ID_, vel, this->client_->simxServiceCall());
        this->client_->simxSetJointTargetVelocity(this->finger12Motor2ID_, vel, this->client_->simxServiceCall());
        this->client_->simxSetJointTargetVelocity(this->finger3Motor1ID_, vel, this->client_->simxServiceCall());
        this->client_->simxSetJointTargetVelocity(this->finger3Motor2ID_, vel, this->client_->simxServiceCall());
    }else{
        ROS_ERROR("Gripper ID is not set yet");
        return;
    }
}