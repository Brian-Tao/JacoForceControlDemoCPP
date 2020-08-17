#ifndef TEST_TRAC_IK_HPP
#define TEST_TRAC_IK_HPP

#include <ros/ros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <trac_ik/trac_ik.hpp>
#include <boost/date_time.hpp>

#include "JacoForceControlCPP/b0RemoteApi.h"
#include <string>

class test_trac_ik{
    ros::NodeHandle nh_; 
    KDL::Tree robot_tree_;
    KDL::Chain robot_chain_;
    TRAC_IK::TRAC_IK* trac_ik_solver_;
    b0RemoteApi* client_;

    bool isInitialized;
    bool isIDSet;

    int NrOfJnts;
    int jnt1ID_;
    int jnt2ID_;
    int jnt3ID_;
    int jnt4ID_;
    int jnt5ID_;
    int jnt6ID_;

    KDL::JntArray jntLowerLimit;
    KDL::JntArray jntUpperLimit;

    //auxiliary function
    void pause(double duration);


public:
    test_trac_ik(ros::NodeHandle& nh);
    ~test_trac_ik();

    // test member function
    int getHandle(std::string objName );
    double getJointValue(int objHandle );
    void setJointValue(int objHandle, float jntValue);

    // might be useful function
    bool getJntID();
    bool getJntValue(KDL::JntArray& jntArray);
    void printJntID();
    bool moveToTargetJntAngle(std::vector<float> jntValue, double duration = 0.005);
    bool moveToTargetJntAngle(KDL::JntArray jntValue, double duration = 0.005);
    bool moveToTargetPos(KDL::Frame targetPos);

    //auxiliary function
    std::vector<float> getNominalJntAngle();


};

#endif







