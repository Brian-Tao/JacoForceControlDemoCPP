#ifndef BASE_AND_MANIPULATOR_HPP
#define BASE_AND_MANIPULATOR_HPP

#include <ros/ros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <trac_ik/trac_ik.hpp>
#include <boost/date_time.hpp>

#include "JacoForceControlCPP/b0RemoteApi.h"
#include <string>
#include <assert.h>

class base_and_manipulator{
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
    base_and_manipulator(ros::NodeHandle& nh, std::string chainStart = "world", std::string chainEnd = "j2n6s300_link_6", std::string roboType = "j2n6s300");
    ~base_and_manipulator();

    // test member function
    int getHandle(std::string objName );
    double getJointValue(int objHandle );

    // might be useful function
    bool setSingleJntValueForSure(int objHandle, float targetJntValue, double timeout = 5.0, double tolerance = 0.1);

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







