#ifndef BASE_AND_MANIPULATOR_HPP
#define BASE_AND_MANIPULATOR_HPP

#include <ros/ros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <trac_ik/trac_ik.hpp>
#include <boost/date_time.hpp>
#include "JacoForceControlCPP/b0RemoteApi.h"
#include <string>
#include <assert.h>
#include <thread>

class base_and_manipulator{
    ros::NodeHandle nh_; 
    KDL::Tree robot_tree_;
    KDL::Chain robot_chain_;
    TRAC_IK::TRAC_IK* trac_ik_solver_;
    b0RemoteApi* client_;

    bool isInitialized;
    bool isIDSet; // is manipulator ID set
    bool isBaseIDSet;
    bool isGripperIDSet;

    int NrOfJnts;
    int jnt1ID_;
    int jnt2ID_;
    int jnt3ID_;
    int jnt4ID_;
    int jnt5ID_;
    int jnt6ID_;

    int baseWorldConnectorID_;
    int baseLeftMotorID_;
    int baseRightMotorID_;

    int finger12Motor1ID_;
    int finger12Motor2ID_;
    int finger3Motor1ID_;
    int finger3Motor2ID_;

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
    bool getJntID();
    bool getJntValue(KDL::JntArray& jntArray);
    void printJntID();

    bool setSingleJntValueForSure(int objHandle, float targetJntValue, double timeout = 70.0, double tolerance = 0.1);
    bool moveToTargetJntAngle(std::vector<float> jntValue, double duration = 0.005); // deprecated
    bool moveToTargetJntAngle(KDL::JntArray jntValue, double duration = 0.005, double timeout = 30.0, double tolerance = 0.1); 
    bool moveToTargetPos(KDL::Frame targetPos);


    //mobile base function
    bool getBaseID();
    void printBaseID();
    bool moveToTargetBasePos(double x, double y, double theta);
    bool getManipRootPos(KDL::Frame& rootPos);
    bool moveToTargetXPos(double xPos, double tolerance = 0.001);


    //auxiliary function
    std::vector<float> getNominalJntAngle();
    bool moveBaseForward(double vel = 1.0);
    bool moveBaseBackward(double vel = -1.0);
    bool stopBase();

    //pick and place function
    void pick();
    void place();
    bool getGripperID();
    void printGripperID();
    void openManipulator();
    void closeManipulator();


};

#endif







