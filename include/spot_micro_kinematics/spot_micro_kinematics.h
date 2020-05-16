#pragma once

#include <eigen3/Eigen/Geometry>

#include "spot_micro_kinematics/utils.h"
#include "spot_micro_kinematics/spot_micro_leg.h"


namespace smk {

// Struct to hold joint angles for four legs
struct LegsJointAngles {
  JointAngles right_back;
  JointAngles right_front;
  JointAngles left_front;
  JointAngles left_back;
};

struct LegsFootPos {
  Point right_back;
  Point right_front;
  Point left_front;
  Point left_back;
};

// Struct to hold various configuration values of a spot micro robot frame
struct SpotMicroConfig {
  float hip_link_length;
  float upper_leg_link_length;
  float lower_leg_link_length;
};

class SpotMicroKinematics {

 public:
  // Constructor, sets up a spot micro kinematics object
  SpotMicroKinematics(float x, float y, float z, const SpotMicroConfig& smc);

  // Sets the joint angles for the legs in the robot
  void setLegJointAngles(LegsJointAngles four_legs_joint_angs);

  // Sets the foot for each leg to the commanded position in a global coordinate
  // system
  void setFeetPosGlobalCoordinates(LegsFootPos four_legs_foot_pos);

  // Sets an absolute body pose for the robot while holding the feet stationary
  void setBodyPose(const Eigen::Matrix4f& ht_body);

  // Sets a body rotation without translating the body or moving the feet
  void setBodyAngles(float phi, float theta, float psi);

  // Returns the joint angles of the four legs
  LegsJointAngles getLegJointAngles();


  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
 private:

  SpotMicroConfig smc_; // Spot micro config struct
 
  // x, y, z position of body center in global coordinate system 
  float x_;
  float y_;
  float z_;
 
  // Euler angles of body in global coordinate system
  float phi_;
  float theta_;
  float psi_;
 
  // Leg objects of the robot 
  SpotMicroLeg right_back_leg;
  SpotMicroLeg right_front_leg;
  SpotMicroLeg left_front_leg;
  SpotMicroLeg left_back_leg;

};




}








