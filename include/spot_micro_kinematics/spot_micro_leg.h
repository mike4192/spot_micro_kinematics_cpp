#pragma once

#include <eigen3/Eigen/Geometry>

#include "spot_micro_kinematics/utils.h"


namespace smk {

class SpotMicroLeg {

 public:
  // Constructor, sets a leg up with initial joint angles and link lengths, and
  // a leg starting point homogeneous transform
  SpotMicroLeg(const JointAngles& joint_angles,
               const LinkLengths& link_lengths,
               const Eigen::Matrix4f& ht_leg_start, bool is_leg_12);
  
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
 
  // Sets the three leg angles and updates homogeneous transformation
  // matrices 
  void setAngles(const JointAngles& joint_angles);

  // Sets the homogenous transform for the leg start coordinate system
  void setHomogTransform(const Eigen::Matrix4f& ht_leg_start);

  // Returns the homogeneous transform of the leg start coordinate system
  Eigen::Matrix4f getHomogTransform();

  // Set the foot position in the leg's local coordinate system
  void setFootPosLocalCoordinates(const Point& point);

  // Set the foot position in global coordinate system
  void setFootPosGlobalCoordinates(const Point& point);

  // Returns the foot position in the global coordinate system
  Point getFootPosGlobalCoordinates();

  // Returns the three joint angles, ang1, ang2, ang3
  JointAngles getLegJointAngles();


 private:
  JointAngles joint_angles_; // Joint angles of the leg

  LinkLengths link_lengths_; // Lengths of the leg links

  Eigen::Matrix4f ht_leg_start_; // Homogeneous transformation of leg coordinate system starting point

  bool is_leg_12_; // Boolean representing whether leg is 1 or 2, (as opposed to 3 or 4)

  // Homogeneous transformations for the portions of the leg
  Eigen::Matrix4f ht_0_to_1_;
  Eigen::Matrix4f ht_1_to_2_;
  Eigen::Matrix4f ht_2_to_3_;
  Eigen::Matrix4f ht_3_to_4_;

};


}
