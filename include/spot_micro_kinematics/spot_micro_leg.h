#pragma once

#include <eigen3/Eigen/Geometry>

//#include "spot_micro_kinematics/utils.h"


namespace smk {

using namespace Eigen;

class SpotMicroLeg 
{
 public:
  // Constructor, sets a leg up with initial joint angles and link lengths, and
  // a leg starting point homogeneous transform
  SpotMicroLeg(float ang1, float ang2, float ang3,
               float l1, float l2, float l3, 
               Matrix4f ht_leg_start, bool is_leg_12);
  
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
 
  // Sets the three leg angles and updates homogeneous transformation
  // matrices 
  void setAngles(float ang1, float ang2, float ang3);

  // Sets the homogenous transform for the leg start coordinate system
  void setHomogTransform(Matrix4f homog_transf);

  // Returns the homogeneous transform of the leg start coordinate system
  Matrix4f getHomogTransform();

  // Set the foot position in the leg's local coordinate system
  void setFootPosLocalCoordinates(float x4, float y4, float z4);

  // Set the foot position in global coordinate system
  void setFootPosGlobalCoordinates(float x4, float y4, float z4);

  // Returns the foot position in the global coordinate system
  std::tuple<float, float, float> getFootPosGlobalCoordinates();

  // Returns the three joint angles, ang1, ang2, ang3
  std::tuple<float, float, float> getLegJointAngles();


 private:
  float ang1_; // Angle of the hip joint, radians
  float ang2_; // Angle of the shoulder joint, radians
  float ang3_; // Angle of the knee joint, radians

  float l1_; // Length of hip link, meters
  float l2_; // Length of upper leg link, meters
  float l3_; // Length of lower leg link, meters

  Matrix4f ht_leg_start_; // Homogeneous transformation of leg coordinate system starting point

  bool is_leg_12_; // Boolean representing whether leg is 1 or 2, (as opposed to 3 or 4)

  // Homogeneous transformations for the portions of the leg
  Matrix4f ht_0_to_1_;
  Matrix4f ht_1_to_2_;
  Matrix4f ht_2_to_3_;
  Matrix4f ht_3_to_4_;


};




















}
