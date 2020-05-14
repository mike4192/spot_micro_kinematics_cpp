#pragma once

#include <eigen3/Eigen/Geometry>

#include "spot_micro_kinematics/utils.h"


namespace smk {

class SpotMicroLeg 
{
 public:
  // Constructor
  SpotMicroLeg();
  
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;


 private:
  float ang1_; // Angle of the hip joint, radians
  float ang2_; // Angle of the shoulder joint, radians
  float ang3_; // ANgle of the knee joint, radians

  float l1_; // Length of hip link, meters
  float l2_; // Length of upper leg link, meters
  float l3_; // Length of lower leg link, meters

  Matrix4f ht_leg_start_; // Homogeneous transformation of leg coordinate system starting point

  bool is_leg_12; // Boolean representing whether leg is 1 or 2, (as opposed to 3 or 4)

  Matrix4f ht_0_to_1_;
  Matrix4f ht_1_to_2_;
  Matrix4f ht_2_to_3_;
  Matrix4f ht_3_to_4_;






};




















}
