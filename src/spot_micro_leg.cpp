#include <eigen3/Eigen/Geometry>

#include "spot_micro_kinematics/utils.h"
#include "spot_micro_kinematics/spot_micro_leg.h"

using namespace Eigen;

namespace smk {


// Constructor
SpotMicroLeg::SpotMicroLeg(const JointAngles& joint_angles,
                           const LinkLengths& link_lengths,
                           const Eigen::Matrix4f& ht_leg_start, bool is_leg_12)
    : joint_angles_(joint_angles),
      link_lengths_(link_lengths),
      ht_leg_start_(ht_leg_start),
      is_leg_12_(is_leg_12) {

  // Create homogeneous trasnformation matrices for the leg links
  ht_0_to_1_ = ht0To1(joint_angles_.ang1,link_lengths_.l1);
  ht_1_to_2_ = ht1To2();
  ht_2_to_3_ = ht2To3(joint_angles_.ang2, link_lengths_.l2);
  ht_3_to_4_ = ht3To4(joint_angles_.ang3, link_lengths_.l3);
}


void SpotMicroLeg::setAngles(const JointAngles& joint_angles) {
  // Update object's joint angles, and update object's homogeneous
  // transformation matrices
  joint_angles_ = joint_angles;

  ht_0_to_1_ = ht0To1(joint_angles_.ang1,link_lengths_.l1);
  ht_1_to_2_ = ht1To2();
  ht_2_to_3_ = ht2To3(joint_angles_.ang2, link_lengths_.l2);
  ht_3_to_4_ = ht3To4(joint_angles_.ang3, link_lengths_.l3);
}


void SpotMicroLeg::setHomogTransform(const Matrix4f& ht_leg_start) {
  // Update object's homogeneous transform
  ht_leg_start_ = ht_leg_start;
}


Matrix4f SpotMicroLeg::getHomogTransform() {
  return ht_leg_start_;
}


void SpotMicroLeg::setFootPosLocalCoordinates(const Point& point) {

  // Run inverse kinematics to find joint angles
  JointAngles joint_angles = ikine(point, link_lengths_, is_leg_12_);

  // Call method to set joint angles of the leg
  setAngles(joint_angles);
}


void SpotMicroLeg::setFootPosGlobalCoordinates(const Point& point) {

  // Need to express the point in the leg's coordinate system, can do so by
  // transforming a vector of the points in global coordinate by the inverse of
  // the leg's starting homogeneous transform

  // Make a homogeneous vector, and store the point in global coords in it
  Eigen::Vector4f p4_ht_vec(point.x, point.y, point.z, 1.0f);

  // Multiply it by the inverse of the homgeneous transform of the leg start.
  // This operation yields a foot position in the foot's local coordinates
  p4_ht_vec = homogInverse(ht_leg_start_) * p4_ht_vec; 

  Point point_local{.x = p4_ht_vec(0), .y = p4_ht_vec(1), .z = p4_ht_vec(2)};

  // Call this leg's method for setting foot position in local cordinates
  setFootPosLocalCoordinates(point_local);
}


Point SpotMicroLeg::getFootPosGlobalCoordinates() {
 // Get homogeneous transform of foot
  Matrix4f ht_foot = ht_leg_start_ *
                     ht_0_to_1_ * 
                     ht_1_to_2_ *
                     ht_2_to_3_ * 
                     ht_3_to_4_;  

  Point return_point = {.x = ht_foot(0,3),
                        .y = ht_foot(1,3),
                        .z = ht_foot(2,3) };

  return return_point;
}


JointAngles SpotMicroLeg::getLegJointAngles() {
  return joint_angles_;
}

}
