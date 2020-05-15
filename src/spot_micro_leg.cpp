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








}
