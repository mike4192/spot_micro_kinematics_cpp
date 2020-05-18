#include <eigen3/Eigen/Geometry>

#include "spot_micro_kinematics/utils.h"
#include "spot_micro_kinematics/spot_micro_leg.h"
#include "spot_micro_kinematics/spot_micro_kinematics.h"

using namespace Eigen;

namespace smk {

SpotMicroKinematics::SpotMicroKinematics(float x, float y, float z,
                                         const SpotMicroConfig& smc) 
    : x_(x),
      y_(y),
      z_(z),
      smc_(smc) {

  // Initialize other class attributes
  phi_ = 0.0f;
  theta_ = 0.0f;
  psi_ = 0.0f;
  
  // Create temporary structs for initializing leg's joint angles and lengths  
  JointAngles joint_angles_temp = {0.0f, 0.0f, 0.0f};
  LinkLengths link_lengths_temp = {smc.hip_link_length,
                                   smc.upper_leg_link_length,
                                   smc.lower_leg_link_length}; 

  // Create legs
  right_back_leg_  = SpotMicroLeg(joint_angles_temp, link_lengths_temp, true);
  right_front_leg_ = SpotMicroLeg(joint_angles_temp, link_lengths_temp, true);
  left_front_leg_  = SpotMicroLeg(joint_angles_temp, link_lengths_temp, false);
  left_back_leg_   = SpotMicroLeg(joint_angles_temp, link_lengths_temp, false);
}


void SpotMicroKinematics::setLegJointAngles(
    const LegsJointAngles& four_legs_joint_angs) {
  // Call each leg's method to set joint angles 
  right_back_leg_.setAngles(four_legs_joint_angs.right_back);
  right_front_leg_.setAngles(four_legs_joint_angs.right_front);
  left_front_leg_.setAngles(four_legs_joint_angs.left_front);
  left_back_leg_.setAngles(four_legs_joint_angs.left_back);
}


void SpotMicroKinematics::setFeetPosGlobalCoordinates(
    const LegsFootPos& four_legs_foot_pos) {
  // Create matrices to represent ht of the body center
  Matrix4f ht_body = homogTransXyz(x_, y_, z_) * homogRotXyz(phi_, theta_, psi_);

  // Create each leg's starting ht matrix. Made in order of right back, right 
  // front, left front, left back
  Matrix4f ht_rb = smk::htLegRightBack(ht_body, smc_.body_length, smc_.body_width);
  Matrix4f ht_rf = smk::htLegRightFront(ht_body, smc_.body_length, smc_.body_width);
  Matrix4f ht_lf = smk::htLegLeftFront(ht_body, smc_.body_length, smc_.body_width);
  Matrix4f ht_lb = smk::htLegLeftBack(ht_body, smc_.body_length, smc_.body_width);


  // Call each leg's method to set foot position in global coordinates
  right_back_leg_.setFootPosGlobalCoordinates(four_legs_foot_pos.right_back,
                                              ht_rb);

  right_back_leg_.setFootPosGlobalCoordinates(four_legs_foot_pos.right_front,
                                              ht_rf);

  right_back_leg_.setFootPosGlobalCoordinates(four_legs_foot_pos.left_front,
                                              ht_lf);

  right_back_leg_.setFootPosGlobalCoordinates(four_legs_foot_pos.left_back,
                                              ht_lb);
}





}
