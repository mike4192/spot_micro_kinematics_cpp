#include <eigen3/Eigen/Geometry>

#include "spot_micro_kinematics/utils.h"
#include "spot_micro_kinematics/spot_micro_leg.h"


namespace smk {

SpotMicroLeg::SpotMicroLeg(float ang1, float ang2, float ang3,
                           float l1, float l2, float l3, 
                           Matrix4f ht_leg_start, bool is_leg_12) 
    : ang1_(ang1),
      ang2_(ang2),
      ang3_(ang3),
      l1_(l1),
      l2_(l2),
      l3_(l3),
      ht_leg_start_(ht_leg_start),
      is_leg_12_(is_leg_12) {








}




}
