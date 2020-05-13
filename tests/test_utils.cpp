#include <math.h>

#include <iostream>

#include <eigen3/Eigen/Geometry>
#include <gtest/gtest.h>

#include "spot_micro_kinematics/utils.h"

using namespace std;
using namespace Eigen;

TEST(homogeneousRotXyz, x_rotation)
{

  // Manually create a rotation matrix with a x rotation
  float ang = 10*M_PI/180.0;
  Matrix4f truth;
  truth << 1.0f,   0.0f,      0.0f,       0.0f,
           0.0f,   cos(ang),  -sin(ang),  0.0f,
           0.0f,   sin(ang),  cos(ang),   0.0f,
           0.00,   0.0f,      0.0f,       1.0f; 

  Matrix4f test_val = smk::homogRotXyz(ang,0.0f,0.0f);

  EXPECT_TRUE(truth.isApprox(test_val));

}



TEST(homogeneousRotXyz, y_rotation)
{

  // Manually create a rotation matrix with a y rotation
  float ang = 10*M_PI/180.0;
  Matrix4f truth;
  truth << cos(ang),   0.0f,     sin(ang),       0.0f,
           0.0f,       1.0f,         0.0f,       0.0f,
          -sin(ang),   0.0f,     cos(ang),       0.0f,
           0.0f,       0.0f,         0.0f,       1.0f; 

  Matrix4f test_val = smk::homogRotXyz(0.0f, ang, 0.0f);

  EXPECT_TRUE(truth.isApprox(test_val));

}


TEST(homogeneousRotXyz, z_rotation)
{

  // Manually create a rotation matrix with a z rotation
  float ang = 10*M_PI/180.0;
  Matrix4f truth;
  truth << cos(ang), -sin(ang),      0.0f,       0.0f,
           sin(ang),  cos(ang),      0.0f,       0.0f,
           0.0f,       0.0f,         1.0f,       0.0f,
           0.0f,       0.0f,         0.0f,       1.0f; 

  Matrix4f test_val = smk::homogRotXyz(0.0f, 0.0f, ang);

  EXPECT_TRUE(truth.isApprox(test_val));

}

TEST(homogeneousRotXyz, combo_rotation)
{

  // Manually create three rotation matrices
  float ang = 10*M_PI/180.0;

  Matrix4f truthx;
  truthx<< 1.0f,   0.0f,      0.0f,       0.0f,
           0.0f,   cos(ang),  -sin(ang),  0.0f,
           0.0f,   sin(ang),  cos(ang),   0.0f,
           0.00,   0.0f,      0.0f,       1.0f; 


  Matrix4f truthy;
  truthy<< cos(ang),   0.0f,     sin(ang),       0.0f,
           0.0f,       1.0f,         0.0f,       0.0f,
          -sin(ang),   0.0f,     cos(ang),       0.0f,
           0.0f,       0.0f,         0.0f,       1.0f; 

  Matrix4f truthz;
  truthz<< cos(ang), -sin(ang),      0.0f,       0.0f,
           sin(ang),  cos(ang),      0.0f,       0.0f,
           0.0f,       0.0f,         1.0f,       0.0f,
           0.0f,       0.0f,         0.0f,       1.0f; 

  Matrix4f truth = truthx * truthy * truthz;

  Matrix4f test_val = smk::homogRotXyz(ang, ang, ang);

  EXPECT_TRUE(truth.isApprox(test_val));

}


TEST(homogeneousTranslate, xyz_translate)
{

  // Manually create a homogeneous translation matrix
  Matrix4f truth;
  truth << 1.0f,       0.0f,         0.0f,       1.4f,
           0.0f,       1.0f,         0.0f,      -3.2f,
           0.0f,       0.0f,         1.0f,      10.8f,
           0.0f,       0.0f,         0.0f,       1.0f; 

  Matrix4f test_val = smk::homogTransXyz(1.4,-3.2,10.8);

  EXPECT_TRUE(truth.isApprox(test_val));

}


TEST(homogeneousInverse, ht_inverse)
{

  // Test by running a forward transformation on a set of coordinates, then 
  // computing the homogeneous transformation inverse, and multiplying on the
  // transformed coordinates, and verifying the result matches the beggining
  // coordinates
  
  Vector3f truth_point = Vector3f(0.23, 10.0, 2.56);

  Matrix4f ht_rot = smk::homogRotXyz(0.3, 0.12, -1.2);

  Matrix4f ht_trans = smk::homogTransXyz(20.1, -100.21, 2.3);

  auto ht = ht_rot * ht_trans;

  auto transformed_point = ht * truth_point.homogeneous();
 
  // Now calculate the inverse homogeneous transform matrix
  auto ht_inv = smk::homogInverse(ht);

  // Multiply transformed point by this matrix to get the original point back
  auto test_point = ht_inv * transformed_point;

  EXPECT_TRUE(truth_point.isApprox(test_point.block<3,1>(0,0)));

}


TEST(htLeg, rightback)
{
  // Create a body homogeneous transform, and verify the leg ht matrix comes out
  // as expected

  float len = 2.0f;
  float width = 4.0f;
  float height = 0.14f;

  Matrix4f body_ht = smk::homogRotXyz(0.0f, 0.0f, 0.0f) *
                     smk::homogTransXyz(0.0f, -height, 0.0f); 

  Matrix4f ht_rightback_test = smk::htLegRightBack(body_ht, len, width);

  Matrix4f ht_rightback_truth = Matrix4f::Identity();

  ht_rightback_truth <<
      cos(M_PI/2.0f),       0.0f,       sin(M_PI/2.0f),     -len/2.0f,
                0.0f,       1.0f,                 0.0f,       -height,
     -sin(M_PI/2.0f),       0.0f,       cos(M_PI/2.0f),    width/2.0f,
                0.0f,       0.0f,                 0.0f,          1.0f;

  EXPECT_TRUE(ht_rightback_truth.isApprox(ht_rightback_test));

}


TEST(htLeg, rightfront)
{
  // Create a body homogeneous transform, and verify the leg ht matrix comes out
  // as expected

  float len = 2.0f;
  float width = 4.0f;
  float height = 0.14f;

  Matrix4f body_ht = smk::homogRotXyz(0.0f, 0.0f, 0.0f) *
                     smk::homogTransXyz(0.0f, -height, 0.0f); 

  Matrix4f ht_rightfront_test = smk::htLegRightFront(body_ht, len, width);

  Matrix4f ht_rightfront_truth = Matrix4f::Identity();

  ht_rightfront_truth <<
      cos(M_PI/2.0f),       0.0f,       sin(M_PI/2.0f),      len/2.0f,
                0.0f,       1.0f,                 0.0f,       -height,
     -sin(M_PI/2.0f),       0.0f,       cos(M_PI/2.0f),    width/2.0f,
                0.0f,       0.0f,                 0.0f,          1.0f;

  EXPECT_TRUE(ht_rightfront_truth.isApprox(ht_rightfront_test));

}


TEST(htLeg, leftfront)
{
  // Create a body homogeneous transform, and verify the leg ht matrix comes out
  // as expected

  float len = 2.0f;
  float width = 4.0f;
  float height = 0.14f;

  Matrix4f body_ht = smk::homogRotXyz(0.0f, 0.0f, 0.0f) *
                     smk::homogTransXyz(0.0f, -height, 0.0f); 

  Matrix4f ht_leftfront_test = smk::htLegLeftFront(body_ht, len, width);

  Matrix4f ht_leftfront_truth = Matrix4f::Identity();

  ht_leftfront_truth <<
      cos(-M_PI/2.0f),       0.0f,       sin(-M_PI/2.0f),      len/2.0f,
                 0.0f,       1.0f,                  0.0f,       -height,
     -sin(-M_PI/2.0f),       0.0f,       cos(-M_PI/2.0f),   -width/2.0f,
                 0.0f,       0.0f,                  0.0f,          1.0f;

  EXPECT_TRUE(ht_leftfront_truth.isApprox(ht_leftfront_test));

}

TEST(htLeg, leftback)
{
  // Create a body homogeneous transform, and verify the leg ht matrix comes out
  // as expected

  float len = 2.0f;
  float width = 4.0f;
  float height = 0.14f;

  Matrix4f body_ht = smk::homogRotXyz(0.0f, 0.0f, 0.0f) *
                     smk::homogTransXyz(0.0f, -height, 0.0f); 

  Matrix4f ht_leftback_test = smk::htLegLeftBack(body_ht, len, width);

  Matrix4f ht_leftback_truth = Matrix4f::Identity();

  ht_leftback_truth <<
      cos(-M_PI/2.0f),       0.0f,       sin(-M_PI/2.0f),     -len/2.0f,
                 0.0f,       1.0f,                  0.0f,       -height,
     -sin(-M_PI/2.0f),       0.0f,       cos(-M_PI/2.0f),   -width/2.0f,
                 0.0f,       0.0f,                  0.0f,          1.0f;

  EXPECT_TRUE(ht_leftback_truth.isApprox(ht_leftback_test));

}


TEST(htLegLinkTransforms, ht0To1)
{
  
  // Test the homogenous transform from link 0 to 1 via a contrived example
  
  float rot_ang = 35.0f;
  float link_length = 0.4;

  Matrix4f ht_0_to_1_test = smk::ht0To1(rot_ang, link_length);

  Matrix4f ht_0_to_1_truth = Matrix4f::Identity();

  <ht_0_to_1_truth
      cos(-M_PI/2.0f),       0.0f,       sin(-M_PI/2.0f),     -len/2.0f,
                 0.0f,       1.0f,                  0.0f,       -height,
     -sin(-M_PI/2.0f),       0.0f,       cos(-M_PI/2.0f),   -width/2.0f,
                 0.0f,       0.0f,                  0.0f,          1.0f;

  EXPECT_TRUE(ht_leftback_truth.isApprox(ht_leftback_test));

}
