#include "spot_micro_kinematics/utils.h"

#include <cmath>

#include <eigen3/Eigen/Geometry>

using namespace Eigen;

namespace smk
{

Matrix4f homogRotXyz(float x_ang, float y_ang, float z_ang)
{
  // Create 3d transformation, and execute x, y, and z rotations
  Transform<float, 3, Affine> t = Transform<float,3,Affine>::Identity();
  t.rotate(AngleAxisf(x_ang, Vector3f::UnitX()));
  t.rotate(AngleAxisf(y_ang, Vector3f::UnitY()));
  t.rotate(AngleAxisf(z_ang, Vector3f::UnitZ()));

  return t.matrix();
}


Matrix4f homogTransXyz(float x, float y, float z)
{
  // Create a linear translation homogenous transformation matrix
  Transform<float, 3, Affine> t;
  t = Translation<float, 3> (Vector3f(x,y,z));

  return t.matrix();
}


Matrix4f homogInverse(const Matrix4f& ht)
{
//The inverse of a homogeneous transformation matrix can be represented as a
//    a matrix product of the following:
//
//                -------------------   ------------------- 
//                |           |  0  |   | 1   0   0  -x_t |
//    ht_inv   =  |   R^-1    |  0  | * | 0   1   0  -y_t |
//                |___________|  0  |   | 0   0   1  -z_t |
//                | 0   0   0 |  1  |   | 0   0   0   1   |
//                -------------------   -------------------
//
//    Where R^-1 is the inverse of the rotation matrix portion of the homogeneous
//    transform (the first three rows and columns). Note that the inverse
//    of a rotation matrix is equal to its transpose. And x_t, y_t, z_t are the
//    linear trasnformation portions of the original transform.  

  Matrix3f temp_rot = ht.block<3,3>(0,0); // Get rotation matrix portion from homogeneous transform via block
  temp_rot.transposeInPlace();    // Transpose, equivalent to inverse for rotation matrix

  // Store linear translation portion and negate directions
  Vector3f temp_translate = ht.block<3,1>(0,3);
  temp_translate = temp_translate * -1.0f;

  // Create left hand portion of ht_inv from comment block above
  Matrix4f ht_inverted1 = Matrix4f::Identity();
  ht_inverted1.block<3,3>(0,0) = temp_rot;

  // Create right hand portion of ht_in from comment block above
  Matrix4f ht_inverted2 = Matrix4f::Identity();
  ht_inverted2.block<3,1>(0,3) = temp_translate;

  // Return product of matrices, the homogeneous transform inverse
  return (ht_inverted1 * ht_inverted2);
}


Matrix4f htLegRightBack(const Matrix4f& ht_body_center, float body_length, float body_width) {

  // Build up matrix representing right back leg ht. First, a pi/2 rotation in y
  Matrix4f htLegRightBack = homogRotXyz(0.0f, M_PI/2.0f, 0.0);

  // Next, add the linear translation portion
  htLegRightBack.block<3,1>(0,3) = Vector3f(-body_length/2.0f, 0.0f, body_width/2.0f);

  // Return the matrix product of ht_body_center and the leg rightback ht
  return (ht_body_center * htLegRightBack);
}


Matrix4f htLegRightFront(const Matrix4f& ht_body_center, float body_length, float body_width) {

  // Build up matrix representing right front leg ht. First, a pi/2 rotation in y
  Matrix4f htLegRightBack = homogRotXyz(0.0f, M_PI/2.0f, 0.0);

  // Next, add the linear translation portion
  htLegRightBack.block<3,1>(0,3) = Vector3f(body_length/2.0f, 0.0f, body_width/2.0f);

  // Return the matrix product of ht_body_center and the leg ht
  return (ht_body_center * htLegRightBack);
}

Matrix4f htLegLeftFront(const Matrix4f& ht_body_center, float body_length, float body_width) {

  // Build up matrix representing right front leg ht. First, a pi/2 rotation in y
  Matrix4f htLegLeftFront = homogRotXyz(0.0f, -M_PI/2.0f, 0.0);

  // Next, add the linear translation portion
  htLegLeftFront.block<3,1>(0,3) = Vector3f(body_length/2.0f, 0.0f, -body_width/2.0f);

  // Return the matrix product of ht_body_center and the leg ht
  return (ht_body_center * htLegLeftFront);
}


Matrix4f htLegLeftBack(const Matrix4f& ht_body_center, float body_length, float body_width) {

  // Build up matrix representing right back leg ht. First, a pi/2 rotation in y
  Matrix4f htLegLeftBack = homogRotXyz(0.0f, -M_PI/2.0f, 0.0);

  // Next, add the linear translation portion
  htLegLeftBack.block<3,1>(0,3) = Vector3f(-body_length/2.0f, 0.0f, -body_width/2.0f);

  // Return the matrix product of ht_body_center and the leg ht
  return (ht_body_center * htLegLeftBack);
}


Matrix4f ht0To1(float rot_ang, float link_length) {
  
  // Build up the matrix as from the paper
  Matrix4f ht_0_to_1 = homogRotXyz(0.0f, 0.0f, rot_ang);

  // Add in remaining terms
  ht_0_to_1(0,3) = -link_length*cos(rot_ang);
  ht_0_to_1(1,3) = -link_length*sin(rot_ang);

  return ht_0_to_1;
}

Matrix4f ht1To2() {
  // Build up the matrix as from the paper
  Matrix4f ht_1_to_2;

  ht_1_to_2 <<
      0.0f,   0.0f,   -1.0f,   0.0f,
     -1.0f,   0.0f,    0.0f,   0.0f,
      0.0f,   1.0f,    0.0f,   0.0f,
      0.0f,   0.0f,    0.0f,   1.0f;

  return ht_1_to_2;
}


Matrix4f ht2To3(float rot_ang, float link_length) {
  
  // Build up the matrix as from the paper
  Matrix4f ht_2_to_3 = homogRotXyz(0.0f, 0.0f, rot_ang);

  // Add in remaining terms
  ht_2_to_3(0,3) = link_length*cos(rot_ang);
  ht_2_to_3(1,3) = link_length*sin(rot_ang);

  return ht_2_to_3;
}

Matrix4f ht3To4(float rot_ang, float link_length) {
  // Same as the 2 to 3 transformation, so just call that function
  
  return ht2To3(rot_ang, link_length);
}


Matrix4f ht0To4(float ang1, float ang2, float ang3, float link1, float link2, float link3) {
// Result is a sequential multiplication of all 4 transform matrices
return (ht0To1(ang1, link1) * ht1To2() * ht2To3(ang2, link2) * ht3To4(ang3, link3));

}

std::tuple<float, float, float> ikine(float x4, float y4, float z4,
                                      float link1, float link2, float link3,
                                      bool is_leg_12) {
using namespace std;

// Initialize return variables
float ang1, ang2, ang3;

// Supporting variable D
float D = (x4*x4 + y4*y4 + z4*z4 - link1*link1 - link2*link2 - link3*link3) /
          (2*link2*link3);

if (is_leg_12) {
  ang3 = atan2(sqrt(1 - D*D), D);
} else {
  ang3 = atan2(-sqrt(1 - D*D), D);
}

ang2 = atan2(z4, sqrt(x4*x4 + y4*y4 - link1*link1)) -
       atan2(link3*sin(ang3), link2 + link3*cos(ang3));

ang1 = atan2(y4, x4) + atan2(sqrt(x4*x4 + y4*y4 - link1*link1), -link1);

return make_tuple(ang1, ang2, ang3);
} 










}
