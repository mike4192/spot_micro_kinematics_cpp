#include "spot_micro_kinematics/utils.h"
//#include <eigen3/Eigen/Core>
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


Matrix4f homogInverse(Matrix4f ht)
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

  Vector3f temp_translate = ht.block<3,1>(0,3);
  temp_translate = temp_translate * -1.0f;

  Matrix4f ht_inverted1 = Matrix4f::Identity();
  ht_inverted1.block<3,3>(0,0) = temp_rot;

  Matrix4f ht_inverted2 = Matrix4f::Identity();
  ht_inverted2.block<3,1>(0,3) = temp_translate;

  Matrix4f ht_inverted = ht_inverted1 * ht_inverted2;

  return ht_inverted;
}


}
