#pragma once // So header is only included once

//#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
//#include <math.h>  


using namespace Eigen;

namespace smk
{

// Returns a 4x4 Matrix that represents a homogeneous rotation matrix
// in the order x, y, z. Input angles are in units radians.
// A convenience wrapper around Eigen::Transform build up
Matrix4f homogRotXyz(float x_ang, float y_ang, float z_ang);

// Returns a 4x4 matrix that represents a homogeneous translation matrix.
// Input distances of x, y, z
// A convenience wrapper around Eigen::Transform
Matrix4f homogTransXyz(float x, float y, float z);


// Returns the inverse of the inputted homogeneous transform. Note that the
// inverse is not just the inverse of the matrix, but a reversed rotation, and a
// reversed linear translation. See definition for more details
Matrix4f homogInverse(Matrix4f ht);
}
