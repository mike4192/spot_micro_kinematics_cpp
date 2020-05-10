#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <math.h>


using namespace Eigen;

void test_func()
{
  std::cout << "Hello from smk test_func" << std::endl;

  Transform<float, 3, Affine> t = Transform<float,3,Affine>::Identity();

  std::cout << t.matrix() << std::endl;

  t.translate(Vector3f( 1.5, 10.1, 20.0));


  std::cout << t.matrix() << std::endl;

  t.rotate(AngleAxisf((45.0*M_PI/180.0), Vector3f::UnitX()));

  std::cout << t.matrix() << std::endl;
}
