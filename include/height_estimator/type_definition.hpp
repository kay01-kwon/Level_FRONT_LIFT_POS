#ifndef TYPE_DEFINITION_HPP
#define TPYE_DEFINITION_HPP

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Quaternion<double> quat;
typedef Eigen::Matrix<double,3,1> mat31;
typedef Eigen::Matrix<double,3,3> mat33;

typedef Eigen::Matrix<double,4,4> mat44;
typedef Eigen::Matrix<double,4,3> mat43;
typedef Eigen::Matrix<double,4,1> mat41;
typedef Eigen::Matrix<double,1,4> mat14;

using std::cout;
using std::endl;

#endif