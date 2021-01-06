#ifndef ASSIGNMENT_1_HPP
#define ASSIGNMENT_1_HPP

#include <tf/transform_datatypes.h>

tf::Point ForwardKinematics(double q1, double q2, double q3);
void Jacobian(double q1, double q2, double q3, double J[3][3]);

#endif /* ASSIGNMENT_1_HPP */
