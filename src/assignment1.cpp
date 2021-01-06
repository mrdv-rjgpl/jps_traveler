/**
 *
 * Cartesian trajectory generation
 *
 */
#include "assignment1.hpp"

// Compute the forward kinematics (only position)
tf::Point ForwardKinematics(double q1, double q2, double q3)
{
  // TODO
  // Fill the values of the forward kinematics
  double x, y, z;
  x = sin(q1) * 1.9145E-1
    - cos(q1) * sin(q2 + 3.141592653589793 * (1.0 / 2.0)) * (1.7E1 / 4.0E1)
    - cos(q1) * cos(q3) * sin(q2 + 3.141592653589793 * (1.0 / 2.0)) * 4.869E-1
    - cos(q1) * cos(q2 + 3.141592653589793 * (1.0 / 2.0)) * sin(q3) * 4.869E-1;

  y = cos(q1) * (-1.9145E-1)
    - sin(q1) * sin(q2 + 3.141592653589793 * (1.0 / 2.0)) * (1.7E1 / 4.0E1)
    - cos(q3) * sin(q1) * sin(q2 + 3.141592653589793 * (1.0 / 2.0)) * 4.869E-1
    - cos(q2 + 3.141592653589793 * (1.0 / 2.0)) * sin(q1) * sin(q3) * 4.869E-1;

  z = cos(q2 + 3.141592653589793 * (1.0 / 2.0)) * (1.7E1 / 4.0E1)
    + cos(q3) * cos(q2 + 3.141592653589793 * (1.0 / 2.0)) * 4.869E-1
    - sin(q3) * sin(q2 + 3.141592653589793 * (1.0 / 2.0)) * 4.869E-1
    + 8.9159E-2;

  return tf::Point(x, y, z);
}

// Compute and return the Jacobian of the robot given the current joint
// positions
// input: the input joint state
// output: the 3x3 Jacobian (position only)
void Jacobian(double q1, double q2, double q3, double J[3][3])
{
  for(int r = 0; r < 3; r++)
    for(int c = 0; c < 3; c++)
      J[r][c] = 0.0;

  // TODO
  // Fill the values of the Jacobian matrix J
  J[0][0] = cos(q1) * 1.9145E-1
    + sin(q1) * sin(q2 + 3.141592653589793 * (1.0 / 2.0)) * (1.7E1 / 4.0E1)
    + cos(q3) * sin(q1) * sin(q2 + 3.141592653589793 * (1.0 / 2.0)) * 4.869E-1
    + cos(q2 + 3.141592653589793 * (1.0 / 2.0)) * sin(q1) * sin(q3) * 4.869E-1;

  J[0][1] =
    cos(q1) * cos(q2 + 3.141592653589793 * (1.0 / 2.0)) * (-1.7E1 / 4.0E1)
    + cos(q1) * sin(q3) * sin(q2 + 3.141592653589793 * (1.0 / 2.0)) * 4.869E-1
    - cos(q1) * cos(q3) * cos(q2 + 3.141592653589793 * (1.0 / 2.0)) * 4.869E-1;

  J[0][2] =
    cos(q1) * sin(q3) * sin(q2 + 3.141592653589793 * (1.0 / 2.0)) * 4.869E-1
    - cos(q1) * cos(q3) * cos(q2 + 3.141592653589793 * (1.0 / 2.0)) * 4.869E-1;

  J[1][0] = sin(q1) * 1.9145E-1
    - cos(q1) * sin(q2 + 3.141592653589793 * (1.0 / 2.0)) * (1.7E1 / 4.0E1)
    - cos(q1) * cos(q3) * sin(q2 + 3.141592653589793 * (1.0 / 2.0)) * 4.869E-1
    - cos(q1) * cos(q2 + 3.141592653589793 * (1.0 / 2.0)) * sin(q3) * 4.869E-1;

  J[1][1] =
    cos(q2 + 3.141592653589793 * (1.0 / 2.0)) * sin(q1) * (-1.7E1 / 4.0E1)
    + sin(q1) * sin(q3) * sin(q2 + 3.141592653589793 * (1.0 / 2.0)) * 4.869E-1
    - cos(q3) * cos(q2 + 3.141592653589793 * (1.0 / 2.0)) * sin(q1) * 4.869E-1;

  J[1][2] =
    sin(q1) * sin(q3) * sin(q2 + 3.141592653589793 * (1.0 / 2.0)) * 4.869E-1
    - cos(q3) * cos(q2 + 3.141592653589793 * (1.0 / 2.0)) * sin(q1) * 4.869E-1;

  J[2][1] = sin(q2 + 3.141592653589793 * (1.0 / 2.0)) * (-1.7E1 / 4.0E1)
    - cos(q3) * sin(q2 + 3.141592653589793 * (1.0 / 2.0)) * 4.869E-1
    - cos(q2 + 3.141592653589793 * (1.0 / 2.0)) * sin(q3) * 4.869E-1;

  J[2][2] = cos(q3) * sin(q2 + 3.141592653589793 * (1.0 / 2.0)) * (-4.869E-1)
    - cos(q2 + 3.141592653589793 * (1.0 / 2.0)) * sin(q3) * 4.869E-1;
}

