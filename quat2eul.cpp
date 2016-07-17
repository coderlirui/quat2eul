// file: quat2eul.cpp, style: indent -kr -ci2 -cli2 -i2 -l80 -nut <file>
//
// License http://opensource.org/licenses/BSD-3-Clause
// Copyright (c) 2016, ItsmeJulian (github.com/ItsmeJulian)
// All rights reserved.
//
// Takes quaternion (imaginary or full) and converts it to Euler angles
//
// Source http://de.mathworks.com/matlabcentral/fileexchange/20696-function-to-convert-between-dcm--euler-angles--quaternions--and-euler-vectors
//        https://scholar.google.de/scholar?cluster=3204262265835591787
//        http://de.mathworks.com/help/robotics/ref/quat2eul.html
//
#include <string>
#include <sstream>
#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

using namespace std;

// user output
void printInfo()
{
  printf
    ("$ ./quat2eul <arg1-5> converts quaternion to euler-angle sequence.\n");
  printf(" arg1: choose rotation sequence (adapted from J. Fuller SpinCalc)\n");
  printf("       xyx, yzy, zxz, xzx, yxy, zyz (group 1) \n");
  printf("       xyz, yzx, zxy, xzy, yxz, zyx (group 2)\n");
  printf(" arg2: q1\n");
  printf(" arg3: q2\n");
  printf(" arg4: q3\n");
  printf("[arg5: q0 (real part)]\n");
  printf("example $ ./quat2eul zyx 0 0 0.7071\n");
  printf("convention: i-axis has psi, j-axis has theta, k-axis has phi\n");
  printf("       psi is always the first angle, then theta and lastly phi\n");
  printf
    ("       v_new = R(psi,theta,phi) * v_bef = R_k(phi) R_j(theta) R_i(psi) * v_bef\n");
  printf("       positive rotation is a right-handed helix\n");
  printf("if second euler-angle is close to singularity user gets notified\n");
}

//singularity check fails if closer than 1 degree, depends on angle group:
// 0----|<--1deg-->sing<--1deg-->|---------
//group 1: one axis repeats itself like xyx, acos --> only 180 deg
//group 2: all three axis involved like xyz, asin --> +90 or -90
//user gets notified to be cautious with the output
void singularitycheck(int group, double theta)
{
  if (group == 1 && (M_PI - theta < M_PI / 180 || theta < M_PI / 180)) {
    printf("singularity check failed: %f || %f < %f rad\n", M_PI - theta, theta,
           M_PI / 180);
  } else if (group == 2 && fabs(theta - M_PI / 2) < M_PI / 180) {
    printf("singularity check failed: %f < %f rad\n", fabs(theta - M_PI / 2),
           M_PI / 180);
  } else if (group != 1 && group != 2) {
    printf("meh...group not 1 or 2\n");
  }
}

// quaternation to euler, from John Fullers SpinCalc Matlab-Function
void quat2eul(string seq, double (&q)[4])
{
  double psi = 0;
  double theta = 0;
  double phi = 0;

  if (seq.compare("xyx") == 0) {
    psi = atan2((q[1] * q[2] + q[3] * q[0]), (q[2] * q[0] - q[1] * q[3]));
    theta = acos(q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
    phi = atan2((q[1] * q[2] - q[3] * q[0]), (q[1] * q[3] + q[2] * q[0]));
    singularitycheck(1, theta);
  } else if (seq.compare("yzy") == 0) {
    psi = atan2((q[1] * q[0] + q[2] * q[3]), (q[3] * q[0] - q[1] * q[2]));
    theta = acos(q[0] * q[0] - q[1] * q[1] + q[2] * q[2] - q[3] * q[3]);
    phi = atan2((q[2] * q[3] - q[1] * q[0]), (q[1] * q[2] + q[3] * q[0]));
    singularitycheck(1, theta);
  } else if (seq.compare("zxz") == 0) {
    psi = atan2((q[1] * q[3] + q[2] * q[0]), (q[1] * q[0] - q[2] * q[3]));
    theta = acos(q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
    phi = atan2((q[1] * q[3] - q[2] * q[0]), (q[1] * q[0] + q[2] * q[3]));
    singularitycheck(1, theta);
  } else if (seq.compare("xzx") == 0) {
    psi = atan2((q[1] * q[3] - q[2] * q[0]), (q[1] * q[2] + q[3] * q[0]));
    theta = acos(q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
    phi = atan2((q[1] * q[3] + q[2] * q[0]), (q[3] * q[0] - q[1] * q[2]));
    singularitycheck(1, theta);
  } else if (seq.compare("yxy") == 0) {
    psi = atan2((q[1] * q[2] - q[3] * q[0]), (q[1] * q[0] + q[2] * q[3]));
    theta = acos(q[0] * q[0] - q[1] * q[1] + q[2] * q[2] - q[3] * q[3]);
    phi = atan2((q[1] * q[2] + q[3] * q[0]), (q[1] * q[0] - q[2] * q[3]));
    singularitycheck(1, theta);
  } else if (seq.compare("zyz") == 0) {
    psi = atan2((q[2] * q[3] - q[1] * q[0]), (q[1] * q[3] + q[2] * q[0]));
    theta = acos(q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
    phi = atan2((q[1] * q[0] + q[2] * q[3]), (q[2] * q[0] - q[1] * q[3]));
    singularitycheck(1, theta);
  } else if (seq.compare("xyz") == 0) {
    psi =
      atan2(2 * (q[1] * q[0] - q[2] * q[3]),
            (q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]));
    theta = asin(2 * (q[1] * q[3] + q[2] * q[0]));
    phi =
      atan2(2 * (q[3] * q[0] - q[1] * q[2]),
            (q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]));
    singularitycheck(2, theta);
  } else if (seq.compare("yzx") == 0) {
    psi =
      atan2(2 * (q[2] * q[0] - q[1] * q[3]),
            (q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]));
    theta = asin(2 * (q[1] * q[2] + q[3] * q[0]));
    phi =
      atan2(2 * (q[1] * q[0] - q[3] * q[2]),
            (q[0] * q[0] - q[1] * q[1] + q[2] * q[2] - q[3] * q[3]));
    singularitycheck(2, theta);
  } else if (seq.compare("zxy") == 0) {
    psi =
      atan2(2 * (q[3] * q[0] - q[1] * q[2]),
            (q[0] * q[0] - q[1] * q[1] + q[2] * q[2] - q[3] * q[3]));
    theta = asin(2 * (q[1] * q[0] + q[2] * q[3]));
    phi =
      atan2(2 * (q[2] * q[0] - q[3] * q[1]),
            (q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]));
    singularitycheck(2, theta);
  } else if (seq.compare("xzy") == 0) {
    psi =
      atan2(2 * (q[1] * q[0] + q[2] * q[3]),
            (q[0] * q[0] - q[1] * q[1] + q[2] * q[2] - q[3] * q[3]));
    theta = asin(2 * (q[3] * q[0] - q[1] * q[2]));
    phi =
      atan2(2 * (q[1] * q[3] + q[2] * q[0]),
            (q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]));
    singularitycheck(2, theta);
  } else if (seq.compare("yxz") == 0) {
    psi =
      atan2(2 * (q[1] * q[3] + q[2] * q[0]),
            (q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]));
    theta = asin(2 * (q[1] * q[0] - q[2] * q[3]));
    phi =
      atan2(2 * (q[1] * q[2] + q[3] * q[0]),
            (q[0] * q[0] - q[1] * q[1] + q[2] * q[2] - q[3] * q[3]));
    singularitycheck(2, theta);
  } else if (seq.compare("zyx") == 0) {
    psi =
      atan2(2 * (q[1] * q[2] + q[3] * q[0]),
            (q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]));
    theta = asin(2 * (q[2] * q[0] - q[1] * q[3]));
    phi =
      atan2(2 * (q[1] * q[0] + q[3] * q[2]),
            (q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]));
    singularitycheck(2, theta);
    // matlab way:
    // psi = atan2( 2.*(q[1]*q[2] + q[0]*q[3]) , q[0]*q[0] + q[1]*q[1]- q[2]*q[2] - q[3]*q[3]);
    // theta = asin( 2.*( - q[1]*q[3] + q[0]*q[2]));
    // phi = atan2( 2.*(q[2]*q[3] + q[0]*q[1]) , q[0]*q[0] - q[1]*q[1]- q[2]*q[2] + q[3]*q[3]);
  } else {
    printf("meh...sequence not supported.\n");
    exit(1);
  }

  //save var. by pushing them back into the array
  q[1] = psi;
  q[2] = theta;
  q[3] = phi;
}

//rad to deg
void rad2deg(double *rads)
{
  rads[1] *= (180. / M_PI);
  rads[2] *= (180. / M_PI);
  rads[3] *= (180. / M_PI);
}

void upper2lowerchar(string & str)
{
  int i = 0;
  char c;
  while (str[i]) {
    c = str[i];
    if (isupper(c)) {
      str[i] = tolower(c);
    }
    i++;
  }
}

// handle I/O stream, functions compute euler-angles
int main(int argc, char **argv)
{
  if (argc < 5) {
    printInfo();
    exit(1);
  }
  // process input
  double q[4];                  //full quaternion
  string seq = "";              // sequence
  stringstream s1(argv[1]);
  stringstream s2(argv[2]);
  stringstream s3(argv[3]);
  stringstream s4(argv[4]);
  s1 >> seq;
  s2 >> q[1];
  s3 >> q[2];
  s4 >> q[3];
  upper2lowerchar(seq);         // user might use XYZ for xyz

  if (argc == 5) {              // we have imaginary part of unit-q
    q[0] = sqrt(1. - q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
    printf("conv. q = [q0, q1, q2, q3] \n");
    printf("your  q = [%f, %f, %f, %f] \n", q[0], q[1], q[2], q[3]);

  } else if (argc == 6) {       // we also have real part
    stringstream s5(argv[5]);
    s5 >> q[0];
    // normalize with euklidean-norm if ||q|| != 1
    double q_norm2 =
      sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (q_norm2 != 1) {
      for (int i = 0; i < 4; i++) {
        q[i] /= q_norm2;
      }
      printf("unit  q = [%f, %f, %f, %f] \n", q[0], q[1], q[2], q[3]);
    }
    printf("your  q = [%f, %f, %f, %f] \n", q[0], q[1], q[2], q[3]);
  }
  // unit-q is known, call functions to convert to euler-angles
  quat2eul(seq, q);

  // output the euler-angles
  printf("angles psi, theta, phi for %s are in \n", seq.c_str());
  printf("rad    %f, %f, %f \n", q[1], q[2], q[3]);
  rad2deg(q);
  printf("deg    %f, %f, %f \n", q[1], q[2], q[3]);
}

// EOF
