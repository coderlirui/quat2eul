// file: quat2eul.cpp, style: indent -kr -ci2 -cli2 -i2 -l80 -nut <file>
//
// License http://opensource.org/licenses/BSD-3-Clause
// Copyright (c) 2016, ItsmeJulian (github.com/ItsmeJulian)
// All rights reserved.
//
// Takes quaternion (imaginary or full) and converts it to Euler angles
// supported sequence: xyz, zyx
//
// Date          Author      	 		Notes
// 12/07/2016    Julian     				Initial release
//
// Source https://scholar.google.de/scholar?cluster=3204262265835591787
//        http://de.mathworks.com/help/robotics/ref/quat2eul.html
//
#include <string>
#include <sstream>
#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

using namespace std;

// quaternation to euler in ZYX (seq:321)
double* quat2eulerzyx(double* q){

  // euler-angles
  double psi = atan2( 2.*(q[1]*q[2] + q[0]*q[3]) , q[0]*q[0] + q[1]*q[1]- q[2]*q[2] - q[3]*q[3]);
  double theta = asin( -2.*(q[1]*q[3] - q[0]*q[2]));
  double phi = atan2( 2.*(q[2]*q[3] + q[0]*q[1]) , q[0]*q[0] - q[1]*q[1]- q[2]*q[2] + q[3]*q[3]);

  // conventional: R(psi,theta,phi) = R_k(phi) R_j(theta) R_i(psi),
  // so psi is always the first angle to rotate with, then theta and last phi
  q[1] = psi;
  q[2] = theta;
  q[3] = phi;//
  return q;
}

// quaternation to euler in XYZ (seq:123)
double* quat2eulerxyz(double* q){

  //euler-angles
  double psi = atan2( -2.*(q[2]*q[3] - q[0]*q[1]) , q[0]*q[0] - q[1]*q[1]- q[2]*q[2] + q[3]*q[3]);
  double theta = asin( 2.*(q[1]*q[3] + q[0]*q[2]));
  double phi = atan2( 2.*(-q[1]*q[2] + q[0]*q[3]) , q[0]*q[0] + q[1]*q[1] - q[2]*q[2] - q[3]*q[3]);

  //save var. by simply pushing them back into the array and return
  q[1] = psi;
  q[2] = theta;
  q[3] = phi;
  return q;
}

//rad to deg
void rad2deg(double* rads) {
  rads[1] *= (180./M_PI);
  rads[2] *= (180./M_PI);
  rads[3] *= (180./M_PI);
}

// user output
void printInfo(){
  printf("arg1: 'img' imaginary parts of a unit-quaternion\n");
  printf("            and converts it to Euler angles\n");
  printf("      'ind' constructs a unit-quaternion from input angles in rad\n");
  printf("      'largeq'   specify a full quaternion q = [q0, q1, q2, q3] any length\n");
  printf("arg2: zyx or xyz rotation sequence\n");
  printf("arg3: q1\n");
  printf("arg4: q2\n");
  printf("arg5: q3\n");
  printf("[arg6: q0 (largeq)]\n");
  printf("  e.g.: ./quat2eul ind zyx 1 0 0\n");
  printf("convention: i-axis having psi, j-axis having theta, k-axis having phi\n");
  printf("psi is always the first angle to rotate with, then theta and at last phi\n");
  printf("R(psi,theta,phi) = R_k(phi) R_j(theta) R_i(psi)\n");
}

// handle I/O stream, functions compute euler-angles
int main(int argc, char **argv)
{
  if(argc <6){
    printInfo();
    exit(1);
  }

  double q [4]; //full quaternion
  string seq = "";// sequence
  string mode = "";
  stringstream s1(argv[1]);
  stringstream s2(argv[2]);
  stringstream s3(argv[3]);
  stringstream s4(argv[4]);
  stringstream s5(argv[5]);
  s1 >> mode;
  s2 >> seq;
  s3 >> q[1];
  s4 >> q[2];
  s5 >> q[3];

  if(mode.compare("img")){// we have imaginary part of unit-q
    q[0] = sqrt(1. - q[1]*q[1] - q[2]*q[2] - q[3]*q[3]);
    printf("your q = [%f, %f, %f, %f]", q[0], q[1], q[2], q[3]);

  } else if(mode.compare("ind")){// we have rad angles and want the q
    q[1] = sin(q[1]/2.);
    q[2] = sin(q[2]/2.);
    q[3] = sin(q[3]/2.);
    q[0] = sqrt(1. - q[1]*q[1] - q[2]*q[2] - q[3]*q[3]);
    printf("your q = [%f, %f, %f, %f]", q[0], q[1], q[2], q[3]);

  } else if (argc == 7){// we also have real part
    stringstream s6(argv[6]);
    s6 >> q[0];

    // normalize with euklidean-norm if ||q|| != 1
    double q_norm2 = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if(q_norm2 != 1) {
      for(int i = 0; i<4; i++) {
        q[i] /= q_norm2;
      }
      printf("unit q = [%f, %f, %f, %f]", q[0], q[1], q[2], q[3]);
    }
    printf("your q = [%f, %f, %f, %f]", q[0], q[1], q[2], q[3]);
  } else {
    cout<<"input failed"<<endl;
    exit(1);
  }

  // unit-q is known, call functions to convert to euler-angles
  if (seq.compare("xyz")) {
    quat2eulerxyz(q);
  } else if (seq.compare("xyz")) {
    quat2eulerzyx(q);
  }

  // output the euler-angles
  printf("The euler angles psi, theta, phi for sequence %s are: ", seq.c_str());
  printf("rad %f, %f, %f", q[1], q[2], q[3]);
  rad2deg(q);
  printf("deg %f, %f, %f", q[1], q[2], q[3]);
}
// EOF
