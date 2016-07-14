// file: quat2eul.cpp, style: indent -kr -ci2 -cli2 -i2 -l80 -nut <file>
//
// Constructs a unit-quaternion from input angles (arg1:ind) and convert to Euler angles,
//
// explanation see printInfo()
//
// or takes the input directly (arg1:d) as unit-quaterion and converts it to Euler-angles,
// or takes the input as a full quaternion (arg1:4) of any length, normalizes it and converts it to Euler-angles;
//
// Licensed under  http://opensource.org/licenses/BSD-3-Clause
//
// Date          Author      	 		Notes
// 12/07/2016    Julian     				Initial release
//
// Source https://scholar.google.de/scholar?cluster=3204262265835591787
// Source http://web.mit.edu/2.998/www/QuaternionReport1.pdf
//
// License http://opensource.org/licenses/BSD-3-Clause
// Copyright (c) 2016, ItsmeJulian (github.com/ItsmeJulian)
// All rights reserved.
//
#include <string>
#include <sstream>
#include <iostream>
#include <math.h>
#include <stdlib.h>

using namespace std;

// quaternation to euler in ZYX (seq:321)
double* quat2eulerzyx(double* q){
	// euler-angles
    double psi = atan2( 2.*(q[1]*q[2] + q[0]*q[3]) , q[0]*q[0] + q[1]*q[1]- q[2]*q[2] - q[3]*q[3]);
    double theta = asin( -2.*(q[1]*q[3] - q[0]*q[2]));
    double phi = atan2( 2.*(q[2]*q[3] + q[0]*q[1]) , q[0]*q[0] - q[1]*q[1]- q[2]*q[2] + q[3]*q[3]);
    // conventional: R(psi,theta,phi) = R_k(phi) R_j(theta) R_i(psi),
        so psi is always the first angle to rotate with, then theta and last phi
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

// user output
void printInfo(){
	printf("arg1: 'img' takes the input directly as the imaginary parts of a UNIT-quaternion\n");
	printf("            and converts it to Euler angles\n");
	printf("      'ind' constructs a unit-quaternion from input angles [sin(arg[3to5]/2.)] in rad\n");
	printf("      'largeq'   specify a full quaternion q = [q0, q1, q2, q3] having any length\n");
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

	//  first to are unit-q, so get real part; in case of '4' normalize if neccessary
	if(!mode.compare("ind")){// ind mode
		q[0] = sqrt(1. - q[1]*q[1] - q[2]*q[2] - q[3]*q[3]);
		cout<<"your q = ["<< q[0]<< ", "<< q[1] << ", "<<q[2] <<", "<<q[3]<<"]" <<endl;
	} else if(!mode.compare("ind")){
	    q[1] = sin(q[1]/2.);
	    q[2] = sin(q[2]/2.);
	    q[3] = sin(q[3]/2.);
		q[0] = sqrt(1. - q[1]*q[1] - q[2]*q[2] - q[3]*q[3]);
		cout<<"your q = ["<< q[0]<< ", "<< q[1] << ", "<<q[2] <<", "<<q[3]<<"]" <<endl;
	} else {
		// assume user chose arg1: 4
    	if(argc == 7) {
			stringstream s6(argv[6]);
			s6 >> q[0];
			// normalize with euklidean-norm if ||q|| != 1
			double q_n2 = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
			if(q_n2 != 1) {
				for(int i = 0; i<4; i++) q[i] /= q_n2;
				cout<<"unit q = ["<< q[0]<< ", "<< q[1] << ", "<<q[2] <<", "<<q[3]<<"]"<<endl;
			}
			cout<<"your q = ["<< q[0]<< ", "<< q[1] << ", "<<q[2] <<", "<<q[3]<<"]" <<endl;
		} else {
			cout<<"arg6 missing"<<endl;
			exit(1);
		}
	}
	// unit-q is known, call euler functions
    if (!seq.compare("xyz")) quat2eulerxyz(q); else quat2eulerzyx(q);

	// output the euler-angles
    cout<<seq<<": "<< "eul[psi,theta,phi] = ["<< q[1] << " "<<q[2] <<" "<<q[3] <<"] (rad)"<<endl;
    q[1] *= (180./M_PI);	//rad to deg
    q[2] *= (180./M_PI);
    q[3] *= (180./M_PI);
    cout<<seq<<": "<< "eul[psi,theta,phi] = ["<< q[1] << " "<<q[2] <<" "<<q[3] <<"] (deg)"<<endl;
}
//%%% EOF %%%
