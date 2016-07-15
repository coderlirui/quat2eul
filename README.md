#### quat2eul
Convert a quaternion to euler angles.

 - Similar to Matlab but runs from the command line.
 - Supported rotation sequences XYZ and  ZYX 
   more Info: [quat2eul.cpp](quat2eul.cpp)
 - The math implementation is adopted from the technical report of [J Diebel](https://scholar.google.de/scholar?cluster=3204262265835591787)
 - Convention: no matter the ijk-sequence the order of the angles stays the same, so psi is always the first angle to rotate with, then theta and then phi

	 - psi for i
     - theta for j
     - phi  for k

 >ijk-sequence: R(psi,theta,phi) = R_k(phi) R_j(theta) R_i(psi)

 Licensed under http://opensource.org/licenses/BSD-3-Clause

 
