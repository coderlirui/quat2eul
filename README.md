### quat2eul
![cmd](printInfo_example.png)


 - Similar to Matlab but runs from the command line.
 - Supported rotation sequences XYZ and  ZYX 
   more Info: [quat2eul.cpp](quat2eul.cpp)
 - The math implementation is adopted from the technical report of [J Diebel](https://scholar.google.de/scholar?cluster=3204262265835591787). It delivers same results as [MatLab](http://www.mathworks.com/help/robotics/ref/quat2eul.html) or [WolframAlpha](https://www.wolframalpha.com/input/?i=quaternion:+0.7071%2B0i%2B0j%2B0.7071k).
 - order of the angles stays the same no matter the ijk-sequence, so psi is always the first angle to rotate with, then theta and lastly phi

	 - psi for i
     - theta for j
     - phi  for k

 Licensed under http://opensource.org/licenses/BSD-3-Clause

 
