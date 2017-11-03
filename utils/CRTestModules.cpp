//=====================================================================
/*
Software License Agreement (BSD-3-Clause License)
Copyright (c) 2017, CoreRobotics.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:

* Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.

* Neither the name of CoreRobotics nor the names of its contributors
may be used to endorse or promote products derived from this
software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

\project CoreRobotics Project
\url     www.corerobotics.org
\author  Parker Owan, Tony Piaskowy, Cameron Devine

*/
//=====================================================================

#include <iostream>
#include "CRTestModules.hpp"
#include "CoreRobotics.hpp"


using namespace std;


int main(int argc, const char * argv[]) {
    
    cout << "Running the CoreRobotics test suite." << endl;
    
    // Run the core test
    test_CRCore();
    
    // Run the math test
    test_CRMath();
    
    // Test the physics models
    CRTestFrameOffset();
    
    // Test the noise models
    test_CRNoiseModel();
    test_CRNoiseGaussian();
    test_CRNoiseDirac();
    test_CRNoiseUniform();
    test_CRNoiseMixture();
    
    // Test the sensor models
    test_CRSensorModel();
    test_CRSensorLinear();
    test_CRSensorProbabilistic();
    
    // Test the motion models
    test_CRMotionModel();
    test_CRMotionLinear();
    test_CRMotionProbabilistic();
    
    // Test the Manipulator
    test_CRManipulator();
    
    // Test controllers
    test_CRInverseKinematics();
	test_CRNullSpace();
	test_CRHardLimits();
    test_CRTrajectoryGenerator();
    return 0;
}
