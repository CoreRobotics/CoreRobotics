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

\author CoreRobotics Project
\author www.corerobotics.org
\author Kevin Bray
\version 0.0

*/
//=====================================================================

#include <iostream>
#include "../src/physics/CRFrame.hpp"
#include "../src/physics/CRFrameEuler.hpp"
//#include "../src/math/CRMath.hpp"

using namespace std;


int main(int argc, const char * argv[]) {

    std::cout << "Hello, CoreRobotics!" << std::endl;

    //// create frame
    //CoreRobotics::CRFrameEuler *myFrame = new CoreRobotics::CRFrameEuler();
    //myFrame->setMode(CoreRobotics::CREulerMode::CR_EULER_MODE_ZYX);

    //// create a rotation and translation 
    //Eigen::Matrix3d r;
    //Eigen::Vector3d t;

    //// get the values from my frame
    //myFrame->getRotationAndTranslation(r, t);
    //cout << t << endl;
    //cout << r << endl;

    //// set rotations
    //double a = 0;
    //double b = 85;
    //double g = 182;
    //cout << "Rotate by: " << a << ", " << b << ", " << g << endl;
    //myFrame->setPositionAndOrientation(1, 2, 3, 
    //    CoreRobotics::CRMath::deg2rad(a), 
    //    CoreRobotics::CRMath::deg2rad(b), 
    //    CoreRobotics::CRMath::deg2rad(g));
    //myFrame->getRotationAndTranslation(r, t);
    //cout << t << endl;
    //cout << r << endl;

    //Eigen::Vector3d pose;
    //myFrame->getEulerPose(CoreRobotics::CREulerMode::CR_EULER_MODE_ZYX, pose);

    ////pose << CoreRobotics::CRMath::rad2deg(pose(0)),
    ////    CoreRobotics::CRMath::rad2deg(pose(1)),
    ////    CoreRobotics::CRMath::rad2deg(pose(2));

    //cout << "Pose: " << endl << pose << endl;

    //myFrame->getEulerPose2(CoreRobotics::CREulerMode::CR_EULER_MODE_ZYX, pose);
    ////pose << CoreRobotics::CRMath::rad2deg(pose(0)),
    ////    CoreRobotics::CRMath::rad2deg(pose(1)),
    ////    CoreRobotics::CRMath::rad2deg(pose(2));

    //cout << "Pose: " << endl << pose << endl;

    //system("pause");

    return 0;
}
