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
 \author Parker Owan
 \version 0.0
 
 */
//=====================================================================

#include <iostream>
#include "CoreRobotics.hpp"

// Use the CoreRobotics namespace
using namespace CoreRobotics;
using namespace Eigen;
using namespace std;


int main(int argc, const char * argv[]) {
    
    cout << "Hello, CoreRobotics!\n";
    
    // Set up a CoreRobotics frame
    CRFrame myFrame = *new CRFrame();
    
    
    // Get the default rot and trans
    Matrix3d r;
    Vector3d t;
    myFrame.getRotationAndTranslation( r, t);
    cout << r << endl;
    cout << t << endl;
    
    
    // Assign the rotation and translation matrices
    Matrix3d R;
    R << 0, -1, 0,
         1, 0, 0,
         0, 0, 1;
    Vector3d T;
    T << 1, 2, 3;
    myFrame.setRotationAndTranslation( R, T);
    myFrame.getRotationAndTranslation( r, t);
    cout << r << endl;
    cout << t << endl;
    
    
    // Return the 4x4 homogeneous forward transformation
    Matrix4d TtoParent;
    myFrame.getTransformToParent(TtoParent);
    cout << TtoParent << endl;
    
    
    // Return the 4x4 homogeneous reverse transformation
    Matrix4d TtoChild;
    myFrame.getTransformToChild(TtoChild);
    cout << TtoChild << endl;
    
    
    // Transform some points through the forward transformation
    Matrix<double, 3, 2> Points;
    Points << 5, 1,
              3, 1,
              4, 9;
    Matrix<double, 3, Dynamic> newPoints;
    myFrame.transformToParent(Points, newPoints);
    cout << newPoints << endl;
    
    // Transform some points through the backward transformation
    Points = newPoints;
    myFrame.transformToChild(Points, newPoints);
    cout << newPoints << endl;
    
    
    
    // Now lets create a modified DH parameter matrix
    cout << "Now create a DH parameter matrix" << endl;
    //CRFrameDh* Frame1 = new CRFrameDh();
    CRFrameDh Frame1;
    
    Frame1.setMode(CR_DH_MODE_CLASSIC);
    Frame1.setParameters(0.12, 0.43, 0.23, 0.74);
    Eigen::Matrix4d FrameTform;
    Frame1.getTransformToParent(FrameTform);
    cout << FrameTform << endl;
    
    
    
    return 0;
}
