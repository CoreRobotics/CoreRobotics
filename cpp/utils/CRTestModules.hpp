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
 \author  Parker Owan
 \version 0.0
 
 */
//=====================================================================

#ifndef CRTestModules_hpp
#define CRTestModules_hpp

// Core functionality test
void test_CRCore(void);

// Math tests
void test_CRMath(void);

// Physics tests
void CRTestFrameOffset(void); // Frame Tests

// Manipulator test
void test_CRManipulator(void);

// Noise model tests
void test_CRNoiseModel(void);           // test CRNoiseModel
void test_CRNoiseGaussian(void);        // test CRNoiseGaussian
void test_CRNoiseDirac(void);           // test CRNoiseDirac
void test_CRNoiseUniform(void);         // test CRNoiseUniform
void test_CRNoiseMixture(void);         // test CRNoiseMixture

// Sensor model tests
void test_CRSensorModel(void);          // test CRSensorModel
void test_CRSensorLinear(void);         // test CRSensorLinear
void test_CRSensorProbabilistic(void);  // test CRSensorProbabilistic

// Motion model tests
void test_CRMotionModel(void);          // test CRMotionModel
void test_CRMotionLinear(void);         // test CRMotionLinear
void test_CRMotionProbabilistic(void);  // test CRMotionProbabilistic

// Test controller modules
void test_CRInverseKinematics(void);    // test IK
void test_CRNullSpace(void);            // test CRNullSpace
void test_CRHardLimits(void);           // test Hard Limits
void test_CRTrajectoryGenerator(void);  // test TG


#endif /* CRTestModules_hpp */
