/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#include "gtest/gtest.h"
#include <cr/physics>
#include <iostream>

// Use the CoreRobotics namespace
using namespace cr;

//
// constructor
//
TEST(FrameDh, Construct) {
  physics::FrameDh frame;
  Eigen::VectorXd p = frame.getPose(physics::CR_EULER_MODE_XYZ);
  EXPECT_DOUBLE_EQ(0, p(0));
  EXPECT_DOUBLE_EQ(0, p(1));
  EXPECT_DOUBLE_EQ(0, p(2));
  EXPECT_DOUBLE_EQ(0, p(3));
  EXPECT_DOUBLE_EQ(0, p(4));
  EXPECT_DOUBLE_EQ(0, p(5));
}

//
// check the regular dh approach (based on the exact values expected
// https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters
//
TEST(FrameDh, ClassicTransformation) {

  // init classic frame
  double r = 0.1;
  double alpha = M_PI / 3;
  double d = 0.2;
  double theta = M_PI / 6;
  physics::FrameDh frame(r, alpha, d, theta, physics::CR_DH_MODE_CLASSIC,
                         physics::CR_DH_FREE_NONE);

  Eigen::Matrix3d R;
  Eigen::Vector3d T;
  frame.getRotationAndTranslation(R, T);

  // check to the matrix layout
  EXPECT_DOUBLE_EQ(cos(theta), R(0, 0));
  EXPECT_DOUBLE_EQ(-sin(theta) * cos(alpha), R(0, 1));
  EXPECT_DOUBLE_EQ(sin(theta) * sin(alpha), R(0, 2));

  EXPECT_DOUBLE_EQ(sin(theta), R(1, 0));
  EXPECT_DOUBLE_EQ(cos(theta) * cos(alpha), R(1, 1));
  EXPECT_DOUBLE_EQ(-cos(theta) * sin(alpha), R(1, 2));

  EXPECT_DOUBLE_EQ(0, R(2, 0));
  EXPECT_DOUBLE_EQ(sin(alpha), R(2, 1));
  EXPECT_DOUBLE_EQ(cos(alpha), R(2, 2));

  EXPECT_DOUBLE_EQ(r * cos(theta), T(0));
  EXPECT_DOUBLE_EQ(r * sin(theta), T(1));
  EXPECT_DOUBLE_EQ(d, T(2));
}

//
// check the modified dh approach (based on the exact values expected
// https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters
//
TEST(FrameDh, ModifiedTransformation) {

  // init classic frame
  double r = 0.1;
  double alpha = M_PI / 3;
  double d = 0.2;
  double theta = M_PI / 6;
  physics::FrameDh frame(r, alpha, d, theta, physics::CR_DH_MODE_MODIFIED,
                         physics::CR_DH_FREE_NONE);

  Eigen::Matrix3d R;
  Eigen::Vector3d T;
  frame.getRotationAndTranslation(R, T);

  // check to the matrix layout
  EXPECT_DOUBLE_EQ(cos(theta), R(0, 0));
  EXPECT_DOUBLE_EQ(-sin(theta), R(0, 1));
  EXPECT_DOUBLE_EQ(0, R(0, 2));

  EXPECT_DOUBLE_EQ(sin(theta) * cos(alpha), R(1, 0));
  EXPECT_DOUBLE_EQ(cos(theta) * cos(alpha), R(1, 1));
  EXPECT_DOUBLE_EQ(-sin(alpha), R(1, 2));

  EXPECT_DOUBLE_EQ(sin(theta) * sin(alpha), R(2, 0));
  EXPECT_DOUBLE_EQ(cos(theta) * sin(alpha), R(2, 1));
  EXPECT_DOUBLE_EQ(cos(alpha), R(2, 2));

  EXPECT_DOUBLE_EQ(r, T(0));
  EXPECT_DOUBLE_EQ(-d * sin(alpha), T(1));
  EXPECT_DOUBLE_EQ(d * cos(alpha), T(2));
}

//
// SetFreeValue/GetFreeValue
//
TEST(FrameDh, GetFreeValue) {
  physics::FrameDh frame(0, 0, 0, M_PI / 2, physics::CR_DH_MODE_MODIFIED,
                         physics::CR_DH_FREE_NONE);
  EXPECT_EQ(NULL, frame.getFreeValue());

  frame.setFreeVariable(physics::CR_DH_FREE_THETA);
  EXPECT_DOUBLE_EQ(M_PI / 2, frame.getFreeValue());

  frame.setFreeValue(0);
  EXPECT_DOUBLE_EQ(0, frame.getFreeValue());
}

//
// SetFreeVariable/GetFreeVariable
//
TEST(FrameDh, GetFreeVariable) {
  physics::FrameDh frame(0, 0, 0, M_PI / 2, physics::CR_DH_MODE_MODIFIED,
                         physics::CR_DH_FREE_NONE);
  EXPECT_EQ(physics::CR_DH_FREE_NONE, frame.getFreeVariable());
  EXPECT_FALSE(frame.isDriven());

  frame.setFreeVariable(physics::CR_DH_FREE_THETA);
  EXPECT_EQ(physics::CR_DH_FREE_THETA, frame.getFreeVariable());
  EXPECT_TRUE(frame.isDriven());
}

//
// SetMode/GetMode
//
TEST(FrameDh, GetMode) {
  physics::FrameDh frame(0, 0, 0, M_PI / 2, physics::CR_DH_MODE_CLASSIC,
                         physics::CR_DH_FREE_NONE);
  EXPECT_EQ(physics::CR_DH_MODE_CLASSIC, frame.getMode());

  frame.setFreeVariable(physics::CR_DH_FREE_THETA);
  frame.setMode(physics::CR_DH_MODE_MODIFIED);
  EXPECT_EQ(physics::CR_DH_MODE_MODIFIED, frame.getMode());
}

//
// GetParameters
//
TEST(FrameDh, GetParameters) {
  physics::FrameDh frame(0.1, 0.2, 0.3, M_PI / 2, physics::CR_DH_MODE_MODIFIED,
                         physics::CR_DH_FREE_NONE);
  double r, alpha, d, theta;
  frame.getParameters(r, alpha, d, theta);

  EXPECT_DOUBLE_EQ(0.1, r);
  EXPECT_DOUBLE_EQ(0.2, alpha);
  EXPECT_DOUBLE_EQ(0.3, d);
  EXPECT_DOUBLE_EQ(M_PI / 2, theta);
}
