/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#include "gtest/gtest.h"
#include <iostream>
#include <string>

#include <cr/aspect>

class Base {
public:
  struct Parameters {
    std::string name = "Base";
  };
  CR_ASPECT_PARAMETER_MUTABLE(Base::Parameters)
};

class Derived : public Base {
public:
  struct Parameters : public Base::Parameters {
    double value{3.1415};
  };
  struct States {
    double value{0.0};
  };
  CR_ASPECT_PARAMETER_MUTABLE(Derived::Parameters)
  CR_ASPECT_STATE_READ(Derived::States)
};

TEST(Aspect, AspectStructue) {

  auto my_base = Base();;
  EXPECT_EQ(my_base.parameters()->name, "Base");

  auto my_class = Derived();
  EXPECT_EQ(my_class.parameters()->name, "Base");
  EXPECT_DOUBLE_EQ(my_class.parameters()->value, 3.1415);
  EXPECT_DOUBLE_EQ(my_class.getState().value, 0.0);
}
