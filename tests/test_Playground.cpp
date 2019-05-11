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

//
// Can we define a static struct
//
TEST(Playground, TemplateClassParameters) {

  auto my_base = Base();;
  // auto params = Base::Parameters();
  std::cout << "Base::Parameters()\n";
  std::cout << my_base.parameters()->name << "\n";

  auto my_class = Derived();
  // auto more_params = Derived::Parameters();
  std::cout << "Derived::Parameters()\n";
  std::cout << my_class.parameters()->name << "\n";
  std::cout << my_class.parameters()->value << "\n";
  std::cout << "Derived::States()\n";
  std::cout << my_class.getState().value << "\n";

  
}
