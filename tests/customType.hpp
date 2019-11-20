/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#ifndef CR_CUSTOM_TYPE_HPP_
#define CR_CUSTOM_TYPE_HPP_

#include <cr/signal>

struct CustomType {
  double value{1.0};
  std::string id{"my_string"};
};

class TypeEmitter {
public:
  Eigen::Vector3d getVector() {
    Eigen::Vector3d p(0.0, 1.0, 2.0);
    return p;
  }
  double getDouble() { return 1.0; }
  float getFloat() { return 2.0; }
  int getInt() { return 5; }
  bool getBool() { return false; }
  CustomType getCustom() { return CustomType(); }
};

class CustomSerializer : public cr::signal::GenericSerializer {
public:
  static void write(std::ostream &i_log, CustomType i_data) {
    i_log << i_data.value  << "," << i_data.id << ",";
  }
  static unsigned size(CustomType /** i_data **/) { return 2; }
};

#endif
