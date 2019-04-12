//
//  MotionTypes.hpp
//  cr-control
//
//  Created by Parker Owan on 4/6/19.
//

#ifndef MotionTypes_hpp
#define MotionTypes_hpp

#include "Eigen/Dense"
#include "core/Step.hpp"

namespace ph = std::placeholders;

namespace cr2 {
namepsace noise {

template<typename DomainType>
class Distribution<DomainType> {};


}

namespace model {

//! This is the base class for motion models
template<typename StateType, typename ActionType>
class Motion
  : public Step {
public:

  //! This function steps the callback and updates the state.
  virtual void step() override;
  
  //! The prototype callback function x_{k+1} = f(t, x, u) must be implemented.
  virtual StateType callback(double t, StateType x, ActionType u) = 0;
  
private:

  //! bound callback function
  std::function<StateType(double, StateType, ActionType)> m_fcn =
    std::bind(&Motion::callback, this, ph::_1, ph::_2, ph::_3);
};

//! This is the dynamical system class
class DynamicalSystem
  : public Motion<Eigen::VectorXd, Eigen::VectorXd> {

  //! Override the meaning of the callback function:
  //! DISCRETE: x_{k+1} = f(t, x, u)
  //! CONTINUOUS: \dot{x} = f(t, x, u)
  virtual void step() override;
};

//! This is the dynamical system class
class MotionLinear
  : public DynamicalSystem {

  //! TODO: Override how the linear simulation is performed
  // virtual void step() override;
  
  //! Implement the linear callback function
  virtual StateType callback(double t, StateType x, ActionType u) override;
};

//! This is the dynamical system class
template<typename StateType, typename ActionType>
class MotionProbabilistic
  : public Motion<Distribution<StateType>, Distribution<ActionType>> {

  //! TODO: Override how the linear simulation is performed
  // virtual void step() override;
  
  //! Implement the linear callback function
  virtual noise::Distribution<StateType> callback(
    double t,
    noise::Distribution<StateType> x,
    noise::Distribution<ActionType> u) override;
};

}
}




#endif /* MotionTypes_hpp */
