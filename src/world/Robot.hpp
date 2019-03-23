/*
 * Copyright (c) 2017-2019, CoreRobotics.  All rights reserved.
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause
 * http://www.corerobotics.org
 */

#ifndef CR_ROBOT_HPP_
#define CR_ROBOT_HPP_

#include "Link.hpp"
#include "Node.hpp"
#include <memory>
#include <vector>

namespace cr {
namespace world {

//! Robot shared pointer
class Robot;
typedef std::shared_ptr<Robot> RobotPtr;

//------------------------------------------------------------------------------
/*!
 \class Robot
 \ingroup world

 \brief This class implements a robotic manipulator composed of a
 series of links.  Currently only supports serial manipulators.

 \details

 ## References
 [1] J. Craig, "Introduction to Robotics: Mechanics and Control", Ed. 3,
 Pearson, 2004.

 [2] R. Murray, Z. Li, and S. Sastry, "A Mathematical Introduction to Robot
 Manipulation", Ed. 1, CRC Press, 1993.
 <a href="http://www.cds.caltech.edu/~murray/mlswiki/index.php/Main_Page">
 Available online.</a> \n\n

 [3] A. Hourtash, "The Kinematic Hessian and Higher Derivatives", IEEE
 International Symposium on Computational Intelligence in Robotics and
 Automation, 169-174, 2005.
 <a href="http://doi.org/10.1109/CIRA.2005.1554272">

 */
//------------------------------------------------------------------------------
class Robot : public Node {

  // Constructor and Destructor
public:
  //! Class constructor
  Robot();

  //! Class destructor
  ~Robot();

  //! Create a pointer
  static RobotPtr create();

  // Tree graph construction (with links)
public:
  //! Register a link with the Manipulator to access Jacobian, Hessian, etc...
  void addLink(LinkPtr i_link);

  // Get/Set Methods
public:
  //! Get the number of links in the list
  unsigned getNumberOfLinks();

  //! Get the number of driven links (degrees of freedom: DOF)
  unsigned getDegreesOfFreedom();

  //! Set the configuration (joint) space positions
  void setConfiguration(const Eigen::VectorXd &i_q);

  //! Get the configuration (joint) space positions
  Eigen::VectorXd getConfiguration();

  //! Set the configuration (joint) space velocities
  void setVelocity(const Eigen::VectorXd &i_qDot);

  //! Get the configuration (joint) space velocities
  Eigen::VectorXd getVelocity();

  //! Get the instantaneous forward kinematics
  // Eigen::MatrixXd getForwardKinematics(void);

  //! Get tool frame for the current manipulator configuration
  // void getToolFrame(unsigned i_toolIndex, Frame& o_tool);

  //! Get a link frame for the current manipulator configuration
  // void getLinkFrame(unsigned i_linkIndex, Frame& o_link);

  //! Get the pose for the specified tool index
  // Eigen::Matrix<double, 6, 1> getToolPose(unsigned i_toolIndex,
  //                                         EulerMode i_mode);

  // Eigen::VectorXd getToolPose(unsigned i_toolIndex,
  //                            EulerMode i_mode,
  //                            Eigen::Matrix<bool, 6, 1> i_poseElements);

  // Eigen::VectorXd getToolPose(unsigned i_toolIndex,
  //                             EulerMode i_mode,
  //                             Eigen::Matrix<int, 6, 1> i_poseElementsInt);

  // Kinematics
public:
  //! compute the jacobian for the indicated child node
  Eigen::MatrixXd jacobian(NodePtr i_node, physics::EulerMode i_mode);

  // Dynamics
public:
  //! compute the generalized mass matrix (w.r.t. the free variables)
  Eigen::MatrixXd mass();

  // Jacobian
public:
  //! Compute the link Jacobian (of the COM)

  //! Compute the instantaneous numerical Jacobian
  // Eigen::MatrixXd jacobian(unsigned i_toolIndex,
  //                         EulerMode i_mode);

  // Eigen::MatrixXd jacobian(unsigned i_toolIndex,
  //                         EulerMode i_mode,
  //                         Eigen::Matrix<bool, 6, 1> i_poseElements);

  // Eigen::MatrixXd jacobian(unsigned i_toolIndex,
  //                         EulerMode i_mode,
  //                         Eigen::Matrix<int, 6, 1> i_poseElementsInt);

  // Hessian
public:
  //! Compute the instantaneous numerical Hessian
  // Eigen::MatrixXd hessian(unsigned i_toolIndex, EulerMode i_mode);

  // Eigen::MatrixXd hessian(unsigned i_toolIndex, EulerMode i_mode,
  // Eigen::Matrix<bool, 6, 1> i_poseElements);

  // Eigen::MatrixXd hessian(unsigned i_toolIndex, EulerMode i_mode,
  // Eigen::Matrix<int, 6, 1> i_poseElementsInt);

  // Add link/tool Methods
public:
  //! Add a link to the kinematic structure, return the index of the added link
  // int addLink(cr::RigidBody* i_link);

  //! Add a tool to the manipulator, return the index of the added tool
  // int addTool(unsigned i_parentIndex, Frame* i_tool);

  // Print details
public:
  //! print the scene
  virtual void print(std::ostream &i_stream);

  // Protected Members
protected:
  //! List of the links in the manipulator
  std::vector<LinkPtr> m_links;
  // std::vector<cr::RigidBody*> m_listLinks;

  //! List of the link parent indices
  // std::vector<int> m_listParents;

  //! List of the driven link indices
  // std::vector<int> m_listDriven;

  //! List of tool frames
  // std::vector<cr::Frame*> m_listToolFrames;

  //! List of tool parents
  // std::vector<int> m_listToolParents;

  //! Arbitrary frame for computing tip position
  // cr::Frame* m_tipFrame;

  //! type (read only)
  std::string m_type = "Robot";
};

} // namespace world
} // namespace cr

#endif
