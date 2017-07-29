#ifndef CRNullSpace_hpp
#define CRNullSpace_hpp

#include "CRManipulator.hpp"
#include "CRTypes.hpp"

namespace CoreRobotics {

class CRNullSpace {

public:

	CRNullSpace(CRManipulator* i_robot,
                unsigned int i_toolIndex,
                CREulerMode i_eulerMode);

	void setRobot(CRManipulator* i_robot) {this->m_robot = i_robot;}

	void setToolIndex(unsigned int i_toolIndex) {this->m_toolIndex = i_toolIndex;}

	void setEulerMode(CREulerMode i_eulerMode) {this->m_eulerMode = i_eulerMode;}

	CREulerMode getEulerMode(void) {return this->m_eulerMode;}

	void setSingularThresh(double i_thresh) {this->m_svdTol = i_thresh;}

	double getSingularThresh(void) {return this->m_svdTol;}

	Eigen::VectorXd solve(Eigen::VectorXd i_velocities,
                          Eigen::VectorXd i_q0);

	Eigen::VectorXd solve(Eigen::VectorXd i_velocities,
                          Eigen::Matrix<bool, 6, 1> i_poseElements,
                          Eigen::VectorXd i_q0);

	Eigen::VectorXd solve(Eigen::VectorXd i_velocities,
                          Eigen::Matrix<int, 6, 1> i_poseElementsInt,
                          Eigen::VectorXd i_q0);

protected:

	CRManipulator* m_robot;

	unsigned int m_toolIndex;

	CREulerMode m_eulerMode;

	double m_svdTol;

};

}

#endif
