#include "CRNullSpace.hpp"
#include "CRMath.hpp"

namespace CoreRobotics {

CRNullSpace::CRNullSpace(CRManipulator* i_robot,
                         unsigned int i_toolIndex,
                         CREulerMode i_eulerMode)
{
	this->setRobot(i_robot);
	this->setToolIndex(i_toolIndex);
	this->setEulerMode(i_eulerMode);
	this->setSingularThresh(1.0e-1);
}

Eigen::VectorXd CRNullSpace::solve(Eigen::VectorXd i_velocities,
                                   Eigen::VectorXd i_q0)
{
	this->m_robot->setConfiguration(i_q0);
	Eigen::MatrixXd J, Jinv;
	J = this->m_robot->jacobian(this->m_toolIndex,
                                this->m_eulerMode);
	CRMath::svdInverse(J, this->m_svdTol, Jinv);
	return (Eigen::MatrixXd::Identity(this->m_robot->getDegreesOfFreedom(), this->m_robot->getDegreesOfFreedom()) - (Jinv * J)) * i_velocities;
}

Eigen::VectorXd CRNullSpace::solve(Eigen::VectorXd i_velocities,
                                   Eigen::Matrix<bool, 6, 1> i_poseElements,
                                   Eigen::VectorXd i_q0)
{
	this->m_robot->setConfiguration(i_q0);
	Eigen::MatrixXd J, Jinv;
	J = this->m_robot->jacobian(this->m_toolIndex,
                                this->m_eulerMode,
								i_poseElements);
	CRMath::svdInverse(J, this->m_svdTol, Jinv);
	return (Eigen::MatrixXd::Identity(this->m_robot->getDegreesOfFreedom(), this->m_robot->getDegreesOfFreedom()) - (Jinv * J)) * i_velocities;
}

Eigen::VectorXd CRNullSpace::solve(Eigen::VectorXd i_velocities,
                                   Eigen::Matrix<int, 6, 1> i_poseElementsInt,
                                   Eigen::VectorXd i_q0)
{
	Eigen::Matrix<bool, 6, 1> i_poseElements = i_poseElementsInt.cast<bool>();
	return CRNullSpace::solve(i_velocities, i_poseElements, i_q0);
}

}
