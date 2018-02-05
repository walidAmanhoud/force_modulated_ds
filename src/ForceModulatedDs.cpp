#include "ForceModulatedDs.h"

ForceModulatedDs::ForceModulatedDs():
_gpr(new GaussianProcessRegression<float>(3,3))
{
	_gpr->SetHyperParams(3.2,1.0,0.02);
}


Eigen::Matrix3f ForceModulatedDs::getModulationMatrix()
{
	return _M;
}


Eigen::Vector3f ForceModulatedDs::getEstimatedNormal()
{
	return _e1;
}


void ForceModulatedDs::setOriginalDynamics(std::function<Eigen::Vector3f(Eigen::Vector3f)> originalDynamics)
{
	_originalDynamics = originalDynamics;
}


Eigen::Vector3f ForceModulatedDs::getModulatedDynamics(Eigen::Vector3f position)
{
	Eigen::Vector3f theta;
	theta = _gpr->DoRegression(position);

	_lambdaC = theta.norm();
	_e1 = theta.normalized();
	Eigen::Vector3f vd;
	vd = _originalDynamics(position);

	_e2 = (Eigen::Matrix3f::Identity()-_e1*_e1.transpose())*vd;
	_e2.normalize();
	_e3 = _e1.cross(_e2);
	_e3.normalize();


	_lambdaB = _lambdaC/(_e1+_e2+_e3).dot(vd);
	float delta;
	delta = 4.0f*std::pow(_lambdaC,2.0f)*std::pow(_e1.dot(vd),2.0f)-4.0f*vd.squaredNorm()*(std::pow(_lambdaC,2.0f)-vd.squaredNorm());
	if(delta<0.0f)
	{
		_M.setIdentity();
		_e1 << 0.0f, 0.0f, -1.0f;
	}
	else
	{
		_lambdaA = (-2.0f*_lambdaC+std::sqrt(delta))/(2.0f*vd.squaredNorm());
		_B.col(0) = _e1;
		_B.col(1) = _e2;
		_B.col(2) = _e3;
		_L.setIdentity();
		_L(0,0) = _lambdaA+_lambdaB;
		_L(1,1) = _lambdaA;
		_L(2,2) = _lambdaA;
		_L(0,1) = _lambdaB;
		_L(0,2) = _lambdaB;

		_M = _B*_L*_B.transpose();
	}

	return _M*_originalDynamics(position);
}


void ForceModulatedDs::addData(Eigen::Vector3f position, Eigen::Vector3f orientation, float force)
{
  Eigen::Vector3f reshapingParameters = computeReshapingParameters(orientation,force);
  _gpr->AddTrainingData(position, reshapingParameters);
}


Eigen::Vector3f ForceModulatedDs::computeReshapingParameters(Eigen::Vector3f orientation, float force)
{
  Eigen::Vector3f theta = force*orientation;
}

