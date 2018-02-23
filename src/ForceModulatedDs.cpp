#include "ForceModulatedDs.h"

ForceModulatedDs::ForceModulatedDs():
_gpr(new GaussianProcessRegression<float>(3,3))
{
	_gpr->SetHyperParams(0.1f, 1.0f, 0.2f);
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


// Eigen::Vector3f ForceModulatedDs::getModulatedDynamics(Eigen::Vector3f position)
// {
// 	Eigen::Vector3f theta;
// 	theta = _gpr->DoRegression(position);
// 	_lambdaC = theta.norm();
// 	// _lambdaC = 15.0f/100.0f;
// 	_e1 = theta/_lambdaC;

// 	Eigen::Vector3f reference;
// 	reference << 0.0f,0.0f,-1.0f;

// 	Eigen::Matrix3f P;

// 	float check = reference.dot(_e1);

// 	Eigen::Vector3f vd;
// 	vd = _originalDynamics(position);
	
// 	std::cerr << "theta: " << theta.transpose() << "_lambdaC: " << _lambdaC  << " check: " << check << std::endl;
	
// 	if(check > 0.85f)
// 	{
//   		P = Eigen::Matrix3f::Identity()-_e1*_e1.transpose();
//   		// vd = P*vd;
// 		_e2 = (Eigen::Matrix3f::Identity()-_e1*_e1.transpose())*vd;
// 		_e2.normalize();
// 		_e3 = _e1.cross(_e2);
// 		_e3.normalize();


// 		_lambdaB = _lambdaC/(_e1+_e2+_e3).dot(vd);
// 		float delta;
// 		// delta = 4.0f*std::pow(_lambdaC,2.0f)*std::pow(_e1.dot(vd),2.0f)-4.0f*vd.squaredNorm()*(std::pow(_lambdaC,2.0f)-vd.squaredNorm());
// 		delta = 4.0f*std::pow(_lambdaC,2.0f)*std::pow(_e1.dot(vd),2.0f)-4.0f*vd.squaredNorm()*(-vd.squaredNorm());
// 		if(delta<0.0f)
// 		{
// 			std::cerr << "a:" << std::endl;
// 			_M.setIdentity();
// 			_e1 << 0.0f, 0.0f, -1.0f;
// 			_lambdaC = 0.0f;
// 			_lambdaA = 1.0f;
// 		}
// 		else
// 		{
// 			_lambdaA = (-2.0f*_lambdaC*_e1.dot(vd)+std::sqrt(delta))/(2.0f*vd.squaredNorm());
// 			_B.col(0) = _e1;
// 			_B.col(1) = _e2;
// 			_B.col(2) = _e3;
// 			_L.setIdentity();
// 			_L(0,0) = _lambdaA+_lambdaB;
// 			_L(1,1) = _lambdaA;
// 			_L(2,2) = _lambdaA;
// 			_L(0,1) = _lambdaB;
// 			_L(0,2) = _lambdaB;

// 			_M = _B*_L*_B.transpose();

// 		}	
// 	}
// 	else
// 	{
// 		std::cerr << "b:" << std::endl;
// 		_M.setIdentity();
// 		_e1 << 0.0f, 0.0f, -1.0f;
// 		_lambdaC = 0.0f;
// 		_lambdaA = 1.0f;
// 	}

// 	Eigen::Vector3f vmd;
// 	vmd = _lambdaA*vd+_lambdaC*_e1;

// 	return vmd;
// }

Eigen::Vector3f ForceModulatedDs::getModulatedDynamics(Eigen::Vector3f position)
{
	Eigen::Vector3f theta;
	theta = _gpr->DoRegression(position);
	_lambdaC = theta.norm();
	// _lambdaC = 15.0f/100.0f;
	_e1 = theta/_lambdaC;

	Eigen::Vector3f reference;
	reference << 0.0f,0.0f,-1.0f;

	Eigen::Matrix3f P;

	float check = reference.dot(_e1);

	Eigen::Vector3f vd;
	vd = _originalDynamics(position);
	
	std::cerr << "theta: " << theta.transpose() << "_lambdaC: " << _lambdaC  << " check: " << check << std::endl;
	
	if(check > 0.85f && _lambdaC >1e-3f)
	{
  		P = Eigen::Matrix3f::Identity()-_e1*_e1.transpose();
  		// vd = P*vd;
		_e2 = (Eigen::Matrix3f::Identity()-_e1*_e1.transpose())*vd;
		_e2.normalize();
		_e3 = _e1.cross(_e2);
		_e3.normalize();


		_lambdaB = (_lambdaC+_e1.dot(vd))/(_e1+_e2+_e3).dot(vd);
		float delta;
		// // delta = 4.0f*std::pow(_lambdaC,2.0f)*std::pow(_e1.dot(vd),2.0f)-4.0f*vd.squaredNorm()*(std::pow(_lambdaC,2.0f)-vd.squaredNorm());
		// delta = (vd.squaredNorm()-std::pow(_lambdaB*(_e1+_e2+_e3).dot(vd),2.0f))/(std::pow(_e2.dot(vd),2.0f)+std::pow(_e3.dot(vd),2.0f));
		delta = vd.squaredNorm()/(std::pow(_e2.dot(vd),2.0f)+std::pow(_e3.dot(vd),2.0f));
		if(delta<0.0f)
		{
			// _M.setIdentity();
			// _e1 << 0.0f, 0.0f, -1.0f;
			_lambdaB = 0.0f;
			_lambdaA = 0.0f;
			delta = (vd.squaredNorm()-(std::pow(_e2.dot(vd),2.0f)+std::pow(_e3.dot(vd),2.0f)))/std::pow((_e1+_e2+_e3).dot(vd),2.0f);
			_lambdaB = sqrt(delta); 
		}
		else
		{
			_lambdaA = sqrt(delta)-1.0f;
			_B.col(0) = _e1;
			_B.col(1) = _e2;
			_B.col(2) = _e3;
			_L.setIdentity();
			_L(0,0) = _lambdaB;
			_L(1,1) = 1.0f+_lambdaA;
			_L(2,2) = 1.0f+_lambdaA;
			_L(0,1) = _lambdaB;
			_L(0,2) = _lambdaB;

			_M = _B*_L*_B.transpose();

		}	
	}
	else
	{
		_M.setIdentity();
		_e1 << 0.0f, 0.0f, -1.0f;
		_lambdaB = 0.0f;
		_lambdaA = 0.0f;
	}
	std::cerr << "_lambdaA: " << _lambdaA << "_lambdaB: " << _lambdaB << std::endl;

	Eigen::Vector3f vmd;
	// vmd = _lambdaA*vd+_lambdaC*_e1;
	vmd = _M*vd;

	return vmd;
}


Eigen::Vector3f ForceModulatedDs::getModulatedDynamics(Eigen::Vector3f position, float Fd)
{
	Eigen::Vector3f theta;
	theta = _gpr->DoRegression(position);
	float beta = theta.norm();
	if(beta>1)
	{
		_e1 = theta/beta;
		beta = 1.0f;
	}
	else if(beta < 0.0f)
	{
		beta = 0.0f;
		_e1 << 0.0f,0.0f,-1.0f;
	}
	else
	{
		_e1 = theta/beta;
	}
	// _lambdaC = 15.0f/100.0f;


	Eigen::Vector3f reference;
	reference << 0.0f,0.0f,-1.0f;

	Eigen::Matrix3f P;

	float check = reference.dot(_e1);

	Eigen::Vector3f vd;
	vd = _originalDynamics(position);
	
	_lambdaC = beta*Fd;
	
	std::cerr << "theta: " << theta.transpose() << "beta: " << beta  << " check: " << check << std::endl;
	
	if(check > 0.85f)
	{
		_e2 = (Eigen::Matrix3f::Identity()-_e1*_e1.transpose())*vd;
		_e2.normalize();
		_e3 = _e1.cross(_e2);
		_e3.normalize();


		_lambdaB = _lambdaC/(_e1+_e2+_e3).dot(vd);
		float delta;
		// delta = 4.0f*std::pow(_lambdaC,2.0f)*std::pow(_e1.dot(vd),2.0f)-4.0f*vd.squaredNorm()*(std::pow(_lambdaC,2.0f)-vd.squaredNorm());
		delta = 4.0f*std::pow(_lambdaC,2.0f)*std::pow(_e1.dot(vd),2.0f)-4.0f*vd.squaredNorm()*(-vd.squaredNorm());
		if(delta<0.0f)
		{
			std::cerr << "a:" << std::endl;
			_M.setIdentity();
			_e1 << 0.0f, 0.0f, -1.0f;
			_lambdaC = 0.0f;
			_lambdaA = 1.0f;
		}
		else
		{
			_lambdaA = (-2.0f*_lambdaC*_e1.dot(vd)+std::sqrt(delta))/(2.0f*vd.squaredNorm());
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
	}
	else
	{
		std::cerr << "b:" << std::endl;
		_M.setIdentity();
		_e1 << 0.0f, 0.0f, -1.0f;
		_lambdaC = 0.0f;
		_lambdaA = 1.0f;
	}

	Eigen::Vector3f vmd;
	vmd = _lambdaA*vd+_lambdaC*_e1;

	return vmd;
}

void ForceModulatedDs::addData(Eigen::Vector3f position, Eigen::Vector3f orientation, float force)
{
  Eigen::Vector3f reshapingParameters = computeReshapingParameters(orientation,force);
  _gpr->AddTrainingData(position, reshapingParameters);
}


void ForceModulatedDs::addData(Eigen::Vector3f position, Eigen::Vector3f orientation, Eigen::Vector3f velocity)
{
  Eigen::Vector3f reshapingParameters = computeReshapingParameters(orientation,velocity);
  _gpr->AddTrainingData(position, reshapingParameters);
}


Eigen::Vector3f ForceModulatedDs::computeReshapingParameters(Eigen::Vector3f orientation, float force)
{
  Eigen::Vector3f theta = force*orientation;

  return theta;
}

Eigen::Vector3f ForceModulatedDs::computeReshapingParameters(Eigen::Vector3f orientation, Eigen::Vector3f velocity)
{
  float vn = fabs(orientation.dot(velocity));
  Eigen::Vector3f theta = (1-std::tanh(50.f*vn))*orientation;

  return theta;
}
