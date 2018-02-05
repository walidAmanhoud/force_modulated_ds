#ifndef FORCE_MODULATED_DS
#define FORCE_MODULATED_DS

#include "eigen3/Eigen/Dense"
#include "gaussian_process_regression/gaussian_process_regression.h"
#include <memory>
#include <functional>


class ForceModulatedDs
{
	private:
		Eigen::Matrix3f _M;
		Eigen::Matrix3f _B;
		Eigen::Matrix3f _L;
		Eigen::Vector3f _e1;
		Eigen::Vector3f _e2;
		Eigen::Vector3f _e3;
		
		float _lambdaA;
		float _lambdaB;
		float _lambdaC;

		std::shared_ptr<GaussianProcessRegression<float> > _gpr;
		std::function<Eigen::Vector3f(Eigen::Vector3f)> _originalDynamics;

	public:
		ForceModulatedDs();
		Eigen::Matrix3f getModulationMatrix();
		Eigen::Vector3f getEstimatedNormal();
		Eigen::Vector3f getModulatedDynamics(Eigen::Vector3f position);
		void setOriginalDynamics(std::function<Eigen::Vector3f(Eigen::Vector3f)> originalDynamics);
		void addData(Eigen::Vector3f position, Eigen::Vector3f orientation, float force);
	
	private:

		Eigen::Vector3f computeReshapingParameters(Eigen::Vector3f orientation, float force);
};


#endif // FORCE_MODULATED_DS
