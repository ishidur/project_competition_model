#ifndef UTILITIES_PARTICLE_FILTER_H
#define UTILITIES_PARTICLE_FILTER_H

#include "LikelihoodFilter.h"
#include "Vector2D.h"
#include "../Positioning/MapInfomation/FieldMap.h"
#include "../AppliedHardware/VehicleHardware/DriveWheels.h"

#define NP 50           // Number of Particle
#define NTH (NP/2)      // Num of resampling
#define RR 0.5          // Covariance

#define QX (1.0/RATIO)
#define QY (1.0/RATIO)
#define QTHETA (0.5*M_PI/180.0)

#define WHITE 100.0

namespace Utilities{

	class ParticleFilter : public LikelihoodFilter{
		private:
			float px[NP]={0}, py[NP]={0}, ptheta[NP]={0}, pw[NP]={0};

			Positioning::MapInfomation::FieldMap mapLikelihood;

		public:
			float GetValue() override;

			ParticleFilter(Positioning::MapInfomation::FieldMap mapLikelihood);
			void Initialize(Utilities::Vector2D pos, float theta);
			void Localization(Utilities::Vector2D* posEst, float* thetaEst, Utilities::Vector2D posDiff, float thetaDiff, float input);
	};

}  // namespace Utilities

float Gauss(float x, float u, float sigma);
double Uniform( void );
double RandNormal( double mu, double sigma );
float Resampling(float* xx, float* yy, float* tt, float* ww);

#endif
