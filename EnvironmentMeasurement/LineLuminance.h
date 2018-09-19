#ifndef ENVIRONMENTMEASUREMENT_LINE_LUMINANCE_H
#define ENVIRONMENTMEASUREMENT_LINE_LUMINANCE_H

#include "../AppliedHardware/EnvironmentSensor/EnvironmentViewer.h"
#include "../DrivingControl/CalcTurn.h"
#include "../Utilities/PIDCalculation.h"
#include "../Utilities/ExponentialSmoothingFilter.h"

namespace EnvironmentMeasurement{

	class LineLuminance : public DrivingControl::CalcTurn{
		private:
			AppliedHardware::EnvironmentSensor::EnvironmentViewer* environmentViewer;
			Utilities::PIDCalculation* pidCalc;
			Utilities::ExponentialSmoothingFilter* filter;

			float turn;

		public:
			LineLuminance();
			void CalcTurnValue();
			void CalcTurnValueByRGB();
			void CalcTurnValueByRGBStand();
			float GetTurn();
	};

}  // namespace EnvironmentMeasurement
#endif
