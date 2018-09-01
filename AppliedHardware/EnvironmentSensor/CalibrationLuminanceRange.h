#ifndef APPLIEDHARDWARE_ENVIRONMENTSENSOR_CALIBRATION_LUMINANCE_RANGE_H
#define APPLIEDHARDWARE_ENVIRONMENTSENSOR_CALIBRATION_LUMINANCE_RANGE_H

#include "../../BaseHardware/ColorSensor.h"

namespace AppliedHardware{
	namespace EnvironmentSensor{

		class CalibrationLuminanceRange{
			private:
				BaseHardware::ColorSensor* colorSensor;

				int white;
				int black;

			public:
				CalibrationLuminanceRange();
				void SetWhite();
				void SetBlack();
				void SetRange();
				void SetRange(int white, int black);
		};

	}  // namespace EnvironmentSensor
}  // namespace AppliedHardware
#endif
