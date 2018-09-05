#ifndef APPLIEDHARDWARE_ENVIRONMENTSENSOR_ENVIRONMENT_VIEWER_H
#define APPLIEDHARDWARE_ENVIRONMENTSENSOR_ENVIRONMENT_VIEWER_H

#include "../../BaseHardware/ColorSensor.h"
#include "../../BaseHardware/UltraSonicSensor.h"
#include "../../BaseHardware/TouchSensor.h"

namespace AppliedHardware{
	namespace EnvironmentSensor{
		class EnvironmentViewer{
			private:
				static EnvironmentViewer* singletonInstance;

				BaseHardware::ColorSensor* colorSensor;
				BaseHardware::UltraSonicSensor* ultraSonicSensor;
				BaseHardware::TouchSensor* touchSensor;

			public:
				static EnvironmentViewer* GetInstance();

				float GetLuminance();
				void GetRGB(int* r, int* g, int* b);
				float GetUSDistance();
				bool GetTouch();

			private:
				EnvironmentViewer();
		};

	}  // namespace EnvironmentSensor
}  // namespace AppliedHardware
#endif
