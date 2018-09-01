#ifndef APPLIEDHARDWARE_VEHICLEHARDWARE_DRIVE_WHEELS_H
#define APPLIEDHARDWARE_VEHICLEHARDWARE_DRIVE_WHEELS_H

#include "../../BaseHardware/Motor.h"

namespace AppliedHardware{
	namespace VehicleHardware{

		class DriveWheels{
			public:
				static const float tireRadius;
				static const float tireDistance;

			private:
				static DriveWheels* singletonInstance;

				BaseHardware::Motor* motorRight;
				BaseHardware::Motor* motorLeft;

				signed char pwmRight;
				signed char pwmLeft;

			public:
				static DriveWheels* GetInstance();

				void SetPWMs(signed char pwmLeft, signed char pwmRight);
				void Stop(bool isStopLeft, bool isStopRight);
				void GetAngles(float* angleLeft, float* angleRight);
				void GetPWMs(signed char* pwmLeft, signed char* pwmRight);

			private:
				DriveWheels();

		};

	}  // namespace VehicleHardware
}  // namespace AppliedHardware
#endif
