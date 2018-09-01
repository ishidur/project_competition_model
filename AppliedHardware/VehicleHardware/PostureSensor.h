#ifndef APPLIEDHARDWARE_VEHICLEHARDWARE_POSTURE_SENSOR_H
#define APPLIEDHARDWARE_VEHICLEHARDWARE_POSTURE_SENSOR_H

#include "../../AppliedHardware/VehicleHardware/CalibrationGyro.h"
#include "../../BaseHardware/GyroSensor.h"

namespace AppliedHardware{
	namespace VehicleHardware{

		class PostureSensor{
			private:
				BaseHardware::GyroSensor* gyro; 
				CalibrationGyro* calibrationGyro;

			public:
				PostureSensor();
				float GetAnglerVelocity();
				float GetGyroOffset();
				void Calibration();
		};

	}  // namespace VehicleHardware
}  // namespace AppliedHardware
#endif
