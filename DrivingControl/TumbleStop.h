#ifndef DRIVINGCONTROL_TUMBLE_STOP_H
#define DRIVINGCONTROL_TUMBLE_STOP_H

#include "../AppliedHardware/VehicleHardware/PostureSensor.h"
#include "../AppliedHardware/VehicleHardware/DriveWheels.h"

namespace DrivingControl{
	class TumbleStop{
		private:
			float anglerVelocityMemory[255] = {0.0};
			float wheelAngleMemory[255] = {0.0};
			float count;

			AppliedHardware::VehicleHardware::PostureSensor postureSensor;
			AppliedHardware::VehicleHardware::DriveWheels* driveWheels;

		public:
			TumbleStop();
			void TumbleStopTask();
	};

}  // namespace DrivingControl
#endif