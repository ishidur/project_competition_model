#ifndef DRIVINGCONTROL_TUMBLE_STOP_H
#define DRIVINGCONTROL_TUMBLE_STOP_H

#include "../AppliedHardware/VehicleHardware/PostureSensor.h"
#include "../AppliedHardware/VehicleHardware/DriveWheels.h"
#include "../AppliedHardware/VehicleHardware/Tail.h"

#define TUNBLE_MEMORY_SIZE 10

namespace DrivingControl{
	class TumbleStop{
		private:
			int count;

			float anglerVelocityMemory[TUNBLE_MEMORY_SIZE] = {0.0};
			float aveAnglerVel;

			signed char pwmLeft, pwmRight;
			float averagePWMMemory[TUNBLE_MEMORY_SIZE] = {0.0};
			float avePWM;

			AppliedHardware::VehicleHardware::PostureSensor postureSensor;
			AppliedHardware::VehicleHardware::DriveWheels* driveWheels;
			AppliedHardware::VehicleHardware::Tail* tail;

		public:
			TumbleStop();
			void TumbleStopTask();
	};

}  // namespace DrivingControl
#endif