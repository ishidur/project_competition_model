#ifndef DRIVINGCONTROL_TUMBLE_STOP_H
#define DRIVINGCONTROL_TUMBLE_STOP_H

#include "../AppliedHardware/VehicleHardware/PostureSensor.h"

namespace DrivingControl{
	class TumbleStop{
		private:
			float anglerVelocityMemory;

			AppliedHardware::VehicleHardware::PostureSensor postureSensor;

		public:
			void TumbleStopTask();
	};

}  // namespace DrivingControl
#endif