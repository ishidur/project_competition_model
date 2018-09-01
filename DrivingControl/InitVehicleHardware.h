#ifndef DRIVINGCONTROL_INIT_VEHICLE_HARDWARE_H
#define DRIVINGCONTROL_INIT_VEHICLE_HARDWARE_H

#include "../AppliedHardware/VehicleHardware/PostureSensor.h"
#include "../AppliedHardware/EnvironmentSensor/EnvironmentViewer.h"
#include "TailControl.h"

namespace DrivingControl{

	class InitVehicleHardware{
		private:
			TailControl tailControl;
			AppliedHardware::VehicleHardware::PostureSensor postureSensor;
			AppliedHardware::EnvironmentSensor::EnvironmentViewer* environmentViewer;

		public:
			InitVehicleHardware();
			void Initialize();
			void Calibration();

	};

}  // namespace DrivingControl
#endif
