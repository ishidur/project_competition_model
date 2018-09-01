#ifndef ENVIRONMENTMEASUREMENT_INIT_ENVIRONMENT_SENSOR_H
#define ENVIRONMENTMEASUREMENT_INIT_ENVIRONMENT_SENSOR_H

#include "../AppliedHardware/EnvironmentSensor/CalibrationLuminanceRange.h"
#include "../AppliedHardware/EnvironmentSensor/EnvironmentViewer.h"
#include "../DrivingControl/TailControl.h"

namespace EnvironmentMeasurement{
	
	class InitEnvironmentSensor{
		private:
			AppliedHardware::EnvironmentSensor::CalibrationLuminanceRange calibrationLuminanceRange;
			AppliedHardware::EnvironmentSensor::EnvironmentViewer* environmentViewer;
			DrivingControl::TailControl tailControl;

		public:
			InitEnvironmentSensor();
			void Initialize();
			void Calibration();

		private:
			bool LoadCalibrationLight(int &white, int &black);
			void SetColor(int type);
	};

}  // namespace EnvironmentMeasurement
#endif
