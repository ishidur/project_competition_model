#ifndef PHASE_PHASE_INITIALIZATION_H
#define PHASE_PHASE_INITIALIZATION_H

#include "../DrivingControl/InitVehicleHardware.h"
#include "../EnvironmentMeasurement/InitEnvironmentSensor.h"
#include "PhaseBase.h"

namespace Phase{
	
	class PhaseInitialization : public PhaseBase{
		private:
			DrivingControl::InitVehicleHardware initVehicleHardware;
			EnvironmentMeasurement::InitEnvironmentSensor initEnvironmentSensor;

		public:
			PhaseInitialization();
			void Execute() override;
			void Calibration();
	};

}  // namespace Phase
#endif
