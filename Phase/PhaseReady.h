#ifndef PHASE_PHASE_READY_H
#define PHASE_PHASE_READY_H

#include "../DrivingControl/PoseDrivingControl.h"
#include "../DrivingControl/TailControl.h"
#include "../AppliedHardware/Communication/Communication.h"
#include "../AppliedHardware/EnvironmentSensor/EnvironmentViewer.h"
#include "PhaseBase.h"

namespace Phase{

	class PhaseReady : public PhaseBase{
		private:
			DrivingControl::TailControl tailControl;
			DrivingControl::PoseDrivingControl poseDrivingControl;
			AppliedHardware::Communication::Communication* com;
			AppliedHardware::EnvironmentSensor::EnvironmentViewer* envViewer;

		public:
			PhaseReady();
			void Execute() override;
			bool IsGetStartCommand();
	};

}  // namespace Phase
#endif
