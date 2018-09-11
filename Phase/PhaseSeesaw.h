#ifndef PHASE_PHASE_SEESAW_H
#define PHASE_PHASE_SEESAW_H

#include "PhaseBase.h"

#include "../BaseHardware/Timer.h"
#include "../DrivingControl/PoseDrivingControl.h"
#include "../AppliedHardware/VehicleHardware/PostureSensor.h"
#include "../AppliedHardware/VehicleHardware/Tail.h"
#include "../AppliedHardware/VehicleHardware/DriveWheels.h"
#include "../AppliedHardware/EnvironmentSensor/EnvironmentViewer.h"
#include "../Positioning/Localization/SelfPos.h"
#include "../Utilities/Vector2D.h"

namespace Phase{

	class PhaseSeesaw : public PhaseBase{
		private:
			DrivingControl::PoseDrivingControl poseDrivingControl;
			AppliedHardware::VehicleHardware::PostureSensor postureSensor; 
			AppliedHardware::VehicleHardware::Tail* tail;

			Positioning::Localization::SelfPos* pos; 

			AppliedHardware::EnvironmentSensor::EnvironmentViewer* envViewer;
			AppliedHardware::VehicleHardware::DriveWheels* driveWheels;
		    BaseHardware::Timer* timer;

			int tmp_log_cnt;
			int tmp_stop_cnt;

		public:
			PhaseSeesaw();
			void Execute() override;
			bool IsFinish();

		private:
			bool IsStartForward( float _x );
			bool IsStartEnter( float _x );
			bool IsStartSlowForward( float _x );
			bool IsStartBackForward( float _x );
			bool IsStartStopForward( float _x );
			bool IsEndStopForward( float _x );
	};

}  // namespace Phase
#endif
