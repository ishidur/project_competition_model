#ifndef PHASE_PHASE_NAVIGATION_H
#define PHASE_PHASE_NAVIGATION_H

#include "../DrivingControl/PoseDrivingControl.h"
#include "../AppliedHardware/EnvironmentSensor/EnvironmentViewer.h"
#include "../AppliedHardware/VehicleHardware/DriveWheels.h"
#include "../Positioning/Localization/SelfPos.h"
#include "../Utilities/Vector2D.h"

#include "../Navigation/Navigation.h"

#include "PhaseBase.h"

namespace Phase{

	class PhaseNavigation : public PhaseBase{
		private:
			DrivingControl::PoseDrivingControl poseDrivingControl;
			Navigation::Navigation navigation;
			AppliedHardware::EnvironmentSensor::EnvironmentViewer* envViewer;
			Positioning::Localization::SelfPos* pos; 
			AppliedHardware::VehicleHardware::DriveWheels* driveWheels;

			char course;

		public:
			PhaseNavigation();
			void Execute() override;
			void Navigation();
			bool IsFinish(Utilities::Vector2D posSelf);

		private:
			int ReadCourse(const char* filename);
	};

}  // namespace Phase
#endif
