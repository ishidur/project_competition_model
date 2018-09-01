#ifndef DRIVINGCONTROL_TAIL_CONTROL_H
#define DRIVINGCONTROL_TAIL_CONTROL_H

#include "../AppliedHardware/VehicleHardware/Tail.h"
#include "../Utilities/PIDCalculation.h"

namespace DrivingControl{

	class TailControl{
		private:
			int targetAngle;

			AppliedHardware::VehicleHardware::Tail* tail;
			Utilities::PIDCalculation* pidCalc;

		public:
			TailControl();
			void SetTargetAngle(int targetAngle);
			void RotateTowardTarget();
			void Stop();
	};

}  // namespace DrivingControl
#endif
