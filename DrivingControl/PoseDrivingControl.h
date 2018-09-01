#ifndef DRIVINGCONTROL_POSE_DRIVING_CONTROL_H
#define DRIVINGCONTROL_POSE_DRIVING_CONTROL_H

#include "DrivingControl/TurnDrivingControl.h"
#include "DrivingControl/TailControl.h"

namespace DrivingControl{
	
	class PoseDrivingControl{
		private:
			float power;
			float turn;
			int poseAngle;
			bool isBalancing;

			bool isStopTail, isStopLeft, isStopRight;

			TurnDrivingControl turnDrivingControl;
			TailControl tailControl;

		public:
			PoseDrivingControl();
			void Driving();
			void SetParams(float power, float turn, int poseAngle, bool isBalancing);
			void SetStop(bool isStopTail, bool isStopLeft, bool isStopRight);

	};

}  // namespace DrivingControl
#endif
