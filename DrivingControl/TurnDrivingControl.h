#ifndef DRIVINGCONTROL_TURN_DRIVING_CONTROL_H
#define DRIVINGCONTROL_TURN_DRIVING_CONTROL_H

#include "AppliedHardware/VehicleHardware/DriveWheels.h"
#include "DrivingControl/CalcTurn2PWM.h"
#include "DrivingControl/CalcBalanceTurn2PWM.h"

namespace DrivingControl{
	class TurnDrivingControl{
		private:
			float power;
			float turn;

			AppliedHardware::VehicleHardware::DriveWheels* driveWheels;
			CalcTurn2PWM calcTurn2PWM;
			CalcBalanceTurn2PWM calcBalanceTurn2PWM;

		public:
			TurnDrivingControl();
			void Driving(bool isBalancing);
			void TurnControl();
			void BalanceControl();
			void Stop(bool isStopLeft, bool isStopRight);

			void SetPower(float power);
			void SetTurn(float turn);
	};

}  // namespace DrivingControl
#endif
