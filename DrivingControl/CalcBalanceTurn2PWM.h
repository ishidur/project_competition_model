#ifndef DRIVINGCONTROL_CALC_BALANCE_TURN2_P_W_M_H
#define DRIVINGCONTROL_CALC_BALANCE_TURN2_P_W_M_H

#include "AppliedHardware/VehicleHardware/PostureSensor.h"
#include "AppliedHardware/VehicleHardware/PowerSource.h"
#include "AppliedHardware/VehicleHardware/DriveWheels.h"
#include "../DrivingControl/CalcPWM.h"

namespace DrivingControl{

	class CalcBalanceTurn2PWM : public CalcPWM{
		private:
			AppliedHardware::VehicleHardware::PostureSensor postureSensor;
			AppliedHardware::VehicleHardware::PowerSource powerSource;
			AppliedHardware::VehicleHardware::DriveWheels* driveWheels;			

		public:
			CalcBalanceTurn2PWM();
			void CalcPWMValue() override;
			void CalcPWMValue(float power, float turn, signed char* leftPWM, signed char* rightPWM);

		private:
			void BacklashCancel(signed char lpwm, signed char rpwm, float *lenc, float *renc);
	};

}  // namespace DrivingControl
#endif
