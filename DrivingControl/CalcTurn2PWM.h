#ifndef DRIVINGCONTROL_CALC_TURN2_P_W_M_H
#define DRIVINGCONTROL_CALC_TURN2_P_W_M_H

#include "../DrivingControl/CalcPWM.h"

namespace DrivingControl{
	
	class CalcTurn2PWM : public CalcPWM	{
		public:
			CalcTurn2PWM();
			void CalcPWMValue() override;
			void CalcPWMValue(float power, float turn, signed char* leftPWM, signed char* rightPWM);
	};

}  // namespace DrivingControl
#endif
