#ifndef DRIVINGCONTROL_CALC_PWM_H
#define DRIVINGCONTROL_CALC_PWM_H

namespace DrivingControl{
	class CalcPWM {
		public:
			virtual void CalcPWMValue()=0;
	};

}  // namespace DrivingControl
#endif
