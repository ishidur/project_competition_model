#ifndef APPLIEDHARDWARE_VEHICLEHARDWARE_TAIL_H
#define APPLIEDHARDWARE_VEHICLEHARDWARE_TAIL_H

#include "../../BaseHardware/Motor.h"

namespace AppliedHardware{
	namespace VehicleHardware{
		class Tail{
			private:
				static Tail* singletonInstance;
				signed char pwm;

				BaseHardware::Motor* tailMotor;

			public:
				static Tail* GetInstance();
				void SetPWM(signed char pwm);
				float GetAngle();
				signed char GetPWM();
				void Stop();

			private:
				Tail();
		};

	}  // namespace VehicleHardware
}  // namespace AppliedHardware
#endif
