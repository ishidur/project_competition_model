#ifndef APPLIEDHARDWARE_VEHICLEHARDWARE_TAIL_H
#define APPLIEDHARDWARE_VEHICLEHARDWARE_TAIL_H

#include "../../BaseHardware/Motor.h"
#include "../../Balancer/balancer_private.h"
#include "PowerSource.h"

namespace AppliedHardware{
	namespace VehicleHardware{
		class Tail{
			private:
				static Tail* singletonInstance;
				signed char pwm;

				BaseHardware::Motor* tailMotor;
				PowerSource powerSource;

			public:
				static Tail* GetInstance();
				void SetPWM(float pwm);
				float GetAngle();
				signed char GetPWM();
				void Stop();

			private:
				Tail();
		};

	}  // namespace VehicleHardware
}  // namespace AppliedHardware
#endif
