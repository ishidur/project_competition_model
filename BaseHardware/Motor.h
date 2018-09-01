#ifndef BASEHARDWARE_MOTOR_H
#define BASEHARDWARE_MOTOR_H

#include "ev3api.h"
#include "BaseHardware/SensorBase.h"

namespace BaseHardware{

	class Motor : public SensorBase{
		private:
			static Motor* singletonInstances[3];
			
			motor_port_t port;			
			signed char pwm;

		public:
			Motor(int index);
			static Motor* GetInstance(int index);

			float GetValue() override;
			void SetPWM(signed char pwm);
			signed char GetPWM();
			void Stop();
	};

}  // namespace BaseHardware
#endif
