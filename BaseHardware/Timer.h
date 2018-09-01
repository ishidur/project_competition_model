#ifndef BASEHARDWARE_TIMER_H
#define BASEHARDWARE_TIMER_H

#include "ev3api.h"
#include "Clock.h"

#include "BaseHardware/SensorBase.h"

namespace BaseHardware{
	
	class Timer : public SensorBase{
		private:
			static Timer* singletonInstance;

			ev3api::Clock* cl;
			float resetTime;

		public:
			Timer();
			static Timer* GetInstance();

			float GetValue();
			float Now();
			void  Reset();
	};

}  // namespace BaseHardware
#endif
