#ifndef BASEHARDWARE_BATTERY_H
#define BASEHARDWARE_BATTERY_H

#include "BaseHardware/SensorBase.h"

namespace BaseHardware{
	
	class Battery : public SensorBase{
		private:
			static Battery* singletonInstance;

		public:
			static Battery* GetInstance();

			float GetValue();	
	};

}  // namespace BaseHardware
#endif
