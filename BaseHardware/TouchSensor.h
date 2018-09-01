#ifndef BASEHARDWARE_TOUCH_SENSOR_H
#define BASEHARDWARE_TOUCH_SENSOR_H

#include "ev3api.h"
#include "BaseHardware/SensorBase.h"

namespace BaseHardware{
	class TouchSensor : public SensorBase{
		private:
			static TouchSensor* singletonInstance;

			sensor_port_t port;

		public:
			TouchSensor(sensor_port_t port);
			static TouchSensor* GetInstance();

			float GetValue();
	};

}  // namespace BaseHardware
#endif
