#ifndef BASEHARDWARE_COLOR_SENSOR_H
#define BASEHARDWARE_COLOR_SENSOR_H

#include "ev3api.h"
#include "BaseHardware/SensorBase.h"

namespace BaseHardware{
	class ColorSensor : public SensorBase{
		private:
			static ColorSensor* singletonInstance;

			sensor_port_t port;
			int whiteValue;
			int blackValue;

		public:
			ColorSensor(sensor_port_t port);
			static ColorSensor* GetInstance();

			float GetValue();
			void SetLuminanceRange(int white, int black);
			void GetRGB(int* r, int* g, int* b);
	};

}  // namespace BaseHardware
#endif
