#ifndef BASEHARDWARE_GYRO_SENSOR_H
#define BASEHARDWARE_GYRO_SENSOR_H

#include "ev3api.h"
#include "BaseHardware/SensorBase.h"

namespace BaseHardware{
	class GyroSensor : public SensorBase{
		private:
			static GyroSensor* singletonInstance;

			sensor_port_t port;
			float offset;

		public:
			GyroSensor(sensor_port_t port);
			static GyroSensor* GetInstance();

			float GetValue();
			void SetOffset(float offset);
			float GetOffset();
	};

}  // namespace BaseHardware
#endif
