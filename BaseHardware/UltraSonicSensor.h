#ifndef BASEHARDWARE_ULTRA_SONIC_SENSOR_H
#define BASEHARDWARE_ULTRA_SONIC_SENSOR_H

#include "ev3api.h"
#include "BaseHardware/SensorBase.h"

namespace BaseHardware{

	class UltraSonicSensor : public SensorBase{
	private:
		static UltraSonicSensor* singletonInstance;

		sensor_port_t port;
	
	public:
		UltraSonicSensor(sensor_port_t port);
		static UltraSonicSensor* GetInstance();

		float GetValue();
	};

}  // namespace BaseHardware
#endif
