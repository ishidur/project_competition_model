#include "ev3api.h"
#include "TouchSensor.h"

#define SENSOR_PORT EV3_PORT_1

using namespace BaseHardware;

TouchSensor* TouchSensor::singletonInstance = nullptr;

TouchSensor::TouchSensor(sensor_port_t port) {
    this->port = port;
}

TouchSensor* TouchSensor::GetInstance(){
	if (singletonInstance == nullptr){
		singletonInstance = new TouchSensor(SENSOR_PORT);
	}
	return singletonInstance;
}

float TouchSensor::GetValue(){
	return (float)ev3_touch_sensor_is_pressed(port);
}
