#include "ev3api.h"
#include "UltraSonicSensor.h"

#define SENSOR_PORT EV3_PORT_3

using namespace BaseHardware;

UltraSonicSensor* UltraSonicSensor::singletonInstance;

UltraSonicSensor::UltraSonicSensor(sensor_port_t port) {
    this->port = port;
}

UltraSonicSensor* UltraSonicSensor::GetInstance(){
	if (singletonInstance == nullptr){
		singletonInstance = new UltraSonicSensor(SENSOR_PORT);
	}
	return singletonInstance;
}

float UltraSonicSensor::GetValue(){
	return (float)ev3_ultrasonic_sensor_get_distance(port);
}
