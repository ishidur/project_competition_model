#include "ev3api.h"
#include "GyroSensor.h"

#define SENSOR_PORT EV3_PORT_4

using namespace BaseHardware;

GyroSensor* GyroSensor::singletonInstance = nullptr;

GyroSensor::GyroSensor(sensor_port_t port) {
    this->port = port;
}

GyroSensor* GyroSensor::GetInstance(){
	if (singletonInstance == nullptr){
		singletonInstance = new GyroSensor(SENSOR_PORT);
	}
	return singletonInstance;
}

float GyroSensor::GetValue(){
    return (float)ev3_gyro_sensor_get_rate(port);
}

void GyroSensor::SetOffset(float offset){
	this->offset = offset;
}

float GyroSensor::GetOffset(){
	return offset;
}

