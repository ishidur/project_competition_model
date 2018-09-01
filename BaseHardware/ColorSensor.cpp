#include "ev3api.h"
#include "ColorSensor.h"

#define SENSOR_PORT EV3_PORT_2

using namespace BaseHardware;

ColorSensor* ColorSensor::singletonInstance = nullptr;

ColorSensor::ColorSensor(sensor_port_t port) : whiteValue(100), blackValue(0){
    this->port = port;
}

ColorSensor* ColorSensor::GetInstance(){
	if (singletonInstance == nullptr){
		singletonInstance = new ColorSensor(SENSOR_PORT);
	}
	return singletonInstance;
}

float ColorSensor::GetValue(){
	float value = (float)ev3_color_sensor_get_reflect(this->port);

	// min_max_normalization
    value = (value - (float)blackValue)/(float)(whiteValue - blackValue) * 100;

    // clipping (0.0 ~ 100.0)
    if(value < 0.0) value = 0.0; else if(value > 100.0) value = 100.0;

    return value;
}

void ColorSensor::SetLuminanceRange(int white, int black){
	this->whiteValue = white;
	this->blackValue = black;
}
