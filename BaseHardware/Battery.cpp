#include "ev3api.h"
#include "Battery.h"

using namespace BaseHardware;

Battery* Battery::singletonInstance = nullptr;

Battery* Battery::GetInstance(){
	if (singletonInstance == nullptr){
		singletonInstance = new Battery();
	}
	return singletonInstance;
}

float Battery::GetValue(){
	return (float)ev3_battery_voltage_mV();
}
