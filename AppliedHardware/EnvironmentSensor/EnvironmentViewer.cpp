#include "EnvironmentViewer.h"

using namespace AppliedHardware::EnvironmentSensor;
using namespace BaseHardware;

EnvironmentViewer* EnvironmentViewer::singletonInstance = nullptr;

EnvironmentViewer::EnvironmentViewer(){
	colorSensor = ColorSensor::GetInstance();
	ultraSonicSensor = UltraSonicSensor::GetInstance();
	touchSensor = TouchSensor::GetInstance();
}

EnvironmentViewer* EnvironmentViewer::GetInstance(){
	if (singletonInstance == nullptr){
		singletonInstance = new EnvironmentViewer();
	}
	return singletonInstance;
}

float EnvironmentViewer::GetLuminance(){
	return colorSensor->GetValue();
}

float EnvironmentViewer::GetUSDistance(){
	return ultraSonicSensor->GetValue();
}

bool EnvironmentViewer::GetTouch(){
	return (bool)(touchSensor->GetValue());
}