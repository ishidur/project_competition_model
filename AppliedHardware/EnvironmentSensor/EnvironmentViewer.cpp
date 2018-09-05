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


void EnvironmentViewer::GetRGB(int* r, int* g, int* b){
	colorSensor->GetRGB(r, g, b);
}

float EnvironmentViewer::GetUSDistance(){
	return ultraSonicSensor->GetValue();
}

bool EnvironmentViewer::GetTouch(){
	return (bool)(touchSensor->GetValue());
}