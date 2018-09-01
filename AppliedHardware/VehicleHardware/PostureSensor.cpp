#include "PostureSensor.h"

using namespace AppliedHardware::VehicleHardware;
using namespace BaseHardware;

PostureSensor::PostureSensor(){
	gyro = GyroSensor::GetInstance();
	calibrationGyro = new CalibrationGyro();
}

float PostureSensor::GetAnglerVelocity(){
	return gyro->GetValue();
}

float PostureSensor::GetGyroOffset(){
	return gyro->GetOffset();
}

void PostureSensor::Calibration(){
	calibrationGyro->SetGyroOffset();
}