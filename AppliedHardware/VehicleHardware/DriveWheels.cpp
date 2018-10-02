#include "DriveWheels.h"

using namespace AppliedHardware::VehicleHardware;
using namespace BaseHardware;

DriveWheels* DriveWheels::singletonInstance = nullptr;
const float DriveWheels::tireRadius = 49.0/10.0*1.035;
const float DriveWheels::tireDistance = 175.0/10.0*1.05;

DriveWheels::DriveWheels():pwmRight(0),pwmLeft(0){
	motorLeft = Motor::GetInstance(2);
	motorRight = Motor::GetInstance(1);
}

DriveWheels* DriveWheels::GetInstance(){
	if (singletonInstance == nullptr){
		singletonInstance = new DriveWheels();
	}
	return singletonInstance;
}

void DriveWheels::SetPWMs(signed char pwmLeft, signed char pwmRight){
	motorLeft->SetPWM(pwmLeft);
	motorRight->SetPWM(pwmRight);

	this->pwmLeft = pwmLeft;
	this->pwmRight = pwmRight;
}

void DriveWheels::Stop(bool isStopLeft, bool isStopRight){
	if(isStopLeft)  motorLeft->Stop();
	if(isStopRight) motorRight->Stop();
}

void DriveWheels::GetAngles(float* angleLeft, float* angleRight){
	*angleLeft = motorLeft->GetValue();
	*angleRight = motorRight->GetValue();
}

void DriveWheels::GetPWMs(signed char* pwmLeft, signed char* pwmRight){
	*pwmLeft = this->pwmLeft;
	*pwmRight = this->pwmRight;
}