#include "Tail.h"

using namespace AppliedHardware::VehicleHardware;
using namespace BaseHardware;

Tail* Tail::singletonInstance = nullptr;

Tail::Tail(){
	tailMotor = Motor::GetInstance(0);
}

Tail* Tail::GetInstance(){
	if (singletonInstance == nullptr){
		singletonInstance = new Tail();
	}
	return singletonInstance;
}

void Tail::SetPWM(float pwm){
    // バッテリーの電圧値で補正？
    float corrected_pwm = pwm * 10.0F / ((BATTERY_GAIN * powerSource.GetVoltage()) - BATTERY_OFFSET);
	tailMotor->SetPWM((signed char) corrected_pwm);
}

float Tail::GetAngle(){
	return tailMotor->GetValue();
}

signed char Tail::GetPWM(){
	return tailMotor->GetPWM();
}

void Tail::Stop(){
	tailMotor->Stop();
}