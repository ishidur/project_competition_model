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

void Tail::SetPWM(signed char pwm){
	tailMotor->SetPWM(pwm);
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