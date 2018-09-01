#include "Motor.h"

using namespace BaseHardware;

Motor* Motor::singletonInstances[3] = {nullptr, nullptr, nullptr};

Motor::Motor(int portIndex){
	switch(portIndex){
		case 0:
			this->port = EV3_PORT_A;
			break;
		case 1:
			this->port = EV3_PORT_B;
			break;
		case 2:
			this->port = EV3_PORT_C;
			break;
		case 3:
			this->port = EV3_PORT_D;
			break;
	}
}

Motor* Motor::GetInstance(int index){
	if (singletonInstances[index] == nullptr){
		singletonInstances[index] = new Motor(index);
	}
	return singletonInstances[index];
}

float Motor::GetValue(){
    return (float)ev3_motor_get_counts(port);
}

void Motor::SetPWM(signed char pwm){
    this->pwm = pwm;
	if(pwm>100) pwm = 100; else if(pwm<-100) pwm = -100;
    ev3_motor_set_power(port, pwm);
}

signed char Motor::GetPWM(){
	return pwm;
}

void Motor::Stop(){
    ev3_motor_stop(port, true);	
}
