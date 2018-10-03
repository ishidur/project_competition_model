#include "TailControl.h"

using namespace DrivingControl;
using namespace AppliedHardware::VehicleHardware;
using namespace Utilities;

TailControl::TailControl(){
    tail = Tail::GetInstance();
    pidCalc = new PIDCalculation("/ev3rt/res/course/pid_params_tail.txt");
}

void TailControl::SetTargetAngle(int targetAngle){
    this->targetAngle = targetAngle;
    pidCalc->PIDReStart();//ターゲットを更新するので再スタート
}

void TailControl::RotateTowardTarget(){
    float pwm = pidCalc->GetPIDValue(tail->GetAngle(), targetAngle);
	if(pwm>100) pwm = 100; else if(pwm<-100) pwm = -100;
    tail->SetPWM((float)pwm);
}

void TailControl::SetPWM(int pwm){
        tail->SetPWM((float)pwm);
}

void TailControl::Stop(){
    tail->Stop();
}
