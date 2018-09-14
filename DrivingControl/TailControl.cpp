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
    tail->SetPWM((int)pwm);
}

void TailControl::Stop(){
    tail->Stop();
}
