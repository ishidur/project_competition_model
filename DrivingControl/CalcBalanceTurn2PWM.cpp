#include "CalcBalanceTurn2PWM.h"
#include "../Balancer/balancer.h"

using namespace DrivingControl;
using namespace AppliedHardware::VehicleHardware;

CalcBalanceTurn2PWM::CalcBalanceTurn2PWM(){
    driveWheels = DriveWheels::GetInstance();
}

void CalcBalanceTurn2PWM::CalcPWMValue(){}
void CalcBalanceTurn2PWM::CalcPWMValue(float power, float turn, signed char* leftPWM, signed char* rightPWM){

    float ang_vel    = postureSensor.GetAnglerVelocity();
    float ang_offset = postureSensor.GetGyroOffset();
    float volt       = powerSource.GetVoltage();
    
    float rotLeft, rotRight;
    driveWheels->GetAngles(&rotLeft, &rotRight);

    BacklashCancel(*leftPWM, *rightPWM, &rotLeft, &rotRight);

    balance_control(power, turn, ang_vel, ang_offset, rotLeft, rotRight, volt, leftPWM, rightPWM);
}

void CalcBalanceTurn2PWM::BacklashCancel(signed char lpwm, signed char rpwm, float *lenc, float *renc){
    const int BACKLASHHALF = 4;   // バックラッシュの半分[deg]

    if(lpwm < 0) {
        *lenc += BACKLASHHALF;
    }else if(lpwm > 0) {
        *lenc -= BACKLASHHALF;
    }

    if(rpwm < 0){
        *renc += BACKLASHHALF;
    }else if(rpwm > 0){
        *renc -= BACKLASHHALF;
    }
}