#include "TurnDrivingControl.h"

using namespace DrivingControl;
using namespace AppliedHardware::VehicleHardware;

TurnDrivingControl::TurnDrivingControl(){
    driveWheels = DriveWheels::GetInstance();
}

void TurnDrivingControl::Driving(bool isBalancing){
    if(isBalancing){
        BalanceControl();
    }else{
        TurnControl();
    }
}

void TurnDrivingControl::TurnControl(){
    signed char leftPWM, rightPWM;
    calcTurn2PWM.CalcPWMValue(power, turn, &leftPWM, &rightPWM);
    driveWheels->SetPWMs(leftPWM, rightPWM);
}

void TurnDrivingControl::BalanceControl(){
    signed char leftPWM, rightPWM;
    calcBalanceTurn2PWM.CalcPWMValue(power, turn, &leftPWM, &rightPWM);
    driveWheels->SetPWMs(leftPWM, rightPWM);
}

void TurnDrivingControl::Stop(bool isStopLeft, bool isStopRight){
    driveWheels->Stop(isStopLeft, isStopRight);
}


void TurnDrivingControl::SetPower(float power){
    this->power = power;
}

void TurnDrivingControl::SetTurn(float turn){
    this->turn = turn;
}