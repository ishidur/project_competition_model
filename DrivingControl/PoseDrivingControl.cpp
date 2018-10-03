#include "PoseDrivingControl.h"

using namespace DrivingControl;

PoseDrivingControl::PoseDrivingControl():
    power(0), turn(0), poseAngle(0), isBalancing(false),
    isStopTail(false), isStopLeft(false), isStopRight(false){

}

void PoseDrivingControl::Driving(){
    turnDrivingControl.Driving(isBalancing);

    if(!isStopTail) tailControl.RotateTowardTarget();
}

void PoseDrivingControl::SetParams(float power, float turn, int poseAngle, bool isBalancing){
    this->power = power;
    turnDrivingControl.SetPower(power);

    this->turn = turn;
    turnDrivingControl.SetTurn(turn);

    if( this->poseAngle != poseAngle ){
		this->poseAngle = poseAngle;
		tailControl.SetTargetAngle(poseAngle);
    }

    this->isBalancing = isBalancing;
}

void PoseDrivingControl::SetStop(bool isStopTail, bool isStopLeft, bool isStopRight){
    if(!(this->isStopTail)&&isStopTail) tailControl.Stop();
    this->isStopTail = isStopTail;

    this->isStopLeft = isStopLeft;
    this->isStopRight = isStopRight;
    if(isStopLeft||isStopRight) turnDrivingControl.Stop(isStopLeft, isStopRight);
}
