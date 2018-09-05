#include "TumbleStop.h"

using namespace DrivingControl;
using namespace AppliedHardware::VehicleHardware;

TumbleStop::TumbleStop() : count(0){
    driveWheels = DriveWheels::GetInstance();
}

void TumbleStop::TumbleStopTask(){
    
}