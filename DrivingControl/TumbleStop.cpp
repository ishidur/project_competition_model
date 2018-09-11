#include "TumbleStop.h"
#include <stdlib.h> 

using namespace DrivingControl;
using namespace AppliedHardware::VehicleHardware;

TumbleStop::TumbleStop() : count(0){
    tail = Tail::GetInstance();
    driveWheels = DriveWheels::GetInstance();
}

void TumbleStop::TumbleStopTask(){
    static bool first_full_flg = false;
    if(count==TUNBLE_MEMORY_SIZE-1 && !first_full_flg) first_full_flg = true;

    anglerVelocityMemory[count] = postureSensor.GetAnglerVelocity();
    driveWheels->GetPWMs(&pwmLeft, &pwmRight);
    averagePWMMemory[count] = ((float)pwmLeft + (float)pwmRight)/2.0;
    count = (count+1)%TUNBLE_MEMORY_SIZE;

    if(first_full_flg){
        aveAnglerVel = 0.0;
        avePWM = 0.0;
        for(int i=0; i<TUNBLE_MEMORY_SIZE; i++){
            aveAnglerVel += anglerVelocityMemory[i];
            avePWM += averagePWMMemory[i];
        }
        aveAnglerVel /= (float)TUNBLE_MEMORY_SIZE;
        avePWM /= (float)TUNBLE_MEMORY_SIZE;

        if(abs(aveAnglerVel)<20.0 && abs(avePWM)==100.0){
            driveWheels->Stop(true,true);
            tail->Stop();

            ter_tsk(PHASE_TASK);
            wup_tsk(MAIN_TASK);
        }
    }
}