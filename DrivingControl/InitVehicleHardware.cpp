#include "ev3api.h"
#include "stdlib.h"
#include "string.h"

#include "InitVehicleHardware.h"

using namespace DrivingControl;
using namespace AppliedHardware::EnvironmentSensor;

InitVehicleHardware::InitVehicleHardware(){
    environmentViewer = EnvironmentViewer::GetInstance();
}

void InitVehicleHardware::Initialize(){}

void InitVehicleHardware::Calibration(){
    while(environmentViewer->GetTouch());   // 離されるまで

    tailControl.SetTargetAngle(85);
    while(true) { // 押されるまで待機
        tailControl.RotateTowardTarget();
        // if(bt!=NULL){
        //     char mes[255] = {0};
        //     bt->GetMessage(mes);
        //     if(touchSensor->IsPressed() || mes[0]=='s') break;
        // }else{
            if(environmentViewer->GetTouch())break;
        // }
    }
    
    tailControl.Stop();
    tslp_tsk(1000);

    postureSensor.Calibration();
}