#include "PhaseGarage.h"

#include "stdlib.h"
#include "../BaseHardware/Timer.h"
#include "../DrivingControl/PoseDrivingControl.h"
#include "../AppliedHardware/VehicleHardware/Tail.h"
#include "../AppliedHardware/VehicleHardware/PostureSensor.h"
#include "../AppliedHardware/VehicleHardware/DriveWheels.h"
#include "../Positioning/Localization/SelfPos.h"
#include "../Utilities/Vector2D.h"

using namespace Phase;
using namespace DrivingControl;
using namespace AppliedHardware::VehicleHardware;
using namespace BaseHardware;
using namespace Positioning::Localization;
using namespace Utilities;


PhaseGarage::PhaseGarage(){
    pos = SelfPos::GetInstance();
}

void PhaseGarage::Execute(){
	printf("PhaseGarage Execute\n");
    PoseDrivingControl poseDrivingControl;
    ev3_speaker_play_tone(440, 50);

    Tail* tail = Tail::GetInstance();
    Timer* timer = Timer::GetInstance();

    PostureSensor postureSensor; 
    DriveWheels* driveWheels = DriveWheels::GetInstance();

    FILE* file;
    for (int i = 0; ; i++) {
        char filename[255] = {};
        sprintf(filename, "/ev3rt/res/log_data_endstop_%03d.csv", i);
        file = fopen(filename, "r");    // ファイルが無いときNULL
        if (file == NULL) {
            fclose(file);
            // ファイルを作成する
            file = fopen(filename, "w");
            break;
        }
        fclose(file);
    }
    fprintf(file,"t,gyroSensor,angleLeft,angleRight,pwmLeft,pwmRight,posSelf.x,posSelf.y,thetaSelf\n");

    int frameCount = 0;
    const int log_refleshrate = 15;

    float angleLeft=0.0, angleRight=0.0;
    signed char pwmLeft=0, pwmRight=0;

    timer->Reset();

    Vector2D startPos = pos->GetSelfPos();
    Vector2D posSelf(0,0);
    float thetaSelf=0.0;
    
    pos->UpdateSelfPos();
    posSelf = pos->GetSelfPos();
    thetaSelf = pos->GetTheta();
    poseDrivingControl.SetStop(false,false,false);

    // 起き上がり
    while(true){
        if (abs(tail->GetAngle() - 75) < 2) {
            poseDrivingControl.SetStop(true,true,true);
            tslp_tsk(1000);
            poseDrivingControl.SetStop(false,false,false);
            break;
        }
        poseDrivingControl.SetParams(-30,0,75,false);
        poseDrivingControl.Driving();

        pos->UpdateSelfPos();
        posSelf = pos->GetSelfPos();
        thetaSelf = pos->GetTheta();

        if ((frameCount++) % log_refleshrate == 0) {
            driveWheels->GetAngles(&angleLeft, &angleRight);
            driveWheels->GetPWMs(&pwmLeft, &pwmRight);
            fprintf(file,"%f,%f,%lf,%lf,%d,%d,%lf,%lf,%lf\n",
                timer->Now(),postureSensor.GetAnglerVelocity(), angleLeft, angleRight, pwmLeft, pwmRight, posSelf.x, posSelf.y, thetaSelf);
        }
        
        tslp_tsk(4);
    }
    // 前進
    int tau = 1000 + timer->Now();
    while (timer->Now() < tau) {
        poseDrivingControl.SetParams(30,0,75,true);
        poseDrivingControl.Driving();

        pos->UpdateSelfPos();
        posSelf = pos->GetSelfPos();
        thetaSelf = pos->GetTheta();

        if ((frameCount++) % log_refleshrate == 0) {
            driveWheels->GetAngles(&angleLeft, &angleRight);
            driveWheels->GetPWMs(&pwmLeft, &pwmRight);
            fprintf(file,"%f,%f,%lf,%lf,%d,%d,%lf,%lf,%lf\n",
                timer->Now(),postureSensor.GetAnglerVelocity(), angleLeft, angleRight, pwmLeft, pwmRight, posSelf.x, posSelf.y, thetaSelf);
        }
        tslp_tsk(4);
    }
    // 停止
    tau = 1000 + timer->Now();
    while (timer->Now() < tau) {
        poseDrivingControl.SetParams(0,0,75,true);
        poseDrivingControl.Driving();

        pos->UpdateSelfPos();
        posSelf = pos->GetSelfPos();
        thetaSelf = pos->GetTheta();

        if ((frameCount++) % log_refleshrate == 0) {
            driveWheels->GetAngles(&angleLeft, &angleRight);
            driveWheels->GetPWMs(&pwmLeft, &pwmRight);
            fprintf(file,"%f,%f,%lf,%lf,%d,%d,%lf,%lf,%lf\n",
                timer->Now(),postureSensor.GetAnglerVelocity(), angleLeft, angleRight, pwmLeft, pwmRight, posSelf.x, posSelf.y, thetaSelf);
        }
        
        tslp_tsk(4);
    }
	finFlg = true;
	printf("PhaseGarage Execute done\n");
}

bool PhaseGarage::IsFinish(){
	return false;
}