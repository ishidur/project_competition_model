#include "PhaseGarage.h"

#include "stdlib.h"
#include "../BaseHardware/Timer.h"
#include "../DrivingControl/PoseDrivingControl.h"
#include "../AppliedHardware/VehicleHardware/Tail.h"
#include "../AppliedHardware/VehicleHardware/PostureSensor.h"
#include "../AppliedHardware/VehicleHardware/DriveWheels.h"
#include "../Positioning/Localization/SelfPos.h"
#include "../Utilities/Vector2D.h"
#include "../EnvironmentMeasurement/LineLuminance.h"

using namespace Phase;
using namespace DrivingControl;
using namespace AppliedHardware::VehicleHardware;
using namespace AppliedHardware::EnvironmentSensor;
using namespace BaseHardware;
using namespace Positioning::Localization;
using namespace Utilities;
using namespace EnvironmentMeasurement;


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
    EnvironmentViewer* envViewer = EnvironmentViewer::GetInstance();
    DriveWheels* driveWheels = DriveWheels::GetInstance();

    LineLuminance line;

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
    fprintf(file,"t,gyroSensor,angleLeft,angleRight,pwmLeft,pwmRight,posSelf.x,posSelf.y,thetaSelf,r,g,b\n");

    int frameCount = 0;
    const int log_refleshrate = 15;

    float angleLeft=0.0, angleRight=0.0;
    signed char pwmLeft=0, pwmRight=0;

    float turn;
    int r, g, b;

    timer->Reset();

    Vector2D startPos = pos->GetSelfPos();
    Vector2D posSelf(0,0);
    float thetaSelf=0.0;
    
    pos->UpdateSelfPos();
    posSelf = pos->GetSelfPos();
    thetaSelf = pos->GetTheta();
    poseDrivingControl.SetStop(false,false,false);

    // ガレージに入るとこまで
    while(posSelf.DistanceFrom(startPos)<60){
        line.CalcTurnValueByRGB();//CalcTurnValue();
        turn = -1.0*line.GetTurn();
        poseDrivingControl.SetParams(20,turn,63,false);
        poseDrivingControl.Driving();

        pos->UpdateSelfPos();
        posSelf = pos->GetSelfPos();
        thetaSelf = pos->GetTheta();

        if ((frameCount++) % log_refleshrate == 0) {
            driveWheels->GetAngles(&angleLeft, &angleRight);
            driveWheels->GetPWMs(&pwmLeft, &pwmRight);
            envViewer->GetRGB(&r, &g, &b);
            fprintf(file,"%f,,%f,%f,%f,%d,%f,%d,%d,%f,%f,%f,%f,%f,%d,%d,%d\n",
                timer->GetValue(), postureSensor.GetAnglerVelocity(),
                20.0,turn,
                tail->GetPWM(),tail->GetAngle(),
                pwmLeft, pwmRight, angleLeft, angleRight,
                posSelf.x, posSelf.y, thetaSelf, r, g, b);
        }

        tslp_tsk(4);
    }
    poseDrivingControl.SetStop(true,true,true);
    tslp_tsk(1000); // タイヤ完全停止待機
    poseDrivingControl.SetStop(false,false,false);

    // 起き上がり
    while(true){
        if (abs(tail->GetAngle() - 70) < 3) {
            break;
        }
        poseDrivingControl.SetParams(-10,0,75,false);
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

    poseDrivingControl.SetStop(true,true,true);
    tslp_tsk(1000); // タイヤ完全停止待機
    poseDrivingControl.SetStop(false,false,false);
    while(true){
        if (abs(tail->GetAngle() - 80) < 3) {
            break;
        }
        poseDrivingControl.SetParams(0,0,80,false);
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
    poseDrivingControl.SetStop(true,true,true);
    
    tslp_tsk(1000); // タイヤ完全停止待機
	finFlg = true;
	printf("PhaseGarage Execute done\n");
}

bool PhaseGarage::IsFinish(){
	return false;
}