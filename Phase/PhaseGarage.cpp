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


    char filename[255] = {};
    sprintf(filename, "/ev3rt/res/log_data_endstopcsv.csv");
    file = fopen(filename, "w");
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
//    while(posSelf.DistanceFrom(startPos)<56){
      while(posSelf.DistanceFrom(startPos)<20){

    line.CalcTurnValueByRGB();//CalcTurnValue();
        turn = -1.0*line.GetTurn();
//        poseDrivingControl.SetParams(20,turn,63,false);
        poseDrivingControl.SetParams(20,0,63,false);
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

    int tail_ang_sum_cnt = 5;
    float now_tail_ang = 0;
    float pre_tail_ang = now_tail_ang*tail_ang_sum_cnt;

    int tail_pwm = 0;
    float forward = -10;
    float tail_update_cnt = 0;
    // 起き上がり
    while(true){
    	now_tail_ang += tail->GetAngle();
        if (abs(tail->GetAngle() - 70) < 3) {
            break;
        }
        tail_update_cnt++;
		if( tail_update_cnt <= tail_ang_sum_cnt ){
		}else{
	        if( now_tail_ang <= pre_tail_ang ){
	        	tail_pwm++;
	        	forward = -10;
	        }else if( now_tail_ang > pre_tail_ang + 1 ){
	        	tail_pwm--;
	        	forward = 0;
	        }

			tail_update_cnt = 0;
			pre_tail_ang = now_tail_ang;
			now_tail_ang = 0;
		}
//        poseDrivingControl.SetParams(-10,0,70,false);
        poseDrivingControl.SetParamsTailPWM(forward,0,tail_pwm,false);

        poseDrivingControl.Driving();

        pos->UpdateSelfPos();
        posSelf = pos->GetSelfPos();
        thetaSelf = pos->GetTheta();

        if ((frameCount++) % log_refleshrate == 0) {
            driveWheels->GetAngles(&angleLeft, &angleRight);
            driveWheels->GetPWMs(&pwmLeft, &pwmRight);
            fprintf(file,"%f,%f,%d,%f,%lf,%lf,%d,%d,%lf,%lf,%lf\n",
                timer->Now(),postureSensor.GetAnglerVelocity(), tail->GetPWM(),tail->GetAngle(),angleLeft, angleRight, pwmLeft, pwmRight, posSelf.x, posSelf.y, thetaSelf);
        }
        tslp_tsk(4);
    }

    poseDrivingControl.SetStop(true,true,true);
    tslp_tsk(1000); // タイヤ完全停止待機
    poseDrivingControl.SetStop(false,false,false);
    now_tail_ang = tail->GetAngle();
    pre_tail_ang = now_tail_ang;

    while(true){
    	now_tail_ang = tail->GetAngle();

    	if (abs(tail->GetAngle() - 80) < 3) {
            break;
        }
#if 0
    	if( now_tail_ang <= pre_tail_ang ){
        	forward = -10;
        }else{
        	forward = 0;
        }
        pre_tail_ang = now_tail_ang;
#endif
    	poseDrivingControl.SetParams(0,0,80,false);
//        poseDrivingControl.SetParams(forward,0,80,false);

    	poseDrivingControl.Driving();

        pos->UpdateSelfPos();
        posSelf = pos->GetSelfPos();
        thetaSelf = pos->GetTheta();

        if ((frameCount++) % log_refleshrate == 0) {
            driveWheels->GetAngles(&angleLeft, &angleRight);
            driveWheels->GetPWMs(&pwmLeft, &pwmRight);
            fprintf(file,"%f,%f,%d,%f,%lf,%lf,%d,%d,%lf,%lf,%lf\n",
                timer->Now(),postureSensor.GetAnglerVelocity(),tail->GetPWM(),tail->GetAngle(), angleLeft, angleRight, pwmLeft, pwmRight, posSelf.x, posSelf.y, thetaSelf);
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
