#include "PhaseLookUpGate.h"

#include "stdlib.h"
#include "../BaseHardware/Timer.h"
#include "../DrivingControl/PoseDrivingControl.h"
#include "../AppliedHardware/VehicleHardware/Tail.h"
#include "../AppliedHardware/VehicleHardware/PostureSensor.h"
#include "../AppliedHardware/VehicleHardware/DriveWheels.h"
#include "../AppliedHardware/EnvironmentSensor/EnvironmentViewer.h"
#include "../Positioning/Localization/SelfPos.h"
#include "../Utilities/Vector2D.h"
#include "../EnvironmentMeasurement/LineLuminance.h"

using namespace Phase;
using namespace	DrivingControl;
using namespace AppliedHardware::VehicleHardware;
using namespace AppliedHardware::EnvironmentSensor;
using namespace BaseHardware;
using namespace Positioning::Localization;
using namespace Utilities;
using namespace EnvironmentMeasurement;

PhaseLookUpGate::PhaseLookUpGate(){
	pos = SelfPos::GetInstance();
    // ログ用ファイル生成
    char filename[255] = {};
    sprintf(filename, "/ev3rt/res/log_data_lookupgate.csv");
    file = fopen(filename, "w");
    fprintf(file,"timer,caribratedBrightness,gyroSensor,power,turn,PWMt,motor_ang_t,PWMl,PWMr,motor_ang_l,motor_ang_r,xEst,yEst,thetaSelf,r,g,b\n");
}

void PhaseLookUpGate::Execute(){
	printf("PhaseLookUpGate Execute\n");

	PoseDrivingControl poseDrivingControl;

	Tail* tail = Tail::GetInstance();
    Timer* timer = Timer::GetInstance();

	PostureSensor postureSensor; 
	EnvironmentViewer* envViewer = EnvironmentViewer::GetInstance();
	DriveWheels* driveWheels = DriveWheels::GetInstance();

	LineLuminance line;

    int frameCount = 0;
    const int log_refleshrate = 15;

    // 初期化
	float angleLeft=0.0, angleRight=0.0;
	signed char pwmLeft=0, pwmRight=0;

	float turn;
    int r, g, b;

    Vector2D startPos = pos->GetSelfPos();
    Vector2D posSelf(0,0);
    float thetaSelf=0.0;

    timer->Reset();
    // 1. ルックアップゲートに入るところ：倒立振子から尻尾着地
    printf("PhaseLookUpGate 1.Deceleration to Land\n");
    int tau = 500;	
	poseDrivingControl.SetParams(-50.0,0,75,true);
    while (true) {
        if (timer->Now()>tau) {
            break;
        }   

		poseDrivingControl.Driving();

		pos->UpdateSelfPos();
		posSelf = pos->GetSelfPos();
		thetaSelf = pos->GetTheta();

        if ((frameCount++) % log_refleshrate == 0) {
            driveWheels->GetAngles(&angleLeft, &angleRight);
            driveWheels->GetPWMs(&pwmLeft, &pwmRight);
            fprintf(file,"%f,%f,%f,%f,%f,%d,%f,%d,%d,%f,%f,%f,%f,%f\n",
                timer->GetValue(), envViewer->GetLuminance(), postureSensor.GetAnglerVelocity(),
                -50.0,0.0,
                tail->GetPWM(),tail->GetAngle(),
                pwmLeft, pwmRight, angleLeft, angleRight,
                posSelf.x, posSelf.y, thetaSelf);
        }
        
        tslp_tsk(4);
    }
    // poseDrivingControl.SetParams(0,0,70,false);
    // poseDrivingControl.SetStop(true,false,false);
    // poseDrivingControl.Driving();

    printf("PhaseLookUpGate 1.5.Landing\n");	
	poseDrivingControl.SetParams(0,0,75,true);
    while(true){
        if (abs(tail->GetAngle() - 75) <= 2) {
            break;
        }

		poseDrivingControl.Driving();

		pos->UpdateSelfPos();
		posSelf = pos->GetSelfPos();
		thetaSelf = pos->GetTheta();

        if ((frameCount++) % log_refleshrate == 0) {
            driveWheels->GetAngles(&angleLeft, &angleRight);
            driveWheels->GetPWMs(&pwmLeft, &pwmRight);
            fprintf(file,"%f,%f,%f,%f,%f,%d,%f,%d,%d,%f,%f,%f,%f,%f\n",
                timer->GetValue(), envViewer->GetLuminance(), postureSensor.GetAnglerVelocity(),
                0.0,0.0,
                tail->GetPWM(),tail->GetAngle(),
                pwmLeft, pwmRight, angleLeft, angleRight,
                posSelf.x, posSelf.y, thetaSelf);
        }
         
        tslp_tsk(4);
    }
    poseDrivingControl.SetParams(10,0,75,false);
    poseDrivingControl.SetStop(true,true,true);
    poseDrivingControl.Driving();
    tslp_tsk(200);

    poseDrivingControl.SetParams(0,0,65,false);
    poseDrivingControl.SetStop(false,true,true);
    while(true){
        if (abs(tail->GetAngle() - 65) <= 2) {
            break;
        }

		poseDrivingControl.Driving();

		pos->UpdateSelfPos();
		posSelf = pos->GetSelfPos();
		thetaSelf = pos->GetTheta();

        if ((frameCount++) % log_refleshrate == 0) {
            driveWheels->GetAngles(&angleLeft, &angleRight);
            driveWheels->GetPWMs(&pwmLeft, &pwmRight);
            fprintf(file,"%f,%f,%f,%f,%f,%d,%f,%d,%d,%f,%f,%f,%f,%f\n",
                timer->GetValue(), envViewer->GetLuminance(), postureSensor.GetAnglerVelocity(),
                0.0,0.0,
                tail->GetPWM(),tail->GetAngle(),
                pwmLeft, pwmRight, angleLeft, angleRight,
                posSelf.x, posSelf.y, thetaSelf);
        }
         
        tslp_tsk(4);
    }
    poseDrivingControl.SetStop(true,true,true);
    tslp_tsk(1000); // タイヤ完全停止待機

    // 1.7. 回転して、ラインを発見
    printf("PhaseLookUpGate 1.7.Turn to find line\n");

    envViewer->GetRGB(&r, &g, &b);
    if(g >= 17){ 
        poseDrivingControl.SetParams(-10.0,100,60,false);
        // poseDrivingControl.SetStop(false,false,false);
        timer->Reset();
        while (true) {
            if (timer->Now()>700) {
                break;
            }   

            poseDrivingControl.Driving();

            pos->UpdateSelfPos();
            posSelf = pos->GetSelfPos();
            thetaSelf = pos->GetTheta();

            if ((frameCount++) % log_refleshrate == 0) {
                driveWheels->GetAngles(&angleLeft, &angleRight);
                driveWheels->GetPWMs(&pwmLeft, &pwmRight);
                fprintf(file,"%f,%f,%f,%f,%f,%d,%f,%d,%d,%f,%f,%f,%f,%f\n",
                    timer->GetValue(), envViewer->GetLuminance(), postureSensor.GetAnglerVelocity(),
                    -50.0,0.0,
                    tail->GetPWM(),tail->GetAngle(),
                    pwmLeft, pwmRight, angleLeft, angleRight,
                    posSelf.x, posSelf.y, thetaSelf);
            }
            
            tslp_tsk(4);
        }
        poseDrivingControl.SetStop(true,true,true);
        tslp_tsk(1000); // タイヤ完全停止待機

        // poseDrivingControl.SetStop(false,false,false);
        while(true){
            poseDrivingControl.SetParams(10,100,60,false);
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
                    10.0,100.0,
                    tail->GetPWM(),tail->GetAngle(),
                    pwmLeft, pwmRight, angleLeft, angleRight,
                    posSelf.x, posSelf.y, thetaSelf, r, g, b);
            }

            envViewer->GetRGB(&r, &g, &b);
            if(g < 17){
                break;
            }

            tslp_tsk(4);
        }
        poseDrivingControl.SetStop(true,true,true);

        tslp_tsk(1000); // タイヤ完全停止待機
    }    

    // 2. 前進して、ゲートを通過
    printf("PhaseLookUpGate 2.Forward\n");
    // poseDrivingControl.SetStop(false,false,false);
    while(abs(posSelf.x-startPos.x)<60){
		line.CalcTurnValueByRGB();//CalcTurnValue();
		turn = line.GetTurn();
		poseDrivingControl.SetParams(20,turn,60,false);
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
    
    printf("PhaseLookUpGate Clear Single\n");
    // ここまででシングル達成


    // 3. そのまま後退
    printf("PhaseLookUpGate 3.Backward\n");
    poseDrivingControl.SetParams(-20,0.0,60,false);
    // poseDrivingControl.SetStop(false,false,false);
    while(abs(posSelf.x-startPos.x)>15){		
        // line.CalcTurnValueByRGB();//CalcTurnValue();
		// turn = line.GetTurn();
		// poseDrivingControl.SetParams(-10,turn,65,false);
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
                -20.0,0.0,
                tail->GetPWM(),tail->GetAngle(),
                pwmLeft, pwmRight, angleLeft, angleRight,
                posSelf.x, posSelf.y, thetaSelf, r, g, b);
       }
       
		tslp_tsk(4);
    }	
	// poseDrivingControl.SetStop(true,true,true);
    
    tslp_tsk(1000); // タイヤ完全停止待機

    // 4. 前進して、再度ゲートを通過する
    printf("PhaseLookUpGate 4.Forward Double\n");
    // poseDrivingControl.SetStop(false,false,false);
    // poseDrivingControl.SetParams(20,0,65,false);
    while(abs(posSelf.x-startPos.x)<60){
		line.CalcTurnValueByRGB();//CalcTurnValue();
		turn = line.GetTurn();
		poseDrivingControl.SetParams(20,turn,60,false);
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
   
    printf("PhaseLookUpGate Clear Double\n");
     // これでダブル

    // ガレージに入るとこまで
    while(abs(posSelf.x-startPos.x)<60+85){
		line.CalcTurnValueByRGB();//CalcTurnValue();
		turn = line.GetTurn();
		poseDrivingControl.SetParams(20,turn,60,false);
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

    fclose(file);
	finFlg = true;
	printf("PhaseLookUpGate Execute done\n");
}