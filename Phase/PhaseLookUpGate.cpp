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

using namespace Phase;
using namespace	DrivingControl;
using namespace AppliedHardware::VehicleHardware;
using namespace AppliedHardware::EnvironmentSensor;
using namespace BaseHardware;
using namespace Positioning::Localization;
using namespace Utilities;

PhaseLookUpGate::PhaseLookUpGate(){
	pos = SelfPos::GetInstance();
}

void PhaseLookUpGate::Execute(){
	printf("PhaseLookUpGate Execute\n");

	PoseDrivingControl poseDrivingControl;

	Tail* tail = Tail::GetInstance();
    Timer* timer = Timer::GetInstance();

	PostureSensor postureSensor; 
	EnvironmentViewer* envViewer = EnvironmentViewer::GetInstance();
	DriveWheels* driveWheels = DriveWheels::GetInstance();


    // ログ用ファイル生成
    FILE* file;
    char filename[255] = {};
    sprintf(filename, "/ev3rt/res/log_data_lookupgate.csv");
    file = fopen(filename, "w");
    fprintf(file,"timer,caribratedBrightness,gyroSensor,power,turn,PWMt,motor_ang_t,PWMl,PWMr,motor_ang_l,motor_ang_r,xEst,yEst,thetaSelf\n");
    
    int frameCount = 0;
    const int log_refleshrate = 15;

    // 初期化
    float now_angle;

	float angleLeft=0.0, angleRight=0.0;
	signed char pwmLeft=0, pwmRight=0;

    int eq_count;
    float integral, lasterror;

    Vector2D startPos = pos->GetSelfPos();
    Vector2D posSelf(0,0);
    float thetaSelf=0.0;
    
    pos->UpdateSelfPos();
    posSelf = pos->GetSelfPos();
    thetaSelf = pos->GetTheta();

    now_angle = thetaSelf;
    
    // 1. ルックアップゲートに入るところ：倒立振子から尻尾着地
    printf("PhaseLookUpGate 1.Deceleration to Land\n");
    int tau = 1000;	
    timer->Reset();
	poseDrivingControl.SetParams(-50.0,0,65,true);
    while (true) {
        if (timer->Now()>tau) {
			poseDrivingControl.SetParams(0,0,65,false);
			poseDrivingControl.Driving();
            poseDrivingControl.SetStop(true,false,false);
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

    printf("PhaseLookUpGate 1.5.Landing\n");	
	poseDrivingControl.SetParams(0,0,65,true);
    while(true){
        if (abs(tail->GetAngle() - 65) < 4) {
			poseDrivingControl.SetParams(30,0,65,false);
			poseDrivingControl.Driving();
            tslp_tsk(200);
            poseDrivingControl.SetStop(true,true,true);
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

    tslp_tsk(1000); // タイヤ完全停止待機

	// poseDrivingControl.SetParams(0,0,65,true);
    // while(true){

    //     if (abs(tail->GetAngle() - 65) < 2) {
            // poseDrivingControl.SetStop(true,true,true);
    //         break;
    //     }

		// pos->UpdateSelfPos();
		// posSelf = pos->GetSelfPos();
		// thetaSelf = pos->GetTheta();

        // if ((frameCount++) % log_refleshrate == 0) {
            // driveWheels->GetAngles(&angleLeft, &angleRight);
            // driveWheels->GetPWMs(&pwmLeft, &pwmRight);
            // fprintf(file,"%f,%f,%f,%f,%f,%d,%f,%d,%d,%f,%f,%f,%f,%f\n",
            //     timer->GetValue(), envViewer->GetLuminance(), postureSensor.GetAnglerVelocity(),
            //     0.0,0.0,
            //     tail->GetPWM(),tail->GetAngle(),
            //     pwmLeft, pwmRight, angleLeft, angleRight,
            //     posSelf.x, posSelf.y, thetaSelf);
        // }
           
    //     tslp_tsk(4);
    // }
    // tslp_tsk(1000); // タイヤ完全停止待機


    // 2. 前進して、ゲートを通過
    printf("PhaseLookUpGate 2.Forward\n");
	poseDrivingControl.SetParams(20,0,65,false);
    while(posSelf.x-startPos.x<60){
        poseDrivingControl.Driving();

		pos->UpdateSelfPos();
		posSelf = pos->GetSelfPos();
		thetaSelf = pos->GetTheta();

        if ((frameCount++) % log_refleshrate == 0) {
            driveWheels->GetAngles(&angleLeft, &angleRight);
            driveWheels->GetPWMs(&pwmLeft, &pwmRight);
            fprintf(file,"%f,%f,%f,%f,%f,%d,%f,%d,%d,%f,%f,%f,%f,%f\n",
                timer->GetValue(), envViewer->GetLuminance(), postureSensor.GetAnglerVelocity(),
                20.0,0.0,
                tail->GetPWM(),tail->GetAngle(),
                pwmLeft, pwmRight, angleLeft, angleRight,
                posSelf.x, posSelf.y, thetaSelf);
        }

        tslp_tsk(4);
    }
	poseDrivingControl.SetStop(true,true,true);

    tslp_tsk(1000); // タイヤ完全停止待機
    
    printf("PhaseLookUpGate Clear Single\n");
    // ここまででシングル達成


    // 3. その場回転
    printf("PhaseLookUpGate 3.Turn\n");
    eq_count = 0;
    integral = 0; lasterror = 0;
    while(eq_count!=100){
        float error = -(3.141592+now_angle - thetaSelf);
        integral = error + integral * 1.0;

        float turn = 5.0 * error + 0.01*integral + 0.0 * (error - lasterror);
        lasterror = error;
		poseDrivingControl.SetParams(turn,100,65,false);
		poseDrivingControl.Driving();

		pos->UpdateSelfPos();
		posSelf = pos->GetSelfPos();
		thetaSelf = pos->GetTheta();

        if ((frameCount++) % log_refleshrate == 0) {
            driveWheels->GetAngles(&angleLeft, &angleRight);
            driveWheels->GetPWMs(&pwmLeft, &pwmRight);
            fprintf(file,"%f,%f,%f,%f,%f,%d,%f,%d,%d,%f,%f,%f,%f,%f\n",
                timer->GetValue(), envViewer->GetLuminance(), postureSensor.GetAnglerVelocity(),
                turn,100.0,
                tail->GetPWM(),tail->GetAngle(),
                pwmLeft, pwmRight, angleLeft, angleRight,
                posSelf.x, posSelf.y, thetaSelf);
       }

        if(abs(abs(thetaSelf - now_angle)-3.141592)<0.001) eq_count++; else eq_count = 0;

        tslp_tsk(4);
    }	
	poseDrivingControl.SetStop(true,true,true);
    
    tslp_tsk(1000); // タイヤ完全停止待機


    // 4. 前進して、戻る方向にゲートを通過
    printf("PhaseLookUpGate 4.Forward\n");
    poseDrivingControl.SetParams(20,0,65,false);
    while(posSelf.x-startPos.x>15){		
		poseDrivingControl.Driving();

		pos->UpdateSelfPos();
		posSelf = pos->GetSelfPos();
		thetaSelf = pos->GetTheta();

        if ((frameCount++) % log_refleshrate == 0) {
            driveWheels->GetAngles(&angleLeft, &angleRight);
            driveWheels->GetPWMs(&pwmLeft, &pwmRight);
            fprintf(file,"%f,%f,%f,%f,%f,%d,%f,%d,%d,%f,%f,%f,%f,%f\n",
                timer->GetValue(), envViewer->GetLuminance(), postureSensor.GetAnglerVelocity(),
                20.0,0.0,
                tail->GetPWM(),tail->GetAngle(),
                pwmLeft, pwmRight, angleLeft, angleRight,
                posSelf.x, posSelf.y, thetaSelf);
        }

        tslp_tsk(4);
    }
	poseDrivingControl.SetStop(true,true,true);
    
    tslp_tsk(1000); // タイヤ完全停止待機


    // 5. その場回転して、最初のゲートに入るときの向きになる
    printf("PhaseLookUpGate 5.Turn\n");
    eq_count = 0;
    integral = 0;  lasterror = 0;
    // now_angle = thetaSelf;
    while(eq_count!=100){
        float error = now_angle - thetaSelf;
        integral = error + integral * 1.0;

        float turn = 5.0 * error + 0.01*integral + 0.0 * (error - lasterror);
        lasterror = error;
		poseDrivingControl.SetParams(turn,100,65,false);
		poseDrivingControl.Driving();

		pos->UpdateSelfPos();
		posSelf = pos->GetSelfPos();
		thetaSelf = pos->GetTheta();

        if ((frameCount++) % log_refleshrate == 0) {
            driveWheels->GetAngles(&angleLeft, &angleRight);
            driveWheels->GetPWMs(&pwmLeft, &pwmRight);
            fprintf(file,"%f,%f,%f,%f,%f,%d,%f,%d,%d,%f,%f,%f,%f,%f\n",
                timer->GetValue(), envViewer->GetLuminance(), postureSensor.GetAnglerVelocity(),
                turn, 100.0,
                tail->GetPWM(),tail->GetAngle(),
                pwmLeft, pwmRight, angleLeft, angleRight,
                posSelf.x, posSelf.y, thetaSelf);
       }

        if(abs(abs(thetaSelf - now_angle)-3.141592)<0.001) eq_count++; else eq_count = 0;

        tslp_tsk(4);
    }
	poseDrivingControl.SetStop(true,true,true);
    
    tslp_tsk(1000); // タイヤ完全停止待機


    // 6. 前進して、再度ゲートを通過する
    printf("PhaseLookUpGate 6.Last Forward\n");
    poseDrivingControl.SetParams(20,0,65,false);
    while(posSelf.x-startPos.x<60){
        poseDrivingControl.Driving();

		pos->UpdateSelfPos();
		posSelf = pos->GetSelfPos();
		thetaSelf = pos->GetTheta();

        if ((frameCount++) % log_refleshrate == 0) {
            driveWheels->GetAngles(&angleLeft, &angleRight);
            driveWheels->GetPWMs(&pwmLeft, &pwmRight);
            fprintf(file,"%f,%f,%f,%f,%f,%d,%f,%d,%d,%f,%f,%f,%f,%f\n",
                timer->GetValue(), envViewer->GetLuminance(), postureSensor.GetAnglerVelocity(),
                20.0,0.0,
                tail->GetPWM(),tail->GetAngle(),
                pwmLeft, pwmRight, angleLeft, angleRight,
                posSelf.x, posSelf.y, thetaSelf);
        }

        tslp_tsk(4);
    }
	poseDrivingControl.SetStop(true,true,true);
    
    tslp_tsk(1000); // タイヤ完全停止待機
   
    printf("PhaseLookUpGate Clear Double\n");
     // これでダブル

    fclose(file);
	finFlg = true;
	printf("PhaseLookUpGate Execute done\n");
}