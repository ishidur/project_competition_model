#include "PhaseSeesaw.h"

#include <stdlib.h>
#include <stdio.h>
#include "../Utilities/Vector2D.h"
#include "../EnvironmentMeasurement/LineLuminance.h"
#include "../Utilities/ExponentialSmoothingFilter.h"

using namespace Phase;
using namespace AppliedHardware::VehicleHardware;
using namespace AppliedHardware::EnvironmentSensor;
using namespace BaseHardware;
using namespace Positioning::Localization;
using namespace Utilities;
using namespace EnvironmentMeasurement;

#define LOG

#define TAIL_ANGLE (55)
#define TAIL_STAND_ANGLE (83)
#define TAIL_CLIBM_ANGLE (105)
#define TAIL_STOP_ANGLE (55)

#define OFFSET_FIRSTPOS (0)

#define SEESAW_ENTER_X_FOR_LINE_TRACE (23)
#define SEESAW_ENTER_X_FOR_NO_LINE_TRACE (23)
//-SEESAW_ENTER_X_FOR_LINE_TRACE) // 70位置でライントレースなし

#define SEESAW_ENTER_X ( SEESAW_ENTER_X_FOR_NO_LINE_TRACE )
#define SEESAW_PRE_ENTER_X ( SEESAW_ENTER_X_FOR_NO_LINE_TRACE - 3 )
#define SEESAW_SLOW ( SEESAW_ENTER_X + 8.8 )
#define SEESAW_BACK ( SEESAW_ENTER_X + 16.7 )
#define SEESAW_STOP ( SEESAW_ENTER_X + 21.9 )
#define SEESAW_CLIBM_STOP ( SEESAW_ENTER_X + 15.06 )
#define SEESAW_PRE_END_STOP ( SEESAW_END_STOP - 2.74 )
#define SEESAW_END_STOP ( SEESAW_ENTER_X + 29.58 )
// #define SEESAW_SLOW ( SEESAW_ENTER_X + 250 )
// #define SEESAW_BACK ( SEESAW_ENTER_X + 610 )
// #define SEESAW_STOP ( SEESAW_ENTER_X + 800 )
// #define SEESAW_CLIBM_STOP ( SEESAW_ENTER_X + 550 )
// #define SEESAW_PRE_END_STOP ( SEESAW_END_STOP - 100 )
// #define SEESAW_END_STOP ( SEESAW_ENTER_X + 1080 )

// #define DISTANCE (angleAve-firstAngleAve)
#define DISTANCE (posSelf.DistanceFrom(startPos)-OFFSET_FIRSTPOS)

#define DEG2RAD     0.01745329238F /* 角度単位変換係数(=pi/180) */
#define RAD2DEG		180.0/3.141592
#define EXEC_PERIOD 0.00500000000F /* バランス制御実行周期(秒) *//* sample_c4の処理時間考慮 */

PhaseSeesaw::PhaseSeesaw(){
	pos = SelfPos::GetInstance();
	tail = Tail::GetInstance();

	envViewer = EnvironmentViewer::GetInstance();
	driveWheels = DriveWheels::GetInstance();
	timer = Timer::GetInstance();

    char filename[255] = {};
    sprintf(filename, "/ev3rt/res/log_data_seesaw.csv");
    file = fopen(filename, "w");
	fprintf(file,"type,timer,x,y,theta,tailAngle,tailPWM,angleLeft,angleRight,pwmLeft,pwmRight,forward,gyro,gyroFilter\n");
}

float PhaseSeesaw::observer(float args_gyro, float args_gyro_offset)
{
    static float ud_psi = 0.0;
	ud_psi += (EXEC_PERIOD * (args_gyro - args_gyro_offset) * DEG2RAD) * RAD2DEG;
    return ud_psi;//rad
}

void PhaseSeesaw::Execute(){
	printf("PhaseSeesaw Execute\n");
    ev3_speaker_play_tone(NOTE_C4, 10);

	LineLuminance line;
    Vector2D posSelf(0,0);
	pos->UpdateSelfPos();
	posSelf = pos->GetSelfPos();
    float thetaSelf = pos->GetTheta();
	Vector2D startPos = pos->GetSelfPos();
	printf("start (x,y) = (%f, %f)\n", startPos.x, startPos.y);

	float angleLeft=0.0, angleRight=0.0, angleAve=0.0;
	signed char pwmLeft=0, pwmRight=0;
	float turn;
	float gyro_sum = 0.0;
	int tmp_forward;
	float process;
	float now_b_angle = 0.0;

	int log_refleshrate = 1;

	driveWheels->GetAngles(&angleLeft, &angleRight);
	angleAve = (angleLeft+angleRight)/2.0;
	float firstAngleAve = angleAve;

	ExponentialSmoothingFilter gyroFilter(0.05,postureSensor.GetAnglerVelocity());	
	float gyro;

    poseDrivingControl.SetStop(false,true,true);
	poseDrivingControl.SetParams(0.0,0,TAIL_CLIBM_ANGLE,false);
    ev3_speaker_play_tone(NOTE_C5, 100);
	tslp_tsk(2000);
	while(1){
		poseDrivingControl.Driving();
        if (envViewer->GetTouch()) {
            break;
        }   
        tslp_tsk(4);
	}
    poseDrivingControl.SetStop(true,true,true);
    ev3_speaker_play_tone(NOTE_C5, 100);
/*
	// 0. 着地
    printf("PhaseSeesaw 0.Landing\n");	
	process = 0.0;
{
    int tau = 1500;	
	poseDrivingControl.SetParams(0.0,0,TAIL_STAND_ANGLE-5,true);
	timer->Reset();
	signed char pwmLeft, pwmRight;
    while (true) {
		pos->UpdateSelfPos();
		posSelf = pos->GetSelfPos();
		thetaSelf = pos->GetTheta();

		poseDrivingControl.Driving();

		// driveWheels->GetAngles(&angleLeft, &angleRight);
		// angleAve = (angleLeft+angleRight)/2.0;
		gyro = gyroFilter.GetValue(postureSensor.GetAnglerVelocity());

	#ifdef LOG
    	if( (tmp_log_cnt++)%log_refleshrate==0 ){
            // driveWheels->GetAngles(&angleLeft, &angleRight);
            driveWheels->GetPWMs(&pwmLeft, &pwmRight);
			fprintf(file,"%f,%f,%f,%f,%f,%f,%d,%f,%f,%d,%d,%d,%f,%f\n",
           		process,timer->GetValue(), posSelf.x, posSelf.y, thetaSelf, tail->GetAngle(),tail->GetPWM(),angleLeft, angleRight, pwmLeft, pwmRight,  tmp_forward, postureSensor.GetAnglerVelocity(), gyro);
		}
	#endif

		driveWheels->GetPWMs(&pwmLeft, &pwmRight);
        if (timer->Now()>tau||((pwmLeft+pwmRight)/2.0<-40&&gyro<-50)) {
            break;
        }   
        tslp_tsk(4);
    }

	float gyro_sum = 0.0;
    poseDrivingControl.SetParams(35,0,TAIL_STAND_ANGLE-5,false);
	timer->Reset();
    while(1){
        poseDrivingControl.Driving();

		// driveWheels->GetAngles(&angleLeft, &angleRight);
		// angleAve = (angleLeft+angleRight)/2.0;
		gyro = gyroFilter.GetValue(postureSensor.GetAnglerVelocity());

	#ifdef LOG
    	if( (tmp_log_cnt++)%log_refleshrate==0 ){
            // driveWheels->GetAngles(&angleLeft, &angleRight);
            driveWheels->GetPWMs(&pwmLeft, &pwmRight);
			fprintf(file,"%f,%f,%f,%f,%f,%f,%d,%f,%f,%d,%d,%d,%f,%f\n",
           		process,timer->GetValue(), posSelf.x, posSelf.y, thetaSelf, tail->GetAngle(),tail->GetPWM(),angleLeft, angleRight, pwmLeft, pwmRight,  tmp_forward, postureSensor.GetAnglerVelocity(), gyro);
		}
	#endif
    	gyro_sum += postureSensor.GetAnglerVelocity();
        if(gyro_sum < -150&&timer->Now()>200){
            break;
        }
        tslp_tsk(4);
    }
    poseDrivingControl.SetStop(true,true,true);
    tslp_tsk(1000); // タイヤ完全停止待機
    poseDrivingControl.SetStop(false,false,false);


	driveWheels->GetAngles(&angleLeft, &angleRight);
	firstAngleAve = (angleLeft+angleRight)/2.0;
    while(true){
        pos->UpdateSelfPos();
        posSelf = pos->GetSelfPos();
        thetaSelf = pos->GetTheta();

        poseDrivingControl.SetParams(-10,0,TAIL_STAND_ANGLE,false);
        poseDrivingControl.Driving();

		driveWheels->GetAngles(&angleLeft, &angleRight);
		angleAve = (angleLeft+angleRight)/2.0;
		
        if (abs(tail->GetAngle() - TAIL_STAND_ANGLE) < 3 && abs(firstAngleAve-angleAve)>40) {
            break;
        }
		// driveWheels->GetAngles(&angleLeft, &angleRight);
		// angleAve = (angleLeft+angleRight)/2.0;
		gyro = gyroFilter.GetValue(postureSensor.GetAnglerVelocity());

	#ifdef LOG
    	if( (tmp_log_cnt++)%log_refleshrate==0 ){
			driveWheels->GetPWMs(&pwmLeft, &pwmRight);
            // driveWheels->GetAngles(&angleLeft, &angleRight);
			fprintf(file,"%f,%f,%f,%f,%f,%f,%d,%f,%f,%d,%d,%d,%f,%f\n",
           		process,timer->GetValue(), posSelf.x, posSelf.y, thetaSelf, tail->GetAngle(),tail->GetPWM(),angleLeft, angleRight, pwmLeft, pwmRight,  tmp_forward, postureSensor.GetAnglerVelocity(), gyro);
		}
	#endif
        tslp_tsk(4);
    }
    poseDrivingControl.SetStop(true,true,true);
    tslp_tsk(1000); // タイヤ完全停止待機
}

	timer->Reset();
    // 1. forward切り替え位置までforward=70で進むx=60
	printf("PhaseSeesaw 1. Forward\n"); 
	process = 1.0;
{
	float lastAngleAve = angleAve;
	long firstTime = timer->GetValue();
	timer->Reset();
    while (true) {
		pos->UpdateSelfPos();
		posSelf = pos->GetSelfPos();
		thetaSelf = pos->GetTheta();

		tmp_forward = 1;
		// if(DISTANCE>SEESAW_PRE_ENTER_X){
		if(timer->GetValue()-firstTime>4000){
			poseDrivingControl.SetParams(1.0,0.0,TAIL_STAND_ANGLE,false);
		}else{
			line.CalcTurnValueByRGB();
			turn = line.GetTurn()*-1;
			poseDrivingControl.SetParams(1.2,turn,TAIL_STAND_ANGLE,false);
		}
		poseDrivingControl.Driving();

		gyro = gyroFilter.GetValue(postureSensor.GetAnglerVelocity());

    	if( (tmp_log_cnt++)%log_refleshrate==0 ){
			driveWheels->GetAngles(&angleLeft, &angleRight);
			angleAve = (angleLeft+angleRight)/2.0;
            // driveWheels->GetAngles(&angleLeft, &angleRight);
	#ifdef LOG
            driveWheels->GetPWMs(&pwmLeft, &pwmRight);
			fprintf(file,"%f,%f,%f,%f,%f,%f,%d,%f,%f,%d,%d,%d,%f,%f\n",
           		process,timer->GetValue(), posSelf.x, posSelf.y, thetaSelf, tail->GetAngle(),tail->GetPWM(),angleLeft, angleRight, pwmLeft, pwmRight,  tmp_forward, postureSensor.GetAnglerVelocity(), gyro);
	#endif
		}
    	//切り替え位置到達かチェック
		if(abs(lastAngleAve-angleAve)<0.2){
		// if(DISTANCE>SEESAW_ENTER_X){
			if(timer->Now()>1500 && timer->GetValue()-firstTime>8000) break; //到達したので終了
    	}else{
			timer->Reset();
		}

		lastAngleAve = angleAve;

    	tslp_tsk(4);
    }
	tslp_tsk(4);
}

    poseDrivingControl.SetStop(true,true,true);
    ev3_speaker_play_tone(NOTE_C4, 200);
	tslp_tsk(1000);

	// 乗り込み助走つけ
{
	driveWheels->GetAngles(&angleLeft, &angleRight);
	firstAngleAve = (angleLeft+angleRight)/2.0;
	while (true) {
		pos->UpdateSelfPos();
		posSelf = pos->GetSelfPos();
		thetaSelf = pos->GetTheta();

		tmp_forward = -5;
		poseDrivingControl.SetParams(tmp_forward,0.0,TAIL_STAND_ANGLE-5,false);
		poseDrivingControl.Driving();

		driveWheels->GetAngles(&angleLeft, &angleRight);
		angleAve = (angleLeft+angleRight)/2.0;
		gyro = gyroFilter.GetValue(postureSensor.GetAnglerVelocity());

	#ifdef LOG
    	if( (tmp_log_cnt++)%log_refleshrate==0 ){
            // driveWheels->GetAngles(&angleLeft, &angleRight);
            driveWheels->GetPWMs(&pwmLeft, &pwmRight);
			fprintf(file,"%f,%f,%f,%f,%f,%f,%d,%f,%f,%d,%d,%d,%f,%f\n",
           		process,timer->GetValue(), posSelf.x, posSelf.y, thetaSelf, tail->GetAngle(),tail->GetPWM(),angleLeft, angleRight, pwmLeft, pwmRight,  tmp_forward, postureSensor.GetAnglerVelocity(), gyro);
		}
	#endif
    	//切り替え位置到達かチェック
		if(abs(firstAngleAve-angleAve)>80){
		// if(DISTANCE>SEESAW_ENTER_X){
    		break; //到達したので終了
    	}

    	tslp_tsk(4);
    }
	tslp_tsk(500);

	while (true) {
		pos->UpdateSelfPos();
		posSelf = pos->GetSelfPos();
		thetaSelf = pos->GetTheta();

		tmp_forward = 50;
		poseDrivingControl.SetParams(tmp_forward,0.0,90,false);
		poseDrivingControl.Driving();

		driveWheels->GetAngles(&angleLeft, &angleRight);
		angleAve = (angleLeft+angleRight)/2.0;
		gyro = gyroFilter.GetValue(postureSensor.GetAnglerVelocity());

	#ifdef LOG
    	if( (tmp_log_cnt++)%log_refleshrate==0 ){
            // driveWheels->GetAngles(&angleLeft, &angleRight);
            driveWheels->GetPWMs(&pwmLeft, &pwmRight);
			fprintf(file,"%f,%f,%f,%f,%f,%f,%d,%f,%f,%d,%d,%d,%f,%f\n",
           		process,timer->GetValue(), posSelf.x, posSelf.y, thetaSelf, tail->GetAngle(),tail->GetPWM(),angleLeft, angleRight, pwmLeft, pwmRight,  tmp_forward, postureSensor.GetAnglerVelocity(), gyro);
		}
	#endif
    	//切り替え位置到達かチェック
		if(abs(firstAngleAve-angleAve)<15){
		// if(DISTANCE>SEESAW_ENTER_X){
    		break; //到達したので終了
    	}

    	tslp_tsk(4);
    }
	tslp_tsk(4);
}
	

    // 2. 傾く
	printf("PhaseSeesaw 2. Tilting\n"); 
	process = 2.0;
{
	tmp_stop_cnt = 0;
	tmp_forward = 0; //PWM = 0を出力（※gyroの更新ありに変更した）
	gyro_sum = 0;
	poseDrivingControl.SetStop(false,false,false);
	poseDrivingControl.SetParams(tmp_forward,0,110,false);	

	timer->Reset();
    while (true) {
		pos->UpdateSelfPos();
		posSelf = pos->GetSelfPos();
		thetaSelf = pos->GetTheta();

		poseDrivingControl.Driving();		

		driveWheels->GetAngles(&angleLeft, &angleRight);
		angleAve = (angleLeft+angleRight)/2.0;
		gyro = gyroFilter.GetValue(postureSensor.GetAnglerVelocity());
	#ifdef LOG
    	if( (tmp_log_cnt++)%log_refleshrate==0 ){
            // driveWheels->GetAngles(&angleLeft, &angleRight);
            driveWheels->GetPWMs(&pwmLeft, &pwmRight);
			fprintf(file,"%f,%f,%f,%f,%f,%f,%d,%f,%f,%d,%d,%d,%f,%f\n",
           		process,timer->GetValue(), posSelf.x, posSelf.y, thetaSelf, tail->GetAngle(),tail->GetPWM(),angleLeft, angleRight, pwmLeft, pwmRight,  tmp_forward, postureSensor.GetAnglerVelocity(), gyro);
		}
	#endif
    	//切り替え条件成立かチェック
        //5ms×30回≒150msマスクする→検討した結果から決定
		// if( (tmp_stop_cnt++) >= 10){
		// 	gyro_sum += gyro;
		// 	if( gyro_sum > 1500 ){//角速度80より大きくなったら次の処理    			
		// 		break;
		// 	}
		// }
		
        if (abs(tail->GetAngle() - 103) < 3) {
            break;
        }
    	tslp_tsk(4);
    }
}
	tslp_tsk(4);

	// poseDrivingControl.SetStop(true,true,true);

    // 4.2. 乗り上げ
	printf("PhaseSeesaw 4.2.Forward to ride the Seesaw\n"); 
	process = 4.2;
{
	driveWheels->GetAngles(&angleLeft, &angleRight);
	firstAngleAve = (angleLeft+angleRight)/2.0;
	tmp_forward = 0;
	ExponentialSmoothingFilter expFilter(0.6,30.0);
	timer->Reset();
	while (true) {
		pos->UpdateSelfPos();
		posSelf = pos->GetSelfPos();
		thetaSelf = pos->GetTheta();

		tmp_forward = expFilter.GetValue(60.0);
		poseDrivingControl.SetParams(tmp_forward,0,80,false);
		poseDrivingControl.Driving();	

		driveWheels->GetAngles(&angleLeft, &angleRight);
		angleAve = (angleLeft+angleRight)/2.0;
		gyro = gyroFilter.GetValue(postureSensor.GetAnglerVelocity());
	#ifdef LOG
    	if( (tmp_log_cnt++)%log_refleshrate==0 ){
			// driveWheels->GetAngles(&angleLeft, &angleRight);
            driveWheels->GetPWMs(&pwmLeft, &pwmRight);
			fprintf(file,"%f,%f,%f,%f,%f,%f,%d,%f,%f,%d,%d,%d,%f,%f\n",
					process,timer->GetValue(), posSelf.x, posSelf.y, thetaSelf, tail->GetAngle(),tail->GetPWM(),angleLeft, angleRight, pwmLeft, pwmRight,  tmp_forward, postureSensor.GetAnglerVelocity(), gyro);
		}
	#endif
		if(abs(firstAngleAve-angleAve)>350){//DISTANCE>SEESAW_SLOW){
    		break; //所定位置まで到達した
    	}

    	tslp_tsk(4);
    }
	tslp_tsk(4);
}
*/
	poseDrivingControl.SetStop(false,false,false);
    // 5. 乗り上げ後は、ゆっくり進む
	printf("PhaseSeesaw 5.Climbing\n"); 
	process = 5.0;
{
	tmp_forward = 5;
	timer->Reset();
	while (true) {
		pos->UpdateSelfPos();
		posSelf = pos->GetSelfPos();
		thetaSelf = pos->GetTheta();

		// line.CalcTurnValue();
		// turn = line.GetTurn()*0.1;
		turn = 0.0;
		poseDrivingControl.SetParams(tmp_forward,turn,TAIL_CLIBM_ANGLE,false);
		poseDrivingControl.Driving();		

		driveWheels->GetAngles(&angleLeft, &angleRight);
		angleAve = (angleLeft+angleRight)/2.0;
		gyro = gyroFilter.GetValue(postureSensor.GetAnglerVelocity());
		now_b_angle = observer(gyro, 0.0);
	#ifdef LOG
    	if( (tmp_log_cnt++)%log_refleshrate==0 ){
			// driveWheels->GetAngles(&angleLeft, &angleRight);
            driveWheels->GetPWMs(&pwmLeft, &pwmRight);
			fprintf(file,"%f,%f,%f,%f,%f,%f,%d,%f,%f,%d,%d,%d,%f,%f\n",
					process,timer->GetValue(), posSelf.x, posSelf.y, thetaSelf, tail->GetAngle(),tail->GetPWM(),angleLeft, angleRight, pwmLeft, pwmRight,  tmp_forward, postureSensor.GetAnglerVelocity(), now_b_angle);
		}
	#endif
		if(now_b_angle>-0.5&&timer->Now()>2300){//abs(firstAngleAve-angleAve)>300){
			ev3_speaker_play_tone(NOTE_C5, 100);
    		break;
    	}

    	tslp_tsk(4);
    }
	tslp_tsk(4);
}
	
    // 6. シーソー下り開始に合わせて、シーソー上で停止 or バックさせる
	printf("PhaseSeesaw 6.Back when descending seesaw\n"); 
	process = 6.0;
{
	driveWheels->GetAngles(&angleLeft, &angleRight);
	angleAve = (angleLeft+angleRight)/2.0;
	firstAngleAve = angleAve;
	
	timer->Reset();
	while (true) {
        //自己位置更新
		pos->UpdateSelfPos();
		posSelf = pos->GetSelfPos();
		thetaSelf = pos->GetTheta();

		poseDrivingControl.SetParams(0,0,35,false);
		poseDrivingControl.Driving();		

		driveWheels->GetAngles(&angleLeft, &angleRight);
		angleAve = (angleLeft+angleRight)/2.0;
		gyro = gyroFilter.GetValue(postureSensor.GetAnglerVelocity());
		now_b_angle = observer(gyro, 0.0);
	#ifdef LOG
    	if( (tmp_log_cnt++)%log_refleshrate==0 ){
			// driveWheels->GetAngles(&angleLeft, &angleRight);
            driveWheels->GetPWMs(&pwmLeft, &pwmRight);
			fprintf(file,"%f,%f,%f,%f,%f,%f,%d,%f,%f,%d,%d,%d,%f,%f\n",
					process,timer->GetValue(), posSelf.x, posSelf.y, thetaSelf, tail->GetAngle(),tail->GetPWM(),angleLeft, angleRight, pwmLeft, pwmRight,  tmp_forward, postureSensor.GetAnglerVelocity(), now_b_angle);
		}
	#endif

		if(timer->Now()>10){
    		break;
    	}

    	tslp_tsk(4);
    }

	tmp_forward = 80.0;
	while (true) {
        //自己位置更新
		pos->UpdateSelfPos();
		posSelf = pos->GetSelfPos();
		thetaSelf = pos->GetTheta();

		poseDrivingControl.SetParams(tmp_forward,0,35,false);
		poseDrivingControl.Driving();		

		driveWheels->GetAngles(&angleLeft, &angleRight);
		angleAve = (angleLeft+angleRight)/2.0;
		gyro = gyroFilter.GetValue(postureSensor.GetAnglerVelocity());
		now_b_angle = observer(gyro, 0.0);
	#ifdef LOG
    	if( (tmp_log_cnt++)%log_refleshrate==0 ){
			// driveWheels->GetAngles(&angleLeft, &angleRight);
            driveWheels->GetPWMs(&pwmLeft, &pwmRight);
			fprintf(file,"%f,%f,%f,%f,%f,%f,%d,%f,%f,%d,%d,%d,%f,%f\n",
					process,timer->GetValue(), posSelf.x, posSelf.y, thetaSelf, tail->GetAngle(),tail->GetPWM(),angleLeft, angleRight, pwmLeft, pwmRight,  tmp_forward, postureSensor.GetAnglerVelocity(), now_b_angle);
		}
	#endif

		if(abs(angleAve - firstAngleAve) > 110){
    		break;
    	}

    	tslp_tsk(4);
    }
	// tslp_tsk(4);
	// tmp_forward = 0.0;
	// poseDrivingControl.SetParams(tmp_forward,0,45,false);
	// while (true) {
    //     //自己位置更新
	// 	pos->UpdateSelfPos();
	// 	posSelf = pos->GetSelfPos();
	// 	thetaSelf = pos->GetTheta();

	// 	poseDrivingControl.Driving();		

	// 	driveWheels->GetAngles(&angleLeft, &angleRight);
	// 	angleAve = (angleLeft+angleRight)/2.0;
	// 	gyro = gyroFilter.GetValue(postureSensor.GetAnglerVelocity());
	// 	now_b_angle = observer(gyro, 0.0);
	// #ifdef LOG
    // 	if( (tmp_log_cnt++)%log_refleshrate==0 ){
	// 		// driveWheels->GetAngles(&angleLeft, &angleRight);
    //         driveWheels->GetPWMs(&pwmLeft, &pwmRight);
	// 		fprintf(file,"%f,%f,%f,%f,%f,%f,%d,%f,%f,%d,%d,%d,%f,%f\n",
	// 				process,timer->GetValue(), posSelf.x, posSelf.y, thetaSelf, tail->GetAngle(),tail->GetPWM(),angleLeft, angleRight, pwmLeft, pwmRight,  tmp_forward, postureSensor.GetAnglerVelocity(), now_b_angle);
	// 	}
	// #endif

	// 	if(abs(angleAve - firstAngleAve) > 110){//DISTANCE>SEESAW_STOP){
    // 		break;
    // 	}

    // 	tslp_tsk(4);
    // }
	tslp_tsk(4);
	tmp_forward = 0.0;
	timer->Reset();
	poseDrivingControl.SetParams(tmp_forward,0,35,false);
	while (true) {
        //自己位置更新
		pos->UpdateSelfPos();
		posSelf = pos->GetSelfPos();
		thetaSelf = pos->GetTheta();

		poseDrivingControl.Driving();		

		driveWheels->GetAngles(&angleLeft, &angleRight);
		angleAve = (angleLeft+angleRight)/2.0;
		gyro = gyroFilter.GetValue(postureSensor.GetAnglerVelocity());
		now_b_angle = observer(gyro, 0.0);
	#ifdef LOG
    	if( (tmp_log_cnt++)%log_refleshrate==0 ){
			// driveWheels->GetAngles(&angleLeft, &angleRight);
            driveWheels->GetPWMs(&pwmLeft, &pwmRight);
			fprintf(file,"%f,%f,%f,%f,%f,%f,%d,%f,%f,%d,%d,%d,%f,%f\n",
					process,timer->GetValue(), posSelf.x, posSelf.y, thetaSelf, tail->GetAngle(),tail->GetPWM(),angleLeft, angleRight, pwmLeft, pwmRight,  tmp_forward, postureSensor.GetAnglerVelocity(), now_b_angle);
		}
	#endif

		if(timer->Now()>500){//DISTANCE>SEESAW_STOP){
    		break;
    	}

    	tslp_tsk(4);
    }
	tslp_tsk(4);
}
	poseDrivingControl.SetStop(true,true,true);
	tslp_tsk(3000);
	poseDrivingControl.SetStop(false,true,true);

	tmp_forward = 0.0;
	timer->Reset();
	poseDrivingControl.SetParams(tmp_forward,0,45,false);
	while (true) {
        //自己位置更新
		pos->UpdateSelfPos();
		posSelf = pos->GetSelfPos();
		thetaSelf = pos->GetTheta();

		poseDrivingControl.Driving();		

		driveWheels->GetAngles(&angleLeft, &angleRight);
		angleAve = (angleLeft+angleRight)/2.0;
		gyro = gyroFilter.GetValue(postureSensor.GetAnglerVelocity());
		now_b_angle = observer(gyro, 0.0);
	#ifdef LOG
    	if( (tmp_log_cnt++)%log_refleshrate==0 ){
			// driveWheels->GetAngles(&angleLeft, &angleRight);
            driveWheels->GetPWMs(&pwmLeft, &pwmRight);
			fprintf(file,"%f,%f,%f,%f,%f,%f,%d,%f,%f,%d,%d,%d,%f,%f\n",
					process,timer->GetValue(), posSelf.x, posSelf.y, thetaSelf, tail->GetAngle(),tail->GetPWM(),angleLeft, angleRight, pwmLeft, pwmRight,  tmp_forward, postureSensor.GetAnglerVelocity(), now_b_angle);
		}
	#endif

		if(timer->Now()>1000){//DISTANCE>SEESAW_STOP){
    		break;
    	}

    	tslp_tsk(4);
    }
	tslp_tsk(4);

#if 0
    // 7. バック＋尻尾押上で、シーソーを背面登頂する
	printf("PhaseSeesaw 7.Back to climb seesaw\n"); 
	tmp_forward = -50.0;
	poseDrivingControl.SetParams(tmp_forward, 0, 80, false);
	while (true) {

		pos->UpdateSelfPos();
		posSelf = pos->GetSelfPos();
		thetaSelf = pos->GetTheta();
		
		poseDrivingControl.Driving();		

        tmp_log_cnt++;
    	if( tmp_log_cnt > 10 ){
			driveWheels->GetAngles(&angleLeft, &angleRight);
            driveWheels->GetPWMs(&pwmLeft, &pwmRight);
			fprintf(file,"%f,%f,%f,%f,%f,%d,%f,%f,%d,%d,%d,%f\n",
           		timer->GetValue(), posSelf.x, posSelf.y, thetaSelf, tail->GetAngle(),tail->GetPWM(),angleLeft, angleRight, pwmLeft, pwmRight,  tmp_forward, postureSensor.GetAnglerVelocity());
	        tmp_log_cnt=0;
		}

		if(DISTANCE<SEESAW_CLIBM_STOP){
    		break;
    	}

    	tslp_tsk(4);
    }
	tslp_tsk(4);

    // 8. 再度シーソー下り開始に合わせて、シーソー上で停止 or バックさせる
	printf("PhaseSeesaw 8.Back when descending seesaw\n"); 
	tmp_forward = 60.0;
	poseDrivingControl.SetParams(tmp_forward, 0, TAIL_STOP_ANGLE, false);
	while (true) {

        //自己位置更新
		pos->UpdateSelfPos();
		posSelf = pos->GetSelfPos();
		thetaSelf = pos->GetTheta();
		
		poseDrivingControl.Driving();		

        tmp_log_cnt++;
    	if( tmp_log_cnt > 10 ){
			driveWheels->GetAngles(&angleLeft, &angleRight);
            driveWheels->GetPWMs(&pwmLeft, &pwmRight);
			fprintf(file,"%f,%f,%f,%f,%f,%d,%f,%f,%d,%d,%d,%f\n",
           		timer->GetValue(), posSelf.x, posSelf.y, thetaSelf, tail->GetAngle(),tail->GetPWM(),angleLeft, angleRight, pwmLeft, pwmRight,  tmp_forward, postureSensor.GetAnglerVelocity());
	        tmp_log_cnt=0;
		}

		if(DISTANCE>SEESAW_STOP){
    		break;
    	}

    	tslp_tsk(4);
    }
	tslp_tsk(4);
	poseDrivingControl.SetStop(true,true,true);

	tslp_tsk(1000);
	poseDrivingControl.SetStop(false,true,true);
#endif

	poseDrivingControl.SetStop(false,false,false);

	// 9. シーソーから降りて前進する
	printf("PhaseSeesaw 9.Forward to get off\n"); 
	process = 9.0;
{
	tmp_forward = 10;
	ExponentialSmoothingFilter expFilter2(0.99,10.0);
	while (true) {
		pos->UpdateSelfPos();
		posSelf = pos->GetSelfPos();
		thetaSelf = pos->GetTheta();
		
		tmp_forward = expFilter2.GetValue(40.0);
		poseDrivingControl.SetParams(tmp_forward, 0, 55, false);
		poseDrivingControl.Driving();	

		driveWheels->GetAngles(&angleLeft, &angleRight);
		angleAve = (angleLeft+angleRight)/2.0;
		gyro = gyroFilter.GetValue(postureSensor.GetAnglerVelocity());
		now_b_angle = observer(gyro, 0.0);

	#ifdef LOG
    	if( (tmp_log_cnt++)%log_refleshrate==0 ){		
            driveWheels->GetPWMs(&pwmLeft, &pwmRight);
			fprintf(file,"%f,%f,%f,%f,%f,%f,%d,%f,%f,%d,%d,%d,%f,%f\n",
					process,timer->GetValue(), posSelf.x, posSelf.y, thetaSelf, tail->GetAngle(),tail->GetPWM(),angleLeft, angleRight, pwmLeft, pwmRight,  tmp_forward, postureSensor.GetAnglerVelocity(), now_b_angle);
		}
	#endif

		if(abs(firstAngleAve-angleAve)>750){
    		break;
    	}

		tslp_tsk(4);
	} 
	// ExponentialSmoothingFilter expFilter3(0.99,tmp_forward);
	// while (true) {
	// 	pos->UpdateSelfPos();
	// 	posSelf = pos->GetSelfPos();
	// 	thetaSelf = pos->GetTheta();
		
	// 	tmp_forward = expFilter2.GetValue(5.0);
	// 	poseDrivingControl.SetParams(tmp_forward, 0, 65, false);
	// 	poseDrivingControl.Driving();	

	// 	driveWheels->GetAngles(&angleLeft, &angleRight);
	// 	angleAve = (angleLeft+angleRight)/2.0;
	// 	gyro = gyroFilter.GetValue(postureSensor.GetAnglerVelocity());
	// 	now_b_angle = observer(gyro, 0.0);
		
	// #ifdef LOG
    // 	if( (tmp_log_cnt++)%log_refleshrate==0 ){	
    //         driveWheels->GetPWMs(&pwmLeft, &pwmRight);
	// 		fprintf(file,"%f,%f,%f,%f,%f,%f,%d,%f,%f,%d,%d,%d,%f,%f\n",
	// 				process,timer->GetValue(), posSelf.x, posSelf.y, thetaSelf, tail->GetAngle(),tail->GetPWM(),angleLeft, angleRight, pwmLeft, pwmRight,  tmp_forward, postureSensor.GetAnglerVelocity(), now_b_angle);
	// 	}
	// #endif

	// 	if(DISTANCE>SEESAW_END_STOP){
    // 		break;
    // 	}

	// 	tslp_tsk(4);
	// }
	poseDrivingControl.SetStop(false,true,true);
	
	tmp_forward = -3;
	poseDrivingControl.SetParams(tmp_forward, 0, 60, false);
	while (true) {
		pos->UpdateSelfPos();
		posSelf = pos->GetSelfPos();
		thetaSelf = pos->GetTheta();
		
		poseDrivingControl.Driving();	

		driveWheels->GetAngles(&angleLeft, &angleRight);
		angleAve = (angleLeft+angleRight)/2.0;
		gyro = gyroFilter.GetValue(postureSensor.GetAnglerVelocity());
		now_b_angle = observer(gyro, 0.0);
		
	#ifdef LOG
    	if( (tmp_log_cnt++)%log_refleshrate==0 ){	
            driveWheels->GetPWMs(&pwmLeft, &pwmRight);
			fprintf(file,"%f,%f,%f,%f,%f,%f,%d,%f,%f,%d,%d,%d,%f,%f\n",
					process,timer->GetValue(), posSelf.x, posSelf.y, thetaSelf, tail->GetAngle(),tail->GetPWM(),angleLeft, angleRight, pwmLeft, pwmRight,  tmp_forward, postureSensor.GetAnglerVelocity(), now_b_angle);
		}
	#endif

		if(abs(tail->GetAngle()-60)<5){
    		break;
    	}

		tslp_tsk(4);
	}
}
	poseDrivingControl.SetStop(true,true,true);

	fclose(file);
	finFlg = true;
	printf("PhaseSeesaw Execute done\n");
}

bool PhaseSeesaw::IsFinish(){
	return false;
}

// ダブル検討
#if 0
#include "PhaseSeesaw.h"

#include <stdlib.h>
#include <stdio.h>
#include "../Utilities/Vector2D.h"
#include "../EnvironmentMeasurement/LineLuminance.h"
#include "../Utilities/ExponentialSmoothingFilter.h"

using namespace Phase;
using namespace AppliedHardware::VehicleHardware;
using namespace AppliedHardware::EnvironmentSensor;
using namespace BaseHardware;
using namespace Positioning::Localization;
using namespace Utilities;
using namespace EnvironmentMeasurement;

#define LOG

#define TAIL_ANGLE (10)
#define TAIL_STOP_ANGLE (50)

#define OFFSET_FIRSTPOS (0)

#define SEESAW_ENTER_X_FOR_LINE_TRACE (0)
#define SEESAW_ENTER_X_FOR_NO_LINE_TRACE (110)
//-SEESAW_ENTER_X_FOR_LINE_TRACE) // 70位置でライントレースなし

#define SEESAW_ENTER_X ( SEESAW_ENTER_X_FOR_NO_LINE_TRACE )
#define SEESAW_PRE_ENTER_X ( SEESAW_ENTER_X_FOR_NO_LINE_TRACE - 60 )
#define SEESAW_SLOW ( SEESAW_ENTER_X + 250 )
#define SEESAW_BACK ( SEESAW_ENTER_X + 525 )
#define SEESAW_STOP ( SEESAW_ENTER_X + 650 )
#define SEESAW_CLIBM_STOP ( SEESAW_ENTER_X + 480 )
#define SEESAW_PRE_END_STOP ( SEESAW_END_STOP - 100 )
#define SEESAW_END_STOP ( SEESAW_ENTER_X + 1080 )

#define DISTANCE (angleAve-firstAngleAve)
//posSelf.DistanceFrom(startPos)-OFFSET_FIRSTPOS

PhaseSeesaw::PhaseSeesaw(){
	pos = SelfPos::GetInstance();
	tail = Tail::GetInstance();

	envViewer = EnvironmentViewer::GetInstance();
	driveWheels = DriveWheels::GetInstance();
	timer = Timer::GetInstance();

    char filename[255] = {};
    sprintf(filename, "/ev3rt/res/log_data_seesaw.csv");
    file = fopen(filename, "w");
	fprintf(file,"type,timer,x,y,theta,tailAngle,tailPWM,angleLeft,angleRight,pwmLeft,pwmRight,forward,gyro\n");
}

void PhaseSeesaw::Execute(){
	printf("PhaseSeesaw Execute\n");
	
	LineLuminance line;
    Vector2D posSelf(0,0);
	pos->UpdateSelfPos();
	posSelf = pos->GetSelfPos();
    float thetaSelf = pos->GetTheta();
	Vector2D startPos = pos->GetSelfPos();
	printf("start (x,y) = (%f, %f)\n", startPos.x, startPos.y);

	float angleLeft=0.0, angleRight=0.0, angleAve=0.0;
	signed char pwmLeft=0, pwmRight=0;
	float turn;
	float gyro_sum = 0.0; 
	float startAngle = 0.0;

	int log_refleshrate = 10;

	driveWheels->GetAngles(&angleLeft, &angleRight);
	angleAve = (angleLeft+angleRight)/2.0;
	float firstAngleAve = angleAve;

	timer->Reset();
    // 1. forward切り替え位置までforward=70で進むx=60
	printf("PhaseSeesaw 1. Forward\n"); 
	int tmp_forward;
    while (true) {
		pos->UpdateSelfPos();
		posSelf = pos->GetSelfPos();
		thetaSelf = pos->GetTheta();

		tmp_forward = 60;
		line.CalcTurnValue();
		turn = line.GetTurn();
		poseDrivingControl.SetParams(tmp_forward,turn,TAIL_ANGLE,true);
		poseDrivingControl.Driving();

		driveWheels->GetAngles(&angleLeft, &angleRight);
		angleAve = (angleLeft+angleRight)/2.0;
        //ログは10回に1回出力する
#ifdef LOG
    	if( (tmp_log_cnt++) > log_refleshrate ){
            // driveWheels->GetAngles(&angleLeft, &angleRight);
            driveWheels->GetPWMs(&pwmLeft, &pwmRight);
			fprintf(file,"1,%f,%f,%f,%f,%f,%d,%f,%f,%d,%d,%d,%f\n",
           		timer->GetValue(), posSelf.x, posSelf.y, thetaSelf, tail->GetAngle(),tail->GetPWM(),angleLeft, angleRight, pwmLeft, pwmRight,  tmp_forward, postureSensor.GetAnglerVelocity());
	        tmp_log_cnt=0;
		}
#endif
    	//切り替え位置到達かチェック
		if(DISTANCE>SEESAW_ENTER_X-4){
    		break; //到達したので終了
    	}

    	tslp_tsk(4);
    }
	tslp_tsk(4);

    // 2. 傾く
	printf("PhaseSeesaw 2. Tilting\n"); 
	tmp_stop_cnt = 0;
	tmp_forward = 0; //PWM = 0を出力（※gyroの更新ありに変更した）
	gyro_sum = 0;
	poseDrivingControl.SetStop(false,false,false);
	poseDrivingControl.SetParams(tmp_forward,0,105,false);
	timer->Reset();
    while (true) {
		pos->UpdateSelfPos();
		posSelf = pos->GetSelfPos();
		thetaSelf = pos->GetTheta();

		poseDrivingControl.Driving();		

		driveWheels->GetAngles(&angleLeft, &angleRight);
		angleAve = (angleLeft+angleRight)/2.0;
#ifdef LOG
    	if( (tmp_log_cnt++) > log_refleshrate){
            // driveWheels->GetAngles(&angleLeft, &angleRight);
            driveWheels->GetPWMs(&pwmLeft, &pwmRight);
			fprintf(file,"2,%f,%f,%f,%f,%f,%d,%f,%f,%d,%d,%d,%f\n",
           		timer->GetValue(), posSelf.x, posSelf.y, thetaSelf, tail->GetAngle(),tail->GetPWM(),angleLeft, angleRight, pwmLeft, pwmRight,  tmp_forward, postureSensor.GetAnglerVelocity());
	        tmp_log_cnt=0;
		}
#endif
    	//切り替え条件成立かチェック
        //5ms×30回≒150msマスクする→検討した結果から決定
		if( (tmp_stop_cnt++) >= 10){
			gyro_sum += postureSensor.GetAnglerVelocity();
			if( gyro_sum > 1300 ){//角速度80より大きくなったら次の処理    			
				break;
			}
		}
	
    	tslp_tsk(4);
    }
	tslp_tsk(4);

#if 0
	// 1. 尻尾で着地
	printf("PhaseSeesaw 1.Deceleration to Land\n"); 
    timer->Reset();
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

        if ((tmp_log_cnt++) > log_refleshrate) {
            driveWheels->GetAngles(&angleLeft, &angleRight);
            driveWheels->GetPWMs(&pwmLeft, &pwmRight);
            fprintf(file,"%f,%f,%f,%f,%f,%d,%f,%d,%d,%f,%f,%f,%f,%f\n",
                timer->GetValue(), envViewer->GetLuminance(), postureSensor.GetAnglerVelocity(),
                -50.0,0.0,
                tail->GetPWM(),tail->GetAngle(),
                pwmLeft, pwmRight, angleLeft, angleRight,
                posSelf.x, posSelf.y, thetaSelf);
			tmp_log_cnt = 0;
        }
        
        tslp_tsk(4);
    }
	poseDrivingControl.SetParams(-10,0,75,true);
    while(true){
        if (abs(tail->GetAngle() - 75) <= 2) {
            break;
        }

		poseDrivingControl.Driving();

		pos->UpdateSelfPos();
		posSelf = pos->GetSelfPos();
		thetaSelf = pos->GetTheta();

        if ((tmp_log_cnt++) > log_refleshrate) {
            driveWheels->GetAngles(&angleLeft, &angleRight);
            driveWheels->GetPWMs(&pwmLeft, &pwmRight);
            fprintf(file,"%f,%f,%f,%f,%f,%d,%f,%d,%d,%f,%f,%f,%f,%f\n",
                timer->GetValue(), envViewer->GetLuminance(), postureSensor.GetAnglerVelocity(),
                0.0,0.0,
                tail->GetPWM(),tail->GetAngle(),
                pwmLeft, pwmRight, angleLeft, angleRight,
                posSelf.x, posSelf.y, thetaSelf);
			tmp_log_cnt = 0;
        }
         
        tslp_tsk(4);
    }
    poseDrivingControl.SetStop(true,true,true);

	printf("PhaseSeesaw 1.2.Landing\n"); 
	gyro_sum = 0.0;
    poseDrivingControl.SetParams(10,0,75,false);
    while(1){
        poseDrivingControl.Driving();

        if ((tmp_log_cnt++) > log_refleshrate) {
            driveWheels->GetAngles(&angleLeft, &angleRight);
            driveWheels->GetPWMs(&pwmLeft, &pwmRight);
            fprintf(file,"%f,%f,%f,%f,%f,%d,%f,%d,%d,%f,%f,%f,%f,%f\n",
                timer->GetValue(), envViewer->GetLuminance(), postureSensor.GetAnglerVelocity(),
                0.0,0.0,
                tail->GetPWM(),tail->GetAngle(),
                pwmLeft, pwmRight, angleLeft, angleRight,
                posSelf.x, posSelf.y, thetaSelf);
			tmp_log_cnt = 0;
        }
    	gyro_sum += postureSensor.GetAnglerVelocity();
        if(gyro_sum < -150){
            break;
        }
        tslp_tsk(4);
    }
    poseDrivingControl.SetStop(true,true,true);
    tslp_tsk(1000); // タイヤ完全停止待機

    // 2. 回転して、ラインを発見
	printf("PhaseSeesaw 2. Find Line\n"); 
    if(envViewer->GetLuminance() >= 50){ 
        poseDrivingControl.SetParams(-10.0,100,75,false);
        timer->Reset();
        while (true) {
            if (timer->Now()>700) {
                break;
            }   

            poseDrivingControl.Driving();

            pos->UpdateSelfPos();
            posSelf = pos->GetSelfPos();
            thetaSelf = pos->GetTheta();

            if ((tmp_log_cnt++) > log_refleshrate) {
                driveWheels->GetAngles(&angleLeft, &angleRight);
                driveWheels->GetPWMs(&pwmLeft, &pwmRight);
                fprintf(file,"%f,%f,%f,%f,%f,%d,%f,%d,%d,%f,%f,%f,%f,%f\n",
                    timer->GetValue(), envViewer->GetLuminance(), postureSensor.GetAnglerVelocity(),
                    -50.0,0.0,
                    tail->GetPWM(),tail->GetAngle(),
                    pwmLeft, pwmRight, angleLeft, angleRight,
                    posSelf.x, posSelf.y, thetaSelf);
				tmp_log_cnt = 0;
            }
            
            tslp_tsk(4);
        }
        poseDrivingControl.SetStop(true,true,true);
        tslp_tsk(1000); // タイヤ完全停止待機

        // poseDrivingControl.SetStop(false,false,false);
        while(true){
            poseDrivingControl.SetParams(10,100,75,false);
            poseDrivingControl.Driving();

            pos->UpdateSelfPos();
            posSelf = pos->GetSelfPos();
            thetaSelf = pos->GetTheta();

            if ((tmp_log_cnt++) > log_refleshrate) {
                driveWheels->GetAngles(&angleLeft, &angleRight);
                driveWheels->GetPWMs(&pwmLeft, &pwmRight);
                fprintf(file,"%f,%f,%f,%f,%f,%d,%f,%d,%d,%f,%f,%f,%f,%f\n",
                    timer->GetValue(), envViewer->GetLuminance(), postureSensor.GetAnglerVelocity(),
                    20.0,100.0,
                    tail->GetPWM(),tail->GetAngle(),
                    pwmLeft, pwmRight, angleLeft, angleRight,
                    posSelf.x, posSelf.y, thetaSelf);
				tmp_log_cnt = 0;
            }

            if(envViewer->GetLuminance() < 50){
                break;
            }

            tslp_tsk(4);
        }
        poseDrivingControl.SetStop(true,true,true);

        tslp_tsk(1000); // タイヤ完全停止待機
    }      

    // 3. ライントレースして、シーソーにぶつける
	printf("PhaseSeesaw 3. Linetrace until Bumping\n");
	int tmp_forward = 20;
	timer->Reset();
    while (true) {
		pos->UpdateSelfPos();
		posSelf = pos->GetSelfPos();
		thetaSelf = pos->GetTheta();

		line.CalcTurnValue();
		turn = line.GetTurn();
		poseDrivingControl.SetParams(5.0,turn,75,false);
		poseDrivingControl.Driving();

        tmp_log_cnt++;
    	if( tmp_log_cnt > log_refleshrate ){
            driveWheels->GetAngles(&angleLeft, &angleRight);
            driveWheels->GetPWMs(&pwmLeft, &pwmRight);
			fprintf(file,"%f,%f,%f,%f,%f,%d,%d,%d,%f\n",
           		timer->Now(), posSelf.x, posSelf.y, angleLeft, angleRight, pwmLeft, pwmRight,  tmp_forward, postureSensor.GetAnglerVelocity());
	        tmp_log_cnt=0;
		}

    	//切り替え位置到達かチェック
    	if(timer->Now()>5000){
    		break;
    	}

    	tslp_tsk(4);
    }

	tslp_tsk(1000);

    // 4.1. 傾ける
	printf("PhaseSeesaw 4.1. Tilting\n"); 
	poseDrivingControl.SetParams(-5.0,0.0,75,false);
	timer->Reset();
    while (true) {
		pos->UpdateSelfPos();
		posSelf = pos->GetSelfPos();
		thetaSelf = pos->GetTheta();

		poseDrivingControl.Driving();

        tmp_log_cnt++;
    	if( tmp_log_cnt > log_refleshrate ){
            driveWheels->GetAngles(&angleLeft, &angleRight);
            driveWheels->GetPWMs(&pwmLeft, &pwmRight);
			fprintf(file,"%f,%f,%f,%f,%f,%d,%d,%d,%f\n",
           		timer->Now(), posSelf.x, posSelf.y, angleLeft, angleRight, pwmLeft, pwmRight,  tmp_forward, postureSensor.GetAnglerVelocity());
	        tmp_log_cnt=0;
		}

    	//切り替え位置到達かチェック
    	if(timer->Now()>200){
    		break;
    	}

    	tslp_tsk(4);
    }
	poseDrivingControl.SetStop(true,true,true);
	tslp_tsk(1000);

	tmp_stop_cnt = 0;
	tmp_forward = 0; //PWM = 0を出力（※gyroの更新ありに変更した）
	gyro_sum = 0;
	poseDrivingControl.SetStop(false,false,false);
	poseDrivingControl.SetParams(tmp_forward,0,102,false);
    while (true) {
		pos->UpdateSelfPos();
		posSelf = pos->GetSelfPos();
		thetaSelf = pos->GetTheta();

		poseDrivingControl.Driving();		

    	if( (tmp_log_cnt++) > log_refleshrate){
			fprintf(file,"%f,%f,%f,%f,%f,%d,%d,%d,%f\n",
           		timer->Now(), posSelf.x, posSelf.y, angleLeft, angleRight, pwmLeft, pwmRight,  tmp_forward, postureSensor.GetAnglerVelocity());
	        tmp_log_cnt=0;
		}
    	gyro_sum += postureSensor.GetAnglerVelocity();

    	//切り替え条件成立かチェック
        //5ms×30回≒150msマスクする→検討した結果から決定
		if( (tmp_stop_cnt++) >= 10 && gyro_sum > 3600 ){//角速度80より大きくなったら次の処理    			
			break;
		}
	
    	tslp_tsk(4);
    }
#endif

    // 4.2. 乗り上げ
    ev3_speaker_play_tone(NOTE_C4, 10);
	printf("PhaseSeesaw 4.2.Forward to ride the Seesaw\n"); 
	tmp_forward = 100;
	ExponentialSmoothingFilter expFilter(0.8,0.0);
	while (true) {
		pos->UpdateSelfPos();
		posSelf = pos->GetSelfPos();
		thetaSelf = pos->GetTheta();

		tmp_forward = expFilter.GetValue(100.0);
		poseDrivingControl.SetParams(tmp_forward,0,TAIL_STOP_ANGLE,true);
		poseDrivingControl.Driving();	

		driveWheels->GetAngles(&angleLeft, &angleRight);
		angleAve = (angleLeft+angleRight)/2.0;
#ifdef LOG
        tmp_log_cnt++;
    	if( tmp_log_cnt > log_refleshrate ){
			// driveWheels->GetAngles(&angleLeft, &angleRight);
            driveWheels->GetPWMs(&pwmLeft, &pwmRight);
			fprintf(file,"4.2,%f,%f,%f,%f,%f,%d,%f,%f,%d,%d,%d,%f\n",
           		timer->GetValue(), posSelf.x, posSelf.y, thetaSelf, tail->GetAngle(),tail->GetPWM(),angleLeft, angleRight, pwmLeft, pwmRight,  tmp_forward, postureSensor.GetAnglerVelocity());
	        tmp_log_cnt=0;
		}
#endif
		if(DISTANCE>SEESAW_SLOW){
    		break; //所定位置まで到達した
    	}

    	tslp_tsk(4);
    }
	tslp_tsk(4);

    // 5. 乗り上げ後は、ゆっくり進む
	printf("PhaseSeesaw 5.Climbing\n"); 
	tmp_forward = 40;
	while (true) {
		pos->UpdateSelfPos();
		posSelf = pos->GetSelfPos();
		thetaSelf = pos->GetTheta();

		line.CalcTurnValue();
		turn = line.GetTurn()*0.4;
		poseDrivingControl.SetParams(tmp_forward,turn,TAIL_STOP_ANGLE,true);
		poseDrivingControl.Driving();		

		driveWheels->GetAngles(&angleLeft, &angleRight);
		angleAve = (angleLeft+angleRight)/2.0;
#ifdef LOG
    	if( (tmp_log_cnt++) > log_refleshrate ){
			// driveWheels->GetAngles(&angleLeft, &angleRight);
            driveWheels->GetPWMs(&pwmLeft, &pwmRight);
			fprintf(file,"5,%f,%f,%f,%f,%f,%d,%f,%f,%d,%d,%d,%f\n",
           		timer->GetValue(), posSelf.x, posSelf.y, thetaSelf, tail->GetAngle(),tail->GetPWM(),angleLeft, angleRight, pwmLeft, pwmRight,  tmp_forward, postureSensor.GetAnglerVelocity());
	        tmp_log_cnt=0;
		}
#endif
		if(DISTANCE>SEESAW_BACK){
    		break;
    	}

    	tslp_tsk(4);
    }
	tslp_tsk(4);
	
    // 6. シーソー下り開始に合わせて、シーソー上で停止 or バックさせる
	printf("PhaseSeesaw 6.Back when descending seesaw\n"); 
	// tmp_forward = 80.0;
	// poseDrivingControl.SetParams(tmp_forward,0,TAIL_STOP_ANGLE,false);
	// timer->Reset();
	// startAngle = angleAve;
	// while (true) {
    //     //自己位置更新
	// 	pos->UpdateSelfPos();
	// 	posSelf = pos->GetSelfPos();
	// 	thetaSelf = pos->GetTheta();

	// 	poseDrivingControl.Driving();		

	// 	driveWheels->GetAngles(&angleLeft, &angleRight);
	// 	angleAve = (angleLeft+angleRight)/2.0;
		
	// 	#ifdef LOG
    //     tmp_log_cnt++;
    // 	if( tmp_log_cnt > log_refleshrate ){
	// 		// driveWheels->GetAngles(&angleLeft, &angleRight);
    //         driveWheels->GetPWMs(&pwmLeft, &pwmRight);
	// 		fprintf(file,"6,%f,%f,%f,%f,%f,%d,%f,%f,%d,%d,%d,%f\n",
    //        		timer->GetValue(), posSelf.x, posSelf.y, thetaSelf, tail->GetAngle(),tail->GetPWM(),angleLeft, angleRight, pwmLeft, pwmRight,  tmp_forward, postureSensor.GetAnglerVelocity());
	//         tmp_log_cnt=0;
	// 	}
	// 	#endif

	// 	// if(timer->Now()>700){
	// 	if(angleAve - startAngle > 180){
    // 		break;
    // 	}

    // 	tslp_tsk(4);
    // }
	// tslp_tsk(4);
	tmp_forward = 40.0;
	poseDrivingControl.SetParams(tmp_forward,0,TAIL_STOP_ANGLE,false);
	while (true) {
        //自己位置更新
		pos->UpdateSelfPos();
		posSelf = pos->GetSelfPos();
		thetaSelf = pos->GetTheta();

		poseDrivingControl.Driving();		

		driveWheels->GetAngles(&angleLeft, &angleRight);
		angleAve = (angleLeft+angleRight)/2.0;
		#ifdef LOG
        tmp_log_cnt++;
    	if( tmp_log_cnt > log_refleshrate ){
			// driveWheels->GetAngles(&angleLeft, &angleRight);
            driveWheels->GetPWMs(&pwmLeft, &pwmRight);
			fprintf(file,"6,%f,%f,%f,%f,%f,%d,%f,%f,%d,%d,%d,%f\n",
           		timer->GetValue(), posSelf.x, posSelf.y, thetaSelf, tail->GetAngle(),tail->GetPWM(),angleLeft, angleRight, pwmLeft, pwmRight,  tmp_forward, postureSensor.GetAnglerVelocity());
	        tmp_log_cnt=0;
		}
		#endif
		if(DISTANCE>SEESAW_STOP){
    		break;
    	}

    	tslp_tsk(4);
    }
	poseDrivingControl.SetStop(true,true,true);
	tslp_tsk(400);
	poseDrivingControl.SetStop(false,true,true);

#if 1
    // 7. バック＋尻尾押上で、シーソーを背面登頂する
	printf("PhaseSeesaw 7.Back to climb seesaw\n"); 
	tmp_forward = -30.0;
	poseDrivingControl.SetParams(tmp_forward, 0, 80, false);
	while (true) {
		pos->UpdateSelfPos();
		posSelf = pos->GetSelfPos();
		thetaSelf = pos->GetTheta();
		
		poseDrivingControl.Driving();		

		driveWheels->GetAngles(&angleLeft, &angleRight);
		angleAve = (angleLeft+angleRight)/2.0;

    	if( (tmp_log_cnt++) > log_refleshrate ){
			// driveWheels->GetAngles(&angleLeft, &angleRight);
            driveWheels->GetPWMs(&pwmLeft, &pwmRight);
			fprintf(file,"%f,%f,%f,%f,%f,%d,%f,%f,%d,%d,%d,%f\n",
           		timer->GetValue(), posSelf.x, posSelf.y, thetaSelf, tail->GetAngle(),tail->GetPWM(),angleLeft, angleRight, pwmLeft, pwmRight,  tmp_forward, postureSensor.GetAnglerVelocity());
	        tmp_log_cnt=0;
		}

		if(DISTANCE<SEESAW_CLIBM_STOP){
    		break;
    	}

    	tslp_tsk(4);
    }
	tslp_tsk(4);
	tmp_forward = 0.0;
	tmp_stop_cnt = 0.0;
	gyro_sum = 0.0;
	poseDrivingControl.SetParams(tmp_forward, 0, 125, false);
	while (true) {
		pos->UpdateSelfPos();
		posSelf = pos->GetSelfPos();
		thetaSelf = pos->GetTheta();
		
		poseDrivingControl.Driving();		

		driveWheels->GetAngles(&angleLeft, &angleRight);
		angleAve = (angleLeft+angleRight)/2.0;

    	if( (tmp_log_cnt++) > log_refleshrate ){
			// driveWheels->GetAngles(&angleLeft, &angleRight);
            driveWheels->GetPWMs(&pwmLeft, &pwmRight);
			fprintf(file,"%f,%f,%f,%f,%f,%d,%f,%f,%d,%d,%d,%f\n",
           		timer->GetValue(), posSelf.x, posSelf.y, thetaSelf, tail->GetAngle(),tail->GetPWM(),angleLeft, angleRight, pwmLeft, pwmRight,  tmp_forward, postureSensor.GetAnglerVelocity());
	        tmp_log_cnt=0;
		}

		if( (tmp_stop_cnt++) >= 15){
			gyro_sum += postureSensor.GetAnglerVelocity();
			if( gyro_sum > 900 ){    			
				break;
			}
		}

    	tslp_tsk(4);
    }
	tslp_tsk(4);

    // 8. 再度シーソー下り開始に合わせて、シーソー上で停止 or バックさせる
	printf("PhaseSeesaw 8.Back when descending seesaw\n"); 
	tmp_forward = 80.0;
	poseDrivingControl.SetParams(tmp_forward,0,TAIL_STOP_ANGLE,false);
	timer->Reset();
	startAngle = angleAve;
	while (true) {
        //自己位置更新
		pos->UpdateSelfPos();
		posSelf = pos->GetSelfPos();
		thetaSelf = pos->GetTheta();

		poseDrivingControl.Driving();		

		driveWheels->GetAngles(&angleLeft, &angleRight);
		angleAve = (angleLeft+angleRight)/2.0;
		
		#ifdef LOG
        tmp_log_cnt++;
    	if( tmp_log_cnt > log_refleshrate ){
			// driveWheels->GetAngles(&angleLeft, &angleRight);
            driveWheels->GetPWMs(&pwmLeft, &pwmRight);
			fprintf(file,"6,%f,%f,%f,%f,%f,%d,%f,%f,%d,%d,%d,%f\n",
           		timer->GetValue(), posSelf.x, posSelf.y, thetaSelf, tail->GetAngle(),tail->GetPWM(),angleLeft, angleRight, pwmLeft, pwmRight,  tmp_forward, postureSensor.GetAnglerVelocity());
	        tmp_log_cnt=0;
		}
		#endif

		// if(timer->Now()>700){
		if(angleAve - startAngle > 270){
    		break;
    	}

    	tslp_tsk(4);
    }
	tslp_tsk(4);
	tmp_forward = 40.0;
	poseDrivingControl.SetParams(tmp_forward,0,TAIL_STOP_ANGLE,false);
	while (true) {
        //自己位置更新
		pos->UpdateSelfPos();
		posSelf = pos->GetSelfPos();
		thetaSelf = pos->GetTheta();

		poseDrivingControl.Driving();		

		driveWheels->GetAngles(&angleLeft, &angleRight);
		angleAve = (angleLeft+angleRight)/2.0;
		#ifdef LOG
        tmp_log_cnt++;
    	if( tmp_log_cnt > log_refleshrate ){
			// driveWheels->GetAngles(&angleLeft, &angleRight);
            driveWheels->GetPWMs(&pwmLeft, &pwmRight);
			fprintf(file,"6,%f,%f,%f,%f,%f,%d,%f,%f,%d,%d,%d,%f\n",
           		timer->GetValue(), posSelf.x, posSelf.y, thetaSelf, tail->GetAngle(),tail->GetPWM(),angleLeft, angleRight, pwmLeft, pwmRight,  tmp_forward, postureSensor.GetAnglerVelocity());
	        tmp_log_cnt=0;
		}
		#endif
		if(DISTANCE>SEESAW_STOP){
    		break;
    	}

    	tslp_tsk(4);
    }
	tslp_tsk(4);
	poseDrivingControl.SetStop(false,true,true);
#endif

	// 9. シーソーから降りて前進する
	printf("PhaseSeesaw 9.Forward to get off\n"); 
	tmp_forward = 60;
	ExponentialSmoothingFilter expFilter2(0.99,60.0);
	while (true) {
		pos->UpdateSelfPos();
		posSelf = pos->GetSelfPos();
		thetaSelf = pos->GetTheta();
		
		tmp_forward = expFilter2.GetValue(30.0);
		poseDrivingControl.SetParams(tmp_forward, 0, 60, false);
		poseDrivingControl.Driving();	

		driveWheels->GetAngles(&angleLeft, &angleRight);
		angleAve = (angleLeft+angleRight)/2.0;
    	if( (tmp_log_cnt++) > log_refleshrate ){
            // driveWheels->GetAngles(&angleLeft, &angleRight);
            driveWheels->GetPWMs(&pwmLeft, &pwmRight);
			fprintf(file,"9,%f,%f,%f,%f,%f,%d,%f,%f,%d,%d,%d,%f\n",
           		timer->GetValue(), posSelf.x, posSelf.y, thetaSelf, tail->GetAngle(),tail->GetPWM(),angleLeft, angleRight, pwmLeft, pwmRight,  tmp_forward, postureSensor.GetAnglerVelocity());
	        tmp_log_cnt=0;
		}

		if(DISTANCE>SEESAW_PRE_END_STOP){
    		break;
    	}

		tslp_tsk(4);
	} 
	ExponentialSmoothingFilter expFilter3(0.99,tmp_forward);
	while (true) {
		pos->UpdateSelfPos();
		posSelf = pos->GetSelfPos();
		thetaSelf = pos->GetTheta();
		
		tmp_forward = expFilter2.GetValue(5.0);
		poseDrivingControl.SetParams(tmp_forward, 0, 61, false);
		poseDrivingControl.Driving();	

		driveWheels->GetAngles(&angleLeft, &angleRight);
		angleAve = (angleLeft+angleRight)/2.0;
    	if( (tmp_log_cnt++) > log_refleshrate ){
            // driveWheels->GetAngles(&angleLeft, &angleRight);
            driveWheels->GetPWMs(&pwmLeft, &pwmRight);
			fprintf(file,"9,%f,%f,%f,%f,%f,%d,%f,%f,%d,%d,%d,%f\n",
           		timer->GetValue(), posSelf.x, posSelf.y, thetaSelf, tail->GetAngle(),tail->GetPWM(),angleLeft, angleRight, pwmLeft, pwmRight,  tmp_forward, postureSensor.GetAnglerVelocity());
	        tmp_log_cnt=0;
		}

		if(DISTANCE>SEESAW_END_STOP){
    		break;
    	}

		tslp_tsk(4);
	}
	poseDrivingControl.SetStop(false,true,true);
	
	tmp_forward = -5;
	poseDrivingControl.SetParams(tmp_forward, 0, 60, false);
	while (true) {
		pos->UpdateSelfPos();
		posSelf = pos->GetSelfPos();
		thetaSelf = pos->GetTheta();
		
		poseDrivingControl.Driving();	

		driveWheels->GetAngles(&angleLeft, &angleRight);
		angleAve = (angleLeft+angleRight)/2.0;
    	if( (tmp_log_cnt++) > log_refleshrate ){
            // driveWheels->GetAngles(&angleLeft, &angleRight);
            driveWheels->GetPWMs(&pwmLeft, &pwmRight);
			fprintf(file,"9,%f,%f,%f,%f,%f,%d,%f,%f,%d,%d,%d,%f\n",
           		timer->GetValue(), posSelf.x, posSelf.y, thetaSelf, tail->GetAngle(),tail->GetPWM(),angleLeft, angleRight, pwmLeft, pwmRight,  tmp_forward, postureSensor.GetAnglerVelocity());
	        tmp_log_cnt=0;
		}

		if(abs(tail->GetAngle()-60)<3){
    		break;
    	}

		tslp_tsk(4);
	}
	poseDrivingControl.SetStop(true,true,true);

	fclose(file);
	finFlg = true;
	printf("PhaseSeesaw Execute done\n");
}

bool PhaseSeesaw::IsFinish(){
	return false;
}

#endif