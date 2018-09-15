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
#define TAIL_STOP_ANGLE (55)

#define OFFSET_FIRSTPOS (0)

#define SEESAW_ENTER_X_FOR_LINE_TRACE (0)
#define SEESAW_ENTER_X_FOR_NO_LINE_TRACE (110)
//-SEESAW_ENTER_X_FOR_LINE_TRACE) // 70位置でライントレースなし

#define SEESAW_ENTER_X ( SEESAW_ENTER_X_FOR_NO_LINE_TRACE )
#define SEESAW_PRE_ENTER_X ( SEESAW_ENTER_X_FOR_NO_LINE_TRACE - 60 )
#define SEESAW_SLOW ( SEESAW_ENTER_X + 250 )
#define SEESAW_BACK ( SEESAW_ENTER_X + 610 )
#define SEESAW_STOP ( SEESAW_ENTER_X + 800 )
#define SEESAW_CLIBM_STOP ( SEESAW_ENTER_X + 550 )
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

    	// if(posSelf.DistanceFrom(startPos)>SEESAW_PRE_ENTER_X){ // 倒立振子で前進
		// 	tmp_forward = expFilter.GetValue(100.0);
		// 	poseDrivingControl.SetParams(tmp_forward,0,TAIL_ANGLE,true);
    	// }else{ // まずはライントレース
			tmp_forward = 70;
			line.CalcTurnValue();
			turn = line.GetTurn();
			poseDrivingControl.SetParams(tmp_forward,turn,TAIL_ANGLE,true);
    	// }
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

#if 1
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
		turn = line.GetTurn()*0.5;
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
	tmp_forward = 80.0;
	poseDrivingControl.SetParams(tmp_forward,0,TAIL_STOP_ANGLE,false);
	timer->Reset();
	float startAngle = angleAve;
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

		if(angleAve - startAngle > 180){
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
	poseDrivingControl.SetStop(true,true,true);

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
			fprintf(file,"2,%f,%f,%f,%f,%f,%d,%f,%f,%d,%d,%d,%f\n",
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
#endif

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