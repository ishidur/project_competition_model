#include "PhaseSeesaw.h"

#include <stdlib.h>
#include <stdio.h>
#include "../Utilities/Vector2D.h"
#include "../EnvironmentMeasurement/LineLuminance.h"

using namespace Phase;
using namespace AppliedHardware::VehicleHardware;
using namespace AppliedHardware::EnvironmentSensor;
using namespace BaseHardware;
using namespace Positioning::Localization;
using namespace Utilities;
using namespace EnvironmentMeasurement;

#define TAIL_ANGLE (45)

PhaseSeesaw::PhaseSeesaw(){
	pos = SelfPos::GetInstance();
	tail = Tail::GetInstance();

	envViewer = EnvironmentViewer::GetInstance();
	driveWheels = DriveWheels::GetInstance();
	timer = Timer::GetInstance();
}

void PhaseSeesaw::Execute(){
	printf("PhaseSeesaw Execute\n");
	
	LineLuminance line;

    FILE* file;
    char filename[255] = {};
    sprintf(filename, "/ev3rt/res/log_data_seesaw.csv");
    file = fopen(filename, "w");
    
    Vector2D posSelf(0,0);
	pos->UpdateSelfPos();
	posSelf = pos->GetSelfPos();
    float thetaSelf = pos->GetTheta();
	Vector2D startPos = pos->GetSelfPos();
	printf("start (x,y) = (%f, %f)\n", startPos.x, startPos.y);

	float angleLeft=0.0, angleRight=0.0;
	signed char pwmLeft=0, pwmRight=0;
	float turn;
	float gyro_sum = 0.0;

	int log_refleshrate = 5;

	poseDrivingControl.SetStop(true,false,false);
	
	timer->Reset();

    // 1. forward切り替え位置までforward=70で進むx=60
	printf("PhaseSeesaw 1. Forward\n"); 
	int tmp_forward = 30;
    while (true) {
		pos->UpdateSelfPos();
		posSelf = pos->GetSelfPos();
		thetaSelf = pos->GetTheta();

    	if(posSelf.DistanceFrom(startPos)>26.0-10.0){ // 倒立振子で前進
			poseDrivingControl.SetParams(tmp_forward,0,TAIL_ANGLE,true);
    	}else{ // まずはライントレース
			line.CalcTurnValue();
			turn = line.GetTurn();
			poseDrivingControl.SetParams(tmp_forward,turn,TAIL_ANGLE,true);
    	}
		poseDrivingControl.Driving();

        //ログは10回に1回出力する
    	if( (tmp_log_cnt++) > log_refleshrate ){
            driveWheels->GetAngles(&angleLeft, &angleRight);
            driveWheels->GetPWMs(&pwmLeft, &pwmRight);
			fprintf(file,"%f,%f,%f,%f,%d,%f,%f,%d,%d,%d,%f\n",
           		timer->GetValue(), posSelf.x, posSelf.y, tail->GetAngle(),tail->GetPWM(),angleLeft, angleRight, pwmLeft, pwmRight,  tmp_forward, postureSensor.GetAnglerVelocity());
	        tmp_log_cnt=0;
		}

    	//切り替え位置到達かチェック
    	// if( IsStartEnter( posSelf.x ) == true ){
		if(posSelf.DistanceFrom(startPos)>26.0){
    		break; //到達したので終了
    	}

    	tslp_tsk(4);
    }
    //※切り替え位置到達時はwhileのtslp_tsk(4)が実行されないので所定時間待つこと
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

    	if( (tmp_log_cnt++) > log_refleshrate){
            driveWheels->GetAngles(&angleLeft, &angleRight);
            driveWheels->GetPWMs(&pwmLeft, &pwmRight);
			fprintf(file,"%f,%f,%f,%f,%d,%f,%f,%d,%d,%d,%f\n",
           		timer->GetValue(), posSelf.x, posSelf.y, tail->GetAngle(),tail->GetPWM(),angleLeft, angleRight, pwmLeft, pwmRight,  tmp_forward, postureSensor.GetAnglerVelocity());
	        tmp_log_cnt=0;
		}

    	//切り替え条件成立かチェック
        //5ms×30回≒150msマスクする→検討した結果から決定
		// if(timer->Now()>3000){    			
		if( (tmp_stop_cnt++) >= 10){
			gyro_sum += postureSensor.GetAnglerVelocity();
			if( gyro_sum > 1300 ){//角速度80より大きくなったら次の処理    			
				break;
			}
		}
	
    	tslp_tsk(4);
    }

	// 1. 尻尾で着地
	// printf("PhaseSeesaw 1.Deceleration to Land\n"); 
    // timer->Reset();
    // int tau = 500;	
	// poseDrivingControl.SetParams(-50.0,0,75,true);
    // while (true) {
    //     if (timer->Now()>tau) {
    //         break;
    //     }   

	// 	poseDrivingControl.Driving();

	// 	pos->UpdateSelfPos();
	// 	posSelf = pos->GetSelfPos();
	// 	thetaSelf = pos->GetTheta();

    //     if ((tmp_log_cnt++) > log_refleshrate) {
    //         driveWheels->GetAngles(&angleLeft, &angleRight);
    //         driveWheels->GetPWMs(&pwmLeft, &pwmRight);
    //         fprintf(file,"%f,%f,%f,%f,%f,%d,%f,%d,%d,%f,%f,%f,%f,%f\n",
    //             timer->GetValue(), envViewer->GetLuminance(), postureSensor.GetAnglerVelocity(),
    //             -50.0,0.0,
    //             tail->GetPWM(),tail->GetAngle(),
    //             pwmLeft, pwmRight, angleLeft, angleRight,
    //             posSelf.x, posSelf.y, thetaSelf);
	// 		tmp_log_cnt = 0;
    //     }
        
    //     tslp_tsk(4);
    // }
	// poseDrivingControl.SetParams(-10,0,75,true);
    // while(true){
    //     if (abs(tail->GetAngle() - 75) <= 2) {
    //         break;
    //     }

	// 	poseDrivingControl.Driving();

	// 	pos->UpdateSelfPos();
	// 	posSelf = pos->GetSelfPos();
	// 	thetaSelf = pos->GetTheta();

    //     if ((tmp_log_cnt++) > log_refleshrate) {
    //         driveWheels->GetAngles(&angleLeft, &angleRight);
    //         driveWheels->GetPWMs(&pwmLeft, &pwmRight);
    //         fprintf(file,"%f,%f,%f,%f,%f,%d,%f,%d,%d,%f,%f,%f,%f,%f\n",
    //             timer->GetValue(), envViewer->GetLuminance(), postureSensor.GetAnglerVelocity(),
    //             0.0,0.0,
    //             tail->GetPWM(),tail->GetAngle(),
    //             pwmLeft, pwmRight, angleLeft, angleRight,
    //             posSelf.x, posSelf.y, thetaSelf);
	// 		tmp_log_cnt = 0;
    //     }
         
    //     tslp_tsk(4);
    // }
    // poseDrivingControl.SetStop(true,true,true);

	// printf("PhaseSeesaw 1.2.Landing\n"); 
	// gyro_sum = 0.0;
    // poseDrivingControl.SetParams(10,0,75,false);
    // while(1){
    //     poseDrivingControl.Driving();

    //     if ((tmp_log_cnt++) > log_refleshrate) {
    //         driveWheels->GetAngles(&angleLeft, &angleRight);
    //         driveWheels->GetPWMs(&pwmLeft, &pwmRight);
    //         fprintf(file,"%f,%f,%f,%f,%f,%d,%f,%d,%d,%f,%f,%f,%f,%f\n",
    //             timer->GetValue(), envViewer->GetLuminance(), postureSensor.GetAnglerVelocity(),
    //             0.0,0.0,
    //             tail->GetPWM(),tail->GetAngle(),
    //             pwmLeft, pwmRight, angleLeft, angleRight,
    //             posSelf.x, posSelf.y, thetaSelf);
	// 		tmp_log_cnt = 0;
    //     }
    // 	gyro_sum += postureSensor.GetAnglerVelocity();
    //     if(gyro_sum < -150){
    //         break;
    //     }
    //     tslp_tsk(4);
    // }
    // poseDrivingControl.SetStop(true,true,true);
    // tslp_tsk(1000); // タイヤ完全停止待機

    // // 2. 回転して、ラインを発見
	// printf("PhaseSeesaw 2. Find Line\n"); 
    // if(envViewer->GetLuminance() >= 50){ 
    //     poseDrivingControl.SetParams(-10.0,100,75,false);
    //     timer->Reset();
    //     while (true) {
    //         if (timer->Now()>700) {
    //             break;
    //         }   

    //         poseDrivingControl.Driving();

    //         pos->UpdateSelfPos();
    //         posSelf = pos->GetSelfPos();
    //         thetaSelf = pos->GetTheta();

    //         if ((tmp_log_cnt++) > log_refleshrate) {
    //             driveWheels->GetAngles(&angleLeft, &angleRight);
    //             driveWheels->GetPWMs(&pwmLeft, &pwmRight);
    //             fprintf(file,"%f,%f,%f,%f,%f,%d,%f,%d,%d,%f,%f,%f,%f,%f\n",
    //                 timer->GetValue(), envViewer->GetLuminance(), postureSensor.GetAnglerVelocity(),
    //                 -50.0,0.0,
    //                 tail->GetPWM(),tail->GetAngle(),
    //                 pwmLeft, pwmRight, angleLeft, angleRight,
    //                 posSelf.x, posSelf.y, thetaSelf);
	// 			tmp_log_cnt = 0;
    //         }
            
    //         tslp_tsk(4);
    //     }
    //     poseDrivingControl.SetStop(true,true,true);
    //     tslp_tsk(1000); // タイヤ完全停止待機

    //     // poseDrivingControl.SetStop(false,false,false);
    //     while(true){
    //         poseDrivingControl.SetParams(10,100,75,false);
    //         poseDrivingControl.Driving();

    //         pos->UpdateSelfPos();
    //         posSelf = pos->GetSelfPos();
    //         thetaSelf = pos->GetTheta();

    //         if ((tmp_log_cnt++) > log_refleshrate) {
    //             driveWheels->GetAngles(&angleLeft, &angleRight);
    //             driveWheels->GetPWMs(&pwmLeft, &pwmRight);
    //             fprintf(file,"%f,%f,%f,%f,%f,%d,%f,%d,%d,%f,%f,%f,%f,%f\n",
    //                 timer->GetValue(), envViewer->GetLuminance(), postureSensor.GetAnglerVelocity(),
    //                 20.0,100.0,
    //                 tail->GetPWM(),tail->GetAngle(),
    //                 pwmLeft, pwmRight, angleLeft, angleRight,
    //                 posSelf.x, posSelf.y, thetaSelf);
	// 			tmp_log_cnt = 0;
    //         }

    //         if(envViewer->GetLuminance() < 50){
    //             break;
    //         }

    //         tslp_tsk(4);
    //     }
    //     poseDrivingControl.SetStop(true,true,true);

    //     tslp_tsk(1000); // タイヤ完全停止待機
    // }      

    // // 3. ライントレースして、シーソーにぶつける
	// printf("PhaseSeesaw 3. Linetrace until Bumping\n");
	// int tmp_forward = 20;
	// timer->Reset();
    // while (true) {
	// 	pos->UpdateSelfPos();
	// 	posSelf = pos->GetSelfPos();
	// 	thetaSelf = pos->GetTheta();

	// 	line.CalcTurnValue();
	// 	turn = line.GetTurn();
	// 	poseDrivingControl.SetParams(5.0,turn,75,false);
	// 	poseDrivingControl.Driving();

    //     tmp_log_cnt++;
    // 	if( tmp_log_cnt > log_refleshrate ){
    //         driveWheels->GetAngles(&angleLeft, &angleRight);
    //         driveWheels->GetPWMs(&pwmLeft, &pwmRight);
	// 		fprintf(file,"%f,%f,%f,%f,%f,%d,%d,%d,%f\n",
    //        		timer->Now(), posSelf.x, posSelf.y, angleLeft, angleRight, pwmLeft, pwmRight,  tmp_forward, postureSensor.GetAnglerVelocity());
	//         tmp_log_cnt=0;
	// 	}

    // 	//切り替え位置到達かチェック
    // 	if(timer->Now()>5000){
    // 		break;
    // 	}

    // 	tslp_tsk(4);
    // }

	// tslp_tsk(1000);

    // // 4.1. 傾ける
	// printf("PhaseSeesaw 4.1. Tilting\n"); 
	// poseDrivingControl.SetParams(-5.0,0.0,75,false);
	// timer->Reset();
    // while (true) {
	// 	pos->UpdateSelfPos();
	// 	posSelf = pos->GetSelfPos();
	// 	thetaSelf = pos->GetTheta();

	// 	poseDrivingControl.Driving();

    //     tmp_log_cnt++;
    // 	if( tmp_log_cnt > log_refleshrate ){
    //         driveWheels->GetAngles(&angleLeft, &angleRight);
    //         driveWheels->GetPWMs(&pwmLeft, &pwmRight);
	// 		fprintf(file,"%f,%f,%f,%f,%f,%d,%d,%d,%f\n",
    //        		timer->Now(), posSelf.x, posSelf.y, angleLeft, angleRight, pwmLeft, pwmRight,  tmp_forward, postureSensor.GetAnglerVelocity());
	//         tmp_log_cnt=0;
	// 	}

    // 	//切り替え位置到達かチェック
    // 	if(timer->Now()>200){
    // 		break;
    // 	}

    // 	tslp_tsk(4);
    // }
	// poseDrivingControl.SetStop(true,true,true);
	// tslp_tsk(1000);

	// tmp_stop_cnt = 0;
	// tmp_forward = 0; //PWM = 0を出力（※gyroの更新ありに変更した）
	// gyro_sum = 0;
	// poseDrivingControl.SetStop(false,false,false);
	// poseDrivingControl.SetParams(tmp_forward,0,102,false);
    // while (true) {
	// 	pos->UpdateSelfPos();
	// 	posSelf = pos->GetSelfPos();
	// 	thetaSelf = pos->GetTheta();

	// 	poseDrivingControl.Driving();		

    // 	if( (tmp_log_cnt++) > log_refleshrate){
	// 		fprintf(file,"%f,%f,%f,%f,%f,%d,%d,%d,%f\n",
    //        		timer->Now(), posSelf.x, posSelf.y, angleLeft, angleRight, pwmLeft, pwmRight,  tmp_forward, postureSensor.GetAnglerVelocity());
	//         tmp_log_cnt=0;
	// 	}
    // 	gyro_sum += postureSensor.GetAnglerVelocity();

    // 	//切り替え条件成立かチェック
    //     //5ms×30回≒150msマスクする→検討した結果から決定
	// 	if( (tmp_stop_cnt++) >= 10 && gyro_sum > 3600 ){//角速度80より大きくなったら次の処理    			
	// 		break;
	// 	}
	
    // 	tslp_tsk(4);
    // }

    //※切り替え位置到達時はwhileのtslp_tsk(4)が実行されないので所定時間待つこと
	tslp_tsk(4);

    // 4.2. forward100で乗り上げ
	printf("PhaseSeesaw 4.2.Forward to ride the Seesaw\n"); 
	tmp_forward = 100;
	poseDrivingControl.SetParams(tmp_forward,0,55,true);
	while (true) {
		pos->UpdateSelfPos();
		posSelf = pos->GetSelfPos();
		thetaSelf = pos->GetTheta();

		poseDrivingControl.Driving();	

        tmp_log_cnt++;
    	if( tmp_log_cnt > log_refleshrate ){
			driveWheels->GetAngles(&angleLeft, &angleRight);
            driveWheels->GetPWMs(&pwmLeft, &pwmRight);
			fprintf(file,"%f,%f,%f,%f,%d,%f,%f,%d,%d,%d,%f\n",
           		timer->GetValue(), posSelf.x, posSelf.y, tail->GetAngle(),tail->GetPWM(),angleLeft, angleRight, pwmLeft, pwmRight,  tmp_forward, postureSensor.GetAnglerVelocity());
	        tmp_log_cnt=0;
		}

    	//乗り上げた後は、速度下げる
    	// if( IsStartSlowForward( posSelf.x ) == true ){
		if(posSelf.DistanceFrom(startPos)>26.0+15.0){
    		break; //所定位置まで到達した
    	}

    	tslp_tsk(4);
    }
    //※切り替え位置到達時はwhileのtslp_tsk(4)が実行されないので所定時間待つこと
	tslp_tsk(4);

    // 5. 乗り上げ後は、forward40でゆっくり進む
	printf("PhaseSeesaw 5.Climbing\n"); 
	tmp_forward = 40;
	while (true) {
		pos->UpdateSelfPos();
		posSelf = pos->GetSelfPos();
		thetaSelf = pos->GetTheta();

		line.CalcTurnValue();
		turn = line.GetTurn()*0.2;
		poseDrivingControl.SetParams(tmp_forward,turn,55,true);
		poseDrivingControl.Driving();		

    	if( (tmp_log_cnt++) > log_refleshrate ){
			driveWheels->GetAngles(&angleLeft, &angleRight);
            driveWheels->GetPWMs(&pwmLeft, &pwmRight);
			fprintf(file,"%f,%f,%f,%f,%d,%f,%f,%d,%d,%d,%f\n",
           		timer->GetValue(), posSelf.x, posSelf.y, tail->GetAngle(),tail->GetPWM(),angleLeft, angleRight, pwmLeft, pwmRight,  tmp_forward, postureSensor.GetAnglerVelocity());
	        tmp_log_cnt=0;
		}

    	//back待ち
		if(posSelf.DistanceFrom(startPos)>26.0+50.0){
        // if( IsStartBackForward( posSelf.x ) == true ){
    		break;
    	}

    	tslp_tsk(4);
    }
    //※切り替え位置到達時はwhileのtslp_tsk(4)が実行されないので所定時間待つこと
	tslp_tsk(4);

    // 6. シーソー下り開始に合わせて、forward=-40でシーソー上で停止 or バックさせる
	printf("PhaseSeesaw 6.Back when descending seesaw\n"); 
	tmp_forward = 60.0;
	poseDrivingControl.SetParams(tmp_forward, 0, 55, false);
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
			fprintf(file,"%f,%f,%f,%f,%d,%f,%f,%d,%d,%d,%f\n",
           		timer->GetValue(), posSelf.x, posSelf.y, tail->GetAngle(),tail->GetPWM(),angleLeft, angleRight, pwmLeft, pwmRight,  tmp_forward, postureSensor.GetAnglerVelocity());
	        tmp_log_cnt=0;
		}

    	//back待ち
		if(posSelf.DistanceFrom(startPos)>26.0+60.0){
        // if( IsStartStopForward( posSelf.x ) == true ){
    		break;
    	}

    	tslp_tsk(4);
    }
    //※切り替え位置到達時はwhileのtslp_tsk(4)が実行されないので所定時間待つこと
	tslp_tsk(4);
	poseDrivingControl.SetStop(true,true,true);

	tslp_tsk(1000);
	// poseDrivingControl.SetStop(false,true,true);

	// 7. シーソーから降りて前進する forward 70
	printf("PhaseSeesaw 7.Forward to get off\n"); 
	tmp_forward = 40;
	poseDrivingControl.SetParams(tmp_forward,0,60,false);
	while (true) {
		pos->UpdateSelfPos();
		posSelf = pos->GetSelfPos();
		thetaSelf = pos->GetTheta();
		
		poseDrivingControl.Driving();	

    	if( (tmp_log_cnt++) > log_refleshrate ){
            driveWheels->GetAngles(&angleLeft, &angleRight);
            driveWheels->GetPWMs(&pwmLeft, &pwmRight);
			fprintf(file,"%f,%f,%f,%f,%d,%f,%f,%d,%d,%d,%f\n",
           		timer->GetValue(), posSelf.x, posSelf.y, tail->GetAngle(),tail->GetPWM(),angleLeft, angleRight, pwmLeft, pwmRight,  tmp_forward, postureSensor.GetAnglerVelocity());
	        tmp_log_cnt=0;
		}

    	//back待ち
		if(posSelf.DistanceFrom(startPos)>26.0+95.0){
        // if( IsEndStopForward( posSelf.x ) == true ){
    		break;
    	}

		tslp_tsk(4);
	}
	poseDrivingControl.SetStop(true,true,true);
	// シーソー上で停止

	tslp_tsk(1000);
	poseDrivingControl.SetStop(false,true,true);

	// 8. 着地
	printf("PhaseSeesaw 8.Landing and Stop\n"); 
    while(true){
        if (abs(tail->GetAngle() - 70) < 3) {
            break;
        }
        poseDrivingControl.SetParams(-10,0,70,false);
        poseDrivingControl.Driving();

        pos->UpdateSelfPos();
        posSelf = pos->GetSelfPos();
        thetaSelf = pos->GetTheta();

    	if( (tmp_log_cnt++) > log_refleshrate ){
            driveWheels->GetAngles(&angleLeft, &angleRight);
            driveWheels->GetPWMs(&pwmLeft, &pwmRight);
			fprintf(file,"%f,%f,%f,%f,%d,%f,%f,%d,%d,%d,%f\n",
           		timer->GetValue(), posSelf.x, posSelf.y, tail->GetAngle(),tail->GetPWM(),angleLeft, angleRight, pwmLeft, pwmRight,  tmp_forward, postureSensor.GetAnglerVelocity());
	        tmp_log_cnt=0;
		}
        tslp_tsk(4);
    }
    poseDrivingControl.SetStop(true,true,true);
	poseDrivingControl.SetParams(10,0,70,false);
	poseDrivingControl.Driving();
	tslp_tsk(500);
    poseDrivingControl.SetStop(true,true,true);
    // tslp_tsk(1000); // タイヤ完全停止待機

    // poseDrivingControl.SetStop(false,false,false);
    // while(true){
    //     if (abs(tail->GetAngle() - 80) < 3) {
    //         break;
    //     }
    //     poseDrivingControl.SetParams(0,0,80,false);
    //     poseDrivingControl.Driving();

    //     pos->UpdateSelfPos();
    //     posSelf = pos->GetSelfPos();
    //     thetaSelf = pos->GetTheta();

    // 	if( (tmp_log_cnt++) > log_refleshrate ){
    //         driveWheels->GetAngles(&angleLeft, &angleRight);
    //         driveWheels->GetPWMs(&pwmLeft, &pwmRight);
	// 		fprintf(file,"%f,%f,%f,%f,%d,%f,%f,%d,%d,%d,%f\n",
    //        		timer->GetValue(), posSelf.x, posSelf.y, tail->GetAngle(),tail->GetPWM(),angleLeft, angleRight, pwmLeft, pwmRight,  tmp_forward, postureSensor.GetAnglerVelocity());
	//         tmp_log_cnt=0;
	// 	}
    //     tslp_tsk(4);
    // }
    // poseDrivingControl.SetStop(true,true,true);
    

	finFlg = true;
	printf("PhaseSeesaw Execute done\n");
}

#define SEESAW_ENTER_X_FOR_LINE_TRACE 48
#define SEESAW_ENTER_X_FOR_NO_LINE_TRACE 60 // 70位置でライントレースなし

#define SEESAW_ENTER_X ( SEESAW_ENTER_X_FOR_LINE_TRACE )
#define SEESAW_PRE_ENTER_X ( SEESAW_ENTER_X_FOR_LINE_TRACE - 10 )
#define SEESAW_SLOW ( SEESAW_ENTER_X + 25 )
#define SEESAW_BACK ( SEESAW_ENTER_X + 45 )
#define SEESAW_STOP ( SEESAW_ENTER_X + 55 )
#define SEESAW_END_STOP ( SEESAW_ENTER_X + 60 )

//以下のパラメータで座標切り替え判断する
bool PhaseSeesaw::IsStartForward( float _x ){
	bool ret = false;
	if( _x > SEESAW_PRE_ENTER_X ){
		ret = true;
	}
	return ret;
}

bool PhaseSeesaw::IsStartEnter( float _x ){
	bool ret = false;
	if( _x > SEESAW_ENTER_X ){
		ret = true;
	}
	return ret;
}

bool PhaseSeesaw::IsStartSlowForward( float _x ){
	bool ret = false;
	if( _x > SEESAW_SLOW ){
		ret = true;
	}
	return ret;
}

bool PhaseSeesaw::IsStartBackForward( float _x ){
	bool ret = false;
	if( _x > SEESAW_BACK ){
		ret = true;
	}
	return ret;
}

bool PhaseSeesaw::IsStartStopForward( float _x ){
	bool ret = false;
	if( _x > SEESAW_STOP ){
		ret = true;
	}
	return ret;
}


bool PhaseSeesaw::IsEndStopForward( float _x ){
	bool ret = false;
	if( _x > SEESAW_END_STOP ){
		ret = true;
	}
	return ret;
}

bool PhaseSeesaw::IsFinish(){
	return false;
}
