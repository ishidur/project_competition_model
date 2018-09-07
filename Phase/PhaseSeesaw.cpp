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

#define TAIL_ANGLE 45

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
    float thetaSelf=-3.141592/2;

	float angleLeft=0.0, angleRight=0.0;
	signed char pwmLeft=0, pwmRight=0;
	float turn;

	poseDrivingControl.SetStop(false,false,false);
	
	timer->Reset();

    //foward切り替え位置までfoward =70で進むx=60
	int tmp_foward = 70;
    while (true) {

        //自己位置更新
		pos->UpdateSelfPos();
		posSelf = pos->GetSelfPos();
		thetaSelf = pos->GetTheta();

    	//倒立振子で前進
    	if( IsStartFoward( posSelf.x ) == true ){
			poseDrivingControl.SetParams(tmp_foward,0,TAIL_ANGLE,true);
    	}
    	//まずはライントレース
    	else{
			line.CalcTurnValue();
			turn = line.GetTurn();
			poseDrivingControl.SetParams(tmp_foward,turn,TAIL_ANGLE,true);
    	}
		poseDrivingControl.Driving();

        //ログは10回に1回出力する
        tmp_log_cnt++;
    	if( tmp_log_cnt > 10 ){
            driveWheels->GetAngles(&angleLeft, &angleRight);
            driveWheels->GetPWMs(&pwmLeft, &pwmRight);
			fprintf(file,"%f,%f,%f,%f,%f,%d,%d,%d,%f\n",
           		timer->Now(), posSelf.x, posSelf.y, angleLeft, angleRight, pwmLeft, pwmRight,  tmp_foward, postureSensor.GetAnglerVelocity());
	        tmp_log_cnt=0;
		}

    	//切り替え位置到達かチェック
    	if( IsStartEnter( posSelf.x ) == true ){
    		//到達したので終了
    		break;
    	}
    	else{
    		//到達していないので待つ
    	}

    	tslp_tsk(4);
    }
    //※切り替え位置到達時はwhileのtslp_tsk(4)が実行されないので所定時間待つこと
	tslp_tsk(4);

    //傾ける
	tmp_stop_cnt = 0;
	tmp_foward = 0;
	float gyro_sum = 0;
	poseDrivingControl.SetParams(tmp_foward,0,TAIL_ANGLE,true);
    while (true) {

        //自己位置更新
		pos->UpdateSelfPos();
		posSelf = pos->GetSelfPos();
		thetaSelf = pos->GetTheta();

        //PWM = 0を出力（※gyroの更新ありに変更した）
		poseDrivingControl.Driving();		

        //ログは10回に1回出力する
        tmp_log_cnt++;
    	if( tmp_log_cnt > 0){
			fprintf(file,"%f,%f,%f,%f,%f,%d,%d,%d,%f\n",
           		timer->Now(), posSelf.x, posSelf.y, angleLeft, angleRight, pwmLeft, pwmRight,  tmp_foward, postureSensor.GetAnglerVelocity());
	        tmp_log_cnt=0;
		}
    	gyro_sum += postureSensor.GetAnglerVelocity();

    	//切り替え条件成立かチェック
        //5ms×30回≒150msマスクする→検討した結果から決定
        if( tmp_stop_cnt < 10 ){
        	//所定回数≒所定時間経過待ち
    	}
        else{
    		//所定回数≒所定時間経過した
//    		if( GyroSensor::anglerVelocity > 70 ){
        	if( gyro_sum > 800 ){
    			//角速度80より大きくなったら次の処理
    			break;
    		}
    	}
    	tmp_stop_cnt++;
    	tslp_tsk(4);
    }

    //※切り替え位置到達時はwhileのtslp_tsk(4)が実行されないので所定時間待つこと
	tslp_tsk(4);

    //シーソー検討
    //foward100で乗り上げ
	tmp_foward = 100;
	poseDrivingControl.SetParams(tmp_foward,0,TAIL_ANGLE,true);
	while (true) {

        //自己位置更新
		pos->UpdateSelfPos();
		posSelf = pos->GetSelfPos();
		thetaSelf = pos->GetTheta();

       	//倒立振子で前進
		poseDrivingControl.Driving();	

        //ログは10回に1回出力する
        tmp_log_cnt++;
    	if( tmp_log_cnt > 10 ){
			fprintf(file,"%f,%f,%f,%f,%f,%d,%d,%d,%f\n",
           		timer->Now(), posSelf.x, posSelf.y, angleLeft, angleRight, pwmLeft, pwmRight,  tmp_foward, postureSensor.GetAnglerVelocity());
	        tmp_log_cnt=0;
		}

    	//乗り上げた後は、速度下げる
    	if( IsStartSlowFoward( posSelf.x ) == true ){
    		//所定位置まで到達した
    		break;
    	}
    	else{
    		//所定位置到達待ち
    	}

    	tslp_tsk(4);
    }
    //※切り替え位置到達時はwhileのtslp_tsk(4)が実行されないので所定時間待つこと
	tslp_tsk(4);

    //乗り上げ後は、foward40でゆっくり進む
	tmp_foward = 40;
	poseDrivingControl.SetParams(tmp_foward,0,TAIL_ANGLE,true);
	while (true) {

        //自己位置更新
		pos->UpdateSelfPos();
		posSelf = pos->GetSelfPos();
		thetaSelf = pos->GetTheta();
		
       	//倒立振子で前進
		poseDrivingControl.Driving();		

        //ログは10回に1回出力する
        tmp_log_cnt++;
    	if( tmp_log_cnt > 10 ){
			fprintf(file,"%f,%f,%f,%f,%f,%d,%d,%d,%f\n",
           		timer->Now(), posSelf.x, posSelf.y, angleLeft, angleRight, pwmLeft, pwmRight,  tmp_foward, postureSensor.GetAnglerVelocity());
	        tmp_log_cnt=0;
		}

    	//back待ち
        if( IsStartBackFoward( posSelf.x ) == true ){
    		break;
    	}
        else{
    	}

    	tslp_tsk(4);
    }
    //※切り替え位置到達時はwhileのtslp_tsk(4)が実行されないので所定時間待つこと
	tslp_tsk(4);

    //シーソー下り開始に合わせて、foward=-40でシーソー上で停止 or バックさせる
	tmp_foward = -40;
	poseDrivingControl.SetParams(tmp_foward,0,TAIL_ANGLE,true);
	while (true) {

        //自己位置更新
		pos->UpdateSelfPos();
		posSelf = pos->GetSelfPos();
		thetaSelf = pos->GetTheta();
		
		poseDrivingControl.Driving();		

        tmp_log_cnt++;
    	if( tmp_log_cnt > 10 ){
			fprintf(file,"%f,%f,%f,%f,%f,%d,%d,%d,%f\n",
           		timer->Now(), posSelf.x, posSelf.y, angleLeft, angleRight, pwmLeft, pwmRight,  tmp_foward, postureSensor.GetAnglerVelocity());
	        tmp_log_cnt=0;
		}

    	//back待ち
        if( IsStartStopFoward( posSelf.x ) == true ){
    		break;
    	}
        else{
    	}

    	tslp_tsk(4);
    }
    //※切り替え位置到達時はwhileのtslp_tsk(4)が実行されないので所定時間待つこと
	tslp_tsk(4);


	//シーソーから降りて前進する foward 70
	tmp_foward = 70;
	poseDrivingControl.SetParams(tmp_foward,0,TAIL_ANGLE,true);
	while (true) {
		pos->UpdateSelfPos();
		posSelf = pos->GetSelfPos();
		thetaSelf = pos->GetTheta();
		
		poseDrivingControl.Driving();	

		tmp_log_cnt++;
		if( tmp_log_cnt > 10 ){
			fprintf(file,"%f,%f,%f,%f,%f,%d,%d,%d,%f\n",
           		timer->Now(), posSelf.x, posSelf.y, angleLeft, angleRight, pwmLeft, pwmRight,  tmp_foward, postureSensor.GetAnglerVelocity());
	        tmp_log_cnt=0;
		}

    	//back待ち
        if( IsEndStopFoward( posSelf.x ) == true ){
    		break;
    	}
        else{
    	}

		tslp_tsk(4);
	}

	// 着地
    int tau = 1000;	
    timer->Reset();
	poseDrivingControl.SetParams(-50.0,0,65,true);
    while (true) {
		pos->UpdateSelfPos();
		posSelf = pos->GetSelfPos();
		thetaSelf = pos->GetTheta();

		poseDrivingControl.Driving();

		tmp_log_cnt++;
		if( tmp_log_cnt > 10 ){
			fprintf(file,"%f,%f,%f,%f,%f,%d,%d,%d,%f\n",
           		timer->Now(), posSelf.x, posSelf.y, angleLeft, angleRight, pwmLeft, pwmRight,  tmp_foward, postureSensor.GetAnglerVelocity());
	        tmp_log_cnt=0;
		}

        if (timer->Now()>tau) {
			poseDrivingControl.SetParams(0,0,65,false);
			poseDrivingControl.Driving();
            poseDrivingControl.SetStop(true,false,false);
            break;
        }   
        
        tslp_tsk(4);
    }

	poseDrivingControl.SetParams(0,0,70,true);
    while(true){
		pos->UpdateSelfPos();
		posSelf = pos->GetSelfPos();
		thetaSelf = pos->GetTheta();

		poseDrivingControl.Driving();

		tmp_log_cnt++;
		if( tmp_log_cnt > 10 ){
			fprintf(file,"%f,%f,%f,%f,%f,%d,%d,%d,%f\n",
           		timer->Now(), posSelf.x, posSelf.y, angleLeft, angleRight, pwmLeft, pwmRight,  tmp_foward, postureSensor.GetAnglerVelocity());
	        tmp_log_cnt=0;
		}

        if (abs(tail->GetAngle() - 70) < 5) {
			poseDrivingControl.SetParams(30,0,65,false);
			poseDrivingControl.Driving();
            tslp_tsk(800);
            poseDrivingControl.SetStop(true,true,true);
            break;
        }
         
        tslp_tsk(4);
    }

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
bool PhaseSeesaw::IsStartFoward( float _x ){
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

bool PhaseSeesaw::IsStartSlowFoward( float _x ){
	bool ret = false;
	if( _x > SEESAW_SLOW ){
		ret = true;
	}
	return ret;
}

bool PhaseSeesaw::IsStartBackFoward( float _x ){
	bool ret = false;
	if( _x > SEESAW_BACK ){
		ret = true;
	}
	return ret;
}

bool PhaseSeesaw::IsStartStopFoward( float _x ){
	bool ret = false;
	if( _x > SEESAW_STOP ){
		ret = true;
	}
	return ret;
}


bool PhaseSeesaw::IsEndStopFoward( float _x ){
	bool ret = false;
	if( _x > SEESAW_END_STOP ){
		ret = true;
	}
	return ret;
}

bool PhaseSeesaw::IsFinish(){
	return false;
}
