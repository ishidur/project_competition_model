#include "PhaseNavigation.h"
#include <stdio.h>

#include "../BaseHardware/Timer.h"
#include "../DrivingControl/PoseDrivingControl.h"
#include "../AppliedHardware/VehicleHardware/Tail.h"
#include "../AppliedHardware/VehicleHardware/PostureSensor.h"
#include "../AppliedHardware/VehicleHardware/DriveWheels.h"
#include "../AppliedHardware/EnvironmentSensor/EnvironmentViewer.h"
#include "../EnvironmentMeasurement/LineLuminance.h"
#include "../Navigation/Navigation.h"
#include "../Positioning/Localization/SelfPos.h"
#include "../Utilities/Vector2D.h"
#include "../Utilities/ExponentialSmoothingFilter.h"

using namespace Phase;
using namespace	DrivingControl;
using namespace AppliedHardware::VehicleHardware;
using namespace AppliedHardware::Communication;
using namespace AppliedHardware::EnvironmentSensor;
using namespace BaseHardware;
using namespace EnvironmentMeasurement;
using namespace Positioning::Localization;
using namespace Utilities;

#define LOG_NAVI

#define TAIL_ANGLE (50)

PhaseNavigation::PhaseNavigation(){
	const char* filename = "/ev3rt/res/course/course.txt";
	if(!ReadCourse(filename)){
		printf("no file %s!\n", filename);
		ter_tsk(MAIN_TASK);
	}
	printf("get course: %c\n", course);

	pos = SelfPos::GetInstance();
	envViewer = EnvironmentViewer::GetInstance();
	driveWheels = DriveWheels::GetInstance();
	navigation = new Navigation::Navigation();
	com = Communication::GetInstance();
}

void PhaseNavigation::Execute(){
	printf("PhaseNavigation Execute\n");

	PostureSensor postureSensor;
	Tail* tail = Tail::GetInstance();
    Timer* cl = Timer::GetInstance();
	
	com->Connect();

    FILE* file;
    char filename[255] = {};
    sprintf(filename, "/ev3rt/res/log_data_linetrace.csv");
    file = fopen(filename, "w");
    fprintf(file,"clock,caribratedBrightness,gyroSensor,USdist,power,turn,PWMt,motor_ang_t,PWMl,PWMr,motor_ang_l,motor_ang_r,xEst,yEst,thetaSelf\n");

    int frameCount = 0;
    const int log_refleshrate = 0;
	int com_count = 0;


	float angleLeft=0.0, angleRight=0.0;
	signed char pwmLeft=0, pwmRight=0;
	float turn=0.0;
    Vector2D posSelf(0,0);
	float thetaSelf = 0.0;
#if 0
    pos->Start();

	pos->UpdateSelfPos();
	posSelf = pos->GetSelfPos();
	thetaSelf = pos->GetTheta();

	//navi開始
	navigation->Start();
#endif

	// 1. 尻尾スタートダッシュ
	printf("PhaseNavigation 1.StartDash\n");
	int tmp_high_speed_start_forward = 0;
	int tmp_high_speed_start_cnt = 0;
	int tmp_high_speed_gyro_sum = 0;
	float now_gyro;

	poseDrivingControl.SetParams(tmp_high_speed_start_forward,0.0,110,false);
	poseDrivingControl.SetStop(false,false,false);
	cl->Reset();
	while(true){
		poseDrivingControl.Driving();
#if 0
		pos->UpdateSelfPos();
		posSelf = pos->GetSelfPos();
		thetaSelf = pos->GetTheta();
#endif
		now_gyro = postureSensor.GetAnglerVelocity();

#ifdef LOG_NAVI
		if ((frameCount++) > log_refleshrate) {
			driveWheels->GetAngles(&angleLeft, &angleRight);
			driveWheels->GetPWMs(&pwmLeft, &pwmRight);
			fprintf(file,"%f,%f,%f,%f,%f,%f,%d,%f,%d,%d,%f,%f,%f,%f,%f\n",
				cl->GetValue(), envViewer->GetLuminance(), postureSensor.GetAnglerVelocity(),envViewer->GetUSDistance(),
				0.0,0.0,
				tail->GetPWM(),tail->GetAngle(),
				pwmLeft, pwmRight, angleLeft, angleRight,
				posSelf.x, posSelf.y, thetaSelf);
			frameCount = 0;
		}
#endif

    	tmp_high_speed_gyro_sum += now_gyro;
    	//切り替え条件成立かチェック
        //5ms×10回≒50msマスクする→検討した結果から決定
        if( (tmp_high_speed_start_cnt++)>10 ){
        	if( tmp_high_speed_gyro_sum > 1500 || (tmp_high_speed_gyro_sum > 900 && now_gyro > 80)){
    			//角速度80より大きくなったら次の処理
    			break;
    		}
    	}

		tslp_tsk(4);
	}

	// 2. 初期安定化前進
	printf("PhaseNavigation 2.BalanceForward\n");
	//移動した
    pos->Start();
	pos->UpdateSelfPos();
	posSelf = pos->GetSelfPos();
	thetaSelf = pos->GetTheta();

	//navi開始
	// navigation->Start();

	Vector2D startPos = pos->GetSelfPos();
	float pre_foward = 150;
	poseDrivingControl.SetParams(pre_foward,0,TAIL_ANGLE,true);
	driveWheels->GetAngles(&angleLeft, &angleRight);
    while (true){
		poseDrivingControl.Driving();

		pos->UpdateSelfPos();
		posSelf = pos->GetSelfPos();
		thetaSelf = pos->GetTheta();
		
		driveWheels->GetAngles(&angleLeft, &angleRight);
#ifdef LOG_NAVI
		if ((frameCount++) > log_refleshrate) {
			driveWheels->GetPWMs(&pwmLeft, &pwmRight);
			fprintf(file,"%f,%f,%f,%f,%f,%f,%d,%f,%d,%d,%f,%f,%f,%f,%f\n",
				cl->GetValue(), envViewer->GetLuminance(), postureSensor.GetAnglerVelocity(),envViewer->GetUSDistance(),
				pre_foward,0.0,
				tail->GetPWM(),tail->GetAngle(),
				pwmLeft, pwmRight, angleLeft, angleRight,
				posSelf.x, posSelf.y, thetaSelf);
			frameCount = 0;
		}
#endif

		if(posSelf.DistanceFrom(startPos)>7&&angleRight>100){
			break;
		}

		tslp_tsk(4);
    }

	// 3. 輝度ライントレース
	printf("PhaseNavigation 3.Linetrace\n");
	LineLuminance line;
	ExponentialSmoothingFilter expFilter(0.5,150.0);
	float tmp_forward = 70;

    int filterSize = 20;
    int cnt = 0;
	bool max_flg = false;
    float caribratedBrightnesses[filterSize]={0.0};
    Vector2D poses[filterSize];
    float greyBottom = 42.0;
    float greyTop = 48.0;
    float varianceCriteria = 5.0;
	float variance, sum, squaredSum;
	float nowluminance, lastluminance = envViewer->GetLuminance(), delta;
	cl->Reset();
		line.CalcTurnValue();
		turn = line.GetTurn();
		poseDrivingControl.SetParams(tmp_forward,turn,TAIL_ANGLE,true);
		poseDrivingControl.Driving();

		nowluminance = envViewer->GetLuminance();
        caribratedBrightnesses[cnt] = nowluminance;
		poses[cnt] = posSelf;
		// lastluminance = nowluminance;
		if(cnt==filterSize-1&&cl->Now()>2000) max_flg = true;

        sum=0.0;
        squaredSum=0.0;
        for (int i = cnt; i < filterSize/2;i++) {
			delta = (caribratedBrightnesses[i]-caribratedBrightnesses[(i-10)%filterSize]);
			delta = delta*delta;
			sum += delta;
			squaredSum += delta*delta;
        }
		sum /= filterSize;
        variance = squaredSum/filterSize - sum*sum;
		if (max_flg&&variance<varianceCriteria &&
				greyBottom<nowluminance&&nowluminance<greyTop && 
				poses[cnt].DistanceFrom(poses[(cnt-10)%filterSize])>3){
            ev3_speaker_play_tone(NOTE_C4, 100);
			poseDrivingControl.SetParams(0,0,TAIL_ANGLE,true);
            while(1){
				poseDrivingControl.Driving();
			}
        }
		cnt = (cnt+1)%filterSize;

#ifdef LOG_NAVI
		if ((frameCount++) > log_refleshrate) {
			driveWheels->GetAngles(&angleLeft, &angleRight);
			driveWheels->GetPWMs(&pwmLeft, &pwmRight);
			fprintf(file,"%f,%f,%f,%f,%f,%f,%d,%f,%d,%d,%f,%f,%f,%f,%f,%f\n",
				cl->GetValue(), envViewer->GetLuminance(), postureSensor.GetAnglerVelocity(),envViewer->GetUSDistance(),
				tmp_forward,turn,
				tail->GetPWM(),tail->GetAngle(),
				pwmLeft, pwmRight, angleLeft, angleRight,
				posSelf.x, posSelf.y, thetaSelf, variance);
			frameCount = 0;
		}
#endif

		// if((com_count++) > 15){
		// 	char mes[255];
		// 	sprintf(mes, "{'x':%f,'y':%f,'theta':%f}\n", posSelf.x, posSelf.y, thetaSelf);
		// 	com->SendString(mes);
		// 	com_count = 0;
		// }

		tslp_tsk(4);
	}

//	仮想ライントレース用
// 	float tmp_turn = 0;
// #if 0	//仮想ライントレースデバッグ用
// 	float dbg=0;
// 	Vector2D dbg_v_self(0,0);
// 	Vector2D dbg_v(0,0);
// 	Vector2D dbg_v_dir(0,0);
// 	float tmp_rho = 0;
// 	Vector2D dbg_near(0,0);
// 	Vector2D dbg_tgt(0,0);
// 	float dbg_turn_base=0;
// 	float dbg_cross = 0;
// 	float dbg_phi = 0;
// 	Vector2D dbg_mean(0,0);
// #endif

//     while (true) {
// 		//自己位置更新
// 		pos->UpdateSelfPos();
// 		posSelf = pos->GetSelfPos();
// 		thetaSelf = pos->GetTheta();

//     	//navi更新
//     	navigation->Update();
//     	navigation->CalcTurn();
//     	tmp_turn = navigation->GetTurn();
//     	navigation->CalcForward();
//     	tmp_forward = navigation->GetForward();

// #if 0	//仮想ライントレースデバッグ用
//     	dbg = navigation->GetDbg();
//     	dbg_v_self = navigation->GetDbgV();
//     	dbg_v = navigation->GetNode();
//     	dbg_v_dir = navigation->GetDir();
//     	tmp_rho = navigation->GetRho();
//     	dbg_near = navigation->GetNear();
//     	dbg_tgt = navigation->GetTgt();
//     	dbg_turn_base = navigation->GetTurnBase();
//     	dbg_cross = navigation->GetCross();

//     	dbg_phi = pos->GetPhi();
//     	dbg_mean = pos->GetMean();
// #endif

// 		line.CalcTurnValue();
// 		turn = line.GetTurn();

// 		//最終nodeの場合のturn
// 		if( navigation->IsEndNode() == true ){
// 			turn = tmp_turn;
// 		}

// //相談する		forward = expFilter.GetValue(90.0);
// 		forward = tmp_forward;
// 		poseDrivingControl.SetParams(forward,turn,TAIL_ANGLE,true);
// 		poseDrivingControl.Driving();


// 		if ((frameCount++) % log_refleshrate == 0) {
// 				driveWheels->GetAngles(&angleLeft, &angleRight);
// 				driveWheels->GetPWMs(&pwmLeft, &pwmRight);
// 				fprintf(file,"%f,%f,%f,%f,%f,%f,%d,%f,%d,%d,%f,%f,%f,%f,%f\n",
// 					cl->GetValue(), envViewer->GetLuminance(), postureSensor.GetAnglerVelocity(),envViewer->GetUSDistance(),
// 					forward,turn,
// 					tail->GetPWM(),tail->GetAngle(),
// 					pwmLeft, pwmRight, angleLeft, angleRight,
// 					posSelf.x, posSelf.y, thetaSelf);
// #if 0	//仮想ライントレースデバッグ用
// 				fprintf(file,"C,%f,%f,%f,%f,%f,%f,%d,%f,%d,%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f, %f,%f,%f, %f, %f,%f,%f,%f,%f,%f,%f,%f\n",
// 					cl->GetValue(), envViewer->GetLuminance(), postureSensor.GetAnglerVelocity(),envViewer->GetUSDistance(),
// 					tmp_forward,tmp_turn,
// 					tail->GetPWM(),tail->GetAngle(),
// 					pwmLeft, pwmRight, angleLeft, angleRight,
// 					posSelf.x, posSelf.y, thetaSelf, dbg,dbg_v.x,dbg_v.y,dbg_v_dir.x,dbg_v_dir.y,dbg_v_self.x,dbg_v_self.y,tmp_rho,dbg_near.x,dbg_near.y,dbg_tgt.x,dbg_tgt.y,dbg_turn_base,dbg_cross,dbg_phi, dbg_mean.x, dbg_mean.y );
// #endif
// 		}

//         if (IsFinish(posSelf)) {
//             break;
//         }

//         tslp_tsk(4);
//     }

<<<<<<< HEAD
=======
        if (cnt == fileterSize) {
          cnt = 0;
        }
        caribratedBrightnesses[cnt] = envViewer->GetLuminance();
        cnt++;
        float sum=0.0;
        float squaredSum=0.0;
        for (int i = 0; i < fileterSize;i++) {
          sum += caribratedBrightnesses[i];
          squaredSum += caribratedBrightnesses[i]* caribratedBrightnesses[i];
        }
        float variance =( squaredSum * fileterSize - sum * sum)/(fileterSize*(fileterSize-1));
		if ((frameCount++) % log_refleshrate == 0) {
				driveWheels->GetAngles(&angleLeft, &angleRight);
				driveWheels->GetPWMs(&pwmLeft, &pwmRight);
				fprintf(file,"%f,%f,%f,%f,%f,%f,%d,%f,%d,%d,%f,%f,%f,%f,%f,%f\n",
					cl->GetValue(), envViewer->GetLuminance(), postureSensor.GetAnglerVelocity(),envViewer->GetUSDistance(),
					forward,turn,
					tail->GetPWM(),tail->GetAngle(),
					pwmLeft, pwmRight, angleLeft, angleRight,
					posSelf.x, posSelf.y, thetaSelf,variance);
		}     
        if (variance<varianceCriteria&&greyBottom<envViewer->GetLuminance()&&envViewer->GetLuminance()<greyTop)
        {
            ev3_speaker_play_tone(NOTE_C4, 10);
            break;
        }
        // if (IsFinish(posSelf)) {
        //     break;
        // }

        frameCount++;  
        tslp_tsk(4);
    }

>>>>>>> 分散値による
	finFlg = true;
	printf("PhaseNavigation Execute done\n");
}

bool PhaseNavigation::IsFinish(Vector2D posSelf) {
    if(this->course=='R'){//R(Seesaw)
		// return ( posSelf.x > 0.0 && posSelf.y > 48.0 );
        // return ( (posSelf.x < 170.0 && posSelf.y < 180.0 )
		return ( navigation->IsFinish() == true );
    }else if(this->course=='L'){//L(LookUpGate)
		// return (posSelf.x < 137.0 && posSelf.y > 335.0);
<<<<<<< HEAD
		return ( navigation->IsFinish() == true );
//		return (posSelf.x < 0.0 && posSelf.y > 80.0);
=======
        return (envViewer->GetTouch()||envViewer->GetUSDistance()<15);
		// return (posSelf.x < 0.0 && posSelf.y > 80.0);
>>>>>>> 分散値による
    }
    return (envViewer->GetTouch()||envViewer->GetUSDistance()<15);
}

void PhaseNavigation::Navigation(){
}

int PhaseNavigation::ReadCourse(const char* filename){
    FILE* option_file = fopen(filename,"r");
	if(option_file==NULL) return 0;

	char param_name[255] = {'\0'};
	fscanf(option_file,"%s %c", &param_name[0], &course);

	fclose(option_file);
	return 1;
}