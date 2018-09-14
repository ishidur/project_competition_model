#include "PhaseNavigation.h"
#include <stdio.h>

#include "../BaseHardware/Timer.h"
#include "../DrivingControl/PoseDrivingControl.h"
#include "../AppliedHardware/VehicleHardware/Tail.h"
#include "../AppliedHardware/VehicleHardware/PostureSensor.h"
#include "../AppliedHardware/VehicleHardware/DriveWheels.h"
#include "../AppliedHardware/EnvironmentSensor/EnvironmentViewer.h"
#include "../EnvironmentMeasurement/LineLuminance.h"
#include "../Positioning/Localization/SelfPos.h"
#include "../Utilities/Vector2D.h"
#include "../Utilities/ExponentialSmoothingFilter.h"

using namespace Phase;
using namespace	DrivingControl;
using namespace AppliedHardware::VehicleHardware;
using namespace AppliedHardware::EnvironmentSensor;
using namespace BaseHardware;
using namespace EnvironmentMeasurement;
using namespace Positioning::Localization;
using namespace Utilities;

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
}

void PhaseNavigation::Execute(){
	printf("PhaseNavigation Execute\n");

	PostureSensor postureSensor; 
	Tail* tail = Tail::GetInstance();
    Timer* cl = Timer::GetInstance();
	

    FILE* file;
    char filename[255] = {};
    sprintf(filename, "/ev3rt/res/log_data_linetrace.csv");
    file = fopen(filename, "w");
    fprintf(file,"clock,caribratedBrightness,gyroSensor,USdist,power,turn,PWMt,motor_ang_t,PWMl,PWMr,motor_ang_l,motor_ang_r,xEst,yEst,thetaSelf\n");
    
    int frameCount = 0;
    const int log_refleshrate = 15;


	float angleLeft=0.0, angleRight=0.0;
	signed char pwmLeft=0, pwmRight=0;
	float turn=0.0;
    Vector2D posSelf(0,0);
	float thetaSelf = 0.0;

    pos->Start();

	pos->UpdateSelfPos();
	posSelf = pos->GetSelfPos();
	thetaSelf = pos->GetTheta();
	poseDrivingControl.SetStop(false,false,false);

	// 1. 尻尾スタートダッシュ
	printf("PhaseNavigation 1.StartDash\n");
	int tmp_high_speed_start_forward = 0;
	int tmp_high_speed_start_cnt = 0;
	int tmp_high_speed_gyro_sum = 0;
	float now_gyro;

	poseDrivingControl.SetParams(tmp_high_speed_start_forward,0.0,110,false);
    cl->Reset();
	while(true){
		poseDrivingControl.Driving();

		pos->UpdateSelfPos();
		posSelf = pos->GetSelfPos();
		thetaSelf = pos->GetTheta();

		now_gyro = postureSensor.GetAnglerVelocity();

		if ((frameCount++) % log_refleshrate == 0) {
				driveWheels->GetAngles(&angleLeft, &angleRight);
				driveWheels->GetPWMs(&pwmLeft, &pwmRight);
				fprintf(file,"%f,%f,%f,%f,%f,%f,%d,%f,%d,%d,%f,%f,%f,%f,%f\n",
					cl->GetValue(), envViewer->GetLuminance(), now_gyro,envViewer->GetUSDistance(),
					0.0,0.0,
					tail->GetPWM(),tail->GetAngle(),
					pwmLeft, pwmRight, angleLeft, angleRight,
					posSelf.x, posSelf.y, thetaSelf);
		}     

    	tmp_high_speed_gyro_sum += now_gyro;
    	//切り替え条件成立かチェック
        //5ms×10回≒50msマスクする→検討した結果から決定
        if( (tmp_high_speed_start_cnt++)>10 ){
        	if( tmp_high_speed_gyro_sum > 1500 || (tmp_high_speed_gyro_sum > 800 && now_gyro > 80)){
    			//角速度80より大きくなったら次の処理
    			break;
    		}
    	}

		tslp_tsk(4);
	}

	// 2. 初期安定化前進
	printf("PhaseNavigation 2.BalanceForward\n");
	Vector2D startPos = pos->GetSelfPos();
	poseDrivingControl.SetParams(150.0,0,TAIL_ANGLE,true);
	driveWheels->GetAngles(&angleLeft, &angleRight);
    while (true){
		poseDrivingControl.Driving();

		pos->UpdateSelfPos();
		posSelf = pos->GetSelfPos();
		thetaSelf = pos->GetTheta();

		if ((frameCount++) % log_refleshrate == 0) {
				driveWheels->GetAngles(&angleLeft, &angleRight);
				driveWheels->GetPWMs(&pwmLeft, &pwmRight);
				fprintf(file,"%f,%f,%f,%f,%f,%f,%d,%f,%d,%d,%f,%f,%f,%f,%f\n",
					cl->GetValue(), envViewer->GetLuminance(), postureSensor.GetAnglerVelocity(),envViewer->GetUSDistance(),
					40.0,0.0,
					tail->GetPWM(),tail->GetAngle(),
					pwmLeft, pwmRight, angleLeft, angleRight,
					posSelf.x, posSelf.y, thetaSelf);
		}     

		driveWheels->GetAngles(&angleLeft, &angleRight);
		if(posSelf.DistanceFrom(startPos)>7){
			break;
		}	

		tslp_tsk(4);
    }

	// 3. 輝度ライントレース
	printf("PhaseNavigation 3.Linetrace\n");
	LineLuminance line;
	ExponentialSmoothingFilter expFilter(0.5,150.0);
    int fileterSize = 100;
    int cnt = 0;
	float forward;
    float caribratedBrightnesses[fileterSize]={0.0};
    float greyBottom = 45.0;
    float greyTop = 55.0;
    float varianceCriteria = 5.0;
    while (true) {        
		line.CalcTurnValue();
		turn = line.GetTurn();

		forward = expFilter.GetValue(90.0);
		poseDrivingControl.SetParams(forward,turn,TAIL_ANGLE,true);
		poseDrivingControl.Driving();

		pos->UpdateSelfPos();
		posSelf = pos->GetSelfPos();
		thetaSelf = pos->GetTheta();

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

	finFlg = true;
	printf("PhaseNavigation Execute done\n");
}

bool PhaseNavigation::IsFinish(Vector2D posSelf) {
    if(this->course=='R'){//R(Seesaw)
		return (posSelf.x > 0.0 && posSelf.y > 48.0);
        // return (posSelf.x < 170.0 && posSelf.y < 180.0);    
    }else if(this->course=='L'){//L(LookUpGate)
		// return (posSelf.x < 137.0 && posSelf.y > 335.0);
        return (envViewer->GetTouch()||envViewer->GetUSDistance()<15);
		// return (posSelf.x < 0.0 && posSelf.y > 80.0);
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