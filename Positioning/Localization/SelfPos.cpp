#include "SelfPos.h"
#include <stdio.h>
#include <string.h>

#define SELF_POS_X_INIT 499.14	//Lコース(検討用初期位置x）
#define SELF_POS_Y_INIT 339.51	//Lコース(検討用初期位置y）
#define SELF_POS_THETA_INIT 3.141592	//Lコース(検討用初期角度）
#define SELF_VEL_X_INIT -1		//Lコース(検討用初期速度x）
#define SELF_VEL_Y_INIT 0		//Lコース(検討用初期速度y）

using namespace Positioning::Localization;
using namespace Utilities;

SelfPos* SelfPos::singletonInstance = nullptr;

SelfPos::SelfPos():vSelf(SELF_POS_X_INIT,SELF_POS_Y_INIT),thetaSelf(SELF_POS_THETA_INIT),vSelfVel(SELF_VEL_X_INIT,SELF_VEL_Y_INIT){
	Odmetry = new CalcSelfPosWithOdmetry();
	ReadFirstPos("/ev3rt/res/course/course.txt", "/ev3rt/res/course/pos_param.txt");
}

SelfPos* SelfPos::GetInstance(){
	if (singletonInstance == nullptr){
		singletonInstance = new SelfPos();
	}
	return singletonInstance;
}

void SelfPos::Start(){
	Odmetry->Start( vSelf, thetaSelf );
}

void SelfPos::UpdateSelfPosComp( Vector2D& _vSelf, Vector2D& _vSelfVel ){
	vSelf = _vSelf;
	vSelfVel = _vSelfVel;
}

void SelfPos::UpdateSelfPos(){
	vSelf = Odmetry->DeadReckoningWithOdmetry( vSelf, vSelfVel );
	thetaSelf = Odmetry->GetTheta();
}

Vector2D SelfPos::GetSelfPos(){
	return vSelf;
}

Vector2D SelfPos::GetSelfVelocity(){
	return vSelfVel;
}

float SelfPos::GetTheta(){
	return thetaSelf;
}

Vector2D SelfPos::GetMean()
{
	return Odmetry->GetMean();
}

float SelfPos::GetPhi()
{
	return Odmetry->GetPhi();
}

int SelfPos::ReadFirstPos(const char* course_filename, const char* pos_filename) {
	// course
	char course;

	FILE* course_file = fopen(course_filename,"r");
	if(course_file==NULL) return 0;

	char course_param_name[255] = {'\0'};
	fscanf(course_file,"%s %c", &course_param_name[0], &course);
	fclose(course_file);
    printf("load course: %c\n", course);

	// first pos of course
    FILE* pos_param_file = fopen(pos_filename,"r");
    if(pos_param_file==NULL) return 0;

    char param_name[255] = {'\0'};
	char course_x_name[255] = {'\0'};
	sprintf(course_x_name, "%c_x:",course);
	char course_y_name[255] = {'\0'};
	sprintf(course_y_name, "%c_y:",course);
	char course_theta_name[255] = {'\0'};
	sprintf(course_theta_name, "%c_theta:",course);
	char course_vel_x_name[255] = {'\0'};
	sprintf(course_vel_x_name, "%c_vel_x:",course);
	char course_vel_y_name[255] = {'\0'};
	sprintf(course_vel_y_name, "%c_vel_y:",course);

    float param;
    while(fscanf(pos_param_file,"%s %f", &param_name[0], &param)!=EOF){
        if(!strcmp(param_name,course_x_name)){
                this->vSelf.x = param;
        }else if(!strcmp(param_name,course_y_name)){
                this->vSelf.y = param;
        }else if(!strcmp(param_name,course_theta_name)){
                this->thetaSelf = param;
        }else if(!strcmp(param_name,course_vel_x_name)){
                this->vSelfVel.x = param;
        }else if(!strcmp(param_name,course_vel_y_name)){
                this->vSelfVel.y = param;
        }
    }
    printf("load pos\n");
    printf("x: %f\n", this->vSelf.x);
    printf("y: %f\n", this->vSelf.y);
    printf("theta: %f\n", this->thetaSelf);
    printf("vel_x: %f\n", this->vSelfVel.x);
    printf("vel_y: %f\n", this->vSelfVel.y);

    fclose(pos_param_file);
    return 1;
}
