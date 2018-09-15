#include "CalcSelfPosWithOdmetry.h"
#include <math.h>

#define THETA_INIT	3.141592		//初期値(θ=π、(x,y)=(-1,0)の方向)

using namespace AppliedHardware::VehicleHardware;
using namespace Positioning::Localization;
using namespace Utilities;

CalcSelfPosWithOdmetry::CalcSelfPosWithOdmetry():rightAngle_bf(0),leftAngle_bf(0),theta(0),wheels( DriveWheels::GetInstance() ){
}

void CalcSelfPosWithOdmetry::Start(float theta_init){
	//角度を取得
	wheels->GetAngles( &leftAngle_bf, &rightAngle_bf );
    theta = theta_init;
}

float CalcSelfPosWithOdmetry::CalcTheta( Vector2D& _vSelfVel ){
	//速度ベクトルから向きを計算する（オドメトリ用）
	return 0;
}

Vector2D CalcSelfPosWithOdmetry::DeadReckoningWithOdmetry( Vector2D& _vSelf, Vector2D& _vSelfVel ){
	float rightAngle; // degree
	float leftAngle;  // degree

	//角度を取得
	wheels->GetAngles( &leftAngle, &rightAngle );

    float rightdphi = (rightAngle - rightAngle_bf)*M_PI/180.0; // radian
    float leftdphi  = (leftAngle - leftAngle_bf)*M_PI/180.0;   // radian

    float u_right = DriveWheels::tireRadius * rightdphi;
    float u_left  = DriveWheels::tireRadius * leftdphi;

    rightAngle_bf = rightAngle;
    leftAngle_bf = leftAngle;

    Vector2D tmp_v(cos(theta) * (u_left + u_right)/2.0, sin(theta) * (u_left + u_right)/2.0);
    theta += (u_right - u_left)/DriveWheels::tireDistance;
    return _vSelf + tmp_v;
}

float CalcSelfPosWithOdmetry::GetTheta(){
    return theta;
}