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

	float delta_rigthAngle = (rightAngle - rightAngle_bf);
	float delta_leftAngle = (leftAngle - leftAngle_bf);

    float rightdphi = delta_rigthAngle*M_PI/180.0; // radian
    float leftdphi  = delta_leftAngle*M_PI/180.0;   // radian

    float u_right = DriveWheels::tireRadius * rightdphi;
    float u_left  = DriveWheels::tireRadius * leftdphi;

    Vector2D tmp_v(0,0);
    tmp_v.x = cos(theta) * (u_left + u_right)/2.0;
    tmp_v.y = sin(theta) * (u_left + u_right)/2.0;

    Vector2D tmp_vself(0,0);
    tmp_vself = _vSelf + tmp_v;

    theta += (u_right - u_left)/DriveWheels::tireDistance;

    rightAngle_bf = rightAngle;
    leftAngle_bf = leftAngle;

    return tmp_vself;
}

float CalcSelfPosWithOdmetry::GetTheta(){
    return theta;
}