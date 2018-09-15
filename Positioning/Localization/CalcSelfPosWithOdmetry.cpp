#include "CalcSelfPosWithOdmetry.h"
#include <math.h>

#define THETA_INIT	3.141592		//初期値(θ=π、(x,y)=(-1,0)の方向)
#define RATIO 10.0      //1pixel=xmm
#define DIST_START (500.0/RATIO) //座標の補正を行う距離
#define DIST (1000.0/RATIO) //座標の補正を行う距離

using namespace AppliedHardware::VehicleHardware;
using namespace Positioning::Localization;
using namespace Utilities;

CalcSelfPosWithOdmetry::CalcSelfPosWithOdmetry():rightAngle_bf(0),leftAngle_bf(0),theta(0),thetaInit(0),vSelfInit(0,0),v_mean(0,0),wheels( DriveWheels::GetInstance() ),mean_cnt(0),is_mean(false){
}

void CalcSelfPosWithOdmetry::Start( Vector2D& _vSelfInit, float _theta_init ){
	//角度を取得
	wheels->GetAngles( &leftAngle_bf, &rightAngle_bf );
	vSelfInit = _vSelfInit;
    theta = _theta_init;
    thetaInit = _theta_init;
    mean_cnt = 0;
    is_mean = true;
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

	Vector2D tmp_v(0,0);
	tmp_v.x = cos(theta) * (u_left + u_right)/2.0;
	tmp_v.y = sin(theta) * (u_left + u_right)/2.0;

	Vector2D tmp_vself(0,0);
	tmp_vself = _vSelf + tmp_v;

	theta += (u_right - u_left)/DriveWheels::tireDistance;

	//補正
    if(is_mean == false ){
    	Vector2D tmp_v_mean(0,0);
    	tmp_v_mean = tmp_vself - vSelfInit;
    	float d2 = _vSelf.DistanceFrom( vSelfInit );
    	if( d2 < DIST ){
    		if( DIST_START < d2 ){
    			v_mean += tmp_v_mean;
    			mean_cnt++;
        	}
    	}
    	else{//DISTを超えたら座標の補正
    		v_mean /= mean_cnt;
        	phi=thetaInit-atan2( v_mean.y,v_mean.x ) ;
        	Vector2D tmp_offset_v(0,0);
        	tmp_offset_v.x = cos(phi)*tmp_v_mean.x - sin(phi)*tmp_v_mean.y;
        	tmp_offset_v.y = sin(phi)*tmp_v_mean.x + cos(phi)*tmp_v_mean.y;
        	tmp_vself = vSelfInit + tmp_offset_v;
        	//暫定で補正する
        	//        	theta = theta - phi;
        	theta = thetaInit;
        	is_mean = true;
        }
    }

	rightAngle_bf = rightAngle;
    leftAngle_bf = leftAngle;

    Vector2D tmp_v(cos(theta) * (u_left + u_right)/2.0, sin(theta) * (u_left + u_right)/2.0);
    theta += (u_right - u_left)/DriveWheels::tireDistance;
    return _vSelf + tmp_v;
}

float CalcSelfPosWithOdmetry::GetTheta(){
    return theta;
}

Vector2D CalcSelfPosWithOdmetry::GetMean()
{
	return v_mean;
}

float CalcSelfPosWithOdmetry::GetPhi()
{
	return phi;
}

