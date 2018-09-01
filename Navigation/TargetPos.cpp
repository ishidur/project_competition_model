#include "TargetPos.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TURN_MAX 20
#define LENGTH_MAX 20
#define TARGET_DIST_RATIO 3
#define PID_PARAM_FILE ("/ev3rt/res/pid_params_navi.txt")
using namespace Utilities;
using namespace Navigation;

TargetPos::TargetPos():turn(0),turn_max(TURN_MAX),length_max(LENGTH_MAX),vNearest(0,0),veTarget(0,0),targetDistRatio(TARGET_DIST_RATIO)
{
	pIDCalculation = new PIDCalculation( PID_PARAM_FILE );
	LoadTgtRatioParam();
}

Vector2D TargetPos::GetTgtVe()
{
	Vector2D ve2(0,0);

	ve2 = vNearest;

	if( ve2.DistanceFrom(v_now) > 1 ){
		ve2 = ve2.DistanceFrom(v_now)*veTarget*targetDistRatio;
	}
	else{
		ve2 = veTarget*targetDistRatio;
	}

	ve2 = vNearest+ve2-v_now;

	return ve2;
}

Vector2D TargetPos::GetNowVe()
{
	return v_now - v_pre;
}

void TargetPos::Set( Vector2D& _v, Vector2D& _ve, Vector2D& _v_now )
{
	vNearest = _v;
	veTarget = _ve;
	v_pre = v_now;
	v_now = _v_now;
}

void TargetPos::CalcTurn( Vector2D& _v_near, Vector2D& _v_e, Vector2D& _v_self_pos )
{
	//現在位置からの最近傍位置v2
	Vector2D v2(0,0);
	v2 = _v_near;

	//最近傍、tgt単位ベクトル、自己位置をセット
	Set( _v_near, _v_e, _v_self_pos );

	//ターゲット方向v3
	Vector2D v3(0,0);
	v3 = GetTgtVe();

	//現在の進行方向v4
	Vector2D v4(0,0);
	v4 = GetNowVe();

	//Turn計算（クロソイド走行する場合も考える）
	//最近傍と現在位置の距離を計算
	float length = _v_self_pos.DistanceFrom( v2 );

	//現在の進行方向v4とターゲット方向v3の外積の符号でturnの方向を決める
	if( v4.Cross(v3) >=  0 ){
	}
	else{
		length *= (-1);
	}

	//lengthの上限チェック
	if( length > length_max ){
		length = length_max;
	}

	//turn量計算する
	turn = CalcLinetraceTurn( length );

	//turnの上限チェック
	if( turn > turn_max ){
		turn = turn_max;
	}else if( turn < (-1)*turn_max ){
		turn = (-1)*turn_max;
	}
}

float TargetPos::CalcLinetraceTurn( float _length )
{
    float pid_turn = pIDCalculation->GetPIDValue( _length, 0 );

    return pid_turn;
}

float TargetPos::GetTurn()
{
	return turn;
}

void TargetPos::LoadTgtRatioParam(){
    char param_name[255];
    FILE* pos_tgt_file = fopen("/ev3rt/res/pos_target_params.txt","r");

    if( pos_tgt_file == NULL ){
    	printf("load tgt_dist_ratio fail\n");
    	//assert(pos_tgt_file != NULL);
    }
    else{
		fscanf(pos_tgt_file,"%s %f", &param_name[0], &this->targetDistRatio);
		printf("load tgt_dist_ratio success:%f\n", this->targetDistRatio);
    }

	fclose(pos_tgt_file);
}
