#include "TargetPos.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define TURN_MAX 20
#define LENGTH_MAX 20
#define TARGET_DIST_RATIO 1
#define CONV_CROSS_TO_TURN (14.32394/4) //180/((2/4)*PI())
#define PID_PARAM_FILE_LENGTH "/ev3rt/res/navigation/pid_params_navi_length.txt"
#define PID_PARAM_FILE_CROSS "/ev3rt/res/navigation/pid_params_navi_cross.txt"
using namespace Utilities;
using namespace Navigation;

#define RHO_NUM 	9
const float tbl_conv_rho_to_forward_turn_term[RHO_NUM][ENUM_FORWARD_TURN_TERM_NUM] =
{		// rho		turn_base_offset
		{	0	,	0	,	100	},
		{	0.0102	,	3.2	,	100	},
		{	0.0177	,	5	,	100	},
		{	0.0262	,	7	,	100	},
		{	0.0271	,	7	,	100	},
		{	0.0272	,	7	,	100	},
		{	0.03	,	7	,	100	},
		{	0.034	,	7.5	,	100	},
		{	1	,	40	,	10	}
};

TargetPos::TargetPos():turn(0),turn_max(TURN_MAX),length_max(LENGTH_MAX),forward(0),vNearest(0,0),veTarget(0,0),v_now(0,0),v_pre(0,0),v_vel_now(0,0),v_vel_pre(0,0),targetDistRatio(TARGET_DIST_RATIO)
{
	pIDCalculationLength = new PIDCalculation( PID_PARAM_FILE_LENGTH );
	pIDCalculationCross = new PIDCalculation( PID_PARAM_FILE_CROSS );
	LoadTgtRatioParam();
}

Vector2D TargetPos::GetTgtVe()
{
	Vector2D ve2(0,0);

	//最近傍セット
	ve2 = vNearest;

	//最近傍と現在位置の距離で追跡ターゲット位置を決める
	if( ve2.DistanceFrom(v_now) > 1 ){
//		ve2 = ve2.DistanceFrom(v_now)*veTarget*targetDistRatio;
		ve2 = veTarget*targetDistRatio;
	}
	else{
		ve2 = veTarget*targetDistRatio;
	}

	ve2 = vNearest+ve2-v_now;

	return ve2;
}

Vector2D TargetPos::GetNowVe()
{
	return v_vel_now;
//	return v_now - v_pre;
}

void TargetPos::Set( Vector2D& _v, Vector2D& _ve, Vector2D& _v_now )
{
	vNearest = _v;
	veTarget = _ve;
	v_pre = v_now;
	v_now = _v_now;

	float tmp_direction = v_vel_pre.Dot( v_now - v_pre );

	if( tmp_direction >= 0 ){
		v_vel_pre = v_vel_now;
		v_vel_now = v_now - v_pre;
	}
}

void TargetPos::CalcTurn( Vector2D& _v_near, Vector2D& _v_e, Vector2D& _v_self_pos, float _rho )
{
	//現在位置からの最近傍位置v2
	Vector2D v2(0,0);
	v2 = _v_near;

	//最近傍、tgt単位ベクトル、自己位置をセット
	Set( _v_near, _v_e, _v_self_pos );

	//ターゲット方向v3
	Vector2D v3(0,0);
	v3 = GetTgtVe();
	Vector2D v3_norm(0,0);
	v3_norm = v3.Normalized();

	tgt = v3;//debug

	//現在の進行方向v4
	Vector2D v4(0,0);
	v4 = GetNowVe();
	Vector2D v4_norm(0,0);
	//停止しているときは計算しない
	if( v4.Length() != 0 ){
		v4_norm = v4.Normalized();
		cross = v4_norm.Cross(v3_norm);
		trun_theta = asin( cross )*CONV_CROSS_TO_TURN;
	}else{
		//後ろはどうする？
	}

	//Turn計算(PID)
	//最近傍と現在位置の距離を計算
	float tmp_length = _v_self_pos.DistanceFrom( v2 );
	//現在の進行方向v4とターゲット方向v3の外積の符号でturnの方向を決める
	if( v4.Cross(v3) >=  0 ){
	}
	else{
		tmp_length *= (-1);
	}

	//lengthの上限チェック
	if( tmp_length > length_max ){
		tmp_length = length_max;
	}

	//Turn計算(Base)
	CalcTurnBaseOffset( _rho );
	float turn_length_ratio = 1.0;

	if( abs(tmp_length) > 3 ){
		turn_length_ratio = 2.0;
	}

	//turn量計算する
	turn = CalcLinetraceTurn( tmp_length, trun_theta*turn_length_ratio );

	//turnの上限チェック
	if( turn > turn_max ){
		turn = turn_max;
	}else if( turn < (-1)*turn_max ){
		turn = (-1)*turn_max;
	}
}

float TargetPos::CalcLinetraceTurn( float _length, float _turn_theta )
{
    float tmp_pid_turn_length = pIDCalculationLength->GetPIDValue( _length, 0 );
    float tmp_pid_turn_cross = pIDCalculationCross->GetPIDValue( _turn_theta, 0 );
    float ret = 0;
	//Lineから遠い場合は最近傍との距離で計算
	if( _length > 18 ){
		ret = tmp_pid_turn_length;
	}
	//Lineから近い場合は外積で計算
	else{
		ret = tmp_pid_turn_cross;
	}

    return ( turn_base + ret );
}

void TargetPos::CalcTurnBaseOffset( float _rho )
{
	//曲率半径rhoでturnのbaseオフセットを決定する
	//計算式でやりたいが調整必要（とりあえず、テーブルで探索）
	now_rho = _rho;
	turn_base = GetValue( ENUM_FORWARD_TURN_TERM_TURN_BASE_OFFSET, _rho );

	//符号をセットする
	//符号が正
	if( now_rho >= 0 ){
		//そのまま
	}
	//符号が負
	else{
		turn_base = (-1)*turn_base;
	}
}

float TargetPos::GetValue( ENUM_FORWARD_TURN_TERM _term, float _rho )
{
	float ret = 0;
	float abs_rho = _rho;
	int i;

    //テーブルで値を参照する
    for( i=0; i<(int)RHO_NUM; i++){
    	if( abs_rho < 0 ){
    		abs_rho *= (-1);
    	}

    	if( abs_rho <= tbl_conv_rho_to_forward_turn_term[i][ENUM_FORWARD_TURN_TERM_RHO] ){
			ret = tbl_conv_rho_to_forward_turn_term[i][_term];
			break;
    	}
    }
    return ret;
}

float TargetPos::GetTurn()
{
	return turn;
}

void TargetPos::CalcForward( float _rho )
{
	forward = CalcLinetraceForward( _rho );
}

float TargetPos::CalcLinetraceForward( float _rho )
{
    float ret = GetValue( ENUM_FORWARD_TURN_TERM_FORWARD_VALUE, _rho );

    return ret;
}

float TargetPos::GetForward()
{
	return forward;
}

Vector2D TargetPos::GetTgt()
{
	return tgt+v_now;
}

float TargetPos::GetTurnBase()
{
	return turn_base;
}

float TargetPos::GetCross()
{
	return cross;
}

void TargetPos::LoadTgtRatioParam(){
    char param_name[255];
    FILE* pos_tgt_file = fopen("/ev3rt/res/navigation/pos_target_params.txt","r");

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
