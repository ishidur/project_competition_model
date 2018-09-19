#include "Navigation.h"
#include "../Utilities/Vector2D.h"

#define NAVIGATION_GOAL_DIST 20//ゴール直前は20cm手前とする
#define NAVIGATION_FORWARD_DEFAULT 125	//デフォルト速度
#define NAVIGATION_FORWARD_SPEEDUP 140	//加速
#define NAVIGATION_FORWARD_SPEEDDOWN 70	//減速
#define NAVIGATION_TURN_END_NODE 0	//最終Nodeはturn 0とする

using namespace Positioning::Localization;
using namespace Utilities;

namespace Navigation
{

Navigation::Navigation():update_cnt(0)
{
	route = new Route();
	selfPos = SelfPos::GetInstance();
	targetPos = new TargetPos();
}

void Navigation::Start()
{
	update_cnt = 0;
	//Phaseに移動した
//	selfPos->Start();
	route->Start();
}

void Navigation::Stop()
{
}

void Navigation::Update()
{
	//自己位置は更新はPhaseに移動した
//	selfPos->UpdateSelfPos();
	//※補正はルートで決定する
	Vector2D v_self_pos(0,0);

	//自己位置補正
	if( route->IsNodeChage() == true ){
		//曲率で補正有無判断(暫定)
		if( route->GetRho() == 0 ){
			Vector2D tmp_v_self_pos_comp(0,0);
			tmp_v_self_pos_comp = selfPos->GetSelfPos();
			Vector2D tmp_v_next_node(0,0);
			tmp_v_next_node = route->GetNodeNow();

			Vector2D tmp_v_next_dir(0,0);
			tmp_v_next_dir = route->GetDirNow();

			//直線方向を判断して補正する
			if( tmp_v_next_dir.x == 0 ){
				tmp_v_self_pos_comp.x = tmp_v_next_node.x;	//yを補正する
			}else if( tmp_v_next_dir.y == 0 ){
				tmp_v_self_pos_comp.y = tmp_v_next_node.y;	//xを補正する
			}else{
				//補正しない
			}
			Vector2D tmp_v_self_Vel(0,0);
			tmp_v_self_Vel = selfPos->GetSelfVelocity();
			selfPos->UpdateSelfPosComp( tmp_v_self_pos_comp ,tmp_v_self_Vel );
		}
	}
	v_self_pos = selfPos->GetSelfPos();

	update_cnt++;
	//更新周期は1/10回にする
	if( update_cnt > 10 ){
		route->Update( v_self_pos );
		update_cnt = 0;
	}else{
	}
}

float Navigation::GetTurn()
{
	//最終Nodeのturnに対応
	return GetEndNodeTurn();
//	return targetPos->GetTurn();
}

float Navigation::GetEndNodeTurn()
{
	return NAVIGATION_TURN_END_NODE;
}

void Navigation::CalcTurn()
{
	//自己位置取得
	Vector2D v_self_pos(0,0);
	v_self_pos = selfPos->GetSelfPos();

	//最近傍位置取得
	Vector2D tmp_v_nearest(0,0);
	tmp_v_nearest = route->GetNearestPos( v_self_pos );

	near = tmp_v_nearest;

	//進行方向ベクトル取得
	Vector2D tmp_v_e(0,0);
	tmp_v_e = route->GetVe();

	//曲率半径取得
	float tmp_rho = route->GetRho();

	targetPos->CalcTurn( tmp_v_nearest, tmp_v_e, v_self_pos, tmp_rho );

}

float Navigation::GetForward()
{
	float ret = NAVIGATION_FORWARD_DEFAULT;//default

   	//ゴール直前のスピードアップチェック
	if( IsNearGoalSpeedUp() == true ){
		ret = NAVIGATION_FORWARD_SPEEDUP;
	}
	//ゴール直前のスピードダウンチェック
	else if( IsArriveGoalSpeedDown() == true ){
		ret = NAVIGATION_FORWARD_SPEEDDOWN;
	}
	//ゴール後
	else if( IsAfterGoalSpeedDown() == true ){
		ret = NAVIGATION_FORWARD_SPEEDDOWN;
	}
	//上記以外（ゴール前）
	else{
		//default
	}
	return ret;
//	return targetPos->GetForward();
}

void Navigation::CalcForward()
{
	//曲率半径取得
	float tmp_rho = route->GetRho();

	targetPos->CalcForward( tmp_rho );
}

bool Navigation::IsNearGoalSpeedUp()
{
	bool ret = false;

	//自己位置取得
	Vector2D v_self_pos(0,0);
	v_self_pos = selfPos->GetSelfPos();

	//GoalのNode取得
	Vector2D tmp_v_next_node(0,0);
	tmp_v_next_node = route->GetNode();

	//次のNodeがゴールか確認
	if( route->IsNextNodeGoal() == true ){
		//ゴールから20cmより遠い
		if( tmp_v_next_node.DistanceFrom( v_self_pos ) > NAVIGATION_GOAL_DIST ){
			ret = true;
		}
	}

	return ret;
}

bool Navigation::IsArriveGoalSpeedDown()
{
	bool ret = false;

	//自己位置取得
	Vector2D v_self_pos(0,0);
	v_self_pos = selfPos->GetSelfPos();

	//GoalのNode取得
	Vector2D tmp_v_next_node(0,0);
	tmp_v_next_node = route->GetNode();

	//次のNodeがゴールか確認
	if( route->IsNextNodeGoal() == true ){
		//到達手前20cm以下
		if( tmp_v_next_node.DistanceFrom( v_self_pos ) <= NAVIGATION_GOAL_DIST ){
			ret = true;
		}
	}

	return ret;
}

bool Navigation::IsAfterGoalSpeedDown()
{
	bool ret = false;

	//ゴール到達したか確認する
	if( route->IsArriveNodeGoal() == true ){
		ret = true;
	}

	return ret;
}

bool Navigation::IsEndNode()
{
	bool ret = false;

	//ゴール到達したか確認する
	if( route->IsArriveEndNode() == true ){
		ret = true;
	}

	return ret;
}

bool Navigation::IsFinish()
{
	bool ret = false;

	//目的地に到達した
	if( route->IsFinish() == true ){
		ret = true;
	}

	return ret;
}

float Navigation::GetDbg()
{
	return route->GetDbg();

}

Vector2D Navigation::GetDbgV()
{
	return route->GetDbgV();

}

Vector2D Navigation::GetNode()
{
	return route->GetNode();
}

Vector2D Navigation::GetDir()
{
	return route->GetDir();
}

float Navigation::GetRho()
{
	return route->GetRho();
}

Vector2D Navigation::GetNear()
{
	return near;
}

Vector2D Navigation::GetTgt()
{
	return targetPos->GetTgt();
}

float Navigation::GetTurnBase()
{
	return targetPos->GetTurnBase();
}

float Navigation::GetCross()
{
	return targetPos->GetCross();
}

}  // namespace Navigation



