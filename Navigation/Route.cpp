#include "Route.h"
#include <stdio.h>
#include "ev3api.h"

//Lコースは10、Rコースは11(node数)
#define ROUTE_ARRIVE_INDEX_R 11
#define ROUTE_ARRIVE_INDEX_L 10

#define ROUTE_NEXT_NODE_IS_GOAL_INDEX_DIFF 2//ゴール直前は2つ手前（２つは難所につなぐ）

using namespace Utilities;
using namespace Positioning::MapInfomation;
using namespace Navigation;

Route::Route():NodeNow(NULL),NodeNext(NULL),node_next_index(0),node_max_index(ROUTE_ARRIVE_INDEX_R),is_finish(false)
{
	virtualMap = new VirtualMap();

	//ルート読み込み
	ReadRoute();

	//初期nodeを読み込み
	Next();
}

void Route::Start()
{
	//目的地セット
	SetDestination();

	//初期nodeを読み込んだので更新する
	Next();
}

void Route::Next()
{
	NodeNow = NodeNext;
	//最終node到達で呼ばれる
	if( IsArriveEndNode() == true ){
		is_finish = true;	//終了
		//終了時はNext更新しない
	}
	//nodeがまだある
	else{
		NodeNext = virtualMap->GetNode( node_next_index );
		EdgeNow = virtualMap->GetEdge( node_next_index );
		//index更新しておく
		node_next_index++;
		is_node_change = true;//Node変化したのでセットする
	}
}

void Route::Update( Vector2D& _v_selfpos )
{
	dbg = NodeNext->GetDist();
	dbg_v = _v_selfpos;

	//Node変化したフラグをクリアする
	is_node_change = false;

	//終了していない
	if( IsFinish() == false ){
		//次のnodeに到達
		if( NodeNext->IsReachNode( _v_selfpos ) == true ){
			//次のマップを取得
			Next();
		}
	}
}

void Route::Stop()
{
}

void Route::SetDestination()
{
	//目的地をセットする
	is_finish = false;
}

bool Route::IsFinish()
{
	return is_finish;
}

bool Route::IsArriveEndNode()
{
	bool ret = true;

	//nodeがまだある場合は目的地に到達していない
	if( node_next_index < node_max_index ){
		ret = false;
	}

	return ret;
}

bool Route::IsNodeChage()
{
	return is_node_change;
}

bool Route::IsNextNodeGoal()
{
	bool ret = false;

	int tmp_index = node_next_index - ROUTE_NEXT_NODE_IS_GOAL_INDEX_DIFF;

	//NextNodeがゴールかチェックする
	if( tmp_index == node_max_index ){
		ret = true;
	}
	return ret;
}

bool Route::IsArriveNodeGoal()
{
	bool ret = false;

	int tmp_index = node_max_index - ROUTE_NEXT_NODE_IS_GOAL_INDEX_DIFF;

	//NextNodeがゴール後かチェックする
	if( node_next_index > tmp_index ){
		ret = true;
	}
	return ret;
}

void Route::SetFollowRatio(float _ratio)
{
	followRatio = _ratio;
}

Vector2D Route::GetNearestPos( Vector2D& _v_selfpos )
{

	return EdgeNow->GetNearestPos( _v_selfpos );
}

Vector2D Route::GetVe()
{
	return EdgeNow->GetVe();
}

float Route::GetRho()
{
	return EdgeNow->GetRho();
}

float Route::GetDbg()
{
	return dbg;
}

Vector2D Route::GetDbgV()
{
	return dbg_v;
}

Vector2D Route::GetNodeNow()
{
	return NodeNow->GetPos();
}

Vector2D Route::GetDirNow()
{
	return NodeNow->GetDir();
}

Vector2D Route::GetNode()
{
	return NodeNext->GetPos();
}

Vector2D Route::GetDir()
{
	return NodeNext->GetDir();
}

void Route::ReadRoute()
{
	// course
	char course;

	FILE* course_file = fopen("/ev3rt/res/course/course.txt","r");
	if(course_file==NULL){
		printf("load course file fail\n");
		assert(course_file != NULL);
	}

	char course_param_name[255] = {'\0'};
	fscanf(course_file,"%s %c", &course_param_name[0], &course);
	fclose(course_file);
	printf("load course: %c\n", course);

	//R
	if( course == 'R' ){
		node_max_index = ROUTE_ARRIVE_INDEX_R;
	}
	//L
	else{
		node_max_index = ROUTE_ARRIVE_INDEX_L;
	}
}