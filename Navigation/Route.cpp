#include "Route.h"
#include <stdio.h>

//検討 シーソーとかは3、Lコースは10、Rコースは11
#define ROUTE_ARRIVE_INDEX 10

using namespace Utilities;
using namespace Positioning::MapInfomation;
using namespace Navigation;

Route::Route():NodeNow(NULL),NodeNext(NULL),node_next_index(0),node_max_index(ROUTE_ARRIVE_INDEX)
{
	virtualMap = new VirtualMap();

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
	NodeNext = virtualMap->GetNode( node_next_index );
	EdgeNow = virtualMap->GetEdge( node_next_index );

	//index更新しておく
	node_next_index++;
}

void Route::Update( Vector2D& _v_selfpos )
{
	//次のnodeに到達
	if( NodeNext->IsReachNode( _v_selfpos ) == true ){
		//最終nodeに到達していない
		if( IsFinish() == false ){
			//次のマップを取得
			Next();
		}
		//node終了
		else{

		}
	}
}

void Route::Stop()
{
}

void Route::SetDestination()
{

}

bool Route::IsFinish()
{
	bool ret = true;

	//nodeがまだある場合は目的地に到達していない
	if( node_next_index < node_max_index ){
		ret = false;
	}

	return ret;
}

void Route::SetFollowRatio(float ratio)
{
}

Vector2D Route::GetNearestPos( Vector2D& _v_selfpos )
{
	return EdgeNow->GetNearestPos( _v_selfpos );
}

Vector2D Route::GetVe()
{
	return EdgeNow->Get_Ve();
}

