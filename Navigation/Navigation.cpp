#include "Navigation.h"
#include "../Utilities/Vector2D.h"

using namespace Positioning::Localization;
using namespace Utilities;

namespace Navigation
{

Navigation::Navigation():update_cnt(0)
{
	route = new Route();
	selfPos = new Positioning::Localization::SelfPos();
	targetPos = new TargetPos();
}

void Navigation::Start()
{
	update_cnt = 0;
	selfPos->Start();
	route->Start();
}

void Navigation::Stop()
{
}

void Navigation::Update()
{
	selfPos->UpdateSelfPos();
	Vector2D v_self_pos(0,0);
	v_self_pos = selfPos->GetSelfPos();

	update_cnt++;
	if( update_cnt > 10 ){
		route->Update( v_self_pos );
		update_cnt = 0;
	}else{
	}
}

float Navigation::GetTurn()
{
	return targetPos->GetTurn();
}

void Navigation::CalcTurn()
{
	Vector2D v_self_pos(0,0);
	v_self_pos = selfPos->GetSelfPos();

	Vector2D tmp_v_nearest(0,0);
	tmp_v_nearest = route->GetNearestPos( v_self_pos );

	Vector2D tmp_v_e(0,0);
	tmp_v_e = route->GetVe();

	targetPos->CalcTurn( tmp_v_nearest, tmp_v_e, v_self_pos );
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

}  // namespace Navigation
