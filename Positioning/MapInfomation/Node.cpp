#include "Node.h"

using namespace Utilities;
using namespace Positioning::MapInfomation;

Node::Node( Vector2D& _pos, float _detect_radius ):Pos( _pos.x, _pos.y ),DetectRadius( _detect_radius )
{
}

Vector2D Node::GetPos()
{
	return Pos;
}

bool Node::IsReachNode( Vector2D& _pos_now )
{
	bool ret = false;
	float tmp_dist = 0;
	tmp_dist = Pos.DistanceFrom( _pos_now );

	//到達閾値以下は到達と判定
	if( tmp_dist <= DetectRadius ){
		ret = true;
	}

	return ret;
}

