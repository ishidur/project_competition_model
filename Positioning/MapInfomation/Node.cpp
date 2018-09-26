#include "Node.h"

using namespace Utilities;
using namespace Positioning::MapInfomation;

Node::Node( Vector2D& _pos, Vector2D& _direction, float _detect_radius ):pos( _pos.x, _pos.y ),direction( _direction.x, _direction.y ), detect_radius( _detect_radius )
{
}

Vector2D Node::GetPos()
{
	return pos;
}

bool Node::IsReachNode( Vector2D& _pos_now )
{
	bool ret = false;
	float tmp_dist = 0;

	detect_radius = 2;//検討用

	tmp_dist = pos.DistanceFrom( _pos_now );
	dist = tmp_dist;

	//Nodeから現在地へのベクトル
	Vector2D v_node_to_now(0,0);
	v_node_to_now = _pos_now - pos;
	//コース幅半径30cm
	if( tmp_dist <= 30 ){
		//到達閾値未満は到達と判定
		if( ( tmp_dist <= detect_radius )
		||	( direction.Dot( v_node_to_now ) >= 0 )){//内積判定(0以上で進行方向に対し90°以下＝通過した)

			ret = true;
		}
	}

	return ret;
}

Vector2D Node::GetDir()
{
	return direction;
}

float Node::GetDetect()
{
	return detect_radius;
}

float Node::GetDist()
{
	return dist;
}
