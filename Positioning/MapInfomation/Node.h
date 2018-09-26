#ifndef POSITIONING_MAPINFOMATION_NODE_H
#define POSITIONING_MAPINFOMATION_NODE_H

#include "../../Utilities/Vector2D.h"

using namespace Utilities;

namespace Positioning{
	namespace MapInfomation{
		class Node{
			private:
				Vector2D pos;		//Node位置
				Vector2D direction;	//進行方向
				float detect_radius;	//Node到達半径
				float dist;
			public:
				Node( Vector2D& _pos, Vector2D& _direction, float _detect_radius );	//コンストラクタ
				Vector2D GetPos();		//Nodeを取得する
				bool IsReachNode( Vector2D& _pos_now );//指定位置_pos_nowがNodeに到達しているか判定する

				Vector2D GetDir();		//方向を取得する
				float GetDetect();
				float GetDist();
		};

	}  // namespace MapInfomation
}  // namespace Positioning
#endif
