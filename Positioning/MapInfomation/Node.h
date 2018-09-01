#ifndef POSITIONING_MAPINFOMATION_NODE_H
#define POSITIONING_MAPINFOMATION_NODE_H

#include "../../Utilities/Vector2D.h"

using namespace Utilities;

namespace Positioning{
	namespace MapInfomation{
		class Node{
			private:
				Vector2D Pos;		//Node位置
				float DetectRadius;	//Node到達半径

			public:
				Node( Vector2D& _pos, float _detect_radius );	//コンストラクタ
				Vector2D GetPos();		//Nodeを取得する
				bool IsReachNode( Vector2D& _pos_now );//指定位置_pos_nowがNodeに到達しているか判定する

		};

	}  // namespace MapInfomation
}  // namespace Positioning
#endif
