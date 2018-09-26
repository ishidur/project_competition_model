#ifndef POSITIONING_MAPINFOMATION_EDGE_H
#define POSITIONING_MAPINFOMATION_EDGE_H

#include "../../Utilities/Vector2D.h"
using namespace Utilities;

namespace Positioning{
	namespace MapInfomation{
		class Edge{
			private:

			public:
				virtual float GetNearestPosDistance()=0;
				virtual Vector2D GetNearestPos( Vector2D& _v1 )=0;//最近傍位置
				virtual Vector2D GetVe()=0;	//進行方向単位ベクトル
				virtual float GetRho()=0;	//曲率
		};
	}  // namespace MapInfomation
}  // namespace Positioning
#endif
