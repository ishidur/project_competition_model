#ifndef POSITIONING_MAPINFOMATION_CIRCLE_H
#define POSITIONING_MAPINFOMATION_CIRCLE_H

#include "Positioning/MapInfomation/Edge.h"
#include "../../Utilities/Vector2D.h"

using namespace Utilities;

namespace Positioning{
	namespace MapInfomation{
		class Circle : public Edge{
			private:
				Vector2D posCenter;	//円の中心
				float r;			//半径
				float direction;	//回転方向は(1 or -1とする)
				float rho;			//曲率

				Vector2D ve;		//進行方向単位ベクトル

			public:
				Circle( Vector2D& _vc, float _r, float _direction, float _rho );
				float GetNearestPosDistance();
				Vector2D GetNearestPos( Vector2D& _v1 );
				Vector2D GetVe();
				float GetRho();

		};
	}  // namespace MapInfomation
}  // namespace Positioning
#endif
