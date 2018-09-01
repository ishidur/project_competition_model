#ifndef POSITIONING_MAPINFOMATION_CIRCLE_H
#define POSITIONING_MAPINFOMATION_CIRCLE_H

#include "Positioning/MapInfomation/Edge.h"
#include "../../Utilities/Vector2D.h"

using namespace Utilities;

namespace Positioning{
	namespace MapInfomation{
		class Circle : public Edge{
			private:
				Vector2D posCenter;
				float r;
				float direction;
				Vector2D ve;

			public:
				Circle( Vector2D& _vc, float _r, float _direction );
				float GetNearestPosDistance();
				Vector2D GetNearestPos( Vector2D& _v1 );
				Vector2D Get_Ve();

		};
	}  // namespace MapInfomation
}  // namespace Positioning
#endif
