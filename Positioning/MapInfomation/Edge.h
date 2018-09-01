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
				virtual Vector2D GetNearestPos( Vector2D& _v1 )=0;
				virtual Vector2D Get_Ve()=0;
		};
	}  // namespace MapInfomation
}  // namespace Positioning
#endif
