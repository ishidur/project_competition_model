#ifndef POSITIONING_MAPINFOMATION_LINE_H
#define POSITIONING_MAPINFOMATION_LINE_H

#include "Positioning/MapInfomation/Edge.h"
#include "../../Utilities/Vector2D.h"

using namespace Utilities;

namespace Positioning{
	namespace MapInfomation{
		class Line : public Edge{
			private:
				float aParam;
				float bParam;
				float cParam;
				Vector2D veDirection;

			public:
				Line( float _a, float _b, float _c, Vector2D& _ve );
				float GetNearestPosDistance();
				Vector2D GetNearestPos( Vector2D& _v1 );
				Vector2D Get_Ve();

		};

	}  // namespace MapInfomation
}  // namespace Positioning
#endif
