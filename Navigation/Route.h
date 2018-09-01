#ifndef NAVIGATION_ROUTE_H
#define NAVIGATION_ROUTE_H

#include "Positioning/MapInfomation/Node.h"
#include "Positioning/MapInfomation/Edge.h"
#include "Positioning/MapInfomation/VirtualMap.h"
#include "../Utilities/Vector2D.h"

using namespace Utilities;
using namespace Positioning::MapInfomation;

namespace Navigation{
	class Route{
		private:
			Positioning::MapInfomation::Node* NodeNow;
			Positioning::MapInfomation::Node* NodeNext;
			Positioning::MapInfomation::Edge* EdgeNow;
			int node_next_index;
			int node_max_index;
			float followRatio;

			Positioning::MapInfomation::VirtualMap* virtualMap;

		public:
			Route();
			void Start();
			void Next();
			void Update( Vector2D& _v_selfpos );
			void Stop();

			void SetDestination();
			bool IsFinish();
			void SetFollowRatio(float ratio);
			Vector2D GetNearestPos( Vector2D& _v_selfpos );
			Vector2D GetVe();
	};

}  // namespace Navigation
#endif
