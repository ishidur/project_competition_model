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
			bool is_node_change;
			bool is_finish;
			float dbg;
			Vector2D dbg_v;
		public:
			Route();
			void Start();
			void Next();
			void Update( Vector2D& _v_selfpos );
			void Stop();

			void SetDestination();
			bool IsArriveEndNode();
			bool IsFinish();
			bool IsNodeChage();
			bool IsNextNodeGoal();
			bool IsArriveNodeGoal();
			void SetFollowRatio(float _ratio);
			Vector2D GetNearestPos( Vector2D& _v_selfpos );
			Vector2D GetVe();
			float GetRho();


			float GetDbg();
			Vector2D GetDbgV();
			Vector2D GetNode();
			Vector2D GetDir();
			Vector2D GetNodeNow();
			Vector2D GetDirNow();
			void ReadRoute();
	};

}  // namespace Navigation
#endif