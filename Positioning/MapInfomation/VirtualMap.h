#ifndef POSITIONING_MAPINFOMATION_VIRTUAL_MAP_H
#define POSITIONING_MAPINFOMATION_VIRTUAL_MAP_H

#include "Positioning/MapInfomation/Node.h"
#include "Positioning/MapInfomation/Edge.h"

#define LIST_SIZE 20	//登録可能最大数

using namespace Positioning::MapInfomation;

namespace Positioning{
	namespace MapInfomation{
		class VirtualMap{
			private:
				Node* NodeList[LIST_SIZE];
				Edge* EdgeList[LIST_SIZE];

				int node_index;
				int edge_index;
			public:
				VirtualMap();
				Node* GetNode(int index);
				Edge* GetEdge(int index);
				void ReadMap();
		};

	}  // namespace MapInfomation
}  // namespace Positioning
#endif
