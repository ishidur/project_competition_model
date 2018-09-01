#ifndef POSITIONING_MAPINFOMATION_FIELD_MAP_H
#define POSITIONING_MAPINFOMATION_FIELD_MAP_H

#include "../../Utilities/Vector2D.h"

#define FIELD_WIDTH 1200//3710
#define FIELD_HEIGHT 1200//5510
#define RATIO 10      //1pixel=xmm

#define WIDTH 120  //(int)(FIELD_WIDTH/RATIO)
#define HEIGHT 120 //(int)(FIELD_HEIGHT/RATIO)

namespace Positioning{
	namespace MapInfomation	{

		class FieldMap{
			private:
				int mapValue[WIDTH][HEIGHT];
				int mapWidth;
				int mapHeight;
				int gridSize = RATIO; // mm
			
			public:
				FieldMap();
				int ReadMap(const char* filename);

				int GetMapValue(Utilities::Vector2D pos){ return mapValue[(int)(pos.x)][(int)(pos.y)]; }
				int GetMapValue(int x, int y){ return mapValue[x][y]; }

				int GetWidth()   { return mapWidth;  }
				int GetHeight()  { return mapHeight; }
				int GetGridSize(){ return gridSize;  }
		};

	}  // namespace MapInfomation
}  // namespace Positioning
#endif
