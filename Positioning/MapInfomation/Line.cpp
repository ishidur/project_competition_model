#include "Line.h"
#include <math.h>

using namespace Utilities;
using namespace Positioning::MapInfomation;

Line::Line( float _a, float _b, float _c, Vector2D& _ve ):aParam(_a),bParam(_b),cParam(_c),veDirection(_ve)
{
}

float Line::GetNearestPosDistance()
{
	return 0;
}

Vector2D Line::GetNearestPos( Vector2D& _v1 )
{
	Vector2D v2( 0,0 );

	//y = ax+c
	//y = c/b( a=0 )
	if( bParam != 0 ){
		if( aParam != 0 ){
			//最短座標x2
			//x2 = (a*(y1-c)+x1)/(a^2+1)
			v2.x = (aParam*(_v1.y-cParam)+_v1.x)/(pow(aParam,2)+1);
		}
		else{
			v2.x = _v1.x;
		}
		//最短座標y2
		//y2 = (a*x2 + c)/(-b)
		v2.y = (aParam*v2.x+cParam)/(-bParam);
	}
	//x=(-c)/a
	else if( aParam != 0 ){
		//最短座標x2 = (-c)/a - x1
		v2.x = (-cParam)/aParam;

		//最短座標y2 = y1
		v2.y=_v1.y;
	}
	//計算できない
	else{
	}

	return v2;
}

Vector2D Line::Get_Ve()
{
	return veDirection;
}
