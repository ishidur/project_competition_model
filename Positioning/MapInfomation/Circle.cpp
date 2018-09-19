#include "Circle.h"
#include <math.h>

using namespace Utilities;
using namespace Positioning::MapInfomation;

Circle::Circle( Vector2D& _vc, float _r, float _direction, float _rho ):posCenter(_vc),r(_r),direction(_direction),rho(_rho),ve(0,0)
{
}

float Circle::GetNearestPosDistance()
{
	return 0;
}

Vector2D Circle::GetNearestPos( Vector2D& _v1 )
{
	Vector2D vc_1( 0,0 );
	//中心を原点として計算。最後に座標変換する

	//中心座標（交点）への単位ベクトルe
	// (xc-xs)
	vc_1 = posCenter - _v1;

	// ex=(xc-xs)/L
	// ey=(yc-ys)/L
	Vector2D ve_tmp(0,0);
	//中心方向単位ベクトル
	ve_tmp = vc_1.Normalized();

	//法線ベクトルvn
	//vn_x=-ey
	//vn_y=ex
	Vector2D vn( -ve_tmp.y,ve_tmp.x );

	//単位回転方向ベクトルをセットする
	ve = vn*direction;

	//中心方向交点1（中心座標系）
	Vector2D v5_c(0,0);
	v5_c = ve_tmp*r;

	//中心方向交点2（中心座標系）
	Vector2D v6_c(0,0);
	v6_c = -ve_tmp*r;

	//点1、2の近いほうを最短座標として選択
	//L1 = LenSqure(v1,_v_now)
	Vector2D v5c_1c(0,0);

	//現在地を1c座標系に変換
	Vector2D v1_c(0,0);
	v1_c = _v1 - posCenter;

	//1c座標系で近い点を距離比較で算出
	v5c_1c = v5_c-v1_c;
	float L1 = v5c_1c.LengthSquare();

	//L2 = LenSqure(v2,_v_now)
	Vector2D v6c_1c(0,0);
	v6c_1c = v6_c-v1_c;

	float L2 = v6c_1c.LengthSquare();

	Vector2D v_ret(0,0);

	if( L1 < L2 ){
		v_ret = v5_c;
	}
	else{
		v_ret = v6_c;
	}

	//中心座標を加算して、座標変換する
	v_ret += posCenter;
	return v_ret;
}

Vector2D Circle::GetVe()
{
	return ve;
}

float Circle::GetRho()
{
	return rho;
}
