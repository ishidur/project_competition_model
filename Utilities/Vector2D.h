#ifndef UTILITIES_VECTOR2_D_H
#define UTILITIES_VECTOR2_D_H

namespace Utilities{
	class Vector2D{
		public:
			float x;
			float y;

		public:
			Vector2D();
			Vector2D(float _x,float _y);
			Vector2D& operator=(const Vector2D& v);
			Vector2D& operator+=(const Vector2D& v);
			Vector2D& operator-=(const Vector2D& v);
			Vector2D& operator*=(float k);
			Vector2D& operator/=(float k);
			Vector2D operator+();
			Vector2D operator-();

			float Length();
			float LengthSquare();
			float Dot(const Vector2D& _v);
			float Cross(const Vector2D& _v);
			float DistanceFrom(const Vector2D& _v);
			Vector2D Normalized();
			bool IsZero();
	};
	//二項演算子
	Vector2D operator+(const Vector2D& u,const Vector2D& v);
	Vector2D operator-(const Vector2D& u,const Vector2D& v);
	float operator*(const Vector2D& u,const Vector2D& v);
	Vector2D operator*(const Vector2D& v, float k);
	Vector2D operator*(float k ,const Vector2D& v);
	Vector2D operator/(const Vector2D& v, float k);

}  // namespace Utilities
#endif
