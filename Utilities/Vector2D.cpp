#include "Vector2D.h"
#include <math.h>

namespace Utilities
{

Vector2D::Vector2D():x(0),y(0)
{
}

Vector2D::Vector2D(float _x,float _y):x(_x),y(_y)
{
}

//代入演算子の定義
Vector2D& Vector2D::operator=(const Vector2D& v){
	this->x=v.x;	this->y=v.y;
	return *this;
}

// +=の定義
Vector2D& Vector2D::operator+=(const Vector2D& v){
	this->x += v.x;	this->y += v.y;	return *this;
}
// -=の定義
Vector2D& Vector2D::operator-=(const Vector2D& v){
	this->x -= v.x;	this->y -= v.y;	return *this;
}
// *=の定義
Vector2D& Vector2D::operator*=(float k){
	this->x *= k;	this->y *= k;	return *this;
}
// /=の定義
Vector2D& Vector2D::operator/=(float k){
	this->x /= k;	this->y /= k;	return *this;
}

//+の定義:	+v
Vector2D Vector2D::operator+(){
	return *this;
}
//-の定義:	-v
Vector2D Vector2D::operator-(){
	return Vector2D(-x,-y);
}


//二項演算子
Vector2D operator+(const Vector2D& u,const Vector2D& v){	//vector+vector
	Vector2D w;
	w.x = u.x + v.x;
	w.y = u.y + v.y;
	return w;
}

Vector2D operator-(const Vector2D& u,const Vector2D& v){	//vector-vector
	Vector2D w;
	w.x = u.x - v.x;
	w.y = u.y - v.y;
	return w;
}

float operator*(const Vector2D& u,const Vector2D& v){	//内積 vector*vector
	return u.x * v.x + u.y * v.y;
}

Vector2D operator*(const Vector2D& v, float k){	//vector*scalar
	Vector2D w;
	w.x = v.x * k;
	w.y = v.y * k;
	return w;
}
Vector2D operator*(float k ,const Vector2D& v){	//scalar*vector
	Vector2D w;
	w.x = v.x * k;
	w.y = v.y * k;
	return w;
}
Vector2D operator/(const Vector2D& v, float k){	//vector/scalar
	Vector2D w;
	w.x = v.x / k;
	w.y = v.y / k;
	return w;
}

float Vector2D::Length()
{
	return sqrt(x * x + y * y);
}

float Vector2D::LengthSquare()
{
	return x * x + y * y;
}

float Vector2D::Dot(const Vector2D& _v)
{
	return x * _v.x + y * _v.y;
}

float Vector2D::Cross(const Vector2D& _v)
{
	return x * _v.y - y * _v.x;
}

float Vector2D::DistanceFrom(const Vector2D& _v)
{
	return sqrt((_v.x - x) * (_v.x - x) + (_v.y - y) * (_v.y - y));
}

Vector2D Vector2D::Normalized()
{
	return{ x / Length() , y / Length() };
}

bool Vector2D::IsZero()
{
	return x == 0.0 && y == 0.0;
}

}// namespace Utilities
