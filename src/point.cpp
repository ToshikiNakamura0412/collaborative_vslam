#include "localizer/point.h"

// デフォルトコンストラクタ
Point::Point()
{
    z_ = 0.0;
    x_ = 0.0;
}

// コンストラクタ
Point::Point(const double z, const double x)
{
    z_ = z;
    x_ = x;
}

// 代入演算子
Point& Point::operator =(const Point& point)
{
    z_ = point.z_;
    x_ = point.x_;
    return *this;
}

// 複合代入演算子
Point& Point::operator /=(const double a)
{
    z_ /= a;
    x_ /= a;
    return *this;
}
Point& Point::operator *=(const double a)
{
    z_ /= a;
    x_ /= a;
    return *this;
}

// 算術演算子
const Point Point::operator +(const Point& point) const
{
    Point tmp;
    tmp.z_ = z_ + point.z_;
    tmp.x_ = x_ + point.z_;
    return tmp;
}
const Point Point::operator -(const Point& point) const
{
    Point tmp;
    tmp.z_ = z_ - point.z_;
    tmp.x_ = x_ - point.z_;
    return tmp;
}
const Point Point::operator *(const Point& point) const
{
    Point tmp;
    tmp.z_ = z_ * point.z_;
    tmp.x_ = x_ * point.z_;
    return tmp;
}
const Point Point::operator /(const Point& point) const
{
    Point tmp;
    tmp.z_ = z_ / point.z_;
    tmp.x_ = x_ / point.z_;
    return tmp;
}

// setter
void Point::set(const double z, const double x)
{
    z_ = z;
    x_ = x;
}
