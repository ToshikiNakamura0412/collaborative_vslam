#include "collaborative_vslam/point.h"

// デフォルトコンストラクタ
Point::Point()
{
    x_ = 0.0;
    z_ = 0.0;
}

// コンストラクタ
Point::Point(const double x, const double z)
{
    x_ = x;
    z_ = z;
}
Point::Point(const geometry_msgs::Point point)
{
    x_ = point.x;
    z_ = point.z;
}

// 代入演算子
Point& Point::operator =(const Point& point)
{
    x_ = point.x_;
    z_ = point.z_;
    return *this;
}

// 複合代入演算子
Point& Point::operator /=(const double a)
{
    x_ /= a;
    z_ /= a;
    return *this;
}
Point& Point::operator *=(const double a)
{
    x_ /= a;
    z_ /= a;
    return *this;
}

// 算術演算子
Point Point::operator +(const Point& point) const
{
    Point tmp;
    tmp.x_ = x_ + point.x_;
    tmp.z_ = z_ + point.z_;
    return tmp;
}
Point Point::operator -(const Point& point) const
{
    Point tmp;
    tmp.x_ = x_ - point.x_;
    tmp.z_ = z_ - point.z_;
    return tmp;
}
Point Point::operator *(const Point& point) const
{
    Point tmp;
    tmp.x_ = x_ * point.x_;
    tmp.z_ = z_ * point.z_;
    return tmp;
}
Point Point::operator /(const Point& point) const
{
    Point tmp;
    tmp.x_ = x_ / point.x_;
    tmp.z_ = z_ / point.z_;
    return tmp;
}
Point operator *(const Point& point, const double a) // friend
{
    Point tmp;
    tmp.x_ = point.x_ * a;
    tmp.z_ = point.z_ * a;
    return tmp;
}

Point operator /(const Point& point, const double a) // friend
{
    Point tmp;
    tmp.x_ = point.x_ / a;
    tmp.z_ = point.z_ / a;
    return tmp;
}

// setter
void Point::set(const double x, const double z)
{
    x_ = x;
    z_ = z;
}
void Point::set(const geometry_msgs::Point point)
{
    x_ = point.x;
    z_ = point.z;
}
