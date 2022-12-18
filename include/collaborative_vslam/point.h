#ifndef POINT_H
#define POINT_H

#include <geometry_msgs/Point.h>

class Point
{
public:
    Point(); // デフォルトコンストラクタ
    Point(const double x, const double z);   // コンストラクタ
    Point(const geometry_msgs::Point point); // コンストラクタ
    Point& operator =(const Point& point); // 代入演算子
    Point& operator /=(const double a); // 複合代入演算子/=
    Point& operator *=(const double a); // 複合代入演算子*=
    Point operator +(const Point& point) const; // 算術演算子+
    Point operator -(const Point& point) const; // 算術演算子-
    Point operator *(const Point& point) const; // 算術演算子*
    Point operator /(const Point& point) const; // 算術演算子/
    friend Point operator *(const Point& point, const double a); // 算術演算子*
    friend Point operator /(const Point& point, const double a); // 算術演算子/

    // accessor
    void set(const double x, const double z);
    void set(const geometry_msgs::Point point);
    double x() const { return x_; }
    double z() const { return z_; }

private:
    double x_; // [m]
    double z_; // [m]
};

#endif
