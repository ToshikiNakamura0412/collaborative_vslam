#ifndef POINT_H
#define POINT_H

#include <cmath>

class Point
{
public:
    Point(); // デフォルトコンストラクタ
    Point(const double z, const double x); // コンストラクタ
    Point& operator =(const Point& point); // 代入演算子
    Point& operator /=(const double a); // 複合代入演算子/=
    Point& operator *=(const double a); // 複合代入演算子*=
    const Point operator +(const Point& point) const; // 算術演算子+
    const Point operator -(const Point& point) const; // 算術演算子-
    const Point operator *(const Point& point) const; // 算術演算子*
    const Point operator /(const Point& point) const; // 算術演算子/

    // accessor
    void set(const double z, const double x);
    double z() const { return z_; }
    double x() const { return x_; }

private:
    double z_; // [m]
    double x_; // [m]
};

#endif
