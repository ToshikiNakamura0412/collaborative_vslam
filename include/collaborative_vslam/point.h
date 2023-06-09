#ifndef POINT_H
#define POINT_H

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_ros/point_cloud.h>

class Point
{
public:
    // デフォルトコンストラクタ
    Point();

    // コンストラクタ
    Point(const double x, const double y, const double z);
    Point(const double x, const double z);
    Point(const geometry_msgs::PoseStamped& pose);
    Point(const geometry_msgs::PointStamped& point);
    Point(const geometry_msgs::Point& point);

    Point& operator =(const Point& point); // 代入演算子
    Point& operator /=(const double a); // 複合代入演算子/=
    Point& operator *=(const double a); // 複合代入演算子*=
    Point  operator +() const; // 単項演算子+
    Point  operator -() const; // 単項演算子-
    Point  operator +(const Point& point) const; // 算術演算子+
    Point  operator -(const Point& point) const; // 算術演算子-
    Point  operator *(const Point& point) const; // 算術演算子*
    Point  operator /(const Point& point) const; // 算術演算子/
    friend Point operator *(const Point& point, const double a); // 算術演算子*
    friend Point operator /(const Point& point, const double a); // 算術演算子/

    friend std::ostream& operator<<(std::ostream& os, const Point& point);

    // accessor
    void set(const double x, const double y, const double z);
    void set(const geometry_msgs::Point& point);
    void set_xz(const double x, const double z);
    void set_x(const double x){ x_ = x; }
    void set_y(const double y){ y_ = y; }
    void set_z(const double z){ z_ = z; }
    void output(geometry_msgs::PoseStamped& pose);
    void output(geometry_msgs::PointStamped& point);
    void output_xz(pcl::PointXYZ& point);
    void output_xz(geometry_msgs::PointStamped& point);
    double x() const { return x_; }
    double y() const { return y_; }
    double z() const { return z_; }

private:
    double x_;
    double y_;
    double z_;
};

#endif
