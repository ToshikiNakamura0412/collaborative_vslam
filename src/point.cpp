#include "collaborative_vslam/point.h"

// デフォルトコンストラクタ
Point::Point()
{
    x_ = 0.0;
    y_ = 0.0;
    z_ = 0.0;
}

// コンストラクタ
Point::Point(const double x, const double y, const double z)
{
    x_ = x;
    y_ = y;
    z_ = z;
}
Point::Point(const double x, const double z)
{
    x_ = x;
    z_ = z;
}
Point::Point(const geometry_msgs::PoseStamped& pose)
{
    x_ = pose.pose.position.x;
    y_ = pose.pose.position.y;
    z_ = pose.pose.position.z;
}
Point::Point(const geometry_msgs::PointStamped& point)
{
    x_ = point.point.x;
    y_ = point.point.y;
    z_ = point.point.z;
}
Point::Point(const geometry_msgs::Point& point)
{
    x_ = point.x;
    y_ = point.y;
    z_ = point.z;
}

// 代入演算子
Point& Point::operator =(const Point& point)
{
    x_ = point.x_;
    y_ = point.y_;
    z_ = point.z_;
    return *this;
}

// 複合代入演算子
Point& Point::operator /=(const double a)
{
    x_ /= a;
    y_ /= a;
    z_ /= a;
    return *this;
}
Point& Point::operator *=(const double a)
{
    x_ /= a;
    y_ /= a;
    z_ /= a;
    return *this;
}

// 単項演算子
Point Point::operator +() const
{
    return *this;
}
Point Point::operator -() const
{
    Point tmp;
    tmp.x_ = -x_;
    tmp.y_ = -y_;
    tmp.z_ = -z_;
    return tmp;
}

// 算術演算子
Point Point::operator +(const Point& point) const
{
    Point tmp;
    tmp.x_ = x_ + point.x_;
    tmp.y_ = y_ + point.y_;
    tmp.z_ = z_ + point.z_;
    return tmp;
}
Point Point::operator -(const Point& point) const
{
    Point tmp;
    tmp.x_ = x_ - point.x_;
    tmp.y_ = y_ - point.y_;
    tmp.z_ = z_ - point.z_;
    return tmp;
}
Point Point::operator *(const Point& point) const
{
    Point tmp;
    tmp.x_ = x_ * point.x_;
    tmp.y_ = y_ * point.y_;
    tmp.z_ = z_ * point.z_;
    return tmp;
}
Point Point::operator /(const Point& point) const
{
    Point tmp;
    tmp.x_ = x_ / point.x_;
    tmp.y_ = y_ / point.y_;
    tmp.z_ = z_ / point.z_;
    return tmp;
}
Point operator *(const Point& point, const double a) // friend
{
    Point tmp;
    tmp.x_ = point.x_ * a;
    tmp.y_ = point.y_ * a;
    tmp.z_ = point.z_ * a;
    return tmp;
}

Point operator /(const Point& point, const double a) // friend
{
    Point tmp;
    tmp.x_ = point.x_ / a;
    tmp.y_ = point.y_ / a;
    tmp.z_ = point.z_ / a;
    return tmp;
}
std::ostream& operator<<(std::ostream& os, const Point& point)
{
    return os << "point:" << std::endl
        << "  x: " << point.x_ << std::endl
        << "  y: " << point.y_ << std::endl
        << "  z: " << point.z_ << std::endl
        << "---" << std::endl;
}


// setter
void Point::set(const double x, const double y, const double z)
{
    x_ = x;
    y_ = y;
    z_ = z;
}
void Point::set(const geometry_msgs::Point& point)
{
    x_ = point.x;
    y_ = point.y;
    z_ = point.z;
}
void Point::set_xz(const double x, const double z)
{
    x_ = x;
    z_ = z;
}

// output
void Point::output(geometry_msgs::PoseStamped& pose)
{
    pose.pose.position.x = x_;
    pose.pose.position.y = y_;
    pose.pose.position.z = z_;
}
void Point::output(geometry_msgs::PointStamped& point)
{
    point.point.x = x_;
    point.point.y = y_;
    point.point.z = z_;
}
void Point::output_xz(pcl::PointXYZ& point)
{
    point.x = x_;
    point.z = z_;
}
void Point::output_xz(geometry_msgs::PointStamped& point)
{
    point.point.x = x_;
    point.point.z = z_;
}
