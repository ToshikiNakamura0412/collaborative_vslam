#ifndef SCALE_RATIO_INITIALIZER_H
#define SCALE_RATIO_INITIALIZER_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

class ScaleRatioInitializer
{
public:
    ScaleRatioInitializer();
    void process();

private:
    // ----- Function -----
    // callback
    void visual_sign_callback(const std_msgs::Bool::ConstPtr& msg);
    void lost_sign_callback(const std_msgs::Bool::ConstPtr& msg);
    void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void wheel_odom_callback(const nav_msgs::Odometry::ConstPtr& msg);

    // others
    void init_scale_ratio();
    void init_parameters();
    void init_dist();
    void init_flag();
    double calc_hypot(geometry_msgs::Point last_point, geometry_msgs::Point prev_point);


    // ----- Variable -----
    int    hz_;
    bool   flag_init_visual_;
    bool   flag_lost_;
    bool   accumulation_mode_;
    double scale_ratio_th_percent_;
    double duration_time_;
    double wheel_dist_for_init_;
    double visual_dist_for_init_;

    std_msgs::Float64  scale_ratio_;
    nav_msgs::Odometry last_odom_; // 最新のodometry
    nav_msgs::Odometry prev_odom_; // 1制御周期前のodometry
    geometry_msgs::PoseStamped last_pose_; // 最新のcamera pose
    geometry_msgs::PoseStamped prev_pose_; // 1制御周期前のcamera pose

    ros::Time init_begin_; // 計算開始時間


    // ----- Ohters -----
    // NodeHandle
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // Subscriber
    ros::Subscriber visual_sign_sub_;
    ros::Subscriber lost_sign_sub_;
    ros::Subscriber pose_sub_;
    ros::Subscriber wheel_odom_sub_;

    // Publisher
    ros::Publisher ratio_pub_;
};

#endif
