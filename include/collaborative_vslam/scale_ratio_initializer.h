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
    // Function
    void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void wheel_odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
    void lost_sign_callback(const std_msgs::Bool::ConstPtr& msg);
    void visual_sign_callback(const std_msgs::Bool::ConstPtr& msg);
    void init_scale_ratio();
    double calc_hypot(geometry_msgs::Point last_point, geometry_msgs::Point prev_point);

    // Variable
    int    hz_;
    bool   is_initialized_;
    bool   flag_lost_;
    bool   flag_init_visual_;
    double move_dist_th_;
    double scale_ratio_th_percent_;
    double duration_init_;
    double wheel_dist_for_init_;
    double visual_dist_for_init_;
    ros::Time init_begin_;
    // std_msgs::Bool flag_init_visual_;
    std_msgs::Bool flag_init_ratio_;
    std_msgs::Float64 scale_ratio_;

    // NodeHandle
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // Subscriber
    ros::Subscriber pose_sub_;
    ros::Subscriber wheel_odom_sub_;
    ros::Subscriber lost_sign_sub_;
    ros::Subscriber visual_sign_sub_;

    // Publisher
    ros::Publisher init_visual_pub_;
    ros::Publisher init_ratio_pub_;
    ros::Publisher ratio_pub_;

    // Info
    geometry_msgs::PoseStamped last_pose_;
    geometry_msgs::PoseStamped prev_pose_;
    nav_msgs::Odometry         last_odom_; // 最新のodometry
    nav_msgs::Odometry         prev_odom_; // 1制御周期前のodometry
};

#endif
