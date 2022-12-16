#include "collaborative_vslam/scale_ratio_initializer.h"

ScaleRatioInitializer::ScaleRatioInitializer():private_nh_("~")
{
    // Parameter Server
    private_nh_.getParam("hz", hz_);
    private_nh_.getParam("move_dist_th", move_dist_th_);
    private_nh_.getParam("scale_ratio_th_percent", scale_ratio_th_percent_);
    private_nh_.getParam("duration_init", duration_init_);

    // Other Parameters
    wheel_dist_for_init_   = 0.0;
    visual_dist_for_init_  = 0.0;
    scale_ratio_.data      = 0.0;
    is_initialized_        = false;
    flag_lost_             = false;
    flag_init_visual_.data = false;
    
    // Subscriber
    pose_sub_       = nh_.subscribe("/pose", 1, &ScaleRatioInitializer::pose_callback, this);
    wheel_odom_sub_ = nh_.subscribe("/wheel_odom", 1, &ScaleRatioInitializer::wheel_odom_callback, this);
    lost_sign_sub_  = nh_.subscribe("/lost_sign", 1, &ScaleRatioInitializer::lost_sign_callback, this);

    // Publisher
    init_visual_pub_ = nh_.advertise<std_msgs::Bool>("/flag_init_visual", 1);
    ratio_pub_       = nh_.advertise<std_msgs::Float64>("/scale_ratio", 1);
}

void ScaleRatioInitializer::process()
{
    ros::Rate loop_rate(hz_);

    while(ros::ok())
    {
        init_scale_ratio();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void ScaleRatioInitializer::pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    if(flag_lost_) return;
    prev_pose_ = last_pose_;
    last_pose_ = *msg;

    if(not flag_init_visual_.data)
    {
        const double move_dist = calc_hypot(last_pose_.pose.position, prev_pose_.pose.position);
        ROS_INFO_STREAM("move_dist = " << move_dist << " | th = " << move_dist_th_ << " | flag_init_visual-> " << (move_dist_th_ < move_dist));
        if(move_dist_th_ < move_dist)
        {
            flag_init_visual_.data = true;  // 動き出したらフラグを立てる
            init_begin_ = ros::Time::now(); // スケール比の初回化スタート時間の設定
        }

        if(flag_init_visual_.data)
            init_visual_pub_.publish(flag_init_visual_);
    }
}

void ScaleRatioInitializer::wheel_odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    prev_odom_ = last_odom_;
    last_odom_ = *msg;
}

void ScaleRatioInitializer::lost_sign_callback(const std_msgs::Bool::ConstPtr& msg)
{
    flag_lost_ = msg->data;
}

void ScaleRatioInitializer::init_scale_ratio()
{
    if(not flag_init_visual_.data or is_initialized_) return;

    const double elapsed_time = ros::Time::now().toSec() - init_begin_.toSec();
    if(elapsed_time < duration_init_) // 移動量を加算
    {
        wheel_dist_for_init_  += calc_hypot(last_pose_.pose.position, prev_pose_.pose.position);
        visual_dist_for_init_ += calc_hypot(last_odom_.pose.pose.position, prev_odom_.pose.pose.position);
    }
    else // スケール比を計算
    {
        const double new_ratio = visual_dist_for_init_ / wheel_dist_for_init_;
        is_initialized_ = fabs((scale_ratio_.data - new_ratio)/scale_ratio_.data) < scale_ratio_th_percent_/100.0;

        ROS_INFO_STREAM("---");
        ROS_INFO_STREAM("old_ratio = " << scale_ratio_.data << "[%]");
        ROS_INFO_STREAM("new_ratio = " << new_ratio << "[%]");
        ROS_INFO_STREAM("ratio_diff = " << fabs((scale_ratio_.data - new_ratio)/scale_ratio_.data)*100.0 << "[%] | goal = " << scale_ratio_th_percent_ << "[%] | flag_init -> " << is_initialized_);

        scale_ratio_.data = new_ratio;
        init_begin_ = ros::Time::now(); // 初回化スタート時間の設定

        if(is_initialized_)
            ratio_pub_.publish(scale_ratio_);
    }
}

double ScaleRatioInitializer::calc_hypot(geometry_msgs::Point last_point, geometry_msgs::Point prev_point)
{
    const double dx = last_point.x - prev_point.x;
    const double dy = last_point.y - prev_point.y;
    const double dz = last_point.z - prev_point.z;
    return hypot(dx, dy, dz);
}