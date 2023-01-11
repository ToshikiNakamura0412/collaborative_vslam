#include "collaborative_vslam/scale_ratio_initializer.h"

ScaleRatioInitializer::ScaleRatioInitializer():private_nh_("~")
{
    // Parameter Server
    private_nh_.getParam("hz", hz_);
    private_nh_.getParam("accumulation_mode", accumulation_mode_);
    private_nh_.getParam("scale_ratio_th_percent", scale_ratio_th_percent_);
    private_nh_.getParam("duration_time", duration_time_);

    // Other Parameters Initialization
    init_parameters();
    init_flag();

    // Subscriber
    visual_sign_sub_ = nh_.subscribe("/visual_sign", 1, &ScaleRatioInitializer::visual_sign_callback, this);
    lost_sign_sub_   = nh_.subscribe("/lost_sign", 1, &ScaleRatioInitializer::lost_sign_callback, this);
    pose_sub_        = nh_.subscribe("/pose", 1, &ScaleRatioInitializer::pose_callback, this);
    wheel_odom_sub_  = nh_.subscribe("/wheel_odom", 1, &ScaleRatioInitializer::wheel_odom_callback, this);

    // Publisher
    ratio_pub_ = nh_.advertise<std_msgs::Float64>("/scale_ratio", 1);
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

void ScaleRatioInitializer::visual_sign_callback(const std_msgs::Bool::ConstPtr& msg)
{
    if(msg->data)
    {
        flag_init_visual_ = true;
        ROS_INFO_STREAM("tracking start!!");
        init_begin_ = ros::Time::now();
        init_parameters();
    }
}

void ScaleRatioInitializer::lost_sign_callback(const std_msgs::Bool::ConstPtr& msg)
{
    if(not flag_lost_ and flag_lost_!=msg->data) ROS_INFO_STREAM("lost");
    flag_lost_ = msg->data;
}

void ScaleRatioInitializer::pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    prev_pose_ = last_pose_;
    last_pose_ = *msg;

    // 移動量を加算
    if(not flag_lost_)
        visual_dist_for_init_ += calc_hypot(last_pose_.pose.position, prev_pose_.pose.position);
}

void ScaleRatioInitializer::wheel_odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    prev_odom_ = last_odom_;
    last_odom_ = *msg;

    // 移動量を加算
    if(not flag_lost_)
        wheel_dist_for_init_ += calc_hypot(last_odom_.pose.pose.position, prev_odom_.pose.pose.position);
}

void ScaleRatioInitializer::init_scale_ratio()
{
    if(not flag_init_visual_ or flag_lost_) return;

    // 経過時間を算出
    const double elapsed_time = ros::Time::now().toSec() - init_begin_.toSec();

    // スケール比を計算
    if(elapsed_time > duration_time_)
    {
        // スケール比を計算
        const double new_ratio  = visual_dist_for_init_ / wheel_dist_for_init_; // [-]
        const double ratio_diff = fabs((scale_ratio_.data - new_ratio)/scale_ratio_.data); // [-]
        const bool flag_init_ratio = ratio_diff < scale_ratio_th_percent_/100.0;

        // debug
        {
            ROS_INFO_STREAM("---");
            ROS_INFO_STREAM("visual_dist = " << visual_dist_for_init_);
            ROS_INFO_STREAM("wheel_dist  = " << wheel_dist_for_init_);
            ROS_INFO_STREAM("old_ratio   = " << scale_ratio_.data << "[%]");
            ROS_INFO_STREAM("new_ratio   = " << new_ratio << "[%]");
            ROS_INFO_STREAM("ratio_diff  = " << ratio_diff*100.0 << "[%] | th = " << scale_ratio_th_percent_ << "[%]");
        }

        // renew
        if(not accumulation_mode_) init_dist();
        scale_ratio_.data = new_ratio;
        init_begin_ = ros::Time::now();

        // publish
        if(flag_init_ratio)
        {
            ratio_pub_.publish(scale_ratio_);
            ROS_INFO_STREAM("initialized!!");
            init_parameters();
            init_flag();
        }
    }
}

void ScaleRatioInitializer::init_parameters()
{
    scale_ratio_.data = 0.0;
    init_dist();
}

void ScaleRatioInitializer::init_dist()
{
    wheel_dist_for_init_  = 0.0;
    visual_dist_for_init_ = 0.0;
}

void ScaleRatioInitializer::init_flag()
{
    flag_init_visual_ = false;
    flag_lost_ = false;
}

double ScaleRatioInitializer::calc_hypot(geometry_msgs::Point last_point, geometry_msgs::Point prev_point)
{
    const double dx = last_point.x - prev_point.x;
    const double dy = last_point.y - prev_point.y;
    const double dz = last_point.z - prev_point.z;
    return hypot(dx, dy, dz);
}
