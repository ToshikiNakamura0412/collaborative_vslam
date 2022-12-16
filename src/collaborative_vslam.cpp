#include "collaborative_vslam/collaborative_vslam.h"

CollaborativeVSLAM::CollaborativeVSLAM():private_nh_("~")
{
    // ===== Parameter Server =====
    // for collaborative system
    private_nh_.getParam("hz", hz_);
    private_nh_.getParam("move_dist_th", move_dist_th_);
    private_nh_.getParam("scale_ratio_th_percent", scale_ratio_th_percent_);
    private_nh_.getParam("duration_init", duration_init_);
    private_nh_.param<std::string>("map_frame_id", map_frame_id_, "map");
    // for leader robot
    // for follower robot

    // ===== Other Parameters Initilize =====
    // for collaborative system
    flag_move_ = false;
    is_initialized_ = false;
    scale_ratio_ = 0.0;
    wheel_dist_for_init_ = 0.0;
    visual_dist_for_init_ = 0.0;
    // for leader robot
    leader_lost_count_ = 0;
    leader_flag_visual_init_.data = false;
    leader_flag_lost_.data = false;
    leader_flag_map_merge_.data = false;
    // for follower robot
    follower_lost_count_ = 0;
    follower_flag_visual_init_.data = false;
    follower_flag_lost_.data = false;
    follower_flag_map_merge_.data = false;


    // ===== Subscriber =====
    // from leader robot
    leader_pose_sub_           = nh_.subscribe("/leader/pose", 1, &CollaborativeVSLAM::leader_pose_callback, this);
    leader_active_map_sub_     = nh_.subscribe("/leader/active_map", 1, &CollaborativeVSLAM::leader_active_map_callback, this);
    leader_lost_sign_sub_      = nh_.subscribe("/leader/lost_sign", 1, &CollaborativeVSLAM::leader_lost_sign_callback, this);
    leader_map_merge_sign_sub_ = nh_.subscribe("/leader/map_merge_sign", 1, &CollaborativeVSLAM::leader_map_merge_sign_callback, this);
    leader_relative_angle_sub_ = nh_.subscribe("/leader/follower_relative_angle", 1, &CollaborativeVSLAM::leader_relative_angle_callback, this);
    leader_wheel_odom_sub_     = nh_.subscribe("/leader/wheel_odom", 1, &CollaborativeVSLAM::leader_wheel_odom_callback, this);
    // from follower robot
    follower_pose_sub_         = nh_.subscribe("/follower/pose", 1, &CollaborativeVSLAM::follower_pose_callback, this);
    follower_relative_pos_sub_ = nh_.subscribe("/follower/leader_relative_position", 1, &CollaborativeVSLAM::follower_relative_pos_callback, this);

    // ===== Publisher =====
    leader_pose_pub_   = nh_.advertise<geometry_msgs::PoseStamped>("/leader/collaborative_pose", 1);
    leader_map_pub_    = nh_.advertise<sensor_msgs::PointCloud2>("/leader/collaborative_map", 1);
    follower_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/follower/collaborative_pose", 1);

    // frame idの設定
    leader_last_pose_.header.frame_id = map_frame_id_;
}


void CollaborativeVSLAM::process()
{
    ros::Rate loop_rate(hz_);

    while(ros::ok())
    {
        init_scale_ratio();
        // co_vslam();
        ros::spinOnce();
        loop_rate.sleep();
    }
}


// callback function for leader robot
void CollaborativeVSLAM::leader_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    if(leader_flag_lost_.data) return;
    leader_prev_pose_ = leader_last_pose_;
    leader_last_pose_ = *msg;

    if(not flag_move_)
    {
        const float move_dist = calc_hypot(leader_last_pose_.pose.position, leader_prev_pose_.pose.position);
        // ROS_INFO_STREAM("move_dist = " << move_dist << " | th = " << move_dist_th_ << " | flag_move-> " << (move_dist_th_ < move_dist));
        if(move_dist_th_ < move_dist)
        {
            flag_move_ = true; // 動き出したらフラグを立てる
            init_begin_ = ros::Time::now(); // スケール比の初回化スタート時間の設定
        }
    }
}

void CollaborativeVSLAM::leader_active_map_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    leader_active_map_ = *msg;
}

void CollaborativeVSLAM::leader_visual_init_sign_callback(const std_msgs::Bool::ConstPtr& msg)
{
    leader_flag_visual_init_ = *msg;
}

void CollaborativeVSLAM::leader_lost_sign_callback(const std_msgs::Bool::ConstPtr& msg)
{
    leader_flag_lost_ = *msg;
}

void CollaborativeVSLAM::leader_map_merge_sign_callback(const std_msgs::Bool::ConstPtr& msg)
{
    leader_flag_map_merge_ = *msg;
}

void CollaborativeVSLAM::leader_relative_angle_callback(const color_detector_msgs::TargetAngle::ConstPtr& msg)
{
    leader_relative_angle_ = *msg;
}

void CollaborativeVSLAM::leader_wheel_odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    leader_prev_odom_ = leader_last_odom_;
    leader_last_odom_ = *msg;
}

// callback function for follower robot
void CollaborativeVSLAM::follower_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    follower_pose_ = *msg;
}

void CollaborativeVSLAM::follower_visual_init_sign_callback(const std_msgs::Bool::ConstPtr& msg)
{
    follower_flag_visual_init_ = *msg;
}

void CollaborativeVSLAM::follower_lost_sign_callback(const std_msgs::Bool::ConstPtr& msg)
{
    follower_flag_lost_ = *msg;
}

void CollaborativeVSLAM::follower_map_merge_sign_callback(const std_msgs::Bool::ConstPtr& msg)
{
    follower_flag_map_merge_ = *msg;
}

void CollaborativeVSLAM::follower_relative_pos_callback(const object_detector_msgs::ObjectPositions::ConstPtr& msg)
{
    for(const auto& obj : msg->object_position)
        if(obj.Class == "roomba")
            follower_relative_pos_ = obj;
}

void CollaborativeVSLAM::init_scale_ratio()
{
    if(not flag_move_ or is_initialized_) return;

    const double elapsed_time = ros::Time::now().toSec() - init_begin_.toSec();
    if(elapsed_time < duration_init_) // 移動量を加算
    {
        wheel_dist_for_init_  += calc_hypot(leader_last_pose_.pose.position, leader_prev_pose_.pose.position);
        visual_dist_for_init_ += calc_hypot(leader_last_odom_.pose.pose.position, leader_prev_odom_.pose.pose.position);
    }
    else // スケール比を計算
    {
        const float new_ratio = visual_dist_for_init_ / wheel_dist_for_init_;
        is_initialized_ = fabs((scale_ratio_ - new_ratio)/scale_ratio_) < scale_ratio_th_percent_/100.0;

        // ROS_INFO_STREAM("---");
        // ROS_INFO_STREAM("old_ratio = " << scale_ratio_ << "[%]");
        // ROS_INFO_STREAM("new_ratio = " << new_ratio << "[%]");
        // ROS_INFO_STREAM("ratio_diff = " << fabs((scale_ratio_ - new_ratio)/scale_ratio_)*100.0 << "[%] | goal = " << scale_ratio_th_percent_ << "[%] | flag_init -> " << is_initialized_);

        scale_ratio_    = new_ratio;
        init_begin_ = ros::Time::now(); // 初回化スタート時間の設定
    }
}

float CollaborativeVSLAM::calc_hypot(geometry_msgs::Point last_point, geometry_msgs::Point prev_point)
{
    const float dx = last_point.x - prev_point.x;
    const float dy = last_point.y - prev_point.y;
    const float dz = last_point.z - prev_point.z;
    return hypot(dx, dy, dz);
}

void CollaborativeVSLAM::co_vslam()
{
    co_localize();
    co_mapping();
}

void CollaborativeVSLAM::co_localize()
{
    if(leader_flag_lost_.data)
    {
        calc_leader_pose();
    }

    leader_pose_pub_.publish(leader_last_pose_);
}

void CollaborativeVSLAM::calc_leader_pose()
{
    calc_leader_pos(leader_last_pose_.pose.position.x, leader_last_pose_.pose.position.z);
    calc_leader_quat(leader_last_pose_.pose.orientation);
}

void CollaborativeVSLAM::calc_leader_pos(double& leader_x, double& leader_z)
{
    const double follower_x     = follower_pose_.pose.position.x;
    const double follower_z     = follower_pose_.pose.position.z;
    const double follower_pitch = getPitch(follower_pose_.pose.orientation);

    const double leader_x_from_follower = follower_relative_pos_.x;
    const double leader_z_from_follower = follower_relative_pos_.z;

    leader_x = follower_x + leader_x_from_follower * cos(follower_pitch) + leader_z_from_follower * sin(follower_pitch);
    leader_z = follower_z - leader_x_from_follower * sin(follower_pitch) + leader_z_from_follower * cos(follower_pitch);
}

void CollaborativeVSLAM::calc_leader_quat(geometry_msgs::Quaternion& leader_quat_msg)
{
    const float follower_pitch = getPitch(follower_pose_.pose.orientation);
    const float follower_x     = follower_pose_.pose.position.x;
    const float follower_z     = follower_pose_.pose.position.z;
    float leader_pitch   = -M_PI + follower_pitch + atan2(follower_x, follower_z) + leader_relative_angle_.radian;
    leader_pitch = normalize_angle(leader_pitch);

    // pitchからquaternionを算出
    tf::Quaternion leader_quat;
    leader_quat.setRPY(0, leader_pitch, 0);
    tf::quaternionTFToMsg(leader_quat, leader_quat_msg);
}

float CollaborativeVSLAM::getPitch(geometry_msgs::Quaternion& quat_msg)
{
    double r, p, y;
    tf::Quaternion quat(quat_msg.x, quat_msg.y, quat_msg.z, quat_msg.w);
    tf::Matrix3x3(quat).getRPY(r, p, y);

    return float(p);
}

// 適切な角度(-M_PI ~ M_PI)を返す
float CollaborativeVSLAM::normalize_angle(float angle)
{
    while(M_PI  < angle) angle -= 2.0*M_PI;
    while(angle < -M_PI) angle += 2.0*M_PI;

    return angle;
}

void CollaborativeVSLAM::co_mapping()
{
    if(!leader_flag_lost_.data)
    {

    }
    else
    {
        leader_map_pub_.publish(leader_co_map_);
    }
}
