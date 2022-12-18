#include "collaborative_vslam/collaborative_vslam.h"

CollaborativeVSLAM::CollaborativeVSLAM():private_nh_("~")
{
    // ===== Parameter Server =====
    // for collaborative system
    private_nh_.getParam("hz", hz_);
    private_nh_.param<std::string>("map_frame_id", map_frame_id_, "map");
    // for leader robot
    // for follower robot

    // ===== Other Parameters Initilize =====
    // for collaborative system
    // for leader robot
    leader_lost_count_ = 0;
    leader_flag_init_visual_.data = false;
    leader_flag_lost_.data = false;
    leader_flag_map_merge_.data = false;
    // for follower robot
    follower_flag_get_tf_ = false;
    follower_lost_count_  = 0;
    follower_flag_init_visual_.data = false;
    follower_flag_lost_.data = false;
    follower_flag_map_merge_.data = false;


    // ===== Subscriber =====
    // from leader robot
    leader_pose_sub_             = nh_.subscribe("/leader/pose", 1, &CollaborativeVSLAM::leader_pose_callback, this);
    leader_init_visual_sign_sub_ = nh_.subscribe("/leader/visual_sign", 1, &CollaborativeVSLAM::leader_init_visual_sign_callback, this);
    leader_init_ratio_sign_sub_  = nh_.subscribe("/leader/ratio_sign", 1, &CollaborativeVSLAM::leader_init_ratio_sign_callback, this);
    leader_scale_ratio_sub_      = nh_.subscribe("/leader/scale_ratio", 1, &CollaborativeVSLAM::leader_scale_ratio_callback, this);
    leader_active_map_sub_       = nh_.subscribe("/leader/active_map", 1, &CollaborativeVSLAM::leader_active_map_callback, this);
    leader_lost_sign_sub_        = nh_.subscribe("/leader/lost_sign", 1, &CollaborativeVSLAM::leader_lost_sign_callback, this);
    leader_map_merge_sign_sub_   = nh_.subscribe("/leader/map_merge_sign", 1, &CollaborativeVSLAM::leader_map_merge_sign_callback, this);
    leader_relative_angle_sub_   = nh_.subscribe("/leader/follower_relative_angle", 1, &CollaborativeVSLAM::leader_relative_angle_callback, this);
    // from follower robot
    follower_pose_sub_             = nh_.subscribe("/follower/pose", 1, &CollaborativeVSLAM::follower_pose_callback, this);
    follower_init_visual_sign_sub_ = nh_.subscribe("/follower/visual_sign", 1, &CollaborativeVSLAM::follower_init_visual_sign_callback, this);
    follower_init_ratio_sign_sub_  = nh_.subscribe("/follower/ratio_sign", 1, &CollaborativeVSLAM::follower_init_ratio_sign_callback, this);
    follower_scale_ratio_sub_      = nh_.subscribe("/follower/scale_ratio", 1, &CollaborativeVSLAM::follower_scale_ratio_callback, this);
    follower_relative_pos_sub_     = nh_.subscribe("/follower/relative_position", 1, &CollaborativeVSLAM::follower_relative_pos_callback, this);

    // ===== Publisher =====
    leader_pose_pub_   = nh_.advertise<geometry_msgs::PoseStamped>("/leader/collaborative_pose", 1);
    leader_map_pub_    = nh_.advertise<sensor_msgs::PointCloud2>("/leader/collaborative_map", 1);
    follower_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/follower/collaborative_pose", 1);

    // frame idの設定
    leader_pose_.header.frame_id = map_frame_id_;
}


void CollaborativeVSLAM::process()
{
    ros::Rate loop_rate(hz_);

    while(ros::ok())
    {
        co_vslam();
        ros::spinOnce();
        loop_rate.sleep();
    }
}


// callback function for leader robot
void CollaborativeVSLAM::leader_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    leader_pose_ = *msg;
}

void CollaborativeVSLAM::leader_scale_ratio_callback(const std_msgs::Float64::ConstPtr& msg)
{
    leader_scale_ratio_ = msg->data;
}

void CollaborativeVSLAM::leader_active_map_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    leader_active_map_ = *msg;
}

void CollaborativeVSLAM::leader_init_visual_sign_callback(const std_msgs::Bool::ConstPtr& msg)
{
    leader_flag_init_visual_ = *msg;
}

void CollaborativeVSLAM::leader_init_ratio_sign_callback(const std_msgs::Bool::ConstPtr& msg)
{
    leader_flag_init_ratio_ = *msg;
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

// callback function for follower robot
void CollaborativeVSLAM::follower_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    follower_pose_ = *msg;
}

void CollaborativeVSLAM::follower_scale_ratio_callback(const std_msgs::Float64::ConstPtr& msg)
{
    follower_scale_ratio_ = msg->data;
}

void CollaborativeVSLAM::follower_init_visual_sign_callback(const std_msgs::Bool::ConstPtr& msg)
{
    follower_flag_init_visual_ = *msg;
}

void CollaborativeVSLAM::follower_init_ratio_sign_callback(const std_msgs::Bool::ConstPtr& msg)
{
    follower_flag_init_ratio_ = *msg;
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

    if(can_set_tf_for_map())
        set_tf_for_map();
}

void CollaborativeVSLAM::set_tf_for_map()
{
    const Point leader_pos(leader_pose_.pose.position);
    const Point follower_pos(follower_pose_.pose.position);
    const double follower_pitch = getPitch(follower_pose_.pose.orientation);
    Point relative_pos;

    rotate_pitch(follower_relative_pos_, follower_pitch, relative_pos); // 回転

    follower_tf_to_leader_ =
        leader_pos - adjust_scale_to_leader(relative_pos) - adjust_follower_scale_to_leader(follower_pos);

    follower_flag_get_tf_ = true;

    // debug
    ros::Publisher follower_map_origin_pub;
    follower_map_origin_pub = nh_.advertise<geometry_msgs::Point>("/follower/map_origin", 1);
    geometry_msgs::Point follower_map_origin;
    follower_map_origin.x = follower_tf_to_leader_.x;
    follower_map_origin.y = 0.0;
    follower_map_origin.z = follower_tf_to_leader_.z;
    follower_map_origin_pub.publish(follower_map_origin);
}

Point CollaborativeVSLAM::adjust_follower_scale_to_leader(const Point point)
{
    return adjust_scale_to_leader(point) / follower_scale_ratio_;
}

Point CollaborativeVSLAM::adjust_leader_scale_to_follower(const Point point)
{
    return adjust_scale_to_follower(point) / leader_scale_ratio_;
}

Point CollaborativeVSLAM::adjust_scale_to_leader(const Point point)
{
    return point * leader_scale_ratio_;
}

Point CollaborativeVSLAM::adjust_scale_to_follower(const Point point)
{
    return point * follower_scale_ratio_;
}

void CollaborativeVSLAM::rotate_pitch(const object_detector_msgs::ObjectPosition& input_point, const double pitch, Point& output_point)
{
    const double x =  input_point.x*cos(pitch) + input_point.z*sin(pitch);
    const double z = -input_point.x*sin(pitch) + input_point.z*cos(pitch);
    output_point.set(x, z);
}
void CollaborativeVSLAM::rotate_pitch(const geometry_msgs::Point& input_point, const double pitch, Point& output_point)
{
    const double x =  input_point.x*cos(pitch) + input_point.z*sin(pitch);
    const double z = -input_point.x*sin(pitch) + input_point.z*cos(pitch);
    output_point.set(x, z);
}

bool CollaborativeVSLAM::can_set_tf_for_map()
{
    return is_init_leader() and is_init_follower() and not follower_flag_get_tf_;
}

bool CollaborativeVSLAM::is_init_leader()
{
    return leader_flag_init_visual_.data and leader_flag_init_ratio_.data;
}

bool CollaborativeVSLAM::is_init_follower()
{
    return follower_flag_init_visual_.data and follower_flag_init_ratio_.data;
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

    leader_pose_pub_.publish(leader_pose_);
}

void CollaborativeVSLAM::calc_leader_pose()
{
    calc_leader_pos(leader_pose_.pose.position.x, leader_pose_.pose.position.z);
    calc_leader_quat(leader_pose_.pose.orientation);
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

double CollaborativeVSLAM::getPitch(geometry_msgs::Quaternion& quat_msg)
{
    double r, p, y;
    tf::Quaternion quat(quat_msg.x, quat_msg.y, quat_msg.z, quat_msg.w);
    tf::Matrix3x3(quat).getRPY(r, p, y);

    return p;
}

// 適切な角度(-M_PI ~ M_PI)を返す
double CollaborativeVSLAM::normalize_angle(double angle)
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
