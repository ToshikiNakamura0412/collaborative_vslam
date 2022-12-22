#include "collaborative_vslam/collaborative_vslam.h"

CollaborativeVSLAM::CollaborativeVSLAM():private_nh_("~")
{
    // ===== Parameter Server =====
    // for collaborative system
    private_nh_.getParam("hz", hz_);
    private_nh_.param<std::string>("map_frame_id", map_frame_id_, "world");
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
    // for leader robot
    leader_pose_pub_               = nh_.advertise<geometry_msgs::PoseStamped>("/leader/collaborative_pose", 1);
    leader_pose_from_follower_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/leader/pose_from_follower", 1);
    leader_map_pub_                = nh_.advertise<sensor_msgs::PointCloud2>("/leader/collaborative_map", 1);
    // for follower robot
    follower_pose_pub_       = nh_.advertise<geometry_msgs::PoseStamped>("/follower/collaborative_pose", 1);
    follower_map_origin_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/follower/map_origin", 1);

    // frame idの設定
    leader_pose_from_follower_.header.frame_id = map_frame_id_;
    follower_map_origin_.header.frame_id       = map_frame_id_;
}

// main process
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
    if(leader_flag_init_visual_.data) ROS_INFO_STREAM("leader start tracking!!");
}

void CollaborativeVSLAM::leader_init_ratio_sign_callback(const std_msgs::Bool::ConstPtr& msg)
{
    leader_flag_init_ratio_ = *msg;
    if(leader_flag_init_ratio_.data) ROS_INFO_STREAM("leader initialized scale ratio!!");
}

void CollaborativeVSLAM::leader_lost_sign_callback(const std_msgs::Bool::ConstPtr& msg)
{
    leader_flag_lost_ = *msg;
}

void CollaborativeVSLAM::leader_map_merge_sign_callback(const std_msgs::Bool::ConstPtr& msg)
{
    leader_flag_map_merge_ = *msg;
}

void CollaborativeVSLAM::leader_relative_angle_callback(const color_detector_msgs::TargetAngleList::ConstPtr& msg)
{
    for(const auto& angle : msg->data)
        if(angle.color == "blue")
            leader_relative_angle_ = angle;
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
    if(follower_flag_init_visual_.data) ROS_INFO_STREAM("follower start tracking!!");
}

void CollaborativeVSLAM::follower_init_ratio_sign_callback(const std_msgs::Bool::ConstPtr& msg)
{
    follower_flag_init_ratio_ = *msg;
    if(follower_flag_init_ratio_.data) ROS_INFO_STREAM("follower initialized scale ratio!!");
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
    follower_relative_pos_set_.object_position.clear();
    for(const auto& obj : msg->object_position)
        if(obj.Class == "roomba")
            follower_relative_pos_set_.object_position.push_back(obj);

    const int obj_count = follower_relative_pos_set_.object_position.size();
    if(obj_count > 0)
    {
        int index_of_min_dist = 0;
        double min_dist = calc_hypot(follower_relative_pos_, follower_relative_pos_set_.object_position[0]);
        for(int i=0; i<obj_count; i++)
        {
            if(calc_hypot(follower_relative_pos_set_.object_position[i])/calc_hypot(follower_relative_pos_) > 1.2)
                continue; // 距離が大きく異なる場合スキップ

            const double tmp_dist = calc_hypot(follower_relative_pos_, follower_relative_pos_set_.object_position[i]);
            if(tmp_dist < min_dist)
            {
                min_dist = tmp_dist;
                index_of_min_dist = i;
            }
        }
        follower_relative_pos_ = follower_relative_pos_set_.object_position[index_of_min_dist];
    }
}

double CollaborativeVSLAM::calc_hypot(const object_detector_msgs::ObjectPosition obj1, const object_detector_msgs::ObjectPosition obj2)
{
    const double dx = obj1.x - obj2.x;
    const double dy = obj1.y - obj2.y;
    const double dz = obj1.z - obj2.z;
    return hypot(dx, dy, dz);
}
double CollaborativeVSLAM::calc_hypot(const object_detector_msgs::ObjectPosition obj)
{
    return hypot(obj.x, obj.y, obj.z);
}

void CollaborativeVSLAM::co_vslam()
{
    if(can_set_tf_for_map())
        set_tf_for_map();
    if(follower_flag_get_tf_)
        follower_map_origin_pub_.publish(follower_map_origin_);

    co_localize();
    co_mapping();
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

void CollaborativeVSLAM::set_tf_for_map()
{
    const Point  leader_pos(leader_pose_.pose.position);
    const double follower_pitch = getPitch(follower_pose_.pose.orientation);
    const Point  relative_pos = rotate_pitch(follower_relative_pos_, follower_pitch);
    const Point  follower_pos(follower_pose_.pose.position);

    Point follower_tf_to_leader =
        leader_pos - adjust_scale_to_leader(relative_pos) - adjust_follower_scale_to_leader(follower_pos);

    follower_tf_to_leader.output_xz(follower_map_origin_);
    follower_flag_get_tf_ = true;
    ROS_INFO_STREAM("Set follower map origin!!");
}

double CollaborativeVSLAM::getPitch(geometry_msgs::Quaternion& quat_msg)
{
    double r, p, y;
    tf::Quaternion quat(quat_msg.x, quat_msg.y, quat_msg.z, quat_msg.w);
    tf::Matrix3x3(quat).getRPY(r, p, y);

    return p;
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

Point CollaborativeVSLAM::rotate_pitch(const object_detector_msgs::ObjectPosition& input_point, const double pitch)
{
    const double x =  input_point.x*cos(pitch) + input_point.z*sin(pitch);
    const double z = -input_point.x*sin(pitch) + input_point.z*cos(pitch);
    return Point(x, z);
}

void CollaborativeVSLAM::co_localize()
{
    // if(leader_flag_lost_.data)
    calc_leader_pose();

    // pub
    leader_pose_pub_.publish(leader_pose_);
}

void CollaborativeVSLAM::calc_leader_pose()
{
    if(!follower_flag_get_tf_) return;

    calc_follower_pose_in_leader_map();
    follower_pose_pub_.publish(follower_pose_in_leader_map_);

    calc_leader_pos();
    calc_leader_quat(leader_pose_from_follower_.pose.orientation);
    leader_pose_from_follower_pub_.publish(leader_pose_from_follower_);
}

void CollaborativeVSLAM::calc_follower_pose_in_leader_map()
{
    const Point follower_map_origin(follower_map_origin_);
    const Point follower_pos(follower_pose_.pose.position);

    Point follower_pose_in_leader_map = follower_map_origin + adjust_follower_scale_to_leader(follower_pos);
    follower_pose_in_leader_map.output(follower_pose_in_leader_map_);
}

void CollaborativeVSLAM::calc_leader_pos()
{
    const Point  follower_pos(follower_pose_in_leader_map_.pose.position);
    const double follower_pitch = getPitch(follower_pose_.pose.orientation);
    const Point  relative_pos = rotate_pitch(follower_relative_pos_, follower_pitch);

    Point leader_pos = follower_pos + adjust_scale_to_leader(relative_pos);
    leader_pos.output(leader_pose_from_follower_);
}

void CollaborativeVSLAM::calc_leader_quat(geometry_msgs::Quaternion& leader_quat_msg)
{
    const double follower_pitch = getPitch(follower_pose_.pose.orientation);
    const double follower_x     = follower_pose_.pose.position.x;
    const double follower_z     = follower_pose_.pose.position.z;
    double leader_pitch   = -M_PI + follower_pitch + atan2(follower_x, follower_z) + leader_relative_angle_.radian;
    leader_pitch = normalize_angle(leader_pitch);

    // pitchからquaternionを算出
    tf::Quaternion leader_quat;
    leader_quat.setRPY(0, leader_pitch, 0);
    tf::quaternionTFToMsg(leader_quat, leader_quat_msg);
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
    leader_co_map_ = leader_active_map_;
    // if(leader_flag_lost_.data)
    leader_map_pub_.publish(leader_co_map_);
}
