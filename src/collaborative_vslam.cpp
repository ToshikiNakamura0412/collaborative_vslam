#include "collaborative_vslam/collaborative_vslam.h"

CollaborativeVSLAM::CollaborativeVSLAM():private_nh_("~")
{
    // ----- Parameter Server -----
    private_nh_.getParam("hz", hz_);
    private_nh_.param<std::string>("map_frame_id", map_frame_id_, "world");

    // ----- Other Parameters Initialization -----
    co_flag_tf_map_origin_ = false;
    co_flag_lost_leader_   = false;
    co_flag_lost_follower_ = false;


    // ----- Subscriber -----
    // from leader robot
    leader_scale_ratio_sub_      = nh_.subscribe("/leader/scale_ratio", 1, &CollaborativeVSLAM::leader_scale_ratio_callback, this, ros::TransportHints().reliable().tcpNoDelay());
    leader_init_visual_sign_sub_ = nh_.subscribe("/leader/visual_sign", 1, &CollaborativeVSLAM::leader_init_visual_sign_callback, this, ros::TransportHints().reliable().tcpNoDelay());
    leader_lost_sign_sub_        = nh_.subscribe("/leader/lost_sign", 1, &CollaborativeVSLAM::leader_lost_sign_callback, this);
    leader_map_merge_sign_sub_   = nh_.subscribe("/leader/map_merge_sign", 1, &CollaborativeVSLAM::leader_map_merge_sign_callback, this);
    leader_pose_sub_             = nh_.subscribe("/leader/pose", 1, &CollaborativeVSLAM::leader_pose_callback, this);
    leader_active_map_sub_       = nh_.subscribe("/leader/active_map", 1, &CollaborativeVSLAM::leader_active_map_callback, this);
    leader_relative_angle_sub_   = nh_.subscribe("/leader/follower_relative_angle", 1, &CollaborativeVSLAM::leader_relative_angle_callback, this);
    // from follower robot
    follower_scale_ratio_sub_      = nh_.subscribe("/follower/scale_ratio", 1, &CollaborativeVSLAM::follower_scale_ratio_callback, this, ros::TransportHints().reliable().tcpNoDelay());
    follower_init_visual_sign_sub_ = nh_.subscribe("/follower/visual_sign", 1, &CollaborativeVSLAM::follower_init_visual_sign_callback, this, ros::TransportHints().reliable().tcpNoDelay());
    follower_lost_sign_sub_        = nh_.subscribe("/follower/lost_sign", 1, &CollaborativeVSLAM::follower_lost_sign_callback, this);
    follower_map_merge_sign_sub_   = nh_.subscribe("/follower/map_merge_sign", 1, &CollaborativeVSLAM::follower_map_merge_sign_callback, this);
    follower_pose_sub_             = nh_.subscribe("/follower/pose", 1, &CollaborativeVSLAM::follower_pose_callback, this);
    follower_active_map_sub_       = nh_.subscribe("/follower/active_map", 1, &CollaborativeVSLAM::follower_active_map_callback, this);
    follower_relative_pos_sub_     = nh_.subscribe("/follower/relative_position", 1, &CollaborativeVSLAM::follower_relative_pos_callback, this);

    // ----- Publisher -----
    // for collaborative system
    co_map_origin_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/system/map_origin", 1);
    // for leader robot
    leader_pose_pub_                 = nh_.advertise<geometry_msgs::PoseStamped>("/leader/collaborative_pose", 1);
    leader_pose_from_follower_pub_   = nh_.advertise<geometry_msgs::PoseStamped>("/leader/pose_from_follower", 1);
    leader_pose_on_return_pub_       = nh_.advertise<geometry_msgs::PoseStamped>("/leader_pose_on_return", 1);
    leader_pose_in_follower_map_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/follower/leader_pose", 1);
    leader_map_pub_                  = nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("/leader/collaborative_map", 10);
    // for follower robot
    follower_pose_pub_               = nh_.advertise<geometry_msgs::PoseStamped>("/follower/collaborative_pose", 1);
    follower_pose_from_leader_pub_   = nh_.advertise<geometry_msgs::PoseStamped>("/follower/pose_from_leader", 1);
    follower_pose_in_leader_map_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/leader/follower_pose", 1);
    follower_map_pub_                = nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("/follower/collaborative_map", 10);


    // ----- 基本設定 -----
    // Frame IDの設定
    // for collaborative system
    co_map_origin_.header.frame_id = map_frame_id_;
    // for leader robot
    leader_pose_.header.frame_id                 = map_frame_id_;
    leader_pose_from_follower_.header.frame_id   = map_frame_id_;
    leader_pose_on_return_.header.frame_id       = map_frame_id_;
    leader_pose_in_follower_map_.header.frame_id = map_frame_id_;
    leader_active_map_.header.frame_id           = map_frame_id_;
    leader_stored_map_.header.frame_id           = map_frame_id_;
    // for follower robot
    follower_pose_.header.frame_id               = map_frame_id_;
    follower_pose_from_leader_.header.frame_id   = map_frame_id_;
    follower_pose_in_leader_map_.header.frame_id = map_frame_id_;
    follower_active_map_.header.frame_id         = map_frame_id_;
    follower_stored_map_.header.frame_id         = map_frame_id_;
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
void CollaborativeVSLAM::leader_scale_ratio_callback(const std_msgs::Float64::ConstPtr& msg)
{
    leader_.scale_ratio = msg->data;
    leader_.flag_init_ratio = true;
    ROS_INFO_STREAM("leader initialized scale ratio!!");
    ROS_INFO_STREAM("scale ratio = " << leader_.scale_ratio);
}

void CollaborativeVSLAM::leader_init_visual_sign_callback(const std_msgs::Bool::ConstPtr& msg)
{
    leader_.flag_init_visual = msg->data;
    if(leader_.flag_init_visual)
    {
        ROS_INFO_STREAM("leader start tracking!!");
        leader_pose_on_return_  = leader_pose_from_follower_;
        co_flag_tf_map_origin_  = false;
        leader_.flag_init_ratio = false;
    }
}

void CollaborativeVSLAM::leader_lost_sign_callback(const std_msgs::Bool::ConstPtr& msg)
{
    if(not leader_.flag_lost and leader_.flag_lost!=msg->data) ROS_INFO_STREAM("lost");
    leader_.flag_lost = msg->data;

    if(leader_.flag_lost and not co_flag_lost_leader_)
    {
        co_flag_lost_leader_ = true;
        update_leader_stored_map(); // TF済のactive_mapをstored_mapに追加
        leader_.lost_count++;
    }
    else if(leader_.flag_init_visual)
    {
        co_flag_lost_leader_ = false;
    }
}

// TF済のactive_mapをstored_mapに追加
void CollaborativeVSLAM::update_leader_stored_map()
{
    // 復帰した地点をベースにmappointsをTF
    if(leader_.lost_count > 0)
        tf_leader_active_map(); // TF済の場合，スキップされる

    // stored_mapに追加
    for(const auto& point : leader_active_map_.points)
        leader_stored_map_.points.push_back(point);
}

void CollaborativeVSLAM::leader_map_merge_sign_callback(const std_msgs::Bool::ConstPtr& msg)
{
    if(msg->data)
    {
        co_flag_tf_map_origin_ = false;
        leader_.lost_count = 0;
        leader_stored_map_.points.clear();
    }
}

void CollaborativeVSLAM::leader_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    if(calc_hypot(*msg) > 1e-6) leader_pose_ = *msg;
}

void CollaborativeVSLAM::leader_active_map_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    if(msg->data.size() != 0)
    {
        pcl::fromROSMsg(*msg, leader_active_map_);
        leader_.flag_tf_map = false;
    }
}

bool CollaborativeVSLAM::judge_angle(double angle)
{
    if(angle < 0) angle = 2.0*M_PI+angle;

    if(leader_relative_angle_log_.size() >= 20)
    {
        double sum = 0.0;
        for(const auto& angle : leader_relative_angle_log_) sum += angle;
        const double avg = sum/leader_relative_angle_log_.size();
        if(avg-M_PI/2.0 <= angle and angle <= avg+M_PI/2.0)
        {
            leader_relative_angle_log_.push_back(angle);

            while(leader_relative_angle_log_.size() > 20)
                leader_relative_angle_log_.erase(leader_relative_angle_log_.begin());
            return true;
        }
        else
        {
            return false;
        }
    }
    else
    {
        leader_relative_angle_log_.push_back(angle);
        return true;
    }
}

void CollaborativeVSLAM::leader_relative_angle_callback(const color_detector_msgs::TargetAngleList::ConstPtr& msg)
{
    for(const auto& angle : msg->data)
        if(angle.color == "blue" and not std::isnan(angle.radian) and judge_angle(angle.radian))
            leader_relative_angle_ = angle;
}


// callback function for follower robot
void CollaborativeVSLAM::follower_scale_ratio_callback(const std_msgs::Float64::ConstPtr& msg)
{
    follower_.scale_ratio = msg->data;
    follower_.flag_init_ratio = true;
    ROS_INFO_STREAM("follower initialized scale ratio!!");
    ROS_INFO_STREAM("scale ratio = " << follower_.scale_ratio);
}

void CollaborativeVSLAM::follower_init_visual_sign_callback(const std_msgs::Bool::ConstPtr& msg)
{
    follower_.flag_init_visual = msg->data;
    if(follower_.flag_init_visual)
    {
        ROS_INFO_STREAM("follower start tracking!!");
        follower_pose_on_return_  = follower_co_pose_;
        co_flag_tf_map_origin_    = false;
        follower_.flag_init_ratio = false;
    }
}

void CollaborativeVSLAM::follower_lost_sign_callback(const std_msgs::Bool::ConstPtr& msg)
{
    if(not follower_.flag_lost and follower_.flag_lost!=msg->data) ROS_INFO_STREAM("lost");
    follower_.flag_lost = msg->data;

    if(follower_.flag_lost and not co_flag_lost_follower_)
    {
        co_flag_lost_follower_ = true;
        update_follower_stored_map(); // TF済のactive_mapをstored_mapに追加
        follower_.lost_count++;
    }
    else if(follower_.flag_init_visual)
    {
        co_flag_lost_follower_ = false;
    }
}

// TF済のactive_mapをstored_mapに追加
void CollaborativeVSLAM::update_follower_stored_map()
{
    // 復帰した地点をベースにmappointsをTF
    if(follower_.lost_count > 0)
        tf_follower_active_map(); // TF済の場合，スキップされる

    // stored_mapに追加
    for(const auto& point : follower_active_map_.points)
        follower_stored_map_.points.push_back(point);
}

void CollaborativeVSLAM::follower_map_merge_sign_callback(const std_msgs::Bool::ConstPtr& msg)
{
    if(msg->data)
    {
        co_flag_tf_map_origin_ = false;
        follower_.lost_count = 0;
        follower_stored_map_.points.clear();
    }
}

void CollaborativeVSLAM::follower_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    if(calc_hypot(*msg) > 1e-6) follower_pose_ = *msg;
}

void CollaborativeVSLAM::follower_active_map_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    if(msg->data.size() != 0)
    {
        pcl::fromROSMsg(*msg, follower_active_map_);
        follower_.flag_tf_map = false;
    }
}

void CollaborativeVSLAM::follower_relative_pos_callback(const object_detector_msgs::ObjectPositions::ConstPtr& msg)
{
    // yoloの結果を取得
    follower_relative_pos_set_.object_position.clear();
    for(const auto& obj : msg->object_position)
        if(obj.Class == "roomba" and not std::isnan(obj.x))
            follower_relative_pos_set_.object_position.push_back(obj);

    // 同時に複数のRoombaが検出された場合の対策
    const int obj_count = follower_relative_pos_set_.object_position.size();
    if(obj_count > 0)
    {
        int index_of_min_dist = 0;
        double min_dist = calc_hypot(follower_relative_pos_, follower_relative_pos_set_.object_position[0]);
        for(int i=0; i<obj_count; i++)
        {
            // 距離が大きく異なる場合スキップ
            if(calc_hypot(follower_relative_pos_set_.object_position[i])/calc_hypot(follower_relative_pos_) > 1.5)
            {
                continue;
            }
            // 前回の位置とのユークリッド距離を算出
            const double tmp_dist = calc_hypot(follower_relative_pos_, follower_relative_pos_set_.object_position[i]);
            // 最小値の更新
            if(tmp_dist < min_dist)
            {
                min_dist = tmp_dist;
                index_of_min_dist = i;
            }
        }
        // followerから見たleaderの相対位置の取得
        follower_relative_pos_ = follower_relative_pos_set_.object_position[index_of_min_dist];
    }
}

// 2点間のユークリッド距離を算出
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
double CollaborativeVSLAM::calc_hypot(const geometry_msgs::PoseStamped& pose)
{
    return hypot(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
}


// ===== 協調的vslam =====
void CollaborativeVSLAM::co_vslam()
{
    if(can_set_tf_for_map())
        set_tf_for_map();
    if(co_flag_tf_map_origin_)
        co_map_origin_pub_.publish(co_map_origin_);

    co_localize();
    co_mapping();
}

bool CollaborativeVSLAM::can_set_tf_for_map()
{
    return leader_.flag_init_ratio and follower_.flag_init_ratio and not co_flag_tf_map_origin_;
}

void CollaborativeVSLAM::set_tf_for_map()
{
    const Point  leader_pos(leader_pose_.pose.position);
    const double follower_pitch = getPitch(follower_pose_.pose.orientation);
    const Point  relative_pos = rotate_pitch(follower_relative_pos_, follower_pitch);
    const Point  follower_pos(follower_pose_.pose.position);

    Point follower_tf_to_leader =
        leader_pos - adjust_scale_to_leader(relative_pos) - adjust_follower_scale_to_leader(follower_pos);

    follower_tf_to_leader.output_xz(co_map_origin_);
    co_flag_tf_map_origin_ = true;
    ROS_INFO_STREAM("Set follower map origin!!");
}

double CollaborativeVSLAM::getPitch(geometry_msgs::Quaternion& quat_msg)
{
    double y, p, r;
    tf2::getEulerYPR(quat_msg, y, p, r);

    if(M_PI/2.0 < abs(y) and M_PI/2.0 < abs(r))
        p = calc_normalized_angle(M_PI-p);

    return p;
}
double CollaborativeVSLAM::getPitch(geometry_msgs::PoseStamped& pose_msg)
{
    return getPitch(pose_msg.pose.orientation);
}

Point CollaborativeVSLAM::adjust_follower_scale_to_leader(const Point point)
{
    return adjust_scale_to_leader(point) / follower_.scale_ratio;
}

Point CollaborativeVSLAM::adjust_leader_scale_to_follower(const Point point)
{
    return adjust_scale_to_follower(point) / leader_.scale_ratio;
}

Point CollaborativeVSLAM::adjust_scale_to_leader(const Point point)
{
    return point * leader_.scale_ratio;
}

Point CollaborativeVSLAM::adjust_scale_to_follower(const Point point)
{
    return point * follower_.scale_ratio;
}

Point CollaborativeVSLAM::rotate_pitch(const object_detector_msgs::ObjectPosition& input_point, const double pitch)
{
    const double x =  input_point.x*cos(pitch) + input_point.z*sin(pitch);
    const double z = -input_point.x*sin(pitch) + input_point.z*cos(pitch);
    return Point(x, z);
}
Point CollaborativeVSLAM::rotate_pitch(const geometry_msgs::PoseStamped& input_pose, const double pitch)
{
    return rotate_pitch(input_pose.pose.position, pitch);
}
Point CollaborativeVSLAM::rotate_pitch(const geometry_msgs::Point& input_point, const double pitch)
{
    const double x =  input_point.x*cos(pitch) + input_point.z*sin(pitch);
    const double z = -input_point.x*sin(pitch) + input_point.z*cos(pitch);
    return Point(x, z);
}
Point CollaborativeVSLAM::rotate_pitch(const pcl::PointXYZ& input_point, const double pitch)
{
    const double x =  input_point.x*cos(pitch) + input_point.z*sin(pitch);
    const double z = -input_point.x*sin(pitch) + input_point.z*cos(pitch);
    return Point(x, z);
}

// ===== 協調的localization =====
void CollaborativeVSLAM::co_localize()
{
    // leader
    calc_leader_pose();
    leader_co_pose_.header.stamp = ros::Time::now();
    leader_pose_pub_.publish(leader_co_pose_);
    leader_pose_on_return_pub_.publish(leader_pose_on_return_);
    // follower
    calc_follower_pose();
    follower_co_pose_.header.stamp = ros::Time::now();
    follower_pose_pub_.publish(follower_co_pose_);
}

// leader poseの算出
void CollaborativeVSLAM::calc_leader_pose()
{
    if(leader_.flag_lost) // ロスト中
        calc_leader_pose_from_follower_in_leader_map();
    else if(leader_.lost_count > 0) // 復帰後
        calc_leader_pose_after_return();
    else // ロスト前
        leader_co_pose_ = leader_pose_;
}

// followerから見たleader pose (in leader map)
void CollaborativeVSLAM::calc_leader_pose_from_follower_in_leader_map()
{
    if(!co_flag_tf_map_origin_) return;

    // follower pose (in leader map)
    calc_follower_pose_in_leader_map();
    follower_pose_in_leader_map_pub_.publish(follower_pose_in_leader_map_);

    // followerから見たleader pose (in leader map)
    calc_leader_pos_from_follower_in_leader_map();
    calc_leader_quat_from_follower_in_leader_map();
    leader_pose_from_follower_pub_.publish(leader_pose_from_follower_);

    // ロスト中
    if(leader_.flag_lost) leader_co_pose_ = leader_pose_from_follower_;
}

// follower pose (in leader map)の算出
void CollaborativeVSLAM::calc_follower_pose_in_leader_map()
{
    const Point follower_map_origin(co_map_origin_);
    const Point follower_pos(follower_pose_.pose.position);
    Point follower_pose_in_leader_map = follower_map_origin + adjust_follower_scale_to_leader(follower_pos);

    // positionの決定
    follower_pose_in_leader_map.output(follower_pose_in_leader_map_);
    // orientationの取得
    follower_pose_in_leader_map_.pose.orientation = follower_co_pose_.pose.orientation;
}

// [相対観測] followerから見たleader pos (in leader map)の算出
void CollaborativeVSLAM::calc_leader_pos_from_follower_in_leader_map()
{
    const Point  follower_pos(follower_pose_in_leader_map_.pose.position);
    const double follower_pitch = getPitch(follower_pose_in_leader_map_);
    const Point  relative_pos   = rotate_pitch(follower_relative_pos_, follower_pitch);
    Point leader_pos = follower_pos + adjust_scale_to_leader(relative_pos);

    // positionの決定
    leader_pos.output(leader_pose_from_follower_);
}

// [相対観測] followerから見たleader quat (in leader map)の算出
void CollaborativeVSLAM::calc_leader_quat_from_follower_in_leader_map()
{
    const double follower_pitch = getPitch(follower_pose_in_leader_map_);
    const double leader_x_from_follower = follower_relative_pos_.x;
    const double leader_z_from_follower = follower_relative_pos_.z;
    double leader_pitch = M_PI + follower_pitch - leader_relative_angle_.radian
        + atan2(leader_x_from_follower, leader_z_from_follower);

    if(leader_.flag_lost) // ロスト中
        leader_pitch = calc_normalized_angle(leader_pitch);
    else if(leader_.lost_count > 0) // 復帰後
        leader_pitch = calc_normalized_angle(leader_pitch + getPitch(leader_pose_on_return_));
    else // ロスト前
        leader_pitch = calc_normalized_angle(leader_pitch);

    // orientationの決定
    set_orientation(leader_pose_from_follower_, leader_pitch);
}

void  CollaborativeVSLAM::calc_leader_pose_after_return()
{
    // positionの決定
    Point leader_pos = calc_pos_after_leader_return(leader_pose_.pose.position);
    leader_pos.output(leader_co_pose_);
    // orientationの決定
    const double leader_pitch = calc_normalized_angle(getPitch(leader_pose_on_return_) + getPitch(leader_pose_));
    set_orientation(leader_co_pose_, leader_pitch);
}

Point CollaborativeVSLAM::calc_pos_after_leader_return(const geometry_msgs::Point input_point)
{
    const double map_origin_pitch = getPitch(leader_pose_on_return_);
    const Point  rotated_pos      = rotate_pitch(input_point, map_origin_pitch);

    return Point(leader_pose_on_return_) + rotated_pos;
}
void CollaborativeVSLAM::calc_pos_after_leader_return(pcl::PointXYZ& target_point)
{
    const double map_origin_pitch = getPitch(leader_pose_on_return_);
    const Point  rotated_pos      = rotate_pitch(target_point, map_origin_pitch);
    Point output_point = Point(leader_pose_on_return_) + rotated_pos;

    // positionの決定
    output_point.output_xz(target_point);
}


// follower poseの算出
void CollaborativeVSLAM::calc_follower_pose()
{
    // leaderから見たfollower pose (in follower map)
    calc_follower_pose_from_leader_in_follower_map();

    // follower poseを決定
    if(follower_.flag_lost) // ロスト中
        follower_co_pose_ = follower_pose_from_leader_;
    else if(follower_.lost_count > 0) // 復帰後
        calc_follower_pose_after_return();
    else // ロスト前
        follower_co_pose_ = follower_pose_;
}

// leaderから見たfollower pose (in follower map)
void CollaborativeVSLAM::calc_follower_pose_from_leader_in_follower_map()
{
    if(!co_flag_tf_map_origin_) return;

    // leader pose (in follower map)
    calc_leader_pose_in_follower_map();
    leader_pose_in_follower_map_pub_.publish(leader_pose_in_follower_map_);

    // leaderから見たfollower pose (in follower map)
    calc_follower_pos_from_leader_in_follower_map();
    calc_follower_quat_from_leader_in_follower_map(follower_pose_from_leader_.pose.orientation);
    follower_pose_from_leader_pub_.publish(follower_pose_from_leader_);
}

// leader pose (in follower map)の算出
void CollaborativeVSLAM::calc_leader_pose_in_follower_map()
{
    const Point follower_map_origin(co_map_origin_);
    const Point leader_map_origin(-follower_map_origin);
    const Point leader_pos(leader_pose_.pose.position);
    Point leader_pose_in_follower_map = leader_map_origin + adjust_leader_scale_to_follower(leader_pos);

    // positionの決定
    leader_pose_in_follower_map.output(leader_pose_in_follower_map_);
    // orientationの取得
    leader_pose_in_follower_map_.pose.orientation = leader_co_pose_.pose.orientation;
}

// [相対観測] leaderから見たfollower pos (in follower map)の算出
void CollaborativeVSLAM::calc_follower_pos_from_leader_in_follower_map()
{
    const Point  leader_pos(leader_pose_in_follower_map_.pose.position);
    const double leader_pitch = getPitch(leader_pose_in_follower_map_);
    const double tmp_radian   = leader_pitch + leader_relative_angle_.radian;
    const double dist_F_to_L  = calc_hypot(follower_relative_pos_);
    const Point  relative_pos(dist_F_to_L*sin(tmp_radian), dist_F_to_L*cos(tmp_radian)); // Point(x,z);
    Point follower_pos = leader_pos + adjust_scale_to_follower(relative_pos);

    follower_pos.output(follower_pose_from_leader_);
}

// [相対観測] leaderから見たfollower quat (in follower map)の算出
void CollaborativeVSLAM::calc_follower_quat_from_leader_in_follower_map(geometry_msgs::Quaternion& follower_quat_msg)
{
    const double leader_pitch = getPitch(leader_pose_in_follower_map_);
    const double leader_x_from_follower = follower_relative_pos_.x;
    const double leader_z_from_follower = follower_relative_pos_.z;
    double follower_pitch = -M_PI + leader_pitch + leader_relative_angle_.radian
        - atan2(leader_x_from_follower, leader_z_from_follower);

    if(follower_.flag_lost) // ロスト中
        follower_pitch = calc_normalized_angle(follower_pitch);
    else if(follower_.lost_count > 0) // 復帰後
        follower_pitch = calc_normalized_angle(follower_pitch + getPitch(follower_pose_on_return_));
    else // ロスト前
        follower_pitch = calc_normalized_angle(follower_pitch);

    // pitchからquaternionを算出
    tf::Quaternion follower_quat;
    follower_quat.setRPY(0, follower_pitch, 0);
    tf::quaternionTFToMsg(follower_quat, follower_quat_msg);
}

void CollaborativeVSLAM::calc_follower_pose_after_return()
{
    // positionの決定
    Point follower_pos = calc_pos_after_follower_return(follower_pose_.pose.position);
    follower_pos.output(follower_co_pose_);
    // orientationの決定
    const double follower_pitch = getPitch(follower_pose_on_return_) + getPitch(follower_pose_);
    set_orientation(follower_co_pose_, follower_pitch);
}

Point CollaborativeVSLAM::calc_pos_after_follower_return(const geometry_msgs::Point input_point)
{
    const double map_origin_pitch = getPitch(follower_pose_on_return_);
    const Point  rotated_pos      = rotate_pitch(input_point, map_origin_pitch);

    return Point(follower_pose_on_return_) + rotated_pos;
}
void CollaborativeVSLAM::calc_pos_after_follower_return(pcl::PointXYZ& target_point)
{
    const double map_origin_pitch = getPitch(follower_pose_on_return_);
    const Point  rotated_pos      = rotate_pitch(target_point, map_origin_pitch);
    Point output_point = Point(follower_pose_on_return_) + rotated_pos;

    output_point.output_xz(target_point);
}

// poseのorientationを設定
void CollaborativeVSLAM::set_orientation(geometry_msgs::PoseStamped& pose, const double pitch)
{
    double normalized_pitch = calc_normalized_angle(pitch);

    // pitchからquaternionを算出
    tf::Quaternion quat=tf::createQuaternionFromRPY(0,normalized_pitch,0);
    tf::quaternionTFToMsg(quat, pose.pose.orientation);
}

// 適切な角度(-M_PI ~ M_PI)を返す
double CollaborativeVSLAM::calc_normalized_angle(double angle)
{
    while(M_PI  < angle) angle -= 2.0*M_PI;
    while(angle < -M_PI) angle += 2.0*M_PI;

    return angle;
}

// ===== 協調的mapping =====
void CollaborativeVSLAM::co_mapping()
{
    // leader
    calc_leader_map(); // clac & pub
    // follower
    // calc_follower_map(); // clac & pub
}

// leader mapの算出(+pub)
void CollaborativeVSLAM::calc_leader_map()
{
    if(leader_.flag_lost) // ロスト中
        leader_map_pub_.publish(leader_stored_map_);
    else if(leader_.lost_count > 0) // 復帰後
        calc_leader_map_after_return();
    else // ロスト前
        leader_map_pub_.publish(leader_active_map_);
}

// 復帰後のleader mapの計算
void CollaborativeVSLAM::calc_leader_map_after_return()
{
    // 復帰した地点をベースにmappointsをTF
    tf_leader_active_map();
    // ロスト前のmapと結合
    const int point_size = leader_stored_map_.points.size() + leader_active_map_.points.size(); // 必要dataサイズ
    pcl::PointCloud<pcl::PointXYZ> tmp_map;
    tmp_map.points.reserve(point_size + 10); // dataサイズの確保
    for(const auto& point : leader_stored_map_.points) tmp_map.points.push_back(point);
    for(const auto& point : leader_active_map_.points) tmp_map.points.push_back(point);
    // publish
    tmp_map.header.frame_id = map_frame_id_;
    leader_map_pub_.publish(tmp_map);
}

// 復帰した地点をベースにmappointsをTF
void CollaborativeVSLAM::tf_leader_active_map()
{
    if(leader_.flag_tf_map) return;

    for(auto& point : leader_active_map_.points)
        calc_pos_after_leader_return(point);
    leader_.flag_tf_map = true;
}

// follower mapの算出(+pub)
void CollaborativeVSLAM::calc_follower_map()
{
    if(follower_.flag_lost) // ロスト中
        follower_map_pub_.publish(follower_stored_map_);
    else if(follower_.lost_count > 0) // 復帰後
        calc_follower_map_after_return();
    else // ロスト前
        follower_map_pub_.publish(follower_active_map_);
}

// 復帰後のfollower mapの計算
void CollaborativeVSLAM::calc_follower_map_after_return()
{
    // 復帰した地点をベースにmappointsをTF
    tf_follower_active_map();
    // ロスト前のmapと結合
    const int point_size = follower_stored_map_.points.size() + follower_active_map_.points.size(); // 必要dataサイズ
    pcl::PointCloud<pcl::PointXYZ> tmp_map;
    tmp_map.points.reserve(point_size + 10); // dataサイズの確保
    for(const auto& point : follower_stored_map_.points) tmp_map.points.push_back(point);
    for(const auto& point : follower_active_map_.points) tmp_map.points.push_back(point);
    // publish
    tmp_map.header.frame_id = map_frame_id_;
    follower_map_pub_.publish(tmp_map);
}

// 復帰した地点をベースにmappointsをTF
void CollaborativeVSLAM::tf_follower_active_map()
{
    if(follower_.flag_tf_map) return;

    for(auto& point : follower_active_map_.points)
        calc_pos_after_follower_return(point);
    follower_.flag_tf_map = true;
}
