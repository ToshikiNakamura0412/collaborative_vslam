#ifndef COLLABORATIVE_VSLAM_H
#define COLLABORATIVE_VSLAM_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>

// custom msg
#include <color_detector_msgs/TargetAngle.h>
#include <object_detector_msgs/ObjectPosition.h>
#include <object_detector_msgs/ObjectPositions.h>

class CollaborativeVSLAM
{
public:
    CollaborativeVSLAM();
    void process();

private:
    // ----- Function -----
    // callback function
    // for leader robot
    void leader_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void leader_active_map_callback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void leader_visual_init_sign_callback(const std_msgs::Bool::ConstPtr& msg);
    void leader_lost_sign_callback(const std_msgs::Bool::ConstPtr& msg);
    void leader_map_merge_sign_callback(const std_msgs::Bool::ConstPtr& msg);
    void leader_relative_angle_callback(const color_detector_msgs::TargetAngle::ConstPtr& msg);
    void leader_wheel_odom_callback(const nav_msgs::Odometry::ConstPtr& msg);
    // for leader robot
    void follower_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void follower_visual_init_sign_callback(const std_msgs::Bool::ConstPtr& msg);
    void follower_lost_sign_callback(const std_msgs::Bool::ConstPtr& msg);
    void follower_map_merge_sign_callback(const std_msgs::Bool::ConstPtr& msg);
    void follower_relative_pos_callback(const object_detector_msgs::ObjectPositions::ConstPtr& msg);
    // other function
    void  co_vslam();
    void  co_localize();
    void  co_mapping();
    void  calc_leader_pose();
    void  calc_leader_pos(double& leader_x, double& leader_z);
    void  calc_leader_quat(geometry_msgs::Quaternion& leader_quat_msg);
    void  tf_follow_pose();
    void  broadcast_leader_state();
    void  init_scale_ratio();
    float normalize_angle(float angle);
    float calc_hypot(geometry_msgs::Point last_point, geometry_msgs::Point prev_point);
    float getPitch(geometry_msgs::Quaternion& quat_msg);

    // ----- Variable -----
    // for collaborative system
    // - base
    int   hz_;
    // - init scale ratio
    bool  flag_move_;
    bool  is_initialized_;
    float move_dist_th_;
    float scale_ratio_;
    float scale_ratio_th_percent_;
    float wheel_dist_for_init_;
    float visual_dist_for_init_;
    double duration_init_;
    ros::Time init_begin_;
    // - frame id
    std::string map_frame_id_;

    // for leader robot
    int leader_lost_count_;
    // double leader_map_scale_rate_;
    std_msgs::Bool leader_flag_visual_init_;
    std_msgs::Bool leader_flag_lost_;
    std_msgs::Bool leader_flag_map_merge_;
    // for follower robot
    int follower_lost_count_;
    std_msgs::Bool follower_flag_visual_init_;
    std_msgs::Bool follower_flag_lost_;
    std_msgs::Bool follower_flag_map_merge_;

    // NodeHandle
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // Subscriber
    // from leader robot
    ros::Subscriber leader_pose_sub_;
    ros::Subscriber leader_active_map_sub_;
    ros::Subscriber leader_lost_sign_sub_;
    ros::Subscriber leader_map_merge_sign_sub_;
    ros::Subscriber leader_relative_angle_sub_;
    ros::Subscriber leader_relative_pos_sub_;
    ros::Subscriber leader_wheel_odom_sub_;
    // from follower robot
    ros::Subscriber follower_pose_sub_;
    ros::Subscriber follower_relative_pos_sub_;

    // Publisher
    ros::Publisher leader_pose_pub_;
    ros::Publisher leader_map_pub_;
    ros::Publisher follower_pose_pub_;

    // leader info
    geometry_msgs::PoseStamped            leader_last_pose_;
    geometry_msgs::PoseStamped            leader_prev_pose_;
    sensor_msgs::PointCloud2              leader_active_map_;
    std::vector<sensor_msgs::PointCloud2> leader_all_map_;
    nav_msgs::Odometry                    leader_last_odom_; // 最新のodometry
    nav_msgs::Odometry                    leader_prev_odom_; // 1制御周期前のodometry
    color_detector_msgs::TargetAngle      leader_relative_angle_;
    // follower info
    geometry_msgs::PoseStamped            follower_pose_;
    object_detector_msgs::ObjectPosition  follower_relative_pos_;
    // collaborative leader info
    // sensor_msgs::PointCloud2              leader_co_pose_;
    sensor_msgs::PointCloud2              leader_co_map_;

};

#endif
