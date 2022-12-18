#ifndef COLLABORATIVE_VSLAM_H
#define COLLABORATIVE_VSLAM_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
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
    void leader_scale_ratio_callback(const std_msgs::Float64::ConstPtr& msg);
    void leader_active_map_callback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void leader_init_visual_sign_callback(const std_msgs::Bool::ConstPtr& msg);
    void leader_init_ratio_sign_callback(const std_msgs::Bool::ConstPtr& msg);
    void leader_lost_sign_callback(const std_msgs::Bool::ConstPtr& msg);
    void leader_map_merge_sign_callback(const std_msgs::Bool::ConstPtr& msg);
    void leader_relative_angle_callback(const color_detector_msgs::TargetAngle::ConstPtr& msg);
    // for leader robot
    void follower_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void follower_scale_ratio_callback(const std_msgs::Float64::ConstPtr& msg);
    void follower_init_visual_sign_callback(const std_msgs::Bool::ConstPtr& msg);
    void follower_init_ratio_sign_callback(const std_msgs::Bool::ConstPtr& msg);
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
    bool  can_set_tf_for_map();
    float normalize_angle(float angle);
    float getPitch(geometry_msgs::Quaternion& quat_msg);

    // ----- Variable -----
    // for collaborative system
    int hz_;
    std::string map_frame_id_;

    // for leader robot
    int    leader_lost_count_;
    double leader_scale_ratio_;
    std_msgs::Bool leader_flag_init_visual_;
    std_msgs::Bool leader_flag_init_ratio_;
    std_msgs::Bool leader_flag_lost_;
    std_msgs::Bool leader_flag_map_merge_;
    // for follower robot
    bool   follower_flag_get_tf_;
    int    follower_lost_count_;
    double follower_scale_ratio_;
    std_msgs::Bool follower_flag_init_visual_;
    std_msgs::Bool follower_flag_init_ratio_;
    std_msgs::Bool follower_flag_lost_;
    std_msgs::Bool follower_flag_map_merge_;
    geometry_msgs::Point follower_tf_to_leader_;

    // NodeHandle
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // Subscriber
    // from leader robot
    ros::Subscriber leader_pose_sub_;
    ros::Subscriber leader_scale_ratio_sub_;
    ros::Subscriber leader_active_map_sub_;
    ros::Subscriber leader_init_visual_sign_sub_;
    ros::Subscriber leader_init_ratio_sign_sub_;
    ros::Subscriber leader_lost_sign_sub_;
    ros::Subscriber leader_map_merge_sign_sub_;
    ros::Subscriber leader_relative_angle_sub_;
    // from follower robot
    ros::Subscriber follower_pose_sub_;
    ros::Subscriber follower_init_visual_sign_sub_;
    ros::Subscriber follower_init_ratio_sign_sub_;
    ros::Subscriber follower_scale_ratio_sub_;
    ros::Subscriber follower_relative_pos_sub_;

    // Publisher
    ros::Publisher leader_pose_pub_;
    ros::Publisher leader_map_pub_;
    ros::Publisher follower_pose_pub_;

    // leader info
    geometry_msgs::PoseStamped            leader_pose_;
    sensor_msgs::PointCloud2              leader_active_map_;
    std::vector<sensor_msgs::PointCloud2> leader_all_map_;
    color_detector_msgs::TargetAngle      leader_relative_angle_;
    // follower info
    geometry_msgs::PoseStamped            follower_pose_;
    object_detector_msgs::ObjectPosition  follower_relative_pos_;
    // collaborative leader info
    // sensor_msgs::PointCloud2              leader_co_pose_;
    sensor_msgs::PointCloud2              leader_co_map_;
};

#endif