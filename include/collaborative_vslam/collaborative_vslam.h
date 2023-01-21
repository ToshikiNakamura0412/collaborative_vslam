#ifndef COLLABORATIVE_VSLAM_H
#define COLLABORATIVE_VSLAM_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_broadcaster.h>
#include <tf2/utils.h>
#include <pcl_ros/point_cloud.h>

// custom msg
#include <color_detector_msgs/TargetAngleList.h>
#include <color_detector_msgs/TargetAngle.h>
#include <object_detector_msgs/ObjectPositions.h>
#include <object_detector_msgs/ObjectPosition.h>

// custom type
#include "collaborative_vslam/point.h"


struct SlamState
{
    bool   flag_tf_map      = false;
    bool   flag_init_ratio  = false;
    bool   flag_init_visual = false;
    bool   flag_lost        = false;
    int    lost_count       = 0;
    double scale_ratio      = 0.0;
};

class CollaborativeVSLAM
{
public:
    CollaborativeVSLAM();
    void process();

private:
    // ----- Function -----
    //   for leader robot
    void leader_scale_ratio_callback(const std_msgs::Float64::ConstPtr& msg);
    void leader_init_visual_sign_callback(const std_msgs::Bool::ConstPtr& msg);
    void leader_lost_sign_callback(const std_msgs::Bool::ConstPtr& msg);
    void leader_map_merge_sign_callback(const std_msgs::Bool::ConstPtr& msg);
    void leader_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void leader_active_map_callback(const sensor_msgs::PointCloud2ConstPtr& msg);
    void leader_relative_angle_callback(const color_detector_msgs::TargetAngleList::ConstPtr& msg);

    //   for leader robot
    void follower_scale_ratio_callback(const std_msgs::Float64::ConstPtr& msg);
    void follower_init_visual_sign_callback(const std_msgs::Bool::ConstPtr& msg);
    void follower_lost_sign_callback(const std_msgs::Bool::ConstPtr& msg);
    void follower_map_merge_sign_callback(const std_msgs::Bool::ConstPtr& msg);
    void follower_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void follower_active_map_callback(const sensor_msgs::PointCloud2ConstPtr& msg);
    void follower_relative_pos_callback(const object_detector_msgs::ObjectPositions::ConstPtr& msg);


    // - slam function
    //   for collaborative
    void co_vslam();
    void co_localize();
    void co_mapping();
    void set_tf_for_map();
    bool can_set_tf_for_map();

    //   for leader robot
    //   - pose
    void  calc_leader_pose();
    void  calc_follower_pose_in_leader_map();
    void  calc_leader_pose_from_follower_in_leader_map();
    void  calc_leader_pos_from_follower_in_leader_map();
    void  calc_leader_quat_from_follower_in_leader_map();
    void  calc_leader_pose_after_return();
    void  calc_pos_after_leader_return(pcl::PointXYZ& target_point);
    Point calc_pos_after_leader_return(const geometry_msgs::Point input_point);
    //   - map
    void  calc_leader_map();
    void  calc_leader_map_after_return();
    void  tf_leader_active_map();
    void  update_leader_stored_map();

    //   for follower robot
    //   - pose
    void  calc_follower_pose();
    void  calc_leader_pose_in_follower_map();
    void  calc_follower_pose_from_leader_in_follower_map();
    void  calc_follower_pos_from_leader_in_follower_map();
    void  calc_follower_quat_from_leader_in_follower_map(geometry_msgs::Quaternion& follower_quat_msg);
    void  calc_follower_pose_after_return();
    void  calc_pos_after_follower_return(pcl::PointXYZ& target_point);
    Point calc_pos_after_follower_return(const geometry_msgs::Point input_point);
    //   - map
    void  calc_follower_map();
    void  calc_follower_map_after_return();
    void  tf_follower_active_map();
    void  update_follower_stored_map();


    // - other function
    void   set_orientation(geometry_msgs::PoseStamped& pose, const double pitch);
    bool   judge_angle(double angle);
    double getPitch(geometry_msgs::Quaternion& quat_msg);
    double getPitch(geometry_msgs::PoseStamped& pose_msg);
    double calc_hypot(const object_detector_msgs::ObjectPosition obj1, const object_detector_msgs::ObjectPosition obj2);
    double calc_hypot(const object_detector_msgs::ObjectPosition obj);
    double calc_hypot(const geometry_msgs::PoseStamped& pose);
    double calc_normalized_angle(double angle);
    Point  rotate_pitch(const object_detector_msgs::ObjectPosition& input_point, const double pitch);
    Point  rotate_pitch(const geometry_msgs::PoseStamped& input_pose, const double pitch);
    Point  rotate_pitch(const geometry_msgs::Point& input_point, const double pitch);
    Point  rotate_pitch(const pcl::PointXYZ& input_point, const double pitch);
    Point  adjust_follower_scale_to_leader(const Point point);
    Point  adjust_leader_scale_to_follower(const Point point);
    Point  adjust_scale_to_leader(const Point point);
    Point  adjust_scale_to_follower(const Point point);



    // ----- Variable -----
    // System
    // for collaborative
    int hz_;
    std::string map_frame_id_;
    bool co_flag_tf_map_origin_;
    bool co_flag_lost_leader_;
    bool co_flag_lost_follower_;
    geometry_msgs::PointStamped co_map_origin_;

    // for leader robote
    SlamState leader_;

    // for follower robot
    SlamState follower_;


    // Leader robot status
    // single
    geometry_msgs::PoseStamped       leader_pose_;
    geometry_msgs::PoseStamped       leader_pose_from_follower_;
    geometry_msgs::PoseStamped       leader_pose_in_follower_map_;
    pcl::PointCloud<pcl::PointXYZ>   leader_active_map_;
    pcl::PointCloud<pcl::PointXYZ>   leader_stored_map_;
    color_detector_msgs::TargetAngle leader_relative_angle_;
    std::vector<double>              leader_relative_angle_log_;
    // collaborative
    geometry_msgs::PoseStamped leader_co_pose_;
    geometry_msgs::PoseStamped leader_pose_on_return_;


    // Follower robot status
    // single
    geometry_msgs::PoseStamped            follower_pose_;
    geometry_msgs::PoseStamped            follower_pose_from_leader_;
    geometry_msgs::PoseStamped            follower_pose_in_leader_map_;
    pcl::PointCloud<pcl::PointXYZ>        follower_active_map_;
    pcl::PointCloud<pcl::PointXYZ>        follower_stored_map_;
    object_detector_msgs::ObjectPositions follower_relative_pos_set_;
    object_detector_msgs::ObjectPosition  follower_relative_pos_;
    // collaborative
    geometry_msgs::PoseStamped  follower_co_pose_;
    geometry_msgs::PoseStamped  follower_pose_on_return_;



    // ----- Others -----
    // NodeHandle
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // Subscriber
    // from leader robot
    ros::Subscriber leader_scale_ratio_sub_;
    ros::Subscriber leader_init_visual_sign_sub_;
    ros::Subscriber leader_lost_sign_sub_;
    ros::Subscriber leader_map_merge_sign_sub_;
    ros::Subscriber leader_pose_sub_;
    ros::Subscriber leader_active_map_sub_;
    ros::Subscriber leader_relative_angle_sub_;
    // from follower robot
    ros::Subscriber follower_scale_ratio_sub_;
    ros::Subscriber follower_init_visual_sign_sub_;
    ros::Subscriber follower_lost_sign_sub_;
    ros::Subscriber follower_map_merge_sign_sub_;
    ros::Subscriber follower_pose_sub_;
    ros::Subscriber follower_active_map_sub_;
    ros::Subscriber follower_relative_pos_sub_;

    // Publisher
    // for collaborative system
    ros::Publisher co_map_origin_pub_;
    // for leader robot
    ros::Publisher leader_pose_pub_;
    ros::Publisher leader_pose_from_follower_pub_;
    ros::Publisher leader_pose_in_follower_map_pub_;
    ros::Publisher leader_map_pub_;
    ros::Publisher leader_pose_on_return_pub_;
    // for follower robot
    ros::Publisher follower_pose_pub_;
    ros::Publisher follower_pose_from_leader_pub_;
    ros::Publisher follower_pose_in_leader_map_pub_;
    ros::Publisher follower_map_pub_;
};

#endif
