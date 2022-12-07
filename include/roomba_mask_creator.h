#ifndef ROOMBA_MASK_CREATOR_H
#define ROOMBA_MASK_CREATOR_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

// custom msg
#include "darknet_ros_msgs/BoundingBox.h"
#include "darknet_ros_msgs/BoundingBoxes.h"

struct BBoxInfo
{
    int xmin;
    int ymin;
    int xmax;
    int ymax;
};

class RoombaMaskCreator
{
public:
    RoombaMaskCreator();
    void process();

private:
    void img_callback(const sensor_msgs::ImageConstPtr& msg);
    void bbox_callback(const darknet_ros_msgs::BoundingBoxesConstPtr& msg);

    void to_cv_img(const sensor_msgs::Image& ros_img, cv::Mat& cv_img);
    void creat_mask(const sensor_msgs::Image& ros_img);


    int hz_;
    BBoxInfo bbox_info_;

    // bool flag_img_ = false;
    bool flag_bbox_ = false;


    // NodeHandle
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // Subscriber
    ros::Subscriber sub_img_;
    ros::Subscriber sub_bbox_;

    // Publisher
    ros::Publisher pub_mask_img_;
};



#endif
