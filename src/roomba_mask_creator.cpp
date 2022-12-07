#include "roomba_mask_creator.h"

RoombaMaskCreator::RoombaMaskCreator():private_nh_("~")
{
    // パラメータの取得
    private_nh_.getParam("hz", hz_);

    // Subscriber
    sub_img_  = nh_.subscribe("/image_raw", 1, &RoombaMaskCreator::img_callback, this);
    sub_bbox_ = nh_.subscribe("/bbox", 1, );

    // Publisher
    pub_mask_img_ = nh_.advertise<sensor_msgs::Image>("/mask_image", 1);
}

void RoombaMaskCreator::process()
{
    ros::Rate loop_rate(hz_);

    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void RoombaMaskCreator::bbox_callback(const darknet_ros_msgs::BoundingBoxesConstPtr& msg)
{
    for(const auto &bbox : msg->bounding_boxes)
    {
        if(bbox.Class == "roomba")
        {
            bbox_info_.xmin = bbox.xmin;
            bbox_info_.ymin = bbox.ymin;
            bbox_info_.xmax = bbox.xmax;
            bbox_info_.ymax = bbox.ymax;
            flag_bbox_ = true;
        }
        else
        {
            flag_bbox_ = false;
        }
    }
}

void RoombaMaskCreator::img_callback(const sensor_msgs::ImageConstPtr& msg)
{
    creat_mask(*msg);
}

void RoombaMaskCreator::creat_mask(const sensor_msgs::Image& ros_img)
{
    if(flag_bbox_)
    {

    }
    else
    {
        pub_mask_img_.publish(ros_img);
    }
}

void RoombaMaskCreator::to_cv_img(const sensor_msgs::Image& ros_img, cv::Mat& cv_img)
{
}
