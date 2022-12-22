#include "collaborative_vslam/roomba_mask_creator.h"

RoombaMaskCreator::RoombaMaskCreator():private_nh_("~")
{
    // Parameter
    private_nh_.getParam("hz", hz_);

    // setting
    bboxs_info_.reserve(5);

    // Subscriber
    sub_img_  = nh_.subscribe("/image_raw", 1, &RoombaMaskCreator::img_callback, this);
    sub_bbox_ = nh_.subscribe("/bboxes", 1, &RoombaMaskCreator::bbox_callback, this);

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
    flag_bbox_ = false;
    bboxs_info_.clear();
    BBoxInfo tmp_bbox_info;
    for(const auto &bbox : msg->bounding_boxes)
    {
        if(bbox.Class == "roomba")
        {
            tmp_bbox_info.xmin = bbox.xmin;
            tmp_bbox_info.ymin = bbox.ymin;
            tmp_bbox_info.xmax = bbox.xmax;
            tmp_bbox_info.ymax = bbox.ymax;
            bboxs_info_.push_back(tmp_bbox_info);
            flag_bbox_ = true;
        }
    }
}

void RoombaMaskCreator::img_callback(const sensor_msgs::ImageConstPtr& msg)
{
    creat_mask_img(*msg);
}

void RoombaMaskCreator::creat_mask_img(const sensor_msgs::Image& ros_img)
{
    if(flag_bbox_)
    {
        cv::Mat mask_img;
        mask_roomba(ros_img, mask_img);
        sensor_msgs::ImageConstPtr img_msg = cv_bridge::CvImage(ros_img.header, "bgr8", mask_img).toImageMsg();
        pub_mask_img_.publish(img_msg);
    }
    else
    {
        pub_mask_img_.publish(ros_img);
    }
}

void RoombaMaskCreator::mask_roomba(const sensor_msgs::Image& ros_img, cv::Mat& mask_img)
{
    to_cv_img(ros_img, mask_img);

    for(const auto& bbox_info : bboxs_info_)
    {
        const int x_L = bbox_info.xmin;
        const int x_R = bbox_info.xmax;
        const int y_U = bbox_info.ymin;
        const int y_D = bbox_info.ymax;

        cv::Vec3b *ptr = (cv::Vec3b*)mask_img.ptr();
        const int step = mask_img.step / sizeof(cv::Vec3b);
        for(int y = y_U; y < y_D; ++y)
            for(int x = x_L; x < x_R; ++x)
                ptr[y*step + x] = ptr[y*step + x - 1];
    }
}

void RoombaMaskCreator::to_cv_img(const sensor_msgs::Image& ros_img, cv::Mat& output_img)
{
    cv_bridge::CvImageConstPtr cv_image_ptr;
    try
    {
        cv_image_ptr = cv_bridge::toCvCopy(ros_img, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &ex)
    {
        ROS_ERROR_STREAM("cv_bridge exception: " << ex.what());
        return;
    }

    cv::Mat cv_image(cv_image_ptr->image.rows, cv_image_ptr->image.cols, cv_image_ptr->image.type());
    cv_image = cv_image_ptr->image;
    output_img = std::move(cv_image);
    return;
}
