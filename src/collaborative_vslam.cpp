#include "collaborative.h"

CollaborativeVSLAM::CollaborativeVSLAM():private_nh_("~")
{
    // Parameter
    private_nh_.getParam("hz", hz_);

    // Subscriber
    sub_img_ = nh_.subscribe("/", 1, ,this);
    sub_img_ = nh_.subscribe("/", 1, ,this);
    sub_img_ = nh_.subscribe("/", 1, ,this);

    // Publisher
    pub_ = nh_.advertise<>("/", 1);
}

void CollaborativeVSLAM::process()
{
    ros::Rate loop_rate(hz_);

    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

