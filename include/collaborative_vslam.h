#ifndef COLLABORATIVE_VSLAM_H
#define COLLABORATIVE_VSLAM_H

#include <ros/ros.h>

class CollaborativeVSLAM
{
public:
    CollaborativeVSLAM();
    void process();
private:
    void mappoints_callback(const sensor_msgs::PointCloud2::ConstPtr& msg);
};

#endif
