#include "collaborative_vslam/collaborative_vslam.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "collaborative_vslam");
    CollaborativeVSLAM collaborative_vslam;
    collaborative_vslam.process();

    return 0;
}
