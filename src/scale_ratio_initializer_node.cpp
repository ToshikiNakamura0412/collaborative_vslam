#include "collaborative_vslam/scale_ratio_initializer.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "scale_ratio_initializer");
    ScaleRatioInitializer scale_ratio_initializer;
    scale_ratio_initializer.process();

    return 0;
}
