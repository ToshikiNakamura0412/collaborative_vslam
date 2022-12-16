#include "collaborative_vslam/roomba_mask_creator.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "roomba_mask_creator");
    RoombaMaskCreator mask_creator;
    mask_creator.process();

    return 0;
}
