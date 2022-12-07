#include "roomba_mask_creator.h"

RoombaMaskCreator::RoombaMaskCreator():private_nh_("~")
{
    private_nh_.getParam("hz", hz_;

    sub_img_  = nh_.subscribe("/image", 1, &RoombaMaskCreator::img_callback, this);
    sub_bbox_ = nh_.subscribe("/bbox", 1, );

}
