#include "brain.h"

HubertBrain::HubertBrain():
    nh_("~")
{

}

void HubertBrain::execute(){
    ROS_INFO("STARTING HUBERT BRAIN!!!");
    ros::Rate rate(10);

    while(ros::ok()){
        rate.sleep();
        ros::spinOnce();
    }
}



