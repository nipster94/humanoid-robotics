#include "brain.h"

int main(int argc, char** argv)
{
    ROS_INFO("Starting HUBERT");
    ros::init(argc, argv, "hubert_brain");
    HubertBrain node;
    node.execute();
    return 0;
}
