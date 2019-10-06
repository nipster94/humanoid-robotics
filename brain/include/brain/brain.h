#ifndef BRAIN_H
#define BRAIN_H

#include "ros/ros.h"
#include "ros/package.h"

class HubertBrain
{
    public:
        HubertBrain();

        enum RobortState{
            Idling,
            Tracking,
            Interrogation,
            DecisionMaking
        };

        void execute(void);

    private:
        ros::NodeHandle nh_;
};


#endif
