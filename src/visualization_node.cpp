//
// Created by localization on 10/15/19.
//

#include "scancontext_ros/visualization.h"

int main(int argc, char** argv)
{
    ros::init(argc,argv,"VisualizationNode");
    Visualization vl;

    ros::MultiThreadedSpinner spinner;
    spinner.spin();

    return 0;
}