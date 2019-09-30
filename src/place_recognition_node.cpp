//
// Created by zhou on 9/29/19.
//

#include "scancontext_ros/place_recognition.h"

int main(int argc, char** argv)
{
    ros::init(argc,argv,"PlaceRecognitionNode");
    PlaceRecognition pr;

    ros::MultiThreadedSpinner spinner;
    spinner.spin();

    return 0;
}