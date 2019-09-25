//
// Created by zhibo on 9/24/19.
//

#include "scancontext_ros/database_building.h"

int main(int argc, char** argv)
{
    ros::init(argc,argv,"DatabaseBuildingNode");
    DatabaseBuilding db;

    ros::MultiThreadedSpinner spinner;
    spinner.spin();

    return 0;
}