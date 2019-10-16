//
// Created by zhibo on 9/24/19.
//

#ifndef SRC_DATABASE_BUILDING_H
#define SRC_DATABASE_BUILDING_H

#include "scan_context.h"

#include "util/time_util.h"

// ROS include
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>


class DatabaseBuilding {
public:
    DatabaseBuilding();

private:
    void PointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);

    void SaveScanContextImageFile(ScanContext &sc, std::string &folder_path);

private:
    std::string image_folder_path_;
    std::string pointcloud_topic_name_;

    ros::Subscriber pointcloud_subscriber_;

    // Parameters for VoxelGrid filter.
    double leaf_size_x_;
    double leaf_size_y_;
    double leaf_size_z_;

};


#endif //SRC_DATABASE_BUILDING_H

