//
// Created by zhibo on 9/24/19.
//

#ifndef SRC_PLACE_RECOGNITION_H
#define SRC_PLACE_RECOGNITION_H

#include "scan_context_matcher.h"

#include "util/time_util.h"

// ROS include
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64MultiArray.h>
#include <pcl_conversions/pcl_conversions.h>

class PlaceRecognition {
public:
    PlaceRecognition();

private:
    void PointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);

private:
    ScanContextMatcher scm_;

    std::string image_folder_path_;
    std::string pointcloud_topic_name_;

    ros::Subscriber pointcloud_subscriber_;

    ros::Publisher result_publisher_;

    // Parameters for VoxelGrid filter.
    double leaf_size_x_;
    double leaf_size_y_;
    double leaf_size_z_;


};


#endif //SRC_PLACE_RECOGNITION_H
