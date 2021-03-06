//
// Created by zhibo on 9/24/19.
//

#include "scancontext_ros/database_building.h"

#include <ctime> // For debugging.

DatabaseBuilding::DatabaseBuilding() {
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    pnh.param<double>("VoxelGirdFilter_size_x", leaf_size_x_, 0.0);
    pnh.param<double>("VoxelGirdFilter_size_y", leaf_size_y_, 0.0);
    pnh.param<double>("VoxelGirdFilter_size_z", leaf_size_z_, 0.0);

    pnh.param<std::string>("pointcloud_topic_name", pointcloud_topic_name_, "/input_pointcloud_topic_name");
    pnh.param<std::string>("folder_path_to_save", image_folder_path_, "/input_folder_path_name");

    pointcloud_subscriber_ = nh.subscribe(pointcloud_topic_name_, 1, &DatabaseBuilding::PointCloudCallback, this);
}

void DatabaseBuilding::PointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {

    clock_t start_time = clock(); // For debugging.

    // ROS message to PCL message.
    // Transform sensor_msgs::PointCloud2 to PCL class.
    PointCloudTypePtr cloud_frame(new PointCloudType());
    pcl::fromROSMsg(*cloud_msg, *cloud_frame);
    cloud_frame->header.stamp = cloud_msg->header.stamp.toSec() * 1e6;

    // scan context
    ScanContext sc(20,
                   60,
                   80,
                   cloud_frame,
                   5,
                   leaf_size_x_,
                   leaf_size_y_,
                   leaf_size_z_);

    // Save image
    SaveScanContextImageFile(sc, image_folder_path_);

    // For debugging.
    std::cout << "Runtime per frame: " << GetTimeInterval(start_time) * 1000 << " ms" << std::endl;

}

void DatabaseBuilding::SaveScanContextImageFile(ScanContext &sc, std::string &folder_path) {
    std::string full_path = folder_path + sc.ID_ + ".xml";
    cv::FileStorage fs(full_path, cv::FileStorage::WRITE);
    fs << "scan_context" << sc.image_;
    fs.release();
}


