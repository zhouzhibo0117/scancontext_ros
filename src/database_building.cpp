//
// Created by zhibo on 9/24/19.
//

#include "scancontext_ros/database_building.h"

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

    // TODO: ROS message to PCL message.
    // Transform sensor_msgs::PointCloud2 to PCL class.
    PointCloudTypePtr cloud_frame(new PointCloudType());
    pcl::fromROSMsg(*cloud_msg, *cloud_frame);
    cloud_frame->header.stamp = cloud_msg->header.stamp.toSec() * 1e6;

    // TODO: scan context
    ScanContext sc(20,
                   60,
                   100,
                   cloud_frame,
                   5,
                   leaf_size_x_,
                   leaf_size_y_,
                   leaf_size_z_);

    // TODO: save image
    SaveScanContextImageFile(sc, image_folder_path_);


}

void DatabaseBuilding::SaveScanContextImageFile(ScanContext &sc, std::string &folder_path) {
    std::string full_path = folder_path + sc.ID_ + ".png";
    cv::imwrite(full_path, sc.image_);
}


