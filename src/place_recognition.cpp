//
// Created by zhibo on 9/24/19.
//

#include "scancontext_ros/place_recognition.h"

PlaceRecognition::PlaceRecognition() {
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    pnh.param<double>("VoxelGirdFilter_size_x", leaf_size_x_, 0.0);
    pnh.param<double>("VoxelGirdFilter_size_y", leaf_size_y_, 0.0);
    pnh.param<double>("VoxelGirdFilter_size_z", leaf_size_z_, 0.0);

    pnh.param<std::string>("pointcloud_topic_name", pointcloud_topic_name_, "/input_pointcloud_topic_name");
    pnh.param<std::string>("folder_path_to_save", image_folder_path_, "/input_folder_path_name");

    pointcloud_subscriber_ = nh.subscribe(pointcloud_topic_name_, 1, &PlaceRecognition::PointCloudCallback, this);

    scm_.LoadDatabase(image_folder_path_);
}

void PlaceRecognition::PointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {

    clock_t start_time = clock(); // For debugging.

    // ROS message to PCL message.
    PointCloudTypePtr cloud_frame(new PointCloudType());
    pcl::fromROSMsg(*cloud_msg, *cloud_frame);

    // scan context
    ScanContext sc_query(20,
                   60,
                   100,
                   cloud_frame,
                   5,
                   leaf_size_x_,
                   leaf_size_y_,
                   leaf_size_z_);

    scm_.SetTarget(sc_query);

    scm_.GetCandidateID(10);

    // For debugging.
    std::cout << "Total time: " << GetTimeInterval(start_time) * 1000 << " ms" << std::endl;
}
