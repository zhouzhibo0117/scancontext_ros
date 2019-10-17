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

    result_publisher_ = nh.advertise<std_msgs::Float64MultiArray>("/place_recognition_result/array_info", 1000);

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
                         80,
                         cloud_frame,
                         5,
                         leaf_size_x_,
                         leaf_size_y_,
                         leaf_size_z_);

    scm_.SetTarget(sc_query);

    scm_.Solve(50);

    std::vector<std::vector<double>> result = scm_.GetCandidateInfo();

    // Publish result.
    std_msgs::Float64MultiArray out_msg;
    for (int i = 0; i < result.size(); ++i) {
        out_msg.data.push_back(result[i][0]);
        out_msg.data.push_back(result[i][1]);
    }

    result_publisher_.publish(out_msg);

    // For debugging.
    std::cout << "Total time: " << GetTimeInterval(start_time) * 1000 << " ms" << std::endl;
}
