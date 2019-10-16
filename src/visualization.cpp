//
// Created by localization on 10/15/19.
//

#include "scancontext_ros/visualization.h"

Visualization::Visualization() {
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    pnh.param<std::string>("pose_file_path", pose_file_path_, "/pose_file_path_name");
    pnh.param<std::string>("grid_map_folder_path", grid_map_folder_path_, "/grid_map_folder_path");
    pnh.param<double>("map_resolution", map_resolution_, 0.1);
    pnh.param<int>("map_size", map_size_, 9);
    pnh.param<int>("map_center_x", map_center_x_, 0);
    pnh.param<int>("map_center_y", map_center_y_, 0);

    result_array_subscriber_ = nh.subscribe("/place_recognition_result/array_info", 1000,
                                            &Visualization::ResultArrayCallback, this);

    candidate_publisher_ = nh.advertise<visualization_msgs::MarkerArray>("/place_recognition_result/markers",
                                                                         1);

    map_publisher_ = nh.advertise<nav_msgs::OccupancyGrid>("/grid_map", 1);


    LoadDatabasePoseFile(pose_file_path_);

    LoadMap(map_center_x_, map_center_y_, map_resolution_, map_size_);

}

void Visualization::ResultArrayCallback(const std_msgs::Float64MultiArrayConstPtr &array_msg) {
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;

    for (int i = 0; i < array_msg->data.size() / 2; ++i) {
        geometry_msgs::Point candidate_point = GetPosition(array_msg->data[2 * i], 0.1);
        double prob = 1 - array_msg->data[2 * i + 1];
        marker.id = i;
        marker.pose.position = candidate_point;
        marker.scale.x = marker.scale.y = marker.scale.z = prob * 5;
        marker.scale.z = prob * 10;

        marker_array.markers.push_back(marker);

        std::cout << std::to_string(array_msg->data[2 * i]) << '\t' <<
                  std::to_string(candidate_point.x) << '\t' <<
                  std::to_string(candidate_point.x) << '\n';
    }

    candidate_publisher_.publish(marker_array);


}

void Visualization::LoadMap(int center_x, int center_y,
                            double map_resolution,
                            int map_size) {

    mrpt::maps::COccupancyGridMap2D map;

    double square_size = 50;

    // 读以center为中心的n×n的图
    float x_min = center_x - float(square_size) / 2 * float(map_size);
    float x_max = center_x + float(square_size) / 2 * float(map_size);
    float y_min = center_y - float(square_size) / 2 * float(map_size);
    float y_max = center_y + float(square_size) / 2 * float(map_size);
    int image_size = int(float(square_size) / map_resolution);

    map.clear();
    map.setSize(x_min, x_max, y_min, y_max, map_resolution);

    int image_around_num = 0;

    // Read map.
    for (int i = 0; i < map_size; ++i) {
        //#pragma omp parallel for
        for (int j = 0; j < map_size; ++j) {
            int new_center_x = center_x + square_size * (i - map_size / 2);
            int new_center_y = center_y + square_size * (j - map_size / 2);
            // int to string
            std::string str_x = Int2FixedStringWithSign(new_center_x, 7);
            std::string str_y = Int2FixedStringWithSign(new_center_y, 7);
            std::string file_name = "cloud_" + str_x + "_" + str_y + "_0000000.png";

            std::string file_path = grid_map_folder_path_ + file_name;

            CImage image_in;

            if (!image_in.loadFromFile(file_path, 0)) {
                for (int x = 0; x < image_size; ++x) {
                    //#pragma omp parallel for
                    for (int y = 0; y < image_size; ++y) {
                        int square_index_x = x + i * image_size;
                        int square_index_y = y + j * image_size;
                        map.setCell(square_index_x, square_index_y, 0.99999);
                    }
                }
            } else {
                for (int x = 0; x < image_size; ++x) {
                    //#pragma omp parallel for
                    for (int y = 0; y < image_size; ++y) {
                        int square_index_x = x + i * image_size;
                        int square_index_y = y + j * image_size;
                        map.setCell(square_index_x, square_index_y, image_in.getAsFloat(x, image_size - 1 - y));
                    }
                }
                image_around_num++;
            }
        }
    }

    // Publish map.
    nav_msgs::OccupancyGrid map_msg;
    mrpt_bridge::convert(map, map_msg);
    map_msg.header.frame_id = "map";
    map_publisher_.publish(map_msg);

    std::cout << "[INFO] Map loading finished.\n";

}

void Visualization::LoadDatabasePoseFile(const std::string &file_name) {

    std::ifstream pose_txt_file;
    const char *p = file_name.c_str();
    pose_txt_file.open(p);
    geometry_msgs::PointStamped pose_file_row;
    double timestamp, rx, ry, rz;
    while (pose_txt_file >> timestamp >>
                         pose_file_row.point.x >>
                         pose_file_row.point.y >>
                         pose_file_row.point.z >>
                         rx >>
                         ry >>
                         rz) {
        pose_file_row.point.z = 1;
        pose_file_row.header.stamp = ros::Time().fromSec(timestamp);
        pose_file_row_vec_.push_back(pose_file_row);
    }
    pose_txt_file.close();

    std::cout << "[INFO] Pose file loading finished.\n";
}

geometry_msgs::Point Visualization::GetPosition(double timestamp, double presicion) {
    geometry_msgs::Point result;
    result.x = result.y = 0;
    result.z = -1;

    for (int i = 0; i < pose_file_row_vec_.size(); ++i) {
        if (fabs(pose_file_row_vec_[i].header.stamp.toSec() - timestamp) < presicion) {
            result = pose_file_row_vec_[i].point;
            break;
        }
    }

    return result;
}


