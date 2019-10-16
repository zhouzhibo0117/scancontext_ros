//
// Created by localization on 10/15/19.
//

#ifndef SRC_VISUALIZATION_H
#define SRC_VISUALIZATION_H

#include "util/str_util.h"

// ROS include
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <mrpt_bridge/mrpt_bridge.h>

// MRPT include
#include <mrpt/maps/COccupancyGridMap2D.h>

class Visualization {
public:
    Visualization();

private:
    void ResultArrayCallback(const std_msgs::Float64MultiArrayConstPtr &array_msg);

    void LoadMap(int center_x, int center_y,
                 double map_resolution,
                 int map_size);

    void LoadDatabasePoseFile(const std::string &file_name);

    geometry_msgs::Point GetPosition(double timestamp, double presicion);

private:
    ros::Subscriber result_array_subscriber_;
    ros::Publisher candidate_publisher_;
    ros::Publisher map_publisher_;

    int map_center_x_; // eg. 3500;4750;-2350;...
    int map_center_y_;
    int map_size_;          // eg. 3;5;7;...
    double map_resolution_; // eg. 0.05 meter

    std::string grid_map_folder_path_;
    std::string pose_file_path_;

    std::vector<geometry_msgs::PointStamped> pose_file_row_vec_;


};

#endif //SRC_VISUALIZATION_H
