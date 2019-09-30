//
// Created by zhou on 9/30/19.
//

#include "scancontext_ros/pointcloud_base.h"

void DownsamplePointCloud(PointCloudTypePtr &cloud_to_filter,
                          double leaf_size_x,
                          double leaf_size_y,
                          double leaf_size_z) {
    pcl::VoxelGrid<PointType> sor;
    PointCloudTypePtr cloud_filtered(new PointCloudType());
    sor.setInputCloud(cloud_to_filter);
    sor.setLeafSize(leaf_size_x, leaf_size_y, leaf_size_z);
    sor.filter(*cloud_filtered);

    cloud_to_filter = cloud_filtered;
}