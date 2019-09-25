//
// Created by zhibo on 9/25/19.
//

#ifndef SRC_POINTCLOUD_BASE_H
#define SRC_POINTCLOUD_BASE_H

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/filters/voxel_grid.h"

typedef pcl::PointXYZI PointType;
typedef pcl::PointCloud<PointType> PointCloudType;
typedef PointCloudType::Ptr PointCloudTypePtr;

void DownsamplePointCloud(PointCloudTypePtr &cloud_to_filter,
                          double leaf_size_x,
                          double leaf_size_y,
                          double leaf_size_z);

#endif //SRC_POINTCLOUD_BASE_H
