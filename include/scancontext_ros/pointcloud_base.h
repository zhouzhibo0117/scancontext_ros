//
// Created by zhibo on 9/25/19.
//

#ifndef SRC_POINTCLOUD_BASE_H
#define SRC_POINTCLOUD_BASE_H

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/filters/voxel_grid.h"
#include <pcl/common/transforms.h>

typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> PointCloudType;
typedef PointCloudType::Ptr PointCloudTypePtr;

void DownsamplePointCloud(PointCloudTypePtr &cloud_to_filter,
                          double leaf_size_x,
                          double leaf_size_y,
                          double leaf_size_z);

void TransformPointcloud(PointCloudTypePtr &cloud_to_transform,
                         double pose_x, double pose_y, double pose_z,
                         double pose_rx, double pose_ry, double pose_rz);

#endif //SRC_POINTCLOUD_BASE_H
