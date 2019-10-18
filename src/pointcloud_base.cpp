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

void TransformPointcloud(PointCloudTypePtr &cloud_to_transform,
                         double pose_x, double pose_y, double pose_z,
                         double pose_rx, double pose_ry, double pose_rz) {
    PointCloudTypePtr cloud_to_be_transformed(cloud_to_transform);
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << pose_x, pose_y, pose_z;
    transform.rotate(Eigen::AngleAxisf(pose_rz, Eigen::Vector3f::UnitZ()) *
                     Eigen::AngleAxisf(pose_ry, Eigen::Vector3f::UnitY()) *
                     Eigen::AngleAxisf(pose_rx, Eigen::Vector3f::UnitX()));

    PointCloudTypePtr transformed_cloud(new PointCloudType());
    pcl::transformPointCloud(*cloud_to_be_transformed, *cloud_to_transform, transform);
}