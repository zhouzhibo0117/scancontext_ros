//
// Created by zhibo on 9/24/19.
//

#ifndef SRC_SCAN_CONTEXT_H
#define SRC_SCAN_CONTEXT_H

#include "pointcloud_base.h"

#include "opencv2/opencv.hpp"

#include <vector>

class ScanContext {
public:
    ScanContext(int ring_num,
                int sector_num,
                double max_range,
                PointCloudTypePtr &cloud,
                int bin_points_min,
                double filter_leaf_size_x,
                double filter_leaf_size_y,
                double filter_leaf_size_z);

    ScanContext(const cv::Mat &image, const std::string &id);

private:
    void PointCloud2Image(PointCloudTypePtr &cloud);

    void Image2RingKey();

    void SetScanContextID(const PointCloudTypePtr &cloud);

    void ImageInitialization();

    void GetImageInfo();


public:
    std::string ID_;
    cv::Mat image_;
    std::vector<int> ring_key_;

private:
    int sector_num_;
    int ring_num_;
    double point_range_max_;
    int bin_points_min_;
    double ring_gap_;
    double sector_gap_;

    double filter_leaf_size_x_;
    double filter_leaf_size_y_;
    double filter_leaf_size_z_;

};


#endif //SRC_SCAN_CONTEXT_H
