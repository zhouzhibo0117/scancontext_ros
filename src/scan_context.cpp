//
// Created by zhibo on 9/24/19.
//

#include "scancontext_ros/scan_context.h"

ScanContext::ScanContext(int ring_num,
                         int sector_num,
                         double max_range,
                         PointCloudTypePtr &cloud,
                         int bin_points_min = 5,
                         double filter_leaf_size_x = 0.0,
                         double filter_leaf_size_y = 0.0,
                         double filter_leaf_size_z = 0.0) : ring_num_(ring_num),
                                                            sector_num_(sector_num),
                                                            point_range_max_(max_range),
                                                            bin_points_min_(bin_points_min),
                                                            filter_leaf_size_x_(filter_leaf_size_x),
                                                            filter_leaf_size_y_(filter_leaf_size_y),
                                                            filter_leaf_size_z_(filter_leaf_size_z) {
    ImageInitialization();

    SetScanContextID(cloud);

    PointCloud2Image(cloud);

    Image2RingKey();


}

ScanContext::ScanContext(const cv::Mat &image,
                         const std::string &id) : image_(image),
                                                  ID_(id) {
    GetImageInfo();
    Image2RingKey();
}

void ScanContext::PointCloud2Image(PointCloudTypePtr &cloud) {

    // VoxelGrid filtering.
    DownsamplePointCloud(cloud,
                         filter_leaf_size_x_,
                         filter_leaf_size_y_,
                         filter_leaf_size_z_);

    // Initialization.
    typedef std::vector<double> StdVectorDouble;
    typedef std::vector<PointCloudTypePtr> StdVectorPointCloudTypePtr;
    std::vector<StdVectorDouble> cloud_bin_array_max;
    std::vector<StdVectorPointCloudTypePtr> cloud_bin_array;
    for (int i = 0; i < ring_num_; ++i) {
        std::vector<double> cloud_sector_array_max;
        std::vector<PointCloudTypePtr> cloud_sector_array;
        for (int j = 0; j < sector_num_; ++j) {
            PointCloudTypePtr new_ptr(new PointCloudType());
            cloud_sector_array.push_back(new_ptr);
            cloud_sector_array_max.push_back(0.0);
        }
        cloud_bin_array_max.push_back(cloud_sector_array_max);
        cloud_bin_array.push_back(cloud_sector_array);

    }

    int cloud_size = cloud->points.size();
    for (int i = 0; i < cloud_size; ++i) {
        double point_range = sqrt(cloud->points[i].x * cloud->points[i].x +
                                  cloud->points[i].y * cloud->points[i].y);
        double point_angle = GetTheta(cloud->points[i].x, cloud->points[i].y);

        if (point_range > point_range_max_) continue;

        int ring_id = int(point_range / ring_gap_);
        int sector_id = int(point_angle / sector_gap_);


        try {
            if (ring_id < 0 || ring_id >= ring_num_ || sector_id < 0 || sector_id >= sector_num_) {
                throw "[ERROR] Index Error!";
            } else {
                // Push back a point to a bin.
                cloud_bin_array[ring_id][sector_id]->push_back(cloud->points[i]);
                // Find the maximum height point.
                if (cloud->points[i].z > cloud_bin_array_max[ring_id][sector_id]) {
                    cloud_bin_array_max[ring_id][sector_id] = cloud->points[i].z;
                }
            }
        }
        catch (const char *error_msg) {
            std::cerr << error_msg << std::endl;
        }


    }

    for (int i = 0; i < ring_num_; ++i) {
        for (int j = 0; j < sector_num_; ++j) {

            // Minimum points in a bin.
            if (cloud_bin_array[i][j]->points.size() < bin_points_min_) continue;

            image_.at<uchar>(i, j) = int(cloud_bin_array_max[i][j]);
        }
    }
}

void ScanContext::ImageInitialization() {
    ring_gap_ = point_range_max_ / ring_num_;
    sector_gap_ = 2 * M_PI / sector_num_;

    image_.create(ring_num_, sector_num_, CV_8UC1);
    for (int i = 0; i < ring_num_; ++i) {
        for (int j = 0; j < sector_num_; ++j) {
            image_.at<uchar>(i, j) = 0;
        }
    }
}

void ScanContext::GetImageInfo() {
    ring_num_ = image_.size().height;
    sector_num_ = image_.size().width;
}

void ScanContext::Image2RingKey() {
    for (int i = 0; i < ring_num_; ++i) {
        int non_zero_element_num = 0;
        for (int j = 0; j < sector_num_; ++j) {
            if (image_.at<uchar>(i, j) > 0) {
                non_zero_element_num++;
            }
        }
        ring_key_.push_back(double(non_zero_element_num));
    }

    // Normalization.
    for (int i = 0; i < ring_num_; ++i) {
        ring_key_[i] /= sector_num_;
    }

}

void ScanContext::SetScanContextID(const PointCloudTypePtr &cloud) {
    uint64_t timestamp = cloud->header.stamp;
    uint64_t integer_part = timestamp / 1e6;
    uint64_t decimal_part = (double(timestamp / 1e6) - double(integer_part)) * 1000;

    std::string str_integer = Int2FixedString(integer_part, 10);
    std::string str_decimal = Int2FixedString(decimal_part, 3);

    ID_ = str_integer + "_" + str_decimal;
}

