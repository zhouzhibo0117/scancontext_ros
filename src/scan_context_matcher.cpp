//
// Created by zhou on 9/26/19.
//

#include "scancontext_ros/scan_context_matcher.h"

void ScanContextMatcher::GetCandidatesWithDatabase() {

    vec_scan_context_all_.clear();
    vec_scan_context_candidate_.clear();

    for (int i = 0; i < vec_sc_database_.size(); ++i) {

        double ring_key_dist = CalcRingKeyDistance(sc_target_, vec_sc_database_[i]);

        ScanContextDistance scd(vec_sc_database_[i].ID_, ring_key_dist);
        vec_scan_context_all_.push_back(scd);
    }

    // Sorting.
    std::sort(vec_scan_context_all_.begin(),
              vec_scan_context_all_.end(),
              CompareRingKeyDistanceFunction);

    vec_scan_context_candidate_.assign(vec_scan_context_all_.begin(),
                                       vec_scan_context_all_.begin() + candidate_num_);
}

void ScanContextMatcher::UpdateScanContextDistance() {
    for (int i = 0; i < candidate_num_; ++i) {
        std::string img_filename = vec_scan_context_candidate_[i].id + ".xml";
        std::string file_path = database_folder_path_ + img_filename;
        cv::FileStorage fs(file_path, cv::FileStorage::READ);
        cv::Mat img_in;
        fs["scan_context"] >> img_in;

        ScanContext sc_query(img_in, vec_scan_context_candidate_[i].id);

        double image_dist = CalcImageDistance(sc_target_, sc_query);

        vec_scan_context_candidate_[i].image_dist = image_dist;
    }

    // Sorting.
    std::sort(vec_scan_context_candidate_.begin(),
              vec_scan_context_candidate_.end(),
              CompareImageDistanceFunction);

    // Threshing.vec_scan_context_candidate_
//    std::vector<ScanContextDistance>::iterator iter = vec_scan_context_candidate_.begin();
//    while (iter->image_dist < 0.2 &&
//           iter < vec_scan_context_candidate_.end()) {
//        iter++;
//    }
//    vec_scan_context_candidate_.resize(iter - vec_scan_context_candidate_.begin());

}

void ScanContextMatcher::Solve(int candidate_num) {

    int vec_img_file_size = vec_sc_database_.size();
    candidate_num_ = candidate_num < vec_img_file_size ? candidate_num : vec_img_file_size;

    GetCandidatesWithDatabase();

    UpdateScanContextDistance();
}

void ScanContextMatcher::LoadDatabase(const std::string &database_folder_path) {
    database_folder_path_ = database_folder_path;

    std::vector<std::string> vec_db_filename = GetFilesInFolder(database_folder_path_);

    int vec_db_file_size = vec_db_filename.size();

    std::cout << "[INFO] Loading " << vec_db_file_size << " images in " << database_folder_path_ << ".\n";
    std::cout << "[INFO] Processing... " << "\n";

    for (int i = 0; i < vec_db_file_size; ++i) {
        std::string file_path = database_folder_path_ + vec_db_filename[i];
        std::string img_id(SplitString(vec_db_filename[i], ".")[0] + "."
                           + SplitString(vec_db_filename[i], ".")[1]);
        cv::FileStorage fs(file_path, cv::FileStorage::READ);
        cv::Mat img_in;
        fs["scan_context"] >> img_in;
        ScanContext sc_query(img_in, img_id);

        vec_sc_database_.push_back(sc_query);
    }
    std::cout << "[INFO] Database loading finished." << '\n';
}

void ScanContextMatcher::SetTarget(const ScanContext &target) {
    sc_target_ = target;
}

double ScanContextMatcher::CalcRingKeyDistance(const ScanContext &target,
                                               const ScanContext &query) {
    std::vector<double> rk1 = target.ring_key_;
    std::vector<double> rk2 = query.ring_key_;

    if (rk1.size() != rk2.size()) {
        std::cout << "[ERROR] Inconsistent Parameters with Database." << '\n';
        std::exit(0);
    } else {
        double err_sum = 0.0;
        for (int i = 0; i < target.ring_key_.size(); ++i) {
            err_sum += (rk1[i] - rk2[i]) * (rk1[i] - rk2[i]);
        }
        return err_sum;
    }
}

double ScanContextMatcher::CalcImageDistance(const ScanContext &target, const ScanContext &query) {
    int ring_num = target.image_.size().height;
    int sector_num = target.image_.size().width;
    std::vector<double> vec_cos_similarity_sum;

    for (int loop_num = 0; loop_num < sector_num; ++loop_num) {
        double cos_similarity_sum = 0.0;
        int col_engaged_num = 0;
        for (int j = 0; j < sector_num; ++j) {
            int jj = (j + loop_num) % sector_num;
            double norm_sum_target = 0.0;
            double norm_sum_query = 0.0;
            double dot_sum = 0.0;
            for (int i = 0; i < ring_num; ++i) {
                dot_sum += target.image_.at<float>(i, j) *
                           query.image_.at<float>(i, jj);
                norm_sum_target += target.image_.at<float>(i, j) *
                                   target.image_.at<float>(i, j);
                norm_sum_query += query.image_.at<float>(i, jj) *
                                  query.image_.at<float>(i, jj);
            }
            if (dot_sum == 0.0) {
                continue;
            } else {
                col_engaged_num++;
                cos_similarity_sum += dot_sum / sqrt(norm_sum_query * norm_sum_target);
            }
        }
        vec_cos_similarity_sum.push_back(cos_similarity_sum / double(col_engaged_num));
    }

    double vec_cos_similarity_max = (*max_element(vec_cos_similarity_sum.begin(),
                                                  vec_cos_similarity_sum.end()));

    if (vec_cos_similarity_max > 1) vec_cos_similarity_max = 1;
    if (vec_cos_similarity_max < 0) vec_cos_similarity_max = 0;

    return 1 - vec_cos_similarity_max;
}

bool ScanContextMatcher::CompareRingKeyDistanceFunction(const ScanContextDistance &scd1,
                                                        const ScanContextDistance &scd2) {
    return scd1.ring_key_dist < scd2.ring_key_dist;
}

bool ScanContextMatcher::CompareImageDistanceFunction(const ScanContextDistance &scd1,
                                                      const ScanContextDistance &scd2) {
    return scd1.image_dist < scd2.image_dist;
}

std::vector<std::vector<double>> ScanContextMatcher::GetCandidateInfo() {
    std::vector<std::vector<double>> result;
    for (int i = 0; i < vec_scan_context_candidate_.size(); ++i) {
        std::vector<double> temp_info;
        temp_info.push_back(String2Double(vec_scan_context_candidate_[i].id));
        temp_info.push_back(vec_scan_context_candidate_[i].image_dist);

        result.push_back(temp_info);
    }
    return result;
}

ScanContextMatcher::ScanContextDistance::ScanContextDistance(const std::string &id_input,
                                                             double ring_key_dist_input = 0.0,
                                                             double image_dist_input = 0.0) : id(id_input),
                                                                                              ring_key_dist(
                                                                                                      ring_key_dist_input),
                                                                                              image_dist(
                                                                                                      image_dist_input) {}

ScanContextMatcher::ScanContextDistance::ScanContextDistance(const std::string &id_input,
                                                             double ring_key_dist_input) : id(id_input),
                                                                                           ring_key_dist(
                                                                                                   ring_key_dist_input),
                                                                                           image_dist(0.0) {

}

