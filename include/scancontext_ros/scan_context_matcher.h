//
// Created by zhou on 9/26/19.
//

#ifndef SRC_SCAN_CONTEXT_MATCHER_H
#define SRC_SCAN_CONTEXT_MATCHER_H

#include "scan_context.h"
#include "util/file_io_util.h"

#include <opencv2/opencv.hpp>

class ScanContextMatcher {
public:
    ScanContextMatcher() {};

    void LoadDatabase(const std::string &database_folder_path);

    void SetTarget(const ScanContext &target);

    void Solve(int candidate_num);

    std::vector<std::vector<double>> GetCandidateInfo();

private:
    std::vector<ScanContext> vec_sc_database_;

    ScanContext sc_target_;

    std::string database_folder_path_;

    int candidate_num_;

    class ScanContextDistance {
    public:
        std::string id;
        double ring_key_dist;
        double image_dist;
    public:
        ScanContextDistance() {
            id = "";
            ring_key_dist = 0.0;
            image_dist = 0.0;
        }

        ScanContextDistance(const std::string &id_input,
                            double ring_key_dist_input);

        ScanContextDistance(const std::string &id_input,
                            double ring_key_dist_input,
                            double image_dist_input);
    };

    std::vector<ScanContextDistance> vec_scan_context_all_;

    std::vector<ScanContextDistance> vec_scan_context_candidate_;

private:

    void GetCandidatesWithDatabase();

    void UpdateScanContextDistance();

    static double CalcRingKeyDistance(const ScanContext &target,
                                      const ScanContext &query);

    static double CalcImageDistance(const ScanContext &target,
                                    const ScanContext &query);

    static bool CompareRingKeyDistanceFunction(const ScanContextDistance &scd1,
                                               const ScanContextDistance &scd2);

    static bool CompareImageDistanceFunction(const ScanContextDistance &scd1,
                                             const ScanContextDistance &scd2);

};

#endif //SRC_SCAN_CONTEXT_MATCHER_H
