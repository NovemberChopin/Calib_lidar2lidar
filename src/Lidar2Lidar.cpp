
#include "Lidar2Lidar.h"

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>

Lidar2Lidar::Lidar2Lidar()
{
    calib_matrix_ = Eigen::Matrix4d::Identity();
    orign_calib_matrix_ = Eigen::Matrix4d::Identity();

    // 初始化 modification_list_
    calibScaleChange();
}

Lidar2Lidar::~Lidar2Lidar() {}

Eigen::Matrix4d Lidar2Lidar::getCalibMatrix() {
    return calib_matrix_;
}

Eigen::Matrix4d Lidar2Lidar::getOrignCalibMatrix() {
    return orign_calib_matrix_;
}


void Lidar2Lidar::setCalibMatrix(Eigen::Matrix4d json_param) {
    calib_matrix_ = json_param;
    orign_calib_matrix_ = json_param;
}


void Lidar2Lidar::calibScaleChange() {
    for (int32_t i = 0; i < 12; i++) {
        std::vector<int> transform_flag(6, 0);
        transform_flag[i / 2] = (i % 2) ? (-1) : 1;
        Eigen::Matrix4d tmp = Eigen::Matrix4d::Identity();
        Eigen::Matrix3d rot_tmp;
        rot_tmp =
            Eigen::AngleAxisd(transform_flag[0] * cali_scale_degree_ / 180.0 * M_PI,
                                Eigen::Vector3d::UnitX()) *
            Eigen::AngleAxisd(transform_flag[1] * cali_scale_degree_ / 180.0 * M_PI,
                                Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(transform_flag[2] * cali_scale_degree_ / 180.0 * M_PI,
                                Eigen::Vector3d::UnitZ());
        tmp.block(0, 0, 3, 3) = rot_tmp;
        tmp(0, 3) = transform_flag[3] * cali_scale_trans_;
        tmp(1, 3) = transform_flag[4] * cali_scale_trans_;
        tmp(2, 3) = transform_flag[5] * cali_scale_trans_;
        modification_list_.push_back(tmp);
    }
    std::cout << "=>Calibration scale update done!\n";
}
