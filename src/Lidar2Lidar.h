#ifndef LIDAR2LIDAR_H
#define LIDAR2LIDAR_H

#include "Tools.h"
#include <Eigen/Core>
#include <vector>

class Lidar2Lidar
{

public:

    double cali_scale_degree_ = 0.3;
    double cali_scale_trans_ = 0.06;
    Eigen::Matrix4d calib_matrix_;
    Eigen::Matrix4d orign_calib_matrix_;
    std::vector<Eigen::Matrix4d> modification_list_;    // 所有变化的矩阵

    Lidar2Lidar();
    ~Lidar2Lidar();

    Eigen::Matrix4d getCalibMatrix();
    Eigen::Matrix4d getOrignCalibMatrix();

    void setCalibMatrix(Eigen::Matrix4d json_param);

    void calibScaleChange();

};

#endif // LIDAR2LIDAR_H