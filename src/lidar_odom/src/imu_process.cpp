#include "imu_process/imu_process.h"

IMUProcessor::IMUProcessor(Config &config, std::shared_ptr<IESKF> kf) : config(config), kf(kf)
{
    Q.setIdentity();
    Q.block<3, 3>(0, 0) = M3D::Identity() * config.ng;
    Q.block<3, 3>(3, 3) = M3D::Identity() * config.na;
    Q.block<3, 3>(6, 6) = M3D::Identity() * config.nbg;
    Q.block<3, 3>(9, 9) = M3D::Identity() * config.nba;
    last_acc.setZero();
    last_gyro.setZero();
    imu_cache.clear();
    poses_cache.clear();
}

bool IMUProcessor::initialize(SyncPackage &package)
{
    imu_cache.insert(imu_cache.end(), package.imus.begin(), package.imus.end());
    if (imu_cache.size() < static_cast<size_t>(config.imu_init_num)) return false;

    V3D acc_mean = V3D::Zero();
    V3D gyro_mean = V3D::Zero();
    for (const auto &imu : imu_cache)
    {
        acc_mean += imu.acc;
        gyro_mean += imu.gyro;
    }
    acc_mean /= static_cast<double>(imu_cache.size());
    gyro_mean /= static_cast<double>(imu_cache.size());

    kf->x().r_il = config.r_il;
    kf->x().t_il = config.t_il;
    kf->x().bg = gyro_mean;

    if (config.gravity_align)
    {
        kf->x().r_wi = (Eigen::Quaterniond::FromTwoVectors((-acc_mean).normalized(), V3D(0.0, 0.0, -1.0)).matrix());
        kf->x().initGravityDir(V3D(0, 0, -1.0));
    }
    else
        kf->x().initGravityDir(-acc_mean);

    kf->P().setIdentity();
    kf->P().block<3, 3>(6, 6) = M3D::Identity() * 0.00001;
    kf->P().block<3, 3>(9, 9) = M3D::Identity() * 0.00001;
    kf->P().block<3, 3>(15, 15) = M3D::Identity() * 0.0001;
    kf->P().block<3, 3>(18, 18) = M3D::Identity() * 0.0001;

    last_imu = imu_cache.back();
    last_propagate_end_time = package.cloud_end_time;
    return true;
}

void IMUProcessor::undistort(SyncPackage &package)
{
    imu_cache.clear();
    imu_cache.push_back(last_imu);
    imu_cache.insert(imu_cache.end(), package.imus.begin(), package.imus.end());

    // const double imu_time_begin = imu_cache.front().time;
    const double imu_time_end = imu_cache.back().time;
    const double cloud_time_begin = package.cloud_start_time;
    const double propagate_time_end = package.cloud_end_time;

    poses_cache.clear();
    poses_cache.emplace_back(0.0, last_acc, last_gyro, kf->x().v, kf->x().t_wi, kf->x().r_wi);

    V3D acc_val, gyro_val;
    Input inp;
    inp.acc = imu_cache.back().acc;
    inp.gyro = imu_cache.back().gyro;
    
    double dt = 0.0;
    for (auto it_imu = imu_cache.begin(); it_imu < (imu_cache.end() - 1); it_imu++)
    {
        IMUData &head = *it_imu;
        IMUData &tail = *(it_imu + 1);

        if (tail.time < last_propagate_end_time) continue;

        gyro_val = 0.5 * (head.gyro + tail.gyro);
        acc_val = 0.5 * (head.acc + tail.acc);

        if (head.time < last_propagate_end_time)
            dt = tail.time - last_propagate_end_time;
        else
            dt = tail.time - head.time;

        inp.acc = acc_val;
        inp.gyro = gyro_val;
        kf->predict(inp, dt, Q);

        last_gyro = gyro_val - kf->x().bg;
        last_acc = kf->x().r_wi * (acc_val - kf->x().ba) + kf->x().g;
        double offset = tail.time - cloud_time_begin;
        poses_cache.emplace_back(offset, last_acc, last_gyro, kf->x().v, kf->x().t_wi, kf->x().r_wi);
    }

    dt = propagate_time_end - imu_time_end;
    kf->predict(inp, dt, Q);
    last_imu = imu_cache.back();
    last_propagate_end_time = propagate_time_end;

    M3D cur_r_wi = kf->x().r_wi;
    V3D cur_t_wi = kf->x().t_wi;
    M3D cur_r_il = kf->x().r_il;
    V3D cur_t_il = kf->x().t_il;

    auto it_pcl = package.cloud->points.end() - 1;
    for (auto it_kp = poses_cache.end() - 1; it_kp != poses_cache.begin(); it_kp--)
    {
        auto head = it_kp - 1;
        auto tail = it_kp;

        M3D imu_r_wi = head->rot;
        V3D imu_t_wi = head->trans;
        V3D imu_vel = head->vel;
        V3D imu_acc = tail->acc;
        V3D imu_gyro = tail->gyro;

        for (; it_pcl->curvature / double(1000) > head->offset; it_pcl--)
        {
            dt = it_pcl->curvature / double(1000) - head->offset;
            V3D point(it_pcl->x, it_pcl->y, it_pcl->z);
            M3D point_rot = imu_r_wi * Sophus::SO3d::exp(imu_gyro * dt).matrix();
            V3D point_pos = imu_t_wi + imu_vel * dt + 0.5 * imu_acc * dt * dt;
            V3D p_compensate = cur_r_il.transpose() * (cur_r_wi.transpose() * (point_rot * (cur_r_il * point + cur_t_il) + point_pos - cur_t_wi) - cur_t_il);
            it_pcl->x = p_compensate(0);
            it_pcl->y = p_compensate(1);
            it_pcl->z = p_compensate(2);
            if (it_pcl == package.cloud->points.begin()) break;
        }
    }
}