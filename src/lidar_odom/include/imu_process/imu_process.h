#ifndef IMU_PROCESSOR_H
#define IMU_PROCESSOR_H

#include "ieskf.h"
#include "commons.h"

class IMUProcessor
{
public:
    IMUProcessor(Config &config, std::shared_ptr<IESKF> kf);

    bool initialize(SyncPackage &package);
    void undistort(SyncPackage &package);

private:
    Config config;
    std::shared_ptr<IESKF> kf;
    double last_propagate_end_time;
    Vec<IMUData> imu_cache;
    Vec<Pose> poses_cache;
    V3D last_acc;
    V3D last_gyro;
    M12D Q;
    IMUData last_imu;
};

#endif // IMU_PROCESSOR_H