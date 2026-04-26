#ifndef MAP_BUILDER_H
#define MAP_BUILDER_H

#include "imu_process/imu_process.h"
#include "lidar_process/lidar_process.h"

enum BuilderStatus
{
    IMU_INIT,
    MAP_INIT,
    MAPPING
};

class MapBuilder
{
public:
    MapBuilder(Config &config, std::shared_ptr<IESKF> kf);

    void process(SyncPackage &package);
    BuilderStatus status() { return m_status; }    
    std::shared_ptr<LidarProcessor> lidar_processor(){return m_lidar_processor;}

private:
    Config config;
    BuilderStatus m_status;
    std::shared_ptr<IESKF> kf;
    std::shared_ptr<IMUProcessor> imu_processor;
    std::shared_ptr<LidarProcessor> m_lidar_processor;
};

#endif // MAP_BUILDER_H