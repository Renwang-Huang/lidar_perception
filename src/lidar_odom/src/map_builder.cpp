#include "map_builder/map_builder.h"

MapBuilder::MapBuilder(Config &config, std::shared_ptr<IESKF> kf) : config(config), kf(kf)
{
    imu_processor = std::make_shared<IMUProcessor>(config, kf);
    m_lidar_processor = std::make_shared<LidarProcessor>(config, kf);
    m_status = BuilderStatus::IMU_INIT;
}

void MapBuilder::process(SyncPackage &package)
{
    if (m_status == BuilderStatus::IMU_INIT)
    {
        if (imu_processor->initialize(package))
            m_status = BuilderStatus::MAP_INIT;
        return;
    }

    imu_processor->undistort(package);

    if (m_status == BuilderStatus::MAP_INIT)
    {
        CloudType::Ptr cloud_world = LidarProcessor::transformCloud(package.cloud, m_lidar_processor->r_wl(), m_lidar_processor->t_wl());
        m_lidar_processor->initCloudMap(cloud_world->points);
        m_status = BuilderStatus::MAPPING;
        return;
    }
    
    m_lidar_processor->process(package);
}