#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H
#include <iostream>
#include <fstream>
#include <string>

#include "sensor_factors.h"
#include "dead_reckoning.h"
#include "alignment.h"
#include "constant.h"

#include <boost/thread/thread.hpp>
#include <boost/thread/condition.hpp>

namespace TADR
{

class SensorFusion
{
public:
	SensorFusion(bool fast_replay, std::string triggle_mode,int period = 0);
	virtual ~SensorFusion();
	
public:
	bool Run();
	void FastReplayLog();
	void SetIMUData(ImuData imu_data);
	void SetGnssData(GnssData gnss_data);
	void SetVehicleData(VehicleData vehicle_data);
	PositionInfo GetLatestPosition();
private:
	void TimeControl();
	void Process();
	void PredictCurrentPose();
public:
	SensorData							sensor_data_;
private:
	ImuData								imu_data_;
	GnssData							gnss_data_;
	VehicleData							vehicle_data_;

	bool								exit_;
	bool								initialed_;
	unsigned int						fix_status_;

	boost::shared_ptr<SensorFactors>	sensor_factors_;
	boost::shared_ptr<SystemAlignment>	system_alignment_;
	boost::shared_ptr<DeadReckoning>	dead_reckoning_;

	std::string							triggle_mode_;
	int									period_;

	boost::thread*						process_thread_;
	boost::thread*						time_control_thread_;
	boost::mutex						mutex_;
	boost::condition					condition_;

	PositionInfo						current_position_info_;
};
}
#endif

