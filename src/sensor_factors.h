#ifndef SENSOR_FACTORS_H
#define SENSOR_FACTORS_H
#include <iostream>
#include <string>
#include "constant.h"


#include <boost/thread/thread.hpp>
#include <boost/thread/condition.hpp>
#include <gtsam/navigation/ImuBias.h>
namespace TADR
{
class SensorFactors
{
public:
	SensorFactors(void);
	virtual ~SensorFactors();

public:
	void SetCurrnetVertexIndex(int value_index);
	int GetCurrentVertexIndex();
	void SetCurrentTimeStamp(unsigned long long time_stamp);
	unsigned long long GetCurrentTimeStamp();
	void UpdateCurrentVertexInfo(gtsam::Pose3 pose,gtsam::Vector3 velocity,gtsam::imuBias::ConstantBias imu_bias);
	void SetLatestVertexInfo();

public:
	std::vector<VertexInfo>			factor_graph_buffer_;
	VertexInfo						current_factor_graph_;

private:
	void ExtrapolateNextVertexInfo();
	void UnionVertexInfo();
private:
	int								period_;
	int								max_buffer_size_;

	int 							current_value_index_;
	unsigned long long				current_time_stamp_;
	gtsam::Point3					current_position_;
	gtsam::Rot3						current_rotation_;
	gtsam::Pose3					current_pose_;
	gtsam::Vector3					current_velocity_;
	gtsam::imuBias::ConstantBias	current_imu_bias_;

private:

};
}
#endif
