#include "sensor_factors.h"
using namespace TADR;

SensorFactors::SensorFactors(void)
: current_value_index_(0)
, current_time_stamp_(0)
, current_pose_(gtsam::Pose3::identity())
, current_velocity_(gtsam::Vector3::Zero())
, current_imu_bias_(gtsam::imuBias::ConstantBias())
, max_buffer_size_(1000)
{
	UnionVertexInfo();
}
SensorFactors::~SensorFactors()
{
}

void SensorFactors::SetCurrnetVertexIndex(int value_index)
{
	current_factor_graph_.value_index = value_index;
}
int SensorFactors::GetCurrentVertexIndex()
{
	return current_factor_graph_.value_index;
}
void SensorFactors::SetCurrentTimeStamp(unsigned long long time_stamp)
{
	current_factor_graph_.time_stamp = time_stamp;
}

unsigned long long SensorFactors::GetCurrentTimeStamp()
{
	return current_factor_graph_.time_stamp;
}

void SensorFactors::UpdateCurrentVertexInfo(gtsam::Pose3 pose,gtsam::Vector3 velocity,gtsam::imuBias::ConstantBias imu_bias)
{
	//current_factor_graph_ = factor_graph_buffer_.back();
	current_factor_graph_.pose = pose;
	current_factor_graph_.velocity = velocity;
	current_factor_graph_.imu_bias = imu_bias;
	//current_factor_graph_.values.clear();
	current_factor_graph_.values.insert(X(current_factor_graph_.value_index),pose);
	current_factor_graph_.values.insert(V(current_factor_graph_.value_index),velocity);
	current_factor_graph_.values.insert(B(current_factor_graph_.value_index),imu_bias);

	//factor_graph_buffer_.pop_back();
	//factor_graph_buffer_.push_back(current_factor_graph_);
	ExtrapolateNextVertexInfo();
}
void SensorFactors::ExtrapolateNextVertexInfo()
{

	//current_factor_graph_.time_stamp += period_;
	current_factor_graph_.value_index ++;
	// extrapolate position
    //current_position_= current_position_ + current_factor_graph_.velocity*period_*KMilisecond2Sencond;
	current_position_ = current_factor_graph_.pose.translation();
    current_factor_graph_.pose = gtsam::Pose3(current_factor_graph_.pose.rotation(),current_position_);

	current_factor_graph_.values.clear();
	current_factor_graph_.values.insert(X(current_factor_graph_.value_index),current_factor_graph_.pose);
	current_factor_graph_.values.insert(V(current_factor_graph_.value_index),current_factor_graph_.velocity);
	current_factor_graph_.values.insert(B(current_factor_graph_.value_index),current_factor_graph_.imu_bias);

	//current_factor_graph_.factors.resize(0);

}
void SensorFactors::SetLatestVertexInfo()
{
	// add mutex
//	current_factor_graph_.values.print();
//	current_factor_graph_.factors.print();
	factor_graph_buffer_.push_back(current_factor_graph_);
//	current_factor_graph_.factors.resize(0);
	if(factor_graph_buffer_.size() > max_buffer_size_)
	{
		factor_graph_buffer_.erase(factor_graph_buffer_.begin());
	}

}
void SensorFactors::UnionVertexInfo()
{
	current_factor_graph_.time_stamp = current_time_stamp_;
	current_factor_graph_.pose = current_pose_;
	current_factor_graph_.velocity = current_velocity_;
	current_factor_graph_.imu_bias = current_imu_bias_;

}
