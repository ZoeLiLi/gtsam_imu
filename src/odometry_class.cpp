#include "odometry_class.h"
using namespace TADR;

OdometryPara::OdometryPara(boost::shared_ptr<SensorFactors> sensor_factor)
: sensor_factors_(sensor_factor)
, initialed_(false)
, velocity_sigma_(0.1,1.5,0.1)
{
	velocity_noise_model_ = gtsam::noiseModel::Diagonal::Sigmas(velocity_sigma_);
	//velocity_thread_ = new boost::thread(boost::BOOST_BIND(&OdometryPara::GenerateVelocityFactor,this));
}
OdometryPara::~OdometryPara()
{
}

void OdometryPara::SetVehicleData(VehicleData vehicle_data)
{
	vehicle_data_ = vehicle_data;
	GenerateVelocityFactor();
}
VehicleData OdometryPara::GetVehicleData()
{
	return vehicle_data_;
}
void OdometryPara::UpdateInitialValue()
{
	initialed_ = true;
}
void OdometryPara::GenerateVelocityFactor()
{
	//	while(1)
	//	{
	//		boost::mutex::scoped_lock lock(mutex_);
	//		condition_.wait(lock);
			if(initialed_)
			{

				gtsam::Point3 velocity(0.0,vehicle_data_.speed,0.0);
				if(vehicle_data_.is_still)
				{
					velocity_noise_model_ = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.1,0.1,0.1));
				}
				else
				{
					velocity_noise_model_ = gtsam::noiseModel::Diagonal::Sigmas(velocity_sigma_);
				}

				gtsam::NonlinearFactor::shared_ptr velocity_factor(new VelocityFactor1(
						V(sensor_factors_->current_factor_graph_.value_index),velocity,
						sensor_factors_->current_factor_graph_.pose.rotation(),velocity_noise_model_));

//				gtsam::NonlinearFactor::shared_ptr velocity_factor(new VelocityFactor2(
//										X(sensor_factors_->current_factor_graph_.value_index),
//										V(sensor_factors_->current_factor_graph_.value_index),velocity,
//										velocity_noise_model_));
				sensor_factors_->current_factor_graph_.factors.push_back(velocity_factor);

			}
	//	}
}
