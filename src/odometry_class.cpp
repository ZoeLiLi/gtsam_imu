#include "odometry_class.h"
using namespace TADR;

OdometryPara::OdometryPara()
: initialed_(false)
, speed_(-1.0)
, velocity_sigma_(10,15,10)
, index_(0)
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
	speed_ = vehicle_data_.speed;
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
				gtsam::NonlinearFactor::shared_ptr velocity_factor(new VelocityFactor2(X(value_index),
						V(value_index),velocity,velocity_noise_model_));

				if(odometry_factors_.size()>0)
				{
					odometry_factors_.resize(0);
				}
				odometry_factors_.push_back(velocity_factor);

//				gtsam::NonlinearFactor::shared_ptr velocity_factor(new VelocityFactor1(
//										V(sensor_factors_->current_factor_graph_.value_index),vehicle_data_.speed,velocity_noise_model_));


			}
//		}
}

gtsam::NonlinearFactorGraph OdometryPara::GetOdometryFactors()
{
	return odometry_factors_;
}
void OdometryPara::Reset()
{
	odometry_factors_.resize(0);
}

