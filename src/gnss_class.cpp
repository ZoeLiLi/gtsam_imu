#include "gnss_class.h"
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/slam/PriorFactor.h>

using namespace TADR;

GNSSPara::GNSSPara(boost::shared_ptr<SensorFactors> sensor_factor)
: initialed_(false)
, first_enter_(false)
, sensor_factors_(sensor_factor)
, gnss_local_cartesian_(0.0,0.0,0.0,GeographicLib::Geocentric::WGS84())

{
	gnss_sigma_<< gtsam::Vector3::Constant(0.0),gtsam::Vector3::Constant(1.0/0.07);
	gnss_noise_model_ = gtsam::noiseModel::Diagonal::Precisions(gnss_sigma_,1);
//	gnss_thread_ = new boost::thread(boost::BOOST_BIND(&GNSSPara::GenerateGNSSFactor,this));

}

GNSSPara::~GNSSPara()
{
}

void GNSSPara::SetGNSSData(GnssData gnss_data)
{
	gnss_data_ = gnss_data;
	GenerateGNSSFactor();
//	condition_.notify_all();

}

GnssData GNSSPara::GetGNSSData()
{
	return gnss_data_;
}

void GNSSPara::SetInitialValue(double lat, double lon, double h)
{
	gnss_local_cartesian_.Reset(lat,lon,h);
	initialed_ = true;

}
void GNSSPara::GenerateGNSSFactor()
{
//	while(1)
//	{
//		boost::mutex::scoped_lock lock(mutex_);
//		condition_.wait(lock);
		if(initialed_)
		{
			if(gnss_data_.is_wgs84)
			{
				gnss_local_cartesian_.Forward(gnss_data_.lat,gnss_data_.lon,gnss_data_.height,gnss_data_.x,gnss_data_.y,gnss_data_.z);
			}
			gnss_sigma_(3) = gnss_data_.std_lat;
			gnss_sigma_(4) = gnss_data_.std_lon;
			gnss_sigma_(5) = gnss_data_.std_height;

			gtsam::Point3 gnss_position(gnss_data_.x,gnss_data_.y,gnss_data_.z);
			gtsam::Pose3 gnss_pose(gtsam::Pose3(sensor_factors_->current_factor_graph_.pose.rotation(),gnss_position));
			gtsam::NonlinearFactor::shared_ptr gnss_factor(new gtsam::PriorFactor<gtsam::Pose3>(X(sensor_factors_->current_factor_graph_.value_index),gnss_pose,gnss_noise_model_));
			//sensor_factors_->current_factor_graph_.pose.print();

		//	gnss_factor->print();
			sensor_factors_->current_factor_graph_.factors.push_back(gnss_factor);

		}
		else
		{
			if(gnss_data_.is_wgs84)
			{
				gnss_data_.x = 0;
				gnss_data_.y = 0;
				gnss_data_.z = 0;
			}
		}
//	}
}
