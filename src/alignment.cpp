#include "alignment.h"
#include <gtsam/slam/PriorFactor.h>

using namespace TADR;

SystemAlignment::SystemAlignment(boost::shared_ptr<SensorFactors> sensor_factors,InitSigmaState init_sigma)
: init_sigma_(init_sigma)
, sensor_factors_(sensor_factors)
, align_from_gnss_(true)
{

}
SystemAlignment::~SystemAlignment()
{
}

bool SystemAlignment::Alignment(ImuData imu_data, GnssData gnss_data,VehicleData vehicle_data)
{
	//using GNSS-aided means to align
	if(align_from_gnss_)
	{
		if(gnss_data.time_stamp > KMinimalValue)
		{
			if(sensor_factors_)
			{
				sensor_factors_->current_factor_graph_.value_index = 0;
				sensor_factors_->current_factor_graph_.time_stamp = imu_data.time_stamp;
				gtsam::Point3 position(gnss_data.x,gnss_data.y,gnss_data.z);
				gtsam::Rot3 R;

				sensor_factors_->current_factor_graph_.pose = gtsam::Pose3(R,position);

				// velocity and bias are all zeros

				sensor_factors_->current_factor_graph_.values.insert(X(0),sensor_factors_->current_factor_graph_.pose);
				sensor_factors_->current_factor_graph_.values.insert(V(0),sensor_factors_->current_factor_graph_.velocity);
				sensor_factors_->current_factor_graph_.values.insert(B(0),sensor_factors_->current_factor_graph_.imu_bias);

				 gtsam::NonlinearFactor::shared_ptr posFactor(new
				    	gtsam::PriorFactor<gtsam::Pose3>(X(0),sensor_factors_->current_factor_graph_.pose,init_sigma_.sigma_pose));

				// bias prior
				gtsam::NonlinearFactor::shared_ptr biasFactor(new
						gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(B(0),sensor_factors_->current_factor_graph_.imu_bias,init_sigma_.sigma_bias));
							//vel prior
				gtsam::NonlinearFactor::shared_ptr velFactor(new
						gtsam::PriorFactor<gtsam::Vector3>(V(0),sensor_factors_->current_factor_graph_.velocity,init_sigma_.sigma_vel));
				sensor_factors_->current_factor_graph_.factors.push_back(posFactor);
				sensor_factors_->current_factor_graph_.factors.push_back(biasFactor);
				sensor_factors_->current_factor_graph_.factors.push_back(velFactor);
				sensor_factors_->SetLatestVertexInfo();
				sensor_factors_->UpdateCurrentVertexInfo(
						sensor_factors_->current_factor_graph_.pose,
						sensor_factors_->current_factor_graph_.velocity,
						sensor_factors_->current_factor_graph_.imu_bias);
				std::cout<<"Success to align!"<<std::endl;
			}
			else
			{
				std::cout<<"Failed to duplicate SensorFactor"<<std::endl;
			}
			return true;
		}
		else
			return false;
	}
}
