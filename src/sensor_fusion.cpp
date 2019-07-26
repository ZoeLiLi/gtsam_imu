#include"sensor_fusion.h"
#include <fstream>
#include <time.h>
#include <boost/chrono.hpp>
#include <boost/algorithm/string.hpp>

using namespace TADR;

SensorFusion::SensorFusion( bool fast_replay,std::string triggle_mode, int period)
: exit_(false)
, initialed_(false)
, is_zupt_(false)
, fix_status_(E_FIX_STATUS::E_STATUS_INVALID)
, triggle_mode_(triggle_mode)
, period_(period)
, process_thread_(NULL)
, time_control_thread_(NULL)
{
	sensor_factors_ = boost::shared_ptr<SensorFactors>(new SensorFactors);
	system_alignment_ = boost::shared_ptr<SystemAlignment>(new SystemAlignment());
	dead_reckoning_ = boost::shared_ptr<DeadReckoning>(new DeadReckoning());

	if(!fast_replay)
	{
		process_thread_ = new boost::thread(boost::BOOST_BIND(&SensorFusion::Run,this));
		if(triggle_mode_ == "Period")
		{
			time_control_thread_= new boost::thread(boost::BOOST_BIND(&SensorFusion::TimeControl,this));
		}
	}
}

SensorFusion::~SensorFusion()
{
	exit_ = true;
}
void SensorFusion::TimeControl()
{
	while(!exit_)
	{
		boost::this_thread::sleep_for(boost::chrono::milliseconds(period_));
		condition_.notify_all();
	}

}
bool SensorFusion::Run() 
{
	while (!exit_)
	{
		boost::mutex::scoped_lock lock(mutex_);
		condition_.wait(lock);
		Process();
	}
	return false;
}
void SensorFusion::FastReplayLog()
{
	Process();
}

void SensorFusion::Process()
{
	static int count = 0;
	const clock_t begin_time = clock();
	current_position_info_.fix_status = E_FIX_STATUS::E_STATUS_INVALID;
	if (!initialed_)
	{
		initialed_ = system_alignment_->Alignment(sensor_data_,current_position_info_);

		if(initialed_)
		{
			dead_reckoning_->InsMechanization(sensor_data_.current_imu_data, current_position_info_);
			sensor_factors_->PoseGraphOptimization(current_position_info_);
			std::cout<<"Initialed!"<<std::endl;
			count  = 0;
			former_distance_ = 0.0;
		}
	}
	else
	{
		// predict current pose
		is_zupt_ = ZuptDetection(sensor_data_);
		if(!is_zupt_)
		{
			dead_reckoning_->InsMechanization(sensor_data_.current_imu_data, current_position_info_);
			if(count >= 2)
			{
			//			current_position_info_.time_stamp = sensor_data_.current_imu_data.time_stamp;
				sensor_factors_->PoseGraphOptimization(current_position_info_);
				dead_reckoning_->UpdatePositionInfo(current_position_info_);
				former_distance_ = current_position_info_.distance;
				count = 0;
			}
		}
		else
		{
			current_position_info_.time_stamp = sensor_data_.current_imu_data.time_stamp;
		}
		const clock_t end_time = clock();
		current_position_info_.time_consume = double(end_time - begin_time)/CLOCKS_PER_SEC;

	}
	count ++;
}

void SensorFusion::SetIMUData(ImuData imu_data)
{
	sensor_data_.current_imu_data = imu_data;
	if(sensor_factors_)
	{
		sensor_factors_->SetSensorData(sensor_data_,"IMU_DATA");
	}
	else
	{
		std::cout<<"Fail to set imu data to sensor_factor."<<std::endl;
	}
}
void SensorFusion::SetGnssData(GnssData gnss_data)
{
	sensor_data_.current_gnss_data = gnss_data;
	if(sensor_factors_)
	{
		sensor_factors_->SetSensorData(sensor_data_,"GNSS_DATA");
	}
	else
	{
		std::cout<<"Fail to set gnss data to sensor_factor."<<std::endl;
	}
}
void SensorFusion::SetVehicleData(VehicleData vehicle_data)
{
	sensor_data_.current_vehicle_data = vehicle_data;
	if(sensor_factors_)
	{
		sensor_factors_->SetSensorData(sensor_data_,"VEHICLE_DATA");
	}
	else
	{
		std::cout<<"Fail to set vehicle data to sensor_factor."<<std::endl;
	}
}

PositionInfo SensorFusion::GetLatestPosition()
{
	return current_position_info_;
}
bool SensorFusion::ZuptDetection(SensorData sensor_data)
{
	if(fabs(sensor_data.current_vehicle_data.speed) <= KStaticSpeed)
	{
		return true;
	}
	else
	{
		return false;
	}
}
