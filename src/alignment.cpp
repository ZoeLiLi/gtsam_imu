#include "alignment.h"

using namespace TADR;

SystemAlignment::SystemAlignment()
: align_from_gnss_(true)
{

}
SystemAlignment::~SystemAlignment()
{
}

bool SystemAlignment::Alignment(SensorData sensor_data,PositionInfo& position_info)
{
	//using GNSS-aided means to align
	if(align_from_gnss_)
	{
		if(sensor_data.current_gnss_data.time_stamp > KMinimalValue)
		{
			position_info.time_stamp = sensor_data.current_gnss_data.time_stamp;
			position_info.lat = sensor_data.current_gnss_data.lat;
			position_info.lon = sensor_data.current_gnss_data.lon;
			position_info.height = sensor_data.current_gnss_data.height;
			position_info.ve = 0.0;
			position_info.vn = -6.5;
			position_info.vu = 0.0;
			position_info.yaw = 180.0*KDeg2Rad;
			position_info.pitch = 0.0;
			position_info.roll = 0.0;
			position_info.initialed = true;
			position_info.gyro_bias = gtsam::Z_3x1;
			position_info.acc_bias = gtsam::Z_3x1;
			position_info.index = 0;
			position_info.fix_status = E_FIX_STATUS::E_STATUS_IMU;
			return true;
		}
		else
			return false;
	}
	else
		return false;
}
