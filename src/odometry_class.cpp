#include "odometry_class.h"
using namespace TADR;

OdometryPara::OdometryPara(boost::shared_ptr<SensorFactors> sensor_factor)
:sensor_factors_(sensor_factor)
{
}
OdometryPara::~OdometryPara()
{
}

void OdometryPara::SetVehicleData(VehicleData vehicle_data)
{
	vehicle_data_ = vehicle_data;
}
VehicleData OdometryPara::GetVehicleData()
{
	return vehicle_data_;
}
