#ifndef ODOMETRY_CLASS_H
#define ODOMETRY_CLASS_H
#include <iostream>
#include <string>
#include "constant.h"
#include "sensor_factors.h"

namespace TADR
{
class OdometryPara
{
public:
	OdometryPara(boost::shared_ptr<SensorFactors> sensor_factor);
	virtual ~OdometryPara();

public:
	void SetVehicleData(VehicleData vehicle_data);
	VehicleData GetVehicleData();
private:
	VehicleData									vehicle_data_;
	int											vertex_index_;
	boost::shared_ptr<SensorFactors>			sensor_factors_;

};
}
#endif
