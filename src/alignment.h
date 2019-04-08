#ifndef ALIGNMENT_H
#define ALIGNMENT_H
#include <iostream>
#include <string>
#include "constant.h"
#include "sensor_factors.h"

namespace TADR
{
class SystemAlignment
{
public:
	SystemAlignment(boost::shared_ptr<SensorFactors> sensor_factors,InitSigmaState init_sigma);
	virtual ~SystemAlignment();
	
public:
	bool Alignment(ImuData imu_data,GnssData gnss_data,VehicleData vehicle_data);

private:
	InitSigmaState						init_sigma_;

	boost::shared_ptr<SensorFactors>	sensor_factors_;

	bool								align_from_gnss_;

};
}
#endif
