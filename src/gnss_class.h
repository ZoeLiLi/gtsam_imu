#ifndef GNSS_CLASS_H
#define GNSS_CLASS_H
#include <iostream>
#include <string>
#include "constant.h"
#include "sensor_factors.h"
#include <gtsam/navigation/GPSFactor.h>
#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/Config.h>

#include <boost/thread/thread.hpp>
#include <boost/thread/condition.hpp>

namespace TADR
{

class GNSSPara
{
public:
	GNSSPara(boost::shared_ptr<SensorFactors> sensor_factor);
	virtual ~GNSSPara();
public:
	void SetGNSSData(GnssData gnss_data);
	void SetInitialValue(double lat0,double lon0,double h0);
	GnssData GetGNSSData();
private:
	void GenerateGNSSFactor();
public:
	int		count_gnss_;
	gtsam::Vector3	gnss_pos_;
private:
	bool								initialed_;
	gtsam::Vector6						gnss_sigma_;
	GnssData							gnss_data_;
	gtsam::NonlinearFactor::shared_ptr	gnss_factor_;
	GeographicLib::LocalCartesian		gnss_local_cartesian_;
	gtsam::SharedNoiseModel				gnss_noise_model_;
	boost::shared_ptr<SensorFactors>	sensor_factors_;
	boost::thread*						gnss_thread_;
	boost::mutex						mutex_;
	boost::condition					condition_;
}
;
}
#endif
