#ifndef GNSS_CLASS_H
#define GNSS_CLASS_H
#include <iostream>
#include <string>
#include "constant.h"
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
	GNSSPara();
	virtual ~GNSSPara();
public:
	void SetGNSSData(GnssData gnss_data);
	void SetInitialValue(double lat0,double lon0,double h0);
	GnssData GetGNSSData();
	gtsam::NonlinearFactorGraph GetGnssFactors();
	void Reset();
private:
	void GenerateGNSSFactor();
public:
	bool	first_enter_;
	int		count_gnss_;
	gtsam::Vector3	gnss_pos_;
private:
	bool								initialed_;
	gtsam::Vector6						gnss_sigma_;
	GnssData							gnss_data_;
	gtsam::NonlinearFactorGraph			gnss_factors_;
	gtsam::SharedNoiseModel				gnss_noise_model_;
	boost::thread*						gnss_thread_;
	boost::mutex						mutex_;
	boost::condition					condition_;
	GeographicLib::LocalCartesian		gnss_local_cartesian_;
}
;
}
#endif
