#include<iostream>
#include <fstream>
#include <time.h>
#include "sensor_fusion.h"
#include <boost/algorithm/string.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/Config.h>

using namespace TADR;
using namespace gtsam;
using namespace std;
using namespace GeographicLib;
std::vector<ImuData> imu_data_buffer;
std::vector<GnssData> gnss_data_buffer;
std::vector<VehicleData> vehicle_data_buffer;
bool LoadReplayLogData(std::string input_log_path);
bool LoadIMUData(std::string input_log_path);
bool LoadGNSSData(std::string input_log_path);
bool PrintResult(PositionInfo result);
unsigned long long time_shift = 1543220000000;
ofstream ofs;
int main()
{

	boost::shared_ptr<SensorFusion> sensorfusion = boost::shared_ptr<SensorFusion> (new SensorFusion(true,"Event",100));
	std::string input_file_path("../../data/GPS_IMU_20181126.txt");
	std::string output_file_path("../../data/20181126result.txt");
	LoadReplayLogData(input_file_path);
	ofs.open(output_file_path.c_str());
	if(!ofs.is_open())
	{
		std::cout<<"Open result file failed!"<<std::endl;
	}
	int imu_index = 0;
	int count = 0;
	int gnss_index = 5;
	int vehicle_index = 0;
	bool initialed = false;
	unsigned long long last_vertex_time = gnss_data_buffer[0].time_stamp;
	PositionInfo results;
	double dt = 0.02;

	clock_t start_time = clock();
	while(imu_index < imu_data_buffer.size())
	{
		sensorfusion->SetIMUData(imu_data_buffer[imu_index]);
		if(imu_data_buffer[imu_index].delta_time > 1)
		{
			dt = 0.02;
		}
		else
		{
			dt = imu_data_buffer[imu_index].delta_time;
		}
		if(imu_data_buffer[imu_index].double_time > vehicle_data_buffer[vehicle_index].double_time)
		{
			sensorfusion->SetVehicleData(vehicle_data_buffer[vehicle_index]);
			vehicle_index ++;
			std::cout<<"Vehicle index:"<<vehicle_index<<" "<<vehicle_data_buffer[vehicle_index].double_time<<std::endl;
		}
		if(fabs(imu_data_buffer[imu_index].double_time - vehicle_data_buffer[vehicle_index].double_time) <= dt)
		{
			sensorfusion->SetVehicleData(vehicle_data_buffer[vehicle_index]);
			if(vehicle_index < vehicle_data_buffer.size())
			{
				vehicle_index ++;
			}


		}

		if(!initialed)
		{

			if(fabs(imu_data_buffer[imu_index].double_time - gnss_data_buffer[gnss_index].double_time) <= 0.5*dt)
			{
				std::cout<<"IMU index:"<<imu_index <<" "<< imu_data_buffer[imu_index].double_time<<std::endl;
				std::cout<<"GPS Index:"<<gnss_index<<" "<<gnss_data_buffer[gnss_index].double_time<<std::endl;
				cout<<0.5*imu_data_buffer[imu_index].delta_time<<endl;
				sensorfusion->SetGnssData(gnss_data_buffer[gnss_index]);
				gnss_index ++;
				std::cout<<"GPS Index:"<<gnss_index<<" "<<gnss_data_buffer[gnss_index].double_time<<std::endl;
				sensorfusion->FastReplayLog();
				results = sensorfusion->GetLatestPosition();
				PrintResult(results);
				initialed = true;
				last_vertex_time = imu_data_buffer[imu_index].time_stamp;
				count = 0;
			}
		}
		else
		{
			count ++;
			if(imu_data_buffer[imu_index].double_time > gnss_data_buffer[gnss_index].double_time && gnss_index < gnss_data_buffer.size())
			{
				sensorfusion->SetGnssData(gnss_data_buffer[gnss_index]);
				if(gnss_index < gnss_data_buffer.size())
				{
					gnss_index ++;
				}
				std::cout<<"GPS Index:"<<gnss_index<<" "<<gnss_data_buffer[gnss_index].double_time<<std::endl;
			}
			if(fabs(imu_data_buffer[imu_index].double_time - gnss_data_buffer[gnss_index].double_time) <= dt)
			{

				sensorfusion->SetGnssData(gnss_data_buffer[gnss_index]);
				if(gnss_index < gnss_data_buffer.size())
				{
					gnss_index ++;
				}
				std::cout<<"GPS Index:"<<gnss_index<<" "<<gnss_data_buffer[gnss_index].double_time<<std::endl;
			}

			if(count >= 2)
			{

				std::cout<<"IMU index:"<<imu_index <<" "<< imu_data_buffer[imu_index].double_time<< " "<<imu_data_buffer[imu_index].delta_time<<std::endl;
				sensorfusion->FastReplayLog();
				results = sensorfusion->GetLatestPosition();
				PrintResult(results);
				last_vertex_time = imu_data_buffer[imu_index-1].time_index;
				count  = 0;
			}
		}

		imu_index ++;
	}
	std::cout<<"Total consume time(s): "<<(clock()-start_time)/CLOCKS_PER_SEC<<std::endl;
	return 0;
}
bool PrintResult(PositionInfo result)
{
	double yaw = 2*KPi - result.yaw;
	if(yaw > 2*KPi)
	{
		yaw = yaw - 2*KPi;
	}
	if(yaw < 0 )
	{
		yaw = yaw + 2*KPi;

	}
	ofs<< result.time_stamp<<","<<result.index<<","<<result.fix_status<<",";
	ofs<<std::setprecision(16)<<result.lat<<","<<result.lon<<",";
	ofs<<std::setprecision(6)<<result.height<<",";
	ofs<<result.vn<<","<<result.ve<<","<<result.vu<<",";
	ofs<<result.roll<<","<<result.pitch<<","<<yaw<<",";
	ofs<<result.gyro_bias(0)/KDPH_2_RPS<<","<<result.gyro_bias(1)/KDPH_2_RPS<<","<<result.gyro_bias(2)/KDPH_2_RPS<<",";
	ofs<<result.acc_bias(0)/KMG_2_MPS2<<","<<result.acc_bias(1)/KMG_2_MPS2<<","<<result.acc_bias(2)/KMG_2_MPS2<<std::endl;
	return true;
}

bool LoadReplayLogData(std::string input_log_path)
{
	std::ifstream ifs(input_log_path.c_str());
	if(!ifs.is_open())
	{
		std::cout<< "Fail to open data file!"<<std::endl;
		return false;
	}
	ImuData imu_data;
	GnssData gnss_data;
	VehicleData vehicle_data;
	double previous_imu_time = 0.0;
	double previous_gnss_time = 0.0;

	while(!ifs.eof())
	{
		std::string strline;
		getline(ifs,strline);
		std::vector<std::string> vtemp;
		boost::split(vtemp,strline,boost::is_any_of(","));
		if(vtemp[2] == "GYRODATA")
		{
			imu_data.time_stamp = atoll(vtemp[3].c_str());
			imu_data.gyro_x = atof(vtemp[4].c_str());
			imu_data.gyro_y = atof(vtemp[5].c_str());
			imu_data.gyro_z = atof(vtemp[6].c_str());
		}
		if(vtemp[2] == "ACCDATA")
		{
			if(atoll(vtemp[3].c_str()) == imu_data.time_stamp)
			{
				imu_data.acc_x = atof(vtemp[4].c_str());
				imu_data.acc_y = atof(vtemp[5].c_str());
				imu_data.acc_z = atof(vtemp[6].c_str());
				imu_data.time_stamp = imu_data.time_stamp - time_shift;
				imu_data.double_time = imu_data.time_stamp*KMilisecond2Sencond;
				if(previous_imu_time > 0.0)
				{
					imu_data.delta_time = imu_data.double_time - previous_imu_time;
					if(imu_data.delta_time > 0.0)
					{
						imu_data_buffer.push_back(imu_data);
					}
				}
				else
				{
					imu_data.delta_time = 0.0;
					imu_data_buffer.push_back(imu_data);
				}
				previous_imu_time = imu_data.double_time;

			}
		}
		if(vtemp[2] == "GPSDATA")
		{
			gnss_data.time_stamp = atoll(vtemp[3].c_str()) - time_shift;
			gnss_data.lon = atof(vtemp[4].c_str());
			gnss_data.lat = atof(vtemp[5].c_str());
			gnss_data.height = atof(vtemp[6].c_str());
			gnss_data.std_lat = atof(vtemp[10].c_str());
			gnss_data.std_lat = atof(vtemp[10].c_str());
			gnss_data.std_lat = atof(vtemp[11].c_str())*10;
			gnss_data.is_wgs84 = true;
			gnss_data.double_time = gnss_data.time_stamp*KMilisecond2Sencond;
			if(previous_gnss_time > 0.0)
			{
				if(gnss_data.double_time > previous_gnss_time)
				{
					gnss_data_buffer.push_back(gnss_data);
				}
			}
			else
			{
				gnss_data_buffer.push_back(gnss_data);
			}
			previous_gnss_time = gnss_data.double_time;
		}
		if(vtemp[2] == "VEHICLEDATA")
		{
			vehicle_data.time_stamp = atoll(vtemp[3].c_str()) - time_shift;
			vehicle_data.speed = atof(vtemp[8].c_str());
			vehicle_data.double_time = vehicle_data.time_stamp*KMilisecond2Sencond;
			vehicle_data_buffer.push_back(vehicle_data);
		}
	}
	std::cout<<"Finish to load input log data."<<std::endl;
	return true;
}
bool LoadIMUData(std::string input_log_path)
{
	std::ifstream ifs(input_log_path.c_str());
	if(!ifs.is_open())
	{
		std::cout<< "Fail to open imu data file!"<<std::endl;
		return false;
	}
	ImuData imu_data;

	while(!ifs.eof())
	{
		std::string strline;
		getline(ifs,strline);
		std::vector<std::string> vtemp;
		boost::split(vtemp,strline,boost::is_any_of(" "));
		if((vtemp[0] != "Time" && vtemp[0] != "") && vtemp.size() == 8)
		{
			imu_data.double_time = atof(vtemp[0].c_str());
			imu_data.time_stamp = (unsigned long long)(imu_data.double_time*1000);
			imu_data.delta_time = atof(vtemp[1].c_str());
			imu_data.acc_x = atof(vtemp[2].c_str());
			imu_data.acc_y = atof(vtemp[3].c_str());
			imu_data.acc_z = atof(vtemp[4].c_str());
			imu_data.gyro_x = atof(vtemp[5].c_str());
			imu_data.gyro_y = atof(vtemp[6].c_str());
			imu_data.gyro_z = atof(vtemp[7].c_str());
			imu_data_buffer.push_back(imu_data);
		}
	}
	std::cout<<"Finish to load imu input log data.size: "<<imu_data_buffer.size()<<std::endl;
	return true;

}
bool LoadGNSSData(std::string input_log_path)
{
	std::ifstream ifs(input_log_path.c_str());
	if(!ifs.is_open())
	{
		std::cout<< "Fail to open gnss data file!"<<std::endl;
		return false;
	}

	GnssData gnss_data;
	while(!ifs.eof())
	{
		std::string strline;
		getline(ifs,strline);
		std::vector<std::string> vtemp;
		boost::split(vtemp,strline,boost::is_any_of(","));
		if(vtemp[0] != "%Time"&&vtemp[0]!="")
		{
			gnss_data.double_time =atof(vtemp[0].c_str());
			gnss_data.time_stamp = (unsigned long long)(gnss_data.double_time*1000);
			gnss_data.x = atof(vtemp[1].c_str());
			gnss_data.y = atof(vtemp[2].c_str());
			gnss_data.z = atof(vtemp[3].c_str());
			gnss_data.is_wgs84 = false;
			gnss_data_buffer.push_back(gnss_data);
		}

	}
	std::cout<<"Finish to load gnss input log data.size: "<<gnss_data_buffer.size()<<std::endl;
	return true;
}
