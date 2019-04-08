#include<iostream>
#include <fstream>
#include "sensor_fusion.h"
#include <boost/algorithm/string.hpp>

using namespace TADR;
std::vector<ImuData> imu_data_buffer;
std::vector<GnssData> gnss_data_buffer;
std::vector<VehicleData> vehicle_data_buffer;
bool LoadReplayLogData(std::string input_log_path);
bool LoadIMUData(std::string input_log_path);
bool LoadGNSSData(std::string input_log_path);

int main()
{

	boost::shared_ptr<SensorFusion> sensorfusion = boost::shared_ptr<SensorFusion> (new SensorFusion(true,"Event",100));
//	std::string input_file_path("../../data/GPS_IMU_20181126.txt");
//	LoadReplayLogData(input_file_path);
	std::string imu_file_path("../../data/KittiEquivBiasedImu.txt");
	std::string gnss_file_path("../../data/KittiGps_converted.txt");
	LoadIMUData(imu_file_path);
	LoadGNSSData(gnss_file_path);

	int imu_index = 2;
	int count = 0;
	int gnss_index = 1;
	int vehicle_index = 0;
	bool initialed = false;
	unsigned long long last_vertex_time = gnss_data_buffer[0].time_stamp;

	while(imu_index < imu_data_buffer.size())
	{
		sensorfusion->SetIMUData(imu_data_buffer[imu_index]);
//		std::cout<<imu_index<<std::endl;
		if(!initialed)
		{
			if(fabs(imu_data_buffer[imu_index].double_time - gnss_data_buffer[gnss_index].double_time) < 0.5*imu_data_buffer[imu_index].delta_time)
			{
				std::cout<<gnss_data_buffer[gnss_index].time_stamp<<std::endl;
				sensorfusion->SetGnssData(gnss_data_buffer[gnss_index]);
				std::cout<<"GPS Index"<<gnss_index<<" "<<gnss_data_buffer[gnss_index].double_time<<std::endl;
				gnss_index ++;
				sensorfusion->FastReplayLog();
				initialed = true;
				last_vertex_time = imu_data_buffer[imu_index].time_stamp;
				count = 0;
				std::cout<<"IMU index:"<<imu_index <<" "<< imu_data_buffer[imu_index].double_time<<std::endl;
			}
		}
		else
		{

			count ++;
			if(fabs(imu_data_buffer[imu_index].double_time - gnss_data_buffer[gnss_index].double_time) < 0.5*imu_data_buffer[imu_index].delta_time)
			{
				sensorfusion->SetGnssData(gnss_data_buffer[gnss_index]);
				std::cout<<"GPS Index"<<gnss_index<<std::endl;
				gnss_index ++;
			}
			if(count >= 10)
//			if(imu_data_buffer[imu_index].time_stamp > last_vertex_time + 100)
			{

				sensorfusion->FastReplayLog();
				last_vertex_time = imu_data_buffer[imu_index-1].time_index;
				count  = 0;
			}
		}

		imu_index ++;


	}

	return 0;
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

	while(!ifs.eof())
	{
		std::string strline;
		getline(ifs,strline);
		std::vector<std::string> vtemp;
		boost::split(vtemp,strline,boost::is_any_of(","));
		if(vtemp[2] == "GYRODATA")
		{
			imu_data.time_stamp = atoll(vtemp[3].c_str());
			imu_data.gyro_x = atof(vtemp[5].c_str());
			imu_data.gyro_y = atof(vtemp[4].c_str());
			imu_data.gyro_z = -atof(vtemp[6].c_str());
		}
		if(vtemp[2] == "ACCDATA")
		{
			if(atoll(vtemp[3].c_str()) == imu_data.time_stamp)
			{
				imu_data.acc_x = atof(vtemp[5].c_str());
				imu_data.acc_y = atof(vtemp[4].c_str());
				imu_data.acc_z = -atof(vtemp[6].c_str());
				imu_data_buffer.push_back(imu_data);
			}
		}
		if(vtemp[2] == "GPSDATA")
		{
			gnss_data.time_stamp = atoll(vtemp[3].c_str());
			gnss_data.lon = atof(vtemp[4].c_str())*KDeg2Rad;
			gnss_data.lat = atof(vtemp[5].c_str())*KDeg2Rad;
			gnss_data.height = atof(vtemp[6].c_str());
			gnss_data.std_lat = atof(vtemp[10].c_str());
			gnss_data.std_lat = atof(vtemp[10].c_str());
			gnss_data.std_lat = atof(vtemp[11].c_str())*10;
			gnss_data.is_wgs84 = true;
			gnss_data_buffer.push_back(gnss_data);
		}
		if(vtemp[2] == "VEHICLEDATA")
		{
			vehicle_data.time_stamp = atoll(vtemp[3].c_str());
			vehicle_data.speed = atof(vtemp[8].c_str());
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
