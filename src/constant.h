#ifndef CONSTANT_H
#define CONSTANT_H
#include <math.h>
#include <sophus/se3.h>
#include <sophus/so3.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuBias.h>

using Sophus::SE3;
using Sophus::SO3;

using gtsam::symbol_shorthand::X;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::B;

const double KRe = 6378137;
const double Ke2 =  0.00669437999013;
const double Kwe = 7.2921151467e-5;
const double KPi = (4*atan(1.0));
const double KDeg2Rad = (KPi/180);
const double KRad2Deg = (180/KPi);
const double KtShift = 1543221000;
const double KDPH_2_RPS = (KDeg2Rad/3600.0);
const double KMG_2_MPS2 = (1e-3*9.7803267714);
const double KMilisecond2Sencond = 0.001;
const double KMinimalValue = 0.000001;
typedef struct
 {
	 unsigned long long time_stamp;
	 double	double_time;
	 unsigned int time_index;		
	 double delta_time;
	 double acc_x;
	 double acc_y;
	 double acc_z;
	 double gyro_x;
	 double gyro_y;
	 double gyro_z;
 }ImuData;
 
 typedef struct{
	 unsigned long long time_stamp;
	 double double_time;
	 double lat;				//rad in WGS-84
	 double lon;				//rad in WGS-84
	 double height;				//m in WGS-84
	 double x;				// m in cartesian coordinate
	 double y;				// m in cartesian coordinate
	 double z;				// m in cartesian coordinate
	 double std_lat;
	 double std_lon;
	 double std_height;
	 bool	is_wgs84;
 }GnssData;

 typedef struct{
	 unsigned long long time_stamp;
	 double double_time;
	 double speed;
	 double fl_speed;
	 double fr_speed;
	 double rl_speed;
	 double rr_peed;
	 double steer_angle;
	 double gyro_z;
	 double acc_x;
	 double acc_y;
	 unsigned int gear_status;
 }VehicleData;

 typedef struct{
	 unsigned long long time_stamp;
	 int value_index;
	 gtsam::Pose3 pose;
	 gtsam::Vector3 velocity;
	 gtsam::imuBias::ConstantBias imu_bias;
	 gtsam::Values values;
	 gtsam::NonlinearFactorGraph factors;
 }VertexInfo;

 typedef struct{
		gtsam::SharedNoiseModel sigma_pose;
		gtsam::SharedNoiseModel sigma_vel;
		gtsam::SharedNoiseModel sigma_bias;
 }InitSigmaState;

 typedef enum{
	 E_STATUS_INVALID	= 0x00,
	 E_STATUS_IMU		= 0x01,
	 E_STATUS_VEHICLE	= 0x02,
	 E_STATUS_GNSS		= 0x03,
 }E_FIX_STATUS;

 typedef struct{
	 unsigned long long time_stamp;
	 int index;
	 double lat;
	 double lon;
	 double height;
	 double ve;
	 double vn;
	 double vu;
	 double roll;
	 double pitch;
	 double yaw;
	 gtsam::Vector3 gyro_bias;
	 gtsam::Vector3 acc_bias;
	 unsigned int fix_status;
 }PositionInfo;
 #endif
