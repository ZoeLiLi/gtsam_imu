#include"dead_reckoning.h"
#include <fstream>

using namespace TADR;


DeadReckoning::DeadReckoning()
: Re_(6378137.0)
, e2_(0.00669437999013)
, we_(7.2921151467e-5)
, g0_(9.7803267714)
, beta_(5.27094e-3)
, beta1_(2.32718e-5)
, beta2_(3.086e-6)
, gn_(gtsam::Z_3x1)
{


}

DeadReckoning::~DeadReckoning()
{
}
void DeadReckoning::InsMechanization(ImuData imu_data,PositionInfo& position_info)
{
	static gtsam::Matrix33 matrix_cbn;
	static gtsam::Rot3	rot_cbn;
	static gtsam::Vector3 pre_vel;
	static gtsam::Quaternion pre_qbn;
	if(!initialed_ && position_info.initialed)
	{
		position_info.time_stamp = imu_data.time_stamp;
		pre_position_info_ = position_info;
		pre_imu_data_ = imu_data;
		rot_cbn = gtsam::Rot3::Ypr(position_info.yaw,position_info.pitch,position_info.roll);
		matrix_cbn = rot_cbn.matrix();

		pre_vel << position_info.ve,position_info.vn,position_info.vu;

		pre_qbn = matrix_cbn;
		initialed_ = true;
	}
	else
	{
		gtsam::Vector3 dsculling = gtsam::Z_3x1;
		gtsam::Vector3 drotate = gtsam::Z_3x1;
		gtsam::Vector3 dconing = gtsam::Z_3x1;

		gtsam::Vector3 dv_sf = gtsam::Z_3x1;
		gtsam::Vector3 dv_cor_g = gtsam::Z_3x1;
		gtsam::Vector3 vel = gtsam::Z_3x1;
		gtsam::Vector3 w_inn = gtsam::Z_3x1;
		gtsam::Vector3 vel_b = gtsam::Z_3x1;
		gtsam::Vector3 tmp;
		gtsam::Matrix3 cnn;
		gtsam::Matrix3 cbb;
		gtsam::Matrix3 dv_cnn;
		gtsam::Quaternion q_bb,q_nn,q_bn;
		gtsam::Vector3 dtheta(imu_data.gyro_x*imu_data.delta_time,imu_data.gyro_y*imu_data.delta_time,imu_data.gyro_z*imu_data.delta_time);
		gtsam::Vector3 dvel(imu_data.acc_x*imu_data.delta_time,imu_data.acc_y*imu_data.delta_time,imu_data.acc_z*imu_data.delta_time);

		ErrorCompensate(dtheta,position_info.gyro_bias*imu_data.delta_time);
		ErrorCompensate(dvel,position_info.acc_bias*imu_data.delta_time);
		position_info.time_stamp = imu_data.time_stamp;

		//Velocity Update

		CalculateGn(position_info.lat*KDeg2Rad,position_info.height);
		CalculateWien(position_info.lat*KDeg2Rad);
		CalculateWenn(position_info.lat*KDeg2Rad,position_info.height,position_info.ve,position_info.vn);
		CalculateRm(position_info.lat*KDeg2Rad);
		CalculateRn(position_info.lat*KDeg2Rad);

		w_inn = 2*wien_ + wenn_;
//		std::cout<<w_inn<<std::endl;

		// Velocity Update
		drotate = 0.5 * dtheta.cross(dvel);
		dsculling = CalculateSculling(imu_data);
		tmp = 0.5*(wien_ + wenn_)*imu_data.delta_time;
		dv_cnn = gtsam::I_3x3 -gtsam::skewSymmetric(tmp(0),tmp(1),tmp(2));
		dv_sf = dv_cnn*matrix_cbn*(dvel + drotate + dsculling);
		dv_cor_g =  -w_inn.cross(pre_vel) + gn_;
		dv_cor_g = dv_cor_g * imu_data.delta_time;
		vel = pre_vel + dv_sf + dv_cor_g;

//		// Position Update
		position_info.height = pre_position_info_.height + 0.5*(pre_position_info_.vu+vel(2)) * imu_data.delta_time;
		position_info.lat = pre_position_info_.lat + 0.5*(pre_position_info_.vn+vel(1)) * imu_data.delta_time/(Rm_ + position_info.height)*KRad2Deg;
		position_info.lon = pre_position_info_.lon + 0.5*(pre_position_info_.ve+vel(0)) * imu_data.delta_time/((Rn_ + position_info.height)*cos(position_info.lat*KDeg2Rad))*KRad2Deg;

		position_info.ve = vel(0);
		position_info.vn = vel(1);
		position_info.vu = vel(2);
		pre_vel = vel;

//		// Attitude Update
		CalculateGn(position_info.lat*KDeg2Rad,position_info.height);
		CalculateWien(position_info.lat*KDeg2Rad);
		CalculateWenn(position_info.lat*KDeg2Rad,position_info.height,position_info.ve,position_info.vn);
		CalculateRm(position_info.lat*KDeg2Rad);
		CalculateRn(position_info.lat*KDeg2Rad);

		w_inn =  wien_ + wenn_;
		dconing = CalculateConing(imu_data);
		q_bb = RotVec2Quat(dconing);
		q_nn = RotVec2Quat(-w_inn*imu_data.delta_time);
		q_bn = pre_qbn * q_bb;
		q_bn.normalized();
		pre_qbn = q_nn * q_bn;
		pre_qbn.normalized();
		matrix_cbn = pre_qbn.toRotationMatrix();
		rot_cbn = gtsam::Rot3(matrix_cbn);
		position_info.yaw = rot_cbn.yaw();
		position_info.pitch = rot_cbn.pitch();
		position_info.roll = rot_cbn.roll();

		pre_imu_data_ = imu_data;
		pre_position_info_ = position_info;
		position_info.fix_status = E_FIX_STATUS::E_STATUS_IMU;

	}
}
void DeadReckoning::DeadReckoning2D(VehicleData vehicle_data, PositionInfo& position_info)
{

}

void DeadReckoning::CalculateGn(double lat,double height)
{
	double s2 = sin(lat)*sin(lat);
	double s4 = s2*s2;
	double glh = g0_*(1 + beta_*s2 + beta1_*s4) - beta2_*height;
	gn_(2) = glh;
}
void DeadReckoning::CalculateWien(double lat)
{
	wien_(0) = 0;
	wien_(1) = we_*cos(lat);
	wien_(2) = we_*sin(lat);
}
void DeadReckoning::CalculateWenn(double lat,double height,double ve,double vn)
{
	wenn_(0) = -vn/(Rm_ + height);
	wenn_(1) = ve/(Rn_ + height);
	wenn_(2) = ve*tan(lat)/(Rn_ + height);
}
void DeadReckoning::CalculateRm(double lat)
{
	double s2 = sin(lat)*sin(lat);
	Rm_ = Re_*(1-e2_)/pow(1-e2_*s2,1.5);
}
void DeadReckoning::CalculateRn(double lat)
{
	double s2 = sin(lat)*sin(lat);
	Rn_ = Re_/sqrt(1-e2_*s2);
}
gtsam::Vector3 DeadReckoning::CalculateSculling(ImuData imu_data)
{
	gtsam::Vector3 dsculling;
	gtsam::Vector3 d_theta1(pre_imu_data_.gyro_x,pre_imu_data_.gyro_y,pre_imu_data_.gyro_z);
	gtsam::Vector3 d_theta2(imu_data.gyro_x,imu_data.gyro_y,imu_data.gyro_z);
	gtsam::Vector3 d_vel1(pre_imu_data_.acc_x,pre_imu_data_.acc_y,pre_imu_data_.acc_z);
	gtsam::Vector3 d_vel2(imu_data.acc_x,imu_data.acc_y,imu_data.acc_z);
	d_theta1 = d_theta1 * pre_imu_data_.delta_time;
	d_theta2 = d_theta2 * imu_data.delta_time;

	d_vel1 = d_vel1 * pre_imu_data_.delta_time;
	d_vel2 = d_vel2 * imu_data.delta_time;

	dsculling = 1.0/12.0*(d_theta1.cross(d_vel2)+d_vel1.cross(d_theta2));

	return dsculling;
}
gtsam::Vector3 DeadReckoning::CalculateConing(ImuData imu_data)
{
	gtsam::Vector3 dconing;
	gtsam::Vector3 d_theta1(pre_imu_data_.gyro_x,pre_imu_data_.gyro_y,pre_imu_data_.gyro_z);
	gtsam::Vector3 d_theta2(imu_data.gyro_x,imu_data.gyro_y,imu_data.gyro_z);
	d_theta1 = d_theta1 * pre_imu_data_.delta_time;
	d_theta2 = d_theta2 * imu_data.delta_time;
	dconing = d_theta2 + 1.0/12.0*d_theta1.cross(d_theta2);
	return dconing;
}
gtsam::Quaternion DeadReckoning::RotVec2Quat(gtsam::Vector3 rot_vec)
{
	gtsam::Quaternion quat;
	double norm;
	double q0,s;
	norm = rot_vec.transpose()*rot_vec;
	if (norm < 0.000001)
	{
		q0 = 1 - norm*(1.0/8 - norm/384);
		s = 1.0/2 - norm*(1.0/48 - norm/3840);
	}
	else
	{
		double nm = sqrt(norm);
		q0 = cos(nm/2);
		s = sin(nm/2)/nm;
	}
	quat.w() = q0;
	quat.vec() = s*rot_vec;

	return quat;
}
void DeadReckoning::ErrorCompensate(gtsam::Vector3& vec, gtsam::Vector3 bias)
{
	vec = vec - bias;
}
