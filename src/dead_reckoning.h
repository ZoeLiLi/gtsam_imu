#ifndef DEAD_RECKONING_H
#define DEAD_RECKONING_H
#include <iostream>
#include <fstream>
#include <string>
#include "constant.h"

namespace TADR
{

class DeadReckoning
{
public:
	DeadReckoning(void);
	virtual ~DeadReckoning();

public:
	void InsMechanization(ImuData imu_data,PositionInfo& position_info);
	void DeadReckoning2D(VehicleData vehicle_data, PositionInfo& position_info);
	void UpdatePositionInfo(PositionInfo position_info);
private:
	void CalculateGn(double lat,double height);
	void CalculateWien(double lat);
	void CalculateWenn(double lat,double height,double ve,double vn);
	void CalculateRm(double lat);
	void CalculateRn(double lat);
	gtsam::Vector3 CalculateSculling(ImuData imu_data);
	gtsam::Vector3 CalculateConing(ImuData imu_data);
	gtsam::Quaternion RotVec2Quat(gtsam::Vector3 rot_vec);
	void ErrorCompensate(gtsam::Vector3& vec, gtsam::Vector3 bias);

private:
	double						e2_;
	double						Re_;
	double						we_;
	double						g0_;
	double 						beta_;
	double						beta1_;
	double						beta2_;
	bool						initialed_;
	PositionInfo				pre_position_info_;
	ImuData						pre_imu_data_;
	gtsam::Vector3				gn_;
	gtsam::Vector3				wien_;
	gtsam::Vector3				wenn_;
	double						Rm_;
	double						Rn_;
	gtsam::Matrix33				matrix_cbn_;
	gtsam::Rot3					rot_cbn_;
	gtsam::Quaternion			pre_qbn_;



};
}
#endif

