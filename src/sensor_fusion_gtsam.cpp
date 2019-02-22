#include <iostream>
#include <fstream>
#include <string>
#include <Eigen/Core>

#include <sophus/se3.h>
#include <sophus/so3.h>

#include <gtsam/slam/dataset.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/navigation/PreintegrationBase.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/ImuFactor.h>

#include <boost/algorithm/string.hpp>


using namespace std;
using Sophus::SE3;
using Sophus::SO3;

/************************************************
 * 本程序演示如何用 gtsam 进行位姿图优化
 * sphere.g2o 是人工生成的一个 Pose graph，我们来优化它。
 * 与 g2o 相似，在 gtsam 中添加的是因子，相当于误差
 * **********************************************/
 typedef struct
 {
	 double t;
	 double dt;
	 double ax;
	 double ay;
	 double az;
	 double gx;
	 double gy;
	 double gz;
 }IMUData;
 typedef struct{
	 double t;
	 gtsam::Vector3 pos;
 }GPSData;
 vector<IMUData> imuData;
void findIMUIndex(double tEnd,double tStart,int& startIndex,vector<int>& imuIndex);
int main ( )
{
	int cntVertex = 0,cntEdge = 0;
	int cntIMU = 0,cntGPS = 0;
	int startIndex = 0;
    ifstream imuifs("../../data/KittiEquivBiasedImu.txt");
    ifstream gpsifs("../../data/KittiGps_converted.txt");
    ofstream resultofs("../../data/results.txt");
    if(!imuifs.is_open())
    {
      cout<<"open imu data file failed!"<<endl;
    }
    if(!gpsifs.is_open())
    {
    	cout<<"open gps data file failed!"<<endl;
    }
    vector<GPSData> gpsData;
    while(!imuifs.eof())
    {
    	IMUData data;
    	string str;
    	getline(imuifs,str);
    	vector<string> vTemp;
    	boost::split(vTemp,str,boost::is_any_of(" "));
    	if((vTemp[0] != "Time" && vTemp[0] != "") && vTemp.size() == 8)
    	{
    		data.t = atof(vTemp[0].c_str());
    		data.dt = atof(vTemp[1].c_str());
    		data.ax = atof(vTemp[2].c_str());
    		data.ay = atof(vTemp[3].c_str());
    		data.az = atof(vTemp[4].c_str());
    		data.gx = atof(vTemp[5].c_str());
    		data.gy = atof(vTemp[6].c_str());
    		data.gz = atof(vTemp[7].c_str());
    		imuData.push_back(data);
    		cntIMU++;
    	}
    }
    while(!gpsifs.eof())
    {
    	GPSData data;
    	string str;
    	getline(gpsifs,str);
    	vector<string> vTemp;
    	boost::split(vTemp,str,boost::is_any_of(","));
    	if(vTemp[0]!="Time" && vTemp[0] != "")
    	{
    		data.t = atof(vTemp[0].c_str());
    		data.pos[0] = atof(vTemp[1].c_str());
    		data.pos[1] = atof(vTemp[2].c_str());
    		data.pos[2] = atof(vTemp[3].c_str());
    		gpsData.push_back(data);
    		cntGPS++;
    	}
    }
    cout<<"imuData size:"<<imuData.size()<<endl;
    cout<<"gpsData size:"<<gpsData.size()<<endl;

    gtsam::Vector3 g(0.0,0.0,-9.8);
    gtsam::Vector3 w_coriolis(0.0,0.0,0.0);
    gtsam::Vector3 zero(0.0,0.0,0.0);
    gtsam::Matrix3 unit = gtsam::Matrix3::Identity();
    cout<<"unit matrix"<<unit<<endl;
    int firstGPSPose = 1;
    
    gtsam::Rot3 R;
    gtsam::Point3 t(gpsData[0].pos);

    // bias noise model
    gtsam::Vector biasSigma(6);
    biasSigma<< gtsam::Vector3::Constant(0.1),gtsam::Vector3::Constant(5.0e-5);

    //gps noise model
    gtsam::Vector gpsSigma(6);
    gpsSigma<<gtsam::Vector3::Constant(0.0),gtsam::Vector3::Constant(1.0/0.07);
    gtsam::SharedNoiseModel noiseModelGPS = gtsam::noiseModel::Diagonal::Precisions(gpsSigma, 1);

    // current key , value, and sigma
    gtsam::Key currentPosKey = gtsam::Key(gpsData[0].t*1000*1000);
    gtsam::Key currentVelKey = currentPosKey + 1;
    gtsam::Key currentBiasKey = currentPosKey + 2;
    
    gtsam::Pose3 currentPos(gtsam::Pose3(R,t));
    gtsam::Vector3 currentVel(0.0,0.0,0.0);
    gtsam::imuBias::ConstantBias currentBias(zero,zero);
    
    gtsam::SharedNoiseModel initSigmaP = gtsam::noiseModel::Isotropic::Precision(6,1,true);
    gtsam::SharedNoiseModel initSigmaV = gtsam::noiseModel::Isotropic::Sigma(3, 1000.0, 1);
    gtsam::SharedNoiseModel initSigmaB = gtsam::noiseModel::Isotropic::Sigmas(biasSigma, 1);

    //imu params
    boost::shared_ptr<gtsam::PreintegrationParams> IMU_params(new gtsam::PreintegrationParams(g));
    IMU_params->setAccelerometerCovariance(0.01*unit);
    IMU_params->setGyroscopeCovariance(1.75e-4*1.75e-4*unit);
    IMU_params->setIntegrationCovariance(0*unit);
    IMU_params->setOmegaCoriolis(w_coriolis);

    gtsam::ISAM2Params isamParams;
    isamParams.setFactorization("CHOLESKY");
    isamParams.setRelinearizeSkip(10);
    gtsam::ISAM2 isam(isamParams);

    gtsam::NonlinearFactorGraph newFactors;
    gtsam::Values newValues;

    int i = 1,j=0;
    double currentT,previousT;
    while(i<cntGPS)
    {
    	currentPosKey = gtsam::Key(gpsData[i].t*1000*1000);
    	currentVelKey = currentPosKey + 1;
    	currentBiasKey = currentPosKey + 2;
    	cout<<"Key:"<<currentPosKey<<" "<<currentVelKey<<" "<<currentBiasKey<<endl;
    	currentT = gpsData[i].t;
    	if(i == firstGPSPose)
    	{
    		newValues.insert(currentPosKey,currentPos);
    		newValues.insert(currentVelKey,currentVel);
    		newValues.insert(currentBiasKey,currentBias);
    		gtsam::NonlinearFactor::shared_ptr posFactor(new
    				gtsam::PriorFactor<gtsam::Pose3>(currentPosKey,currentPos,initSigmaP));
    		gtsam::NonlinearFactor::shared_ptr velFactor(new
    				gtsam::PriorFactor<gtsam::Vector>(currentVelKey,currentVel,initSigmaV));
    		gtsam::NonlinearFactor::shared_ptr biasFactor(new
    				gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(currentBiasKey,currentBias,initSigmaB));
    		newFactors.push_back(posFactor);
    		newFactors.push_back(velFactor);
    		newFactors.push_back(biasFactor);
    		resultofs<<"Key:"<<currentPosKey<< currentPos<<" "<<currentVel<<" "<<currentBias<<endl;
    	}
    	else
    	{
    		previousT = gpsData[i-1].t;
            vector<int> IMUindices;
            findIMUIndex(currentT, previousT,startIndex, IMUindices);
    		boost::shared_ptr<gtsam::PreintegratedImuMeasurements> currentSummarizedMeasurement ( new
    				gtsam::PreintegratedImuMeasurements(IMU_params,currentBias));
    		
    		cout<<"start time:"<<previousT<<". End time:"<< currentT<<". startIndex:"<<startIndex<<endl;
    		
    		int imuLength = IMUindices.size();
    		while(j < imuLength)
    		{
    			int  index = IMUindices[j];
    			gtsam::Vector3 accMeas(imuData[index].ax,imuData[index].ay,imuData[index].az);
    			gtsam::Vector3 gyroMeas(imuData[index].gx,imuData[index].gy,imuData[index].gz);
    			double dt = imuData[index].dt;
    			currentSummarizedMeasurement->integrateMeasurement(accMeas,gyroMeas,dt);
    			j++;
    		}
    		gtsam::NonlinearFactor::shared_ptr imuFactor(new
    				gtsam::ImuFactor(currentPosKey-1,currentVelKey-1,currentPosKey,currentVelKey,currentBiasKey,*currentSummarizedMeasurement));

    		gtsam::Vector imuBiasSigmas(6);
    		imuBiasSigmas<<sqrt(imuLength)*0.1670e-3,sqrt(imuLength)*0.1670e-3,sqrt(imuLength)*0.1670e-3,sqrt(imuLength)*0.0029e-3,sqrt(imuLength)*0.0029e-3,sqrt(imuLength)*0.0029e-3;

    		gtsam::NonlinearFactor::shared_ptr betweenFactorConstantBias(new gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>
    				(currentBiasKey-1,currentBiasKey,gtsam::imuBias::ConstantBias(zero,zero),
    						gtsam::noiseModel::Diagonal::Sigmas(imuBiasSigmas)));
    		newFactors.push_back(imuFactor);
    		newFactors.push_back(betweenFactorConstantBias);
    		gtsam::Pose3 gpsPos(gtsam::Pose3(currentPos.rotation(),gpsData[i].pos));

    		if(i%1 == 0)
    		{
    			gtsam::NonlinearFactor::shared_ptr gpsFactor(new gtsam::PriorFactor<gtsam::Pose3>(currentPosKey,gpsPos,noiseModelGPS));
    			newFactors.push_back(gpsFactor);
    		}
    		newValues.insert(currentPosKey,gpsPos);
    		newValues.insert(currentVelKey,currentVel);
    		newValues.insert(currentBiasKey,currentBias);
    		//if(i > 1)
    		//{
    			gtsam::ISAM2Result isam2Result = isam.update(newFactors, newValues);
    			isam2Result.print();
    			
    			gtsam::Values result = isam.calculateEstimate();
    			result.print("current estimate:");
    			currentPos = result.at<gtsam::Pose3>(currentPosKey);
    			currentVel = result.at<gtsam::Vector3>(currentVelKey);
    			currentBias = result.at<gtsam::imuBias::ConstantBias>(currentBiasKey);
    			resultofs<<currentPosKey<< currentPos<<" "<<currentVel<<" "<<currentBias<<endl;
    			newFactors.resize(0);
    			newValues.clear();
    		//}

    	}
    	i++;
    }
	return 0;
}
void findIMUIndex(double tEnd,double tStart,int& startIndex,vector<int>& imuIndex)
{

	while(startIndex < imuData.size())
	{
		if(imuData[startIndex].t >= tStart && imuData[startIndex].t <= tEnd)
		{
			imuIndex.push_back(startIndex);
		}
		else if(imuData[startIndex].t > tEnd)
		{
			break;
		}
		startIndex ++;
	}
}

