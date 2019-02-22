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
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>

#include <boost/algorithm/string.hpp>


using namespace std;
using namespace gtsam;
using gtsam::symbol_shorthand::X;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::B;
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
 struct IMUHelper {
   IMUHelper() {
     {
       auto gaussian = gtsam::noiseModel::Diagonal::Sigmas(
           (gtsam::Vector(6) << Vector3::Constant(5.0e-2), Vector3::Constant(5.0e-3))
               .finished());
       auto huber = noiseModel::Robust::Create(
           noiseModel::mEstimator::Huber::Create(1.345), gaussian);

       biasNoiseModel = huber;
     }

     {
       auto gaussian = noiseModel::Isotropic::Sigma(3, 0.01);
       auto huber = noiseModel::Robust::Create(
           noiseModel::mEstimator::Huber::Create(1.345), gaussian);

       velocityNoiseModel = huber;
     }

     // expect IMU to be rotated in image space co-ords
     auto p = boost::make_shared<PreintegratedCombinedMeasurements::Params>(
         Vector3(0.0, 9.8, 0.0));

     p->accelerometerCovariance =
         I_3x3 * pow(0.0565, 2.0);  // acc white noise in continuous
     p->integrationCovariance =
         I_3x3 * 1e-9;  // integration uncertainty continuous
     p->gyroscopeCovariance =
         I_3x3 * pow(4.0e-5, 2.0);  // gyro white noise in continuous
     p->biasAccCovariance = I_3x3 * pow(0.00002, 2.0);  // acc bias in continuous
     p->biasOmegaCovariance =
         I_3x3 * pow(0.001, 2.0);  // gyro bias in continuous
     p->biasAccOmegaInt = Matrix::Identity(6, 6) * 1e-5;

     // body to IMU rotation
     Rot3 iRb(0.036129, -0.998727, 0.035207,
              0.045417, -0.033553, -0.998404,
              0.998315, 0.037670, 0.044147);

     // body to IMU translation (meters)
     Point3 iTb(0.03, -0.025, -0.06);

     // body in this example is the left camera
     p->body_P_sensor = Pose3(iRb, iTb);

     Rot3 prior_rotation = Rot3(I_3x3);
     Pose3 prior_pose(prior_rotation, Point3(0, 0, 0));

     Vector3 acc_bias(0.0, -0.0942015, 0.0);  // in camera frame
     Vector3 gyro_bias(-0.00527483, -0.00757152, -0.00469968);

     priorImuBias = imuBias::ConstantBias(acc_bias, gyro_bias);

     prevState = NavState(prior_pose, Vector3(0, 0, 0));
     propState = prevState;
     prevBias = priorImuBias;

     preintegrated = new PreintegratedCombinedMeasurements(p, priorImuBias);
   }

   imuBias::ConstantBias priorImuBias;  // assume zero initial bias
   noiseModel::Robust::shared_ptr velocityNoiseModel;
   noiseModel::Robust::shared_ptr biasNoiseModel;
   NavState prevState;
   NavState propState;
   imuBias::ConstantBias prevBias;
   PreintegratedCombinedMeasurements* preintegrated;
 };
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
    if(!resultofs.is_open())
     {
     	cout<<"open result data file failed!"<<endl;
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


    gtsam::Vector3 g(0.0,0.0,-9.8);
    gtsam::Vector3 w_coriolis(0.0,0.0,0.0);
    gtsam::Vector3 zero(0.0,0.0,0.0);
    gtsam::Matrix3 unit = gtsam::Matrix3::Identity();
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

    gtsam::Pose3 currentPos(gtsam::Pose3(R,t));
    gtsam::Vector3 currentVel(0.0,0.0,0.0);
    gtsam::imuBias::ConstantBias currentBias(zero,zero);
    
    gtsam::SharedNoiseModel initSigmaP =gtsam::noiseModel::Diagonal::Sigmas(
    		(gtsam::Vector(6) <<gtsam::Vector3::Constant(0.1),gtsam::Vector3::Constant(0.1)).finished());

    gtsam::SharedNoiseModel initSigmaV = gtsam::noiseModel::Isotropic::Sigma(3, 1000.0, 1);
    gtsam::SharedNoiseModel initSigmaB = gtsam::noiseModel::Isotropic::Sigmas(biasSigma, 1);

    //imu params
    boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> IMU_params(new gtsam::PreintegratedCombinedMeasurements::Params(g));
    IMU_params->setAccelerometerCovariance(0.01*unit);
    IMU_params->setGyroscopeCovariance(1.75e-4*1.75e-4*unit);
    IMU_params->setIntegrationCovariance(0*unit);
    IMU_params->setOmegaCoriolis(w_coriolis);

    gtsam::ISAM2Params isamParams;
//    isamParams.setFactorization("CHOLESKY");
//    isamParams.setRelinearizeSkip(1);
    isamParams.relinearizeThreshold = 0.1;
    gtsam::ISAM2 isam(isamParams);

    gtsam::NonlinearFactorGraph newFactors;
    gtsam::Values newValues;
    IMUHelper imu;

    int i = 1,j=0;
    double currentT,previousT;
    while(i<cntGPS)
    {
    	cout<<"i = "<<i<<endl;
    	currentT = gpsData[i].t;
    	if(i == firstGPSPose)
    	{
    		//pos prior
    		t = gpsData[i].pos;
    		currentPos = gtsam::Pose3(R,t);
    		gtsam::NonlinearFactor::shared_ptr posFactor(new
    		    				gtsam::PriorFactor<gtsam::Pose3>(X(1),currentPos,initSigmaP));
    		newFactors.push_back(posFactor);
    		newValues.insert(X(0),currentPos);

    		// bias prior
    		gtsam::NonlinearFactor::shared_ptr biasFactor(new
    		    				gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(B(1),currentBias,initSigmaB));
    		newFactors.push_back(biasFactor);
    		newValues.insert(B(0),currentBias);

    		//vel prior
    		gtsam::NonlinearFactor::shared_ptr velFactor(new
    				gtsam::PriorFactor<gtsam::Vector3>(V(1),currentVel,initSigmaV));
    		newFactors.push_back(velFactor);
    		newValues.insert(V(0),currentVel);
    	}
    	else
    	{
    		gtsam::Pose3 gpsPos(gtsam::Pose3(currentPos.rotation(),gpsData[i].pos));
    		newValues.insert(X(i-1),gpsPos);
    		newValues.insert(V(i-1),currentVel);
    		newValues.insert(B(i-1),currentBias);

    		// IMU preintegrated
    		previousT = gpsData[i-1].t;
            vector<int> IMUindices;
            findIMUIndex(currentT, previousT,startIndex, IMUindices);
    		int imuLength = IMUindices.size();
    		while(j < imuLength)
    		{
    			int  index = IMUindices[j];
    			gtsam::Vector3 accMeas(imuData[index].ax,imuData[index].ay,imuData[index].az);
    			gtsam::Vector3 gyroMeas(imuData[index].gx,imuData[index].gy,imuData[index].gz);
    			double dt = imuData[index].dt;
    			imu.preintegrated->integrateMeasurement(accMeas,gyroMeas,dt);
    			j++;
    		}
    		gtsam::NonlinearFactor::shared_ptr imuFactor(new
    		    				gtsam::CombinedImuFactor(X(i-2),V(i-2),X(i-1),V(i-1),B(i-2),B(i-1),*imu.preintegrated));
    		newFactors.push_back(imuFactor);

    		// bias factor
    		gtsam::Vector imuBiasSigmas(6);
    		imuBiasSigmas<<sqrt(imuLength)*0.1670e-3,sqrt(imuLength)*0.1670e-3,sqrt(imuLength)*0.1670e-3,sqrt(imuLength)*0.0029e-3,sqrt(imuLength)*0.0029e-3,sqrt(imuLength)*0.0029e-3;
    		gtsam::NonlinearFactor::shared_ptr betweenFactorConstantBias(new gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>
    				(B(i-2),B(i-1),gtsam::imuBias::ConstantBias(zero,zero),
    						gtsam::noiseModel::Diagonal::Sigmas(imuBiasSigmas)));
    		newFactors.push_back(betweenFactorConstantBias);

    		//GPS factor
    		if(i%1 == 0)
    		{
    			gtsam::NonlinearFactor::shared_ptr gpsFactor(new gtsam::PriorFactor<gtsam::Pose3>(X(i-1),gpsPos,noiseModelGPS));
    			newFactors.push_back(gpsFactor);
    		}

    		if(i > 1)
    		{
     			gtsam::ISAM2Result isam2Result = isam.update(newFactors, newValues);
    			gtsam::Values result = isam.calculateEstimate();
    			currentPos = result.at<gtsam::Pose3>(X(i-1));
    			currentVel = result.at<gtsam::Vector3>(V(i-1));
    			currentBias = result.at<gtsam::imuBias::ConstantBias>(B(i-1));
    			resultofs<<currentT<<" "<< currentPos.x()<<" "<<currentPos.y()<<" "<<currentPos.z()<<" "<<currentVel.transpose()<<endl;
    			newFactors.resize(0);
    			newValues.clear();
    		}
    	}
    	i++;
    }
    imuifs.close();
    gpsifs.close();
    resultofs.close();
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

