#include "velocity_factor.h"
#include <gtsam/nonlinear/NonlinearFactor.h>
using namespace TADR;

void VelocityFactor1::print(const std::string& s,const gtsam::KeyFormatter& keyFormatter) const{
	std::cout << s <<"VelocityFactor1("<<keyFormatter(this->key())<<std::endl;
	std::cout << "Velocity measurement: " << velocity_<<std::endl;
	noiseModel_->print(" noise model: ");
}

bool VelocityFactor1::equals(const gtsam::NonlinearFactor& expected, double tol) const{
	const This* e = dynamic_cast<const This*>(&expected);
	return e != NULL && Base::equals(*e, tol) && gtsam::traits<gtsam::Vector3>::Equals(velocity_, e->velocity_, tol);
}

gtsam::Vector VelocityFactor1::evaluateError(const gtsam::Vector3& vel,
	    boost::optional<gtsam::Matrix&> H1) const {
	gtsam::Vector3 hx = bRn_.rotate(vel,boost::none,H1);
	//std::cout<<(hx-velocity_)<<std::endl;
	return (hx-velocity_);
	}



void VelocityFactor2::print(const std::string& s,const gtsam::KeyFormatter& keyFormatter) const{
	      std::cout << s << "VelocityFactor2("
	          << keyFormatter(this->key1()) << ","
	          << keyFormatter(this->key2()) << ")\n";
	      std::cout << "Velocity measurement: " << velocity_<<std::endl;
	      noiseModel_->print("  noise model: ");
}

bool VelocityFactor2::equals(const gtsam::NonlinearFactor& expected, double tol) const{
	const This* e = dynamic_cast<const This*>(&expected);
	return e != NULL && Base::equals(*e, tol) && gtsam::traits<gtsam::Vector3>::Equals(velocity_, e->velocity_, tol);
}

gtsam::Vector VelocityFactor2::evaluateError(const gtsam::Pose3& pose,const gtsam::Vector3& vel,
	    boost::optional<gtsam::Matrix&> H1,boost::optional<gtsam::Matrix&> H2) const {

	gtsam::Rot3 bRn = pose.rotation().inverse();
	if(H1&&H2)
	{
		H1->resize(3,6);
		gtsam::Matrix3 H_rot;
		gtsam::Vector3 hx = bRn.rotate(vel,H_rot,H2);
		H1->block<3,3>(0,0).setZero();
		H1->block<3,3>(0,3) = H_rot;
		//std::cout<<(hx - velocity_)<<std::endl;
		return (hx-velocity_);
	}
	else
	{
		gtsam::Vector3 hx = bRn.rotate(vel,boost::none,H2);
		return (hx-velocity_);
	}

	}
