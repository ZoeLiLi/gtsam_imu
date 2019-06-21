#include "gnss_factor.h"
using namespace TADR;

void GNSSFactor1::print(const std::string& s,const gtsam::KeyFormatter& keyFormatter) const{
	std::cout << s <<"GNSSFactor1("<<keyFormatter(this->key())<<std::endl;
	std::cout << "GNSS measurement: " << x_<<" "<< y_<<std::endl;
	noiseModel_->print(" noise model: ");
}

bool GNSSFactor1::equals(const gtsam::NonlinearFactor& expected, double tol) const{
	const This* e = dynamic_cast<const This*>(&expected);
	return e != NULL && Base::equals(*e, tol) && gtsam::traits<double>::Equals(x_, e->x_, tol)&& gtsam::traits<double>::Equals(y_, e->y_, tol);
}

gtsam::Vector GNSSFactor1::evaluateError(const gtsam::Pose3& pose,
	    boost::optional<gtsam::Matrix&> H1) const {
	if(H1)
	{
		gtsam::Rot3 rot = pose.rotation();
		H1->resize(2,6);
		H1->block<2,3>(0,0).setZero();
		H1->block<2,3>(0,3) = rot.matrix().block(0,0,2,3);
	}
	return gtsam::Vector2(pose.x()-x_,pose.y()-y_);
}

