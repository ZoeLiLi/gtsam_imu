#include "velocity_factor.h"
#include <gtsam/nonlinear/NonlinearFactor.h>
using namespace TADR;

void VelocityFactor::print(const std::string& s,const gtsam::KeyFormatter& keyFormatter) const{
	std::cout << s <<"VelocityFactor("<<keyFormatter(this->key())<<std::endl;
	std::cout << "Velocity measurement: " << velocity_<<std::endl;
	noiseModel_->print(" noise model: ");
}

bool VelocityFactor::equals(const gtsam::NonlinearFactor& expected, double tol) const{
	const This* e = dynamic_cast<const This*>(&expected);
	return e != NULL && Base::equals(*e, tol) && gtsam::traits<gtsam::Vector3>::Equals(velocity_, e->velocity_, tol);
}

gtsam::Vector VelocityFactor::evaluateError(const gtsam::Vector3& vel,
	    boost::optional<gtsam::Matrix&> H1) const {
	gtsam::Vector3 hx = bRn_.rotate(vel,boost::none,H1);
	return (hx-velocity_);
	}

