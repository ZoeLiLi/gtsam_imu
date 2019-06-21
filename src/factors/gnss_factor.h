#ifndef GNSS_FACTOR_H
#define GNSS_FACTOR_H

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose3.h>

namespace TADR{

class GNSSFactor1: public gtsam::NoiseModelFactor1 <gtsam::Pose3> {

private:

  typedef gtsam::NoiseModelFactor1<gtsam::Pose3> Base;

  double x_;
  double y_;
 

public:

  /// shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<GNSSFactor1> shared_ptr;

  /// Typedef to this class
  typedef GNSSFactor1 This;

  /** default constructor - only use for serialization */
  GNSSFactor1(): x_(0.0),y_(0.0) {}

  virtual ~GNSSFactor1() {}

  GNSSFactor1(gtsam::Key key, double x,double y,const gtsam::SharedNoiseModel& model):
	  Base(model,key),x_(x),y_(y){

  }

  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  gtsam::Vector evaluateError(const gtsam::Pose3& pose,
         boost::optional<gtsam::Matrix&> H1 = boost::none) const;
  /// print
  virtual void print(const std::string& s, const gtsam::KeyFormatter& keyFormatter =
      gtsam::DefaultKeyFormatter) const;

  /// equals
  virtual bool equals(const gtsam::NonlinearFactor& expected, double tol = 1e-9) const;


  inline const double& x() const {
    return x_;
  }
  inline const double& y() const{
    return y_;
  }
};

}

#endif
