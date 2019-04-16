#ifndef VELOCITY_FACTOR_H
#define VELOCITY_FACTOR_H

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose3.h>

class VelocityFactor: public gtsam::NoiseModelFactor1 <gtsam::Point3> {

private:

  typedef gtsam::NoiseModelFactor1<gtsam::Point3> Base;

  gtsam::Point3 velocity_; ///< Position measurement in cartesian coordinates
  gtsam::Rot3 bRn_;

public:

  /// shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<VelocityFactor> shared_ptr;

  /// Typedef to this class
  typedef VelocityFactor This;

  /** default constructor - only use for serialization */
  VelocityFactor(): velocity_(0, 0, 0) {}

  virtual ~VelocityFactor() {}

  VelocityFactor(gtsam::Key key1, const gtsam::Point3& velocityIn,
		  const gtsam::Rot3& nRb,const gtsam::SharedNoiseModel& model):
	  Base(model,key1),velocity_(velocityIn),bRn_(nRb.inverse()){

  }

  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  gtsam::Vector evaluateError(const gtsam::Point3& vel,
         boost::optional<gtsam::Matrix&> H1 = boost::none) const;
  /// print
  virtual void print(const std::string& s, const gtsam::KeyFormatter& keyFormatter =
      gtsam::DefaultKeyFormatter) const;

  /// equals
  virtual bool equals(const gtsam::NonlinearFactor& expected, double tol = 1e-9) const;


  inline const gtsam::Point3 & measurementIn() const {
    return velocity_;
  }

private:

  /// Serialization function
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar
        & boost::serialization::make_nvp("NoiseModelFactor1",
            boost::serialization::base_object<Base>(*this));
    ar & BOOST_SERIALIZATION_NVP(velocity_);
  }
};

#endif
