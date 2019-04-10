#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>

class VelocityFactor: public NoiseModelFactor2<gtsam::Pose3,gtsam::Vector3> {

private:

  typedef NoiseModelFactor1<gtsam::Pose3,gtsam::Vector3> Base;

  gtsam::Vector3 velocity_; ///

public:

  typedef boost::shared_ptr<VelocityFactor> shared_ptr;

  /// Typedef to this class
  typedef VelocityFactor This;

  /** default constructor - only use for serialization */
  VelocityFactor(): velocity_(0,0,0) {}

  virtual ~VelocityFactor() {}

  VelocityFactor(Key key, const gtsam::Vector3& velocityIn, const SharedNoiseModel& model) :
      Base(model, key), velocity_(velocityIn) {
  }

  /// @return a deep copy of this factor
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  /// print
  virtual void print(const std::string& s, const KeyFormatter& keyFormatter =
      DefaultKeyFormatter) const;

  /// equals
  virtual bool equals(const NonlinearFactor& expected, double tol = 1e-9) const;

  /// @return measurementIn 
  inline const gtsam::Vector3 & measurementIn() const {
    return velocity_;

  /// vector of errors
  Vector evaluateError(const gtsam::Pose3& p,const gtsam::Vector3& v
      boost::optional<gtsam::Matrix&> H = boost::none) const;
  }

private:

  /// Serialization function
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar
        & boost::serialization::make_nvp("NoiseModelFactor1",
            boost::serialization::base_object<Base>(*this));
    ar & BOOST_SERIALIZATION_NVP(nT_);
  }
};

