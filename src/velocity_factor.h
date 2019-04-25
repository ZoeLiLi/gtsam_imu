#ifndef VELOCITY_FACTOR_H
#define VELOCITY_FACTOR_H

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose3.h>

namespace TADR{
class VelocityFactor1: public gtsam::NoiseModelFactor1 <gtsam::Vector3> {

private:

  typedef gtsam::NoiseModelFactor1<gtsam::Vector3> Base;

  gtsam::Point3 velocity_; ///< Position measurement in cartesian coordinates
  gtsam::Rot3 bRn_;

public:

  /// shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<VelocityFactor1> shared_ptr;

  /// Typedef to this class
  typedef VelocityFactor1 This;

  /** default constructor - only use for serialization */
  VelocityFactor1(): velocity_(0, 0, 0) {}

  virtual ~VelocityFactor1() {}

  VelocityFactor1(gtsam::Key key1, const gtsam::Vector3& velocityIn,
		  const gtsam::Rot3& nRb,const gtsam::SharedNoiseModel& model):
	  Base(model,key1),velocity_(velocityIn),bRn_(nRb.inverse()){

  }

  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  gtsam::Vector evaluateError(const gtsam::Vector3& vel,
         boost::optional<gtsam::Matrix&> H1 = boost::none) const;
  /// print
  virtual void print(const std::string& s, const gtsam::KeyFormatter& keyFormatter =
      gtsam::DefaultKeyFormatter) const;

  /// equals
  virtual bool equals(const gtsam::NonlinearFactor& expected, double tol = 1e-9) const;


  inline const gtsam::Vector3 & measurementIn() const {
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

class VelocityFactor2: public gtsam::NoiseModelFactor2 <gtsam::Pose3,gtsam::Vector3> {

private:

  typedef gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Vector3> Base;

  gtsam::Point3 velocity_; ///< velocity measurement
public:

  /// shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<VelocityFactor2> shared_ptr;

  /// Typedef to this class
  typedef VelocityFactor2 This;

  /** default constructor - only use for serialization */
  VelocityFactor2(): velocity_(0, 0, 0) {}

  virtual ~VelocityFactor2() {}

  VelocityFactor2(gtsam::Key key1, gtsam::Key key2,const gtsam::Vector3& velocityIn,
		  const gtsam::SharedNoiseModel& model):
	  Base(model,key1,key2),velocity_(velocityIn){

  }

  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  gtsam::Vector evaluateError(const gtsam::Pose3& pose,const gtsam::Vector3& vel,
         boost::optional<gtsam::Matrix&> H1 = boost::none,
		 boost::optional<gtsam::Matrix&> H2 = boost::none) const;
  /// print
  virtual void print(const std::string& s, const gtsam::KeyFormatter& keyFormatter =
      gtsam::DefaultKeyFormatter) const;

  /// equals
  virtual bool equals(const gtsam::NonlinearFactor& expected, double tol = 1e-9) const;


  inline const gtsam::Vector3 & measurementIn() const {
    return velocity_;
  }

private:

  /// Serialization function
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar
        & boost::serialization::make_nvp("NoiseModelFactor2",
            boost::serialization::base_object<Base>(*this));
    ar & BOOST_SERIALIZATION_NVP(velocity_);
  }
};

}

#endif
