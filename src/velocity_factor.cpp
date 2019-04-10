#include "VelocityFactor.h"

using namespace std;

//***************************************************************************
void VelocityFactor::print(const string& s, const KeyFormatter& keyFormatter) const {
  cout << s << "VelocityFactor on " << keyFormatter(key()) << "\n";
  cout << "  Velocity measurement: " << velocity_ << "\n";
  noiseModel_->print("  noise model: ");
}

//***************************************************************************
bool VelocityFactor::equals(const NonlinearFactor& expected, double tol) const {
  const This* e = dynamic_cast<const This*>(&expected);
  return e != NULL && Base::equals(*e, tol) && traits<Point3>::Equals(velocity_, e->velocity_, tol);
}

//***************************************************************************
Vector VelocityFactor::evaluateError(const Pose3& p,
    boost::optional<Matrix&> H) const {
  return p.translation(H) -velocity_;
}
