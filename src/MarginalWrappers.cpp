#include "MarginalWrappers.h"
#include <eigen3/Eigen/Core>

namespace jcbb
{


class JointMarginalWrapper
{
    const Eigen::Block<const gtsam::Matrix> JointMarginalWrapper::operator()(int i, int j) const
  {
    return P_joint_.block(2 * i, 2 * j, 2, 2);
  }
    const Eigen::Block<const gtsam::Matrix> at(int i, int j) const { return (*this)(i, j); }
    const gtsam::Matrix& fullMatrix() const;

private:
    gtsam::Matrix P_joint_;
};

class MarginalsWrapper
{
public:
    explicit MarginalsWrapper(const Eigen::MatrixXd &P) : P_full_(P) {}
    const Eigen::Block<const gtsam::Matrix>& marginalCovariance(gtsam::Key variable) const;
    JointMarginalWrapper jointMarginalCovariance(const gtsam::KeyVector &variables) const;

private:
    Eigen::MatrixXd P_full_;
};


} // namespace jcbb