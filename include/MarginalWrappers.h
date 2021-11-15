#ifndef JOINT_MARGINAL_WRAPPER_H
#define JOINT_MARGINAL_WRAPPER_H

#include <eigen3/Eigen/Core>
#include <gtsam/nonlinear/Marginals.h>

namespace jcbb
{

class JointMarginalWrapper
{
public:
    explicit JointMarginalWrapper(const Eigen::MatrixXd &P, const gtsam::KeyVector& keys);
    const Eigen::Block<const gtsam::Matrix> operator()(gtsam::Key iVariable, gtsam::Key jVariable) const;
    const Eigen::Block<const gtsam::Matrix> at(gtsam::Key iVariable, gtsam::Key jVariable) const { return (*this)(iVariable, jVariable); }
    const gtsam::Matrix& fullMatrix() const {return P_joint_;}

private:
    gtsam::Matrix P_joint_;
    gtsam::KeyVector keys_;
};

class MarginalsWrapper
{
public:
    explicit MarginalsWrapper(const Eigen::MatrixXd &P) : P_full_(P) {}
    const Eigen::Block<const gtsam::Matrix> marginalCovariance(gtsam::Key variable) const;
    JointMarginalWrapper jointMarginalCovariance(const gtsam::KeyVector &variables) const;

private:
    Eigen::MatrixXd P_full_;
};

} // namespace jcbb

#endif // JOINT_MARGINAL_WRAPPER_H