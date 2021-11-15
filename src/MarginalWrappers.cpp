#include "MarginalWrappers.h"
#include <eigen3/Eigen/Core>
#include <gtsam/inference/Symbol.h>
#include <iostream>

namespace jcbb
{
  JointMarginalWrapper::JointMarginalWrapper(const Eigen::MatrixXd &P, const gtsam::KeyVector &keys) : P_joint_(P), keys_(keys) {}

  const Eigen::Block<const gtsam::Matrix> JointMarginalWrapper::operator()(gtsam::Key iVariable, gtsam::Key jVariable) const
  {
    return P_joint_.block(2 * iVariable, 2 * jVariable, 2, 2);
  }

  const Eigen::Block<const gtsam::Matrix> MarginalsWrapper::marginalCovariance(gtsam::Key variable) const
  {
    return P_full_.block(0, 0, 1, 1);
  }

  JointMarginalWrapper MarginalsWrapper::jointMarginalCovariance(const gtsam::KeyVector &variables) const
  {
    const int dim = 2 * variables.size() + 1; // Assumes first key is robot pose and rest landmarks
    gtsam::Matrix P_joint(dim, dim);

    // Insert state row on top
    P_joint.block<3, 3>(0, 0) = P_full_.block<3, 3>(0, 0);
    int i = 1, j = 3;
    for (int i = 1; i < variables.size(); i++)
    {
      int lj = gtsam::symbolIndex(variables[i]);
      P_joint.block<3, 2>(0, j) = P_full_.block<3, 2>(0, 3 + 2 * lj); // Assumes landmarks are zero indexed
      j += 2;
    }

    // Insert landmarks
    for (int i = 1; i < variables.size(); i++)
    {
      int li = gtsam::symbolIndex(variables[i]);
      for (int j = 1; j < variables.size(); j++)
      {
        int lj = gtsam::symbolIndex(variables[j]);
        P_joint.block<2, 2>(3 + 2 * (i - 1), 3 + 2 * (j - 1)) = P_full_.block<2, 2>(3 + 2 * li, 3 + 2 * lj);
      }
    }
    P_joint.triangularView<Eigen::Lower>() = P_joint.transpose();

    return JointMarginalWrapper(P_joint, variables);
  }

} // namespace jcbb