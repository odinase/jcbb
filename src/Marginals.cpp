#include "Marginals.h"
#include <eigen3/Eigen/Core>

namespace jcbb
{
  const Eigen::Block<const Eigen::MatrixXd> Marginals::operator()(int i, int j) const
  {
    return S_.block(2 * i, 2 * j, 2, 2);
  }
} // namespace jcbb