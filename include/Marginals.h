#ifndef MARGINALS_H
#define MARGINALS_H

#include <eigen3/Eigen/Core>

namespace jcbb
{

class Marginals
{
public:
    explicit Marginals(const Eigen::MatrixXd &S) : S_(S) {}
    const Eigen::Block<const Eigen::MatrixXd> operator()(int i, int j) const;
    const Eigen::Block<const Eigen::MatrixXd> at(int i, int j) const { return (*this)(i, j); }

private:
    Eigen::MatrixXd S_;
};

} // namespace jcbb

#endif // MARGINALS_H