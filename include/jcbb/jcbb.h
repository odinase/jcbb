#ifndef JCBB_H
#define JCBB_H

#include <vector>
#include <queue>
#include <limits>
#include <eigen3/Eigen/Core>
#include <numeric>
#include <unordered_set>
#include <unordered_map>
#include <iostream>

#include <gtsam/base/FastVector.h>

#include "MarginalWrappers.h"
#include "jcbb/Hypothesis.h"

namespace jcbb
{
    template <class T>
    using FastMinHeap = std::priority_queue<T, gtsam::FastVector<T>, std::greater<T>>;
    double chi2inv(double p, unsigned int dim);

    class JCBB
    {
    public:
        JCBB();

    private:
        Hypothesis jcbb(const Eigen::VectorXd &z, const Eigen::VectorXd &zbar, const Marginals &S, double jc_prob, double ic_prob);
        double jc(const Hypothesis &h, const Eigen::VectorXd &z, const Eigen::VectorXd &zbar, const Marginals &S);
        Hypothesis jcbb(const Eigen::VectorXd &z, const Eigen::VectorXd &zbar, const Marginals &S, double jc_prob, double ic_prob);
        std::vector<Hypothesis> successors(const Hypothesis &h, const Eigen::VectorXd &z, const Eigen::VectorXd &zbar, const Marginals &S, double ic_prob);
        bool feasible(const Hypothesis &h, double jc_prob, const Eigen::VectorXd &z, const Eigen::VectorXd &zbar, const Marginals &S);
        bool prunable(int tot_num_measurements, const Hypothesis &h, const Hypothesis &best);

        // std::unordered_map<>
    };

} // namespace jcbb

#endif // JCBB_H