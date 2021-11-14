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


#include "Marginals.h"
#include "jcbb/Hypothesis.h"

namespace jcbb
{
    template <class T>
    using MinHeap = std::priority_queue<T, std::vector<T>, std::greater<T>>;
    double chi2inv(double p, unsigned int dim);

    class JCBB
    {
    public:
        JCBB();

    private:
        Hypothesis jcbb(const Eigen::VectorXd &z, const Eigen::VectorXd &zbar, const Marginals &S, double jc_prob, double ic_prob);
        double jc(const Hypothesis &h, const Eigen::VectorXd &z, const Eigen::VectorXd &zbar, const Marginals &S);


    };

} // namespace jcbb

#endif // JCBB_H