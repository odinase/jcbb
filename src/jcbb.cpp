#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Cholesky>
#include <boost/math/distributions.hpp>
#include <cmath>
#include <vector>
#include <memory>
#include <unordered_set>
#include <utility>

#include "jcbb/jcbb.h"
#include "jcbb/utils.h"

namespace jcbb
{

  double chi2inv(double p, unsigned int dim)
  {
    boost::math::chi_squared dist(dim);
    return quantile(dist, p);
  }

  bool prunable(int tot_num_measurements, const Hypothesis &h, const Hypothesis &best)
  {
    int num_associations = h.num_associations();
    int num_associated_measurements = h.num_measurements();
    int remaining_measurements = tot_num_measurements - num_associated_measurements;
    int potential_num_associations = num_associations + remaining_measurements;

    // If we can't beat the current number of associations, we give up
    return potential_num_associations <= best.num_associations();
  }

  double jc(const Hypothesis &h, const Eigen::VectorXd &z, const Eigen::VectorXd &zbar, const Marginals &S)
  {
    int N = h.num_associations();
    // std::cout << "num associations: " << N << "\n";
    int d = 2; // Measurement dim
    std::vector<std::pair<int, int>> measurement_landmark_associations;
    for (const auto &a : h.associations)
    {
      if (a->associated())
      {
        measurement_landmark_associations.push_back({a->measurement,
                                                     *a->landmark});
      }
    }
    Eigen::MatrixXd Sjoint(N * d, N * d);
    for (int i = 0; i < measurement_landmark_associations.size(); i++)
    {
      int li = measurement_landmark_associations[i].second;
      for (int j = i; j < measurement_landmark_associations.size(); j++)
      {
        int lj = measurement_landmark_associations[j].second;
        Sjoint.block(d * i, d * j, d, d) = S.at(li, lj);
      }
    }

    Sjoint.triangularView<Eigen::Lower>() = Sjoint.transpose();
    Eigen::VectorXd innov(N * d);
    int k = 0;
    for (const auto &a : measurement_landmark_associations)
    {
      int m = a.first;
      int l = a.second;
      innov.segment(k, d) = z.segment(d * m, d) - zbar.segment(d * l, d);
      innov(k + 1) = wrapToPi(innov(k + 1));
      k += d;
    }

    double nis = innov.transpose() * Sjoint.llt().solve(innov);
    return nis;
  }

  bool feasible(const Hypothesis &h, double jc_prob, const Eigen::VectorXd &z, const Eigen::VectorXd &zbar, const Marginals &S)
  {
    int N = h.num_associations();
    if (N == 0)
    {
      return true;
    }
    double nis = jc(h, z, zbar, S);
    return nis < chi2inv(1 - jc_prob, N * 2);
  }

  bool exists(const std::unordered_set<int> &s, int k)
  {
    return s.find(k) != s.end();
  }

  std::vector<Hypothesis> successors(const Hypothesis &h, const Eigen::VectorXd &z, const Eigen::VectorXd &zbar, const Marginals &S, double ic_prob)
  {
    int curr_measurement = h.num_measurements() - 1; // We use zero indexing, so needs to subtract 1
    int next_measurement = curr_measurement + 1;
    if ((next_measurement + 1) > z.size() / 2)
    { // We have no more measurements to check, so no successors
      return {};
    }
    std::vector<Hypothesis> hypothesis_successors;

    // Add no association
    Hypothesis successor = h.extended(std::make_shared<Association>(next_measurement));
    hypothesis_successors.push_back(successor);

    std::unordered_set<int> associated_landmarks = h.associated_landmarks();
    int num_landmarks = zbar.size() / 2;
    for (int l = 0; l < num_landmarks; l++)
    {
      if (exists(associated_landmarks, l))
      {
        continue;
      }
      Association a(next_measurement, l);
      double nis = a.nis(z, zbar, S);
      double inv = chi2inv(1 - ic_prob, 2);
      if (nis < inv)
      {
        Hypothesis successor = h.extended(std::make_shared<Association>(a));
        double joint_nis = jc(successor, z, zbar, S);
        successor.nis = joint_nis;
        hypothesis_successors.push_back(successor);
      }
    }

    return hypothesis_successors;
  }

  Hypothesis jcbb(const Eigen::VectorXd &z, const Eigen::VectorXd &zbar, const Marginals &S, double jc_prob, double ic_prob)
  {
    int m = z.size() / 2;
    Hypothesis best_hypothesis{Hypothesis::empty_hypothesis()};
    MinHeap<Hypothesis> min_heap;
    min_heap.push(Hypothesis::empty_hypothesis());

    while (!min_heap.empty())
    {
      Hypothesis hypothesis = min_heap.top();
      min_heap.pop();

      if (!prunable(m, hypothesis, best_hypothesis))
      {
        if (feasible(hypothesis, jc_prob, z, zbar, S) && hypothesis.better_than(best_hypothesis))
        {
          best_hypothesis = hypothesis;
        }
        std::vector<Hypothesis> hypothesis_successors = successors(hypothesis, z, zbar, S, ic_prob);
        for (const auto &h : hypothesis_successors)
        {
          min_heap.push(h);
        }
      }
    }

    return best_hypothesis;
  }

} // namespace jcbb
