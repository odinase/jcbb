#ifndef JCBB_H
#define JCBB_H

#include <vector>
#include <queue>
#include <memory>
#include <limits>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Cholesky>
#include <optional>
#include <numeric>
#include <unordered_set>
#include <iostream>

namespace jcbb
{
    template <class T>
    using MinHeap = std::priority_queue<T, std::vector<T>, std::greater<T>>;
    double chi2inv(double p, unsigned int dim);

    class Marginals
    {
    public:
        explicit Marginals(const Eigen::MatrixXd &S) : S_(S) {}
        const Eigen::Block<const Eigen::MatrixXd> operator()(int i, int j) const;
        const Eigen::Block<const Eigen::MatrixXd> at(int i, int j) const { return (*this)(i, j); }

    private:
        Eigen::MatrixXd S_;
    };

    struct Association
    {
        Association(int m, std::optional<int> l = {}) : measurement(m), landmark(l) {}
        typedef std::shared_ptr<Association> shared_ptr;
        int measurement;
        std::optional<int> landmark;
        bool associated() const { return bool(landmark); }
        double nis(const Eigen::VectorXd &z, const Eigen::VectorXd &zbar, const Marginals &S)
        {
            if (!associated())
            {
                return std::numeric_limits<double>::infinity();
            }
            // std::cout << "meas idx: " << measurement << "\nlandmark idx: " << *landmark << "\n";
            // std::cout << "measurement:\n" << z.segment(2*measurement, 2) << "\n";
            // std::cout << "landmark:\n" << zbar.segment(2*(*landmark), 2) << "\n";
            Eigen::VectorXd innov = z.segment(2 * measurement, 2) - zbar.segment(2 * (*landmark), 2);
            Eigen::MatrixXd Sij = S.at(*landmark, *landmark);
            return innov.transpose() * Sij.llt().solve(innov);
        }
    };

    struct Hypothesis
    {
        Hypothesis(const std::vector<Association::shared_ptr> &associations, double nis) : associations(associations), nis(nis) {}
        double nis;
        std::vector<Association::shared_ptr> associations;
        int num_associations() const
        {
            int n = 0;
            for (const auto &a : associations)
            {
                if (a->associated())
                {
                    n++;
                }
            }
            return n;
        }

        int num_measurements() const
        {
            return associations.size();
        }

        std::unordered_set<int> associated_landmarks() const
        {
            std::unordered_set<int> landmarks;
            for (const auto &a : associations)
            {
                if (a->associated())
                {
                    landmarks.insert(*a->landmark);
                }
            }
            return landmarks;
        }
        // Needed for min heap
        bool operator<(const Hypothesis &rhs) const
        {
            return num_associations() > rhs.num_associations() || (num_associations() == rhs.num_associations() && nis < rhs.nis);
        }

        // Needed for min heap
        bool operator>(const Hypothesis &rhs) const
        {
            return num_associations() < rhs.num_associations() || (num_associations() == rhs.num_associations() && nis > rhs.nis);
        }

        // Added for completion of comparision operators
        bool operator==(const Hypothesis &rhs) const
        {
            return num_associations() == rhs.num_associations() && nis == rhs.nis;
        }

        // Added for completion of comparision operators
        bool operator<=(const Hypothesis &rhs) const
        {
            return *this < rhs || *this == rhs;
        }

        // Added for completion of comparision operators
        bool operator>=(const Hypothesis &rhs) const
        {
            return *this > rhs || *this == rhs;
        }

        bool better_than(const Hypothesis &other) const
        {
            return *this < other;
        }

        static Hypothesis empty_hypothesis()
        {
            return Hypothesis{{}, std::numeric_limits<double>::infinity()};
        }

        void extend(const Association::shared_ptr &a)
        {
            associations.push_back(a);
        }

        Hypothesis extended(const Association::shared_ptr &a) const
        {
            Hypothesis h(*this);
            h.extend(a);
            return h;
        }
        // std::vector<std::pair<int, int>> measurement_landmark_associations() const
        // {
        //     std::vector<std::pair<int, int>> asso_pairs;
        //     for (const auto &a : associations)
        //     {
        //         std::pair<int, int> asso_pair;
        //         if (a->associated())
        //         {
        //             asso_pair = {
        //                 a->measurement,
        //                 *a->landmark};
        //         }
        //         asso_pairs.push_back(asso_pair);
        //     }
        // }
    };

    Hypothesis jcbb(const Eigen::VectorXd &z, const Eigen::VectorXd &zbar, const Marginals &S, double jc_prob, double ic_prob);
double jc(const Hypothesis &h, const Eigen::VectorXd &z, const Eigen::VectorXd &zbar, const Marginals &S);

} // namespace jcbb

#endif // JCBB_H