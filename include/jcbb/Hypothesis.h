#ifndef HYPOTHESIS_H
#define HYPOTHESIS_H

#include <vector>
#include <optional>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Cholesky>
#include <memory>
#include <unordered_set>
#include "MarginalWrappers.h"


namespace jcbb
{

    struct Association
    {
        Association(int m, std::optional<int> l = {}) : measurement(m), landmark(l) {}
        typedef std::shared_ptr<Association> shared_ptr;
        int measurement;
        std::optional<int> landmark;
        bool associated() const { return bool(landmark); }
        double nis(const Eigen::VectorXd &z, const Eigen::VectorXd &zbar, const Marginals &S);
    };


    class Hypothesis
    {
        public:
        Hypothesis(const std::vector<Association::shared_ptr> &associations, double nis) : associations(associations), nis(nis) {}
        double nis;
        std::vector<Association::shared_ptr> associations;
        int num_associations() const;
        int num_measurements() const;

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
    };


    
} // namespace jcbb

#endif // HYPOTHESIS_H