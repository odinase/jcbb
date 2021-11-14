#include <unordered_set>

#include "jcbb/Hypothesis.h"

namespace jcbb
{
    double Association::nis(const Eigen::VectorXd &z, const Eigen::VectorXd &zbar, const Marginals &S)
    {
        if (!associated())
        {
            return std::numeric_limits<double>::infinity();
        }
        Eigen::VectorXd innov = z.segment(2 * measurement, 2) - zbar.segment(2 * (*landmark), 2);
        Eigen::MatrixXd Sij = S.at(*landmark, *landmark);
        return innov.transpose() * Sij.llt().solve(innov);
    }

    int Hypothesis::num_associations() const
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

    int Hypothesis::num_measurements() const
    {
        return associations.size();
    }

    std::unordered_set<int> Hypothesis::associated_landmarks() const
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
    bool Hypothesis::operator<(const Hypothesis &rhs) const
    {
        return num_associations() > rhs.num_associations() || (num_associations() == rhs.num_associations() && nis < rhs.nis);
    }

    // Needed for min heap
    bool Hypothesis::operator>(const Hypothesis &rhs) const
    {
        return num_associations() < rhs.num_associations() || (num_associations() == rhs.num_associations() && nis > rhs.nis);
    }

    // Added for completion of comparision operators
    bool Hypothesis::operator==(const Hypothesis &rhs) const
    {
        return num_associations() == rhs.num_associations() && nis == rhs.nis;
    }

    // Added for completion of comparision operators
    bool Hypothesis::operator<=(const Hypothesis &rhs) const
    {
        return *this < rhs || *this == rhs;
    }

    // Added for completion of comparision operators
    bool Hypothesis::operator>=(const Hypothesis &rhs) const
    {
        return *this > rhs || *this == rhs;
    }

    bool Hypothesis::better_than(const Hypothesis &other) const
    {
        return *this < other;
    }

    static Hypothesis empty_hypothesis()
    {
        return Hypothesis{{}, std::numeric_limits<double>::infinity()};
    }

    void Hypothesis::extend(const Association::shared_ptr &a)
    {
        associations.push_back(a);
    }

    Hypothesis Hypothesis::extended(const Association::shared_ptr &a) const
    {
        Hypothesis h(*this);
        h.extend(a);
        return h;
    }

} // namespace jcbb
