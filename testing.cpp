#include "MarginalWrappers.h"
#include "file_parser.h"
#include <eigen3/Eigen/Core>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/inference/Symbol.h>
#include <iostream>
#include <glog/logging.h>


using namespace std;
using namespace Eigen;

using gtsam::symbol_shorthand::L;
using gtsam::symbol_shorthand::X;

int main(int argc, char **argv)
{
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();

    MatrixXd P = txt2mat("./xPtests/test_sim/P.txt");
    cout << P << "\n";
    MatrixXd P_expected(3+2*2, 3+2*2);
    P_expected << P.block<3+2, 3+2>(0,0), P.block<3+2, 2>(0,7),
 P.block<2, 3+2>(7,0), P.block<2,2>(7,7);
    cout << "\nexpected Pjoint:\n" << P_expected << "\n";
    gtsam::KeyVector keys = {X(0), L(0), L(2)};
    jcbb::MarginalsWrapper marginals(P);
    jcbb::JointMarginalWrapper joint_marginals = marginals.jointMarginalCovariance(keys);
    cout << "\noutput joint:\n" << joint_marginals.fullMatrix() << "\n";
}