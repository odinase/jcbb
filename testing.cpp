#include "MarginalMocks.h"
#include "file_parser.h"
#include <eigen3/Eigen/Core>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/inference/Symbol.h>
#include <iostream>
#include <string>
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

//     MatrixXd P = txt2mat("./xPtests/test_sim/P.txt");
//     cout << P << "\n";
//     MatrixXd P_expected(3+2*2, 3+2*2);
//     P_expected << P.block<3+2, 3+2>(0,0), P.block<3+2, 2>(0,7),
//  P.block<2, 3+2>(7,0), P.block<2,2>(7,7);
//     cout << "\nexpected Pjoint:\n" << P_expected << "\n";
//     gtsam::KeyVector keys = {X(0), L(0), L(2)};
//     jcbb::MarginalsMock marginals(P);
//     jcbb::JointMarginalMock joint_marginals = marginals.jointMarginalCovariance(keys);
//     cout << "\noutput joint:\n" << joint_marginals.fullMatrix() << "\n";
//     cout << "\nlandmark "<< gtsam::symbolIndex(L(2)) << ":\n" << joint_marginals.at(L(2), L(2)) << "\n";
//   cout << "\npose and landmark "<< gtsam::symbolIndex(L(2)) << ":\n" << joint_marginals.at(X(0), L(2)) << "\n";
//   cout << "\nlandmark " << gtsam::symbolIndex(L(0)) << " and landmark "<< gtsam::symbolIndex(L(2)) << ":\n" << joint_marginals.at(L(0), L(2)) << "\n";

  
//     gtsam::KeyVector keys2 = {X(0)};
//     jcbb::JointMarginalMock joint_marginals2 = marginals.jointMarginalCovariance(keys2);
//     cout << "just pose:\n" << joint_marginals2.fullMatrix() << "\n";


//     // gtsam::KeyVector keys3 = {L(1)};
//     // jcbb::JointMarginalMock joint_marginals3 = marginals.jointMarginalCovariance(keys3);
//     // cout << "just landmark 1:\n" << joint_marginals3.fullMatrix() << "\n";
}