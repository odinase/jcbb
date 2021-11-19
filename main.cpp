#include <eigen3/Eigen/Core>
#include <iostream>
#include <fstream>
#include <memory>
#include <vector>
#include <cmath>
#include <functional>
#include <string>
#include <sstream>
#include <optional>
#include <variant>
#include <random>
#include <queue>
#include <type_traits>
#include "file_parser.h"
#include "jcbb/jcbb.h"
#include "MarginalMocks.h"
#include <glog/logging.h>
#include <algorithm>
#include <gtsam/base/FastVector.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>


using namespace std;
using namespace Eigen;
using gtsam::symbol_shorthand::X;
using gtsam::symbol_shorthand::L;


// jcbb::Hypothesis assos2hyp(const Eigen::MatrixXd& assos, const jcbb::JCBB& jcbb) {
//   gtsam::FastVector<jcbb::Association::shared_ptr> a;
//   for (int i = 0; i < assos.rows(); i++) {
//     if (assos(i,0) > 0) { 
//     a.push_back(std::make_shared<jcbb::Association>(i, assos(i,0)-1));
//     } else {
//           a.push_back(std::make_shared<jcbb::Association>(i));
//     }
//   }
//   jcbb::Hypothesis h(a, 0);
//   double nis = jcbb.joint_compatability(h);
//   h.set_nis(nis);
//   return h;
// }


void run_test(const std::string& test_folder, double ic_prob, double jc_prob, gtsam::noiseModel::Diagonal::shared_ptr noise_model) {
  MatrixXd P = txt2mat("./xPtests/" + test_folder + "/P.txt");
  MatrixXd z_mat = txt2mat("./xPtests/"+test_folder+"/z.txt");

  gtsam::FastVector<gtsam::Vector2> measurements;
  for (int i = 0; i < z_mat.rows(); i+=2) {
    measurements.push_back(z_mat.block<2,1>(i,0));
  }

  gtsam::Values estimates;

  MatrixXd eta_mat = txt2mat("./xPtests/"+test_folder+"/eta.txt");
  gtsam::Pose2 x_pose(eta_mat(0,0), eta_mat(1,0), eta_mat(2,0));
  estimates.insert(X(0), x_pose);

  for (int i = 3, j = 0; i < eta_mat.rows(); i+=2) {
    gtsam::Point2 lmk(eta_mat(i,0), eta_mat(i+1,0));
    estimates.insert(L(j), lmk);
    j++;
  }

  MatrixXd abest = txt2mat("./xPtests/"+test_folder+"/a.txt");

  jcbb::MarginalsMock P_marginal(P);

  Eigen::MatrixXd sensorOffset = txt2mat("./xPtests/"+test_folder+"/sensorOffset.txt");

  jcbb::JCBB jcbb(estimates, P_marginal, measurements, noise_model, sensorOffset, jc_prob, ic_prob);
  jcbb::Hypothesis h = jcbb.jcbb();
  cout << "\n********* TEST: " << test_folder << " RESULTS: **************\n";
  cout << "\n************* JCBB ****************\n";
  for (const auto &a : h.associations())
  {
    if (a->associated())
    {
      cout << "associated meas " << a->measurement << " with landmark " << gtsam::symbolIndex(*a->landmark) << "\n";
    }
    else
    {
      cout << "meas " << a->measurement << " unassociated\n";
    }
  }
  cout << "nis: " << h.get_nis() << "\n";
  cout << "\n************* GROUND TRUTH ****************\n";
  for (int i = 0; i < abest.rows(); i++) {
    if (abest(i,0) > 0) {
    cout << "Measurement " << i << " associated with landmark " << abest(i,0) -1 << "\n"; 
    } else {
    cout << "Measurement " << i << " unassociated\n"; 
    }
  }

  Eigen::MatrixXd nis_gt_mat = txt2mat("./xPtests/"+test_folder+"/nis.txt");
  double nis_gt = nis_gt_mat(0,0);
  // jcbb::Hypothesis h_gt = assos2hyp(abest, z, zbar, S);
  cout << "nis: " << nis_gt << "\n";
  // std::sort(h.associations.begin(), h.associations.end(), [](jcbb::Association::shared_ptr lhs, jcbb::Association::shared_ptr rhs) {return lhs->measurement < rhs->measurement;});
  //   cout << "\n************* comparison ****************\n";
  //   for 
}


int main(int argc, char **argv)
{
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();

  // first is for joint compatibility, second is individual
  MatrixXd jc_prob_mat = txt2mat("./testing/alpha1.txt");
  MatrixXd ic_prob_mat = txt2mat("./testing/alpha2.txt");
  double jc_prob = jc_prob_mat(0, 0);
  double ic_prob = ic_prob_mat(0, 0);

  // R = diag([0.06 2*pi/180].^2);
  gtsam::Vector sigmas(2);
  sigmas << 0.06, 2*M_PI/180.0;
  gtsam::noiseModel::Diagonal::shared_ptr noise = gtsam::noiseModel::Diagonal::Sigmas(sigmas);

  std::vector<std::string> test_folders = {
    "test_sim", "test_vp"
  };
  cout << "*********** TEST FOR SIM ***************\n";
  run_test(test_folders[0], ic_prob, jc_prob, noise);
  
  MatrixXd alphas = txt2mat("./testing/matrix.txt");
  jc_prob = alphas(0, 0);
  ic_prob = alphas(0, 1);
  // R = diag([1e-2, 1.1e-2]);
  gtsam::Vector sigmas_vp(2);
  sigmas_vp << sqrt(1e-2), sqrt(1.1e-2);
  noise = gtsam::noiseModel::Diagonal::Sigmas(sigmas_vp);
  // Eigen::MatrixXd R = noise->sigmas().array().square().matrix().asDiagonal();
  // cout << "R vp is\n" << R << "\n";

  cout << "*********** TESTS FOR VP ***************\n";
  run_test(test_folders[1], ic_prob, jc_prob, noise);
}