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
#include "jcbb/utils.h"
#include <tuple>


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
// std::tuple<MatrixXd, MatrixXd, MatrixXd, VectorXd> calc_mats(const jcbb::Hypothesis& h, const jcbb::JCBB& jcbb) {
//       int N = h.num_associations();
//     int n = jcbb::State::dimension;
//     int m = jcbb::Landmark::RowsAtCompileTime;
//     int d = jcbb::Measurement::RowsAtCompileTime; // Measurement dim
  
//       gtsam::KeyVector joint_states;
//     // joint_states.reserve(measurement_landmark_associations.size()+1);
//     joint_states.push_back(jcbb.x_key_);
//     int num_associated_meas_to_lmk = 0;
//       for (const auto asso: h.associations()) {
//       if (asso->associated()) {
//         num_associated_meas_to_lmk++;
//       joint_states.push_back(*asso->landmark);
//       }
//     }

//     Eigen::MatrixXd Pjoint = jcbb.marginals_.jointMarginalCovariance(joint_states).fullMatrix();

//     Eigen::MatrixXd H = Eigen::MatrixXd::Zero(num_associated_meas_to_lmk*d, n + num_associated_meas_to_lmk*m);
//     Eigen::MatrixXd R = Eigen::MatrixXd::Zero(num_associated_meas_to_lmk*d, num_associated_meas_to_lmk*d);

//     Eigen::VectorXd innov(N * d);
//     int k = 0, j = 0;

//     for (const auto &a : h.associations())
//     {
//       if (a->associated()) {
//       innov.segment(k, d) = a->error;
//       innov(k + 1) = jcbb::wrapToPi(innov(k + 1));
//       H.block(k, 0, d, n) = a->Hx;
//       H.block(k, n + j, d, m) = a->Hl;

//       // Adding R might be done more cleverly
//       R.block(k, k, d, d) = jcbb.meas_noise_->sigmas().array().square().matrix().asDiagonal();

//       k += d;
//       j += m;
//       }
//     }

//     Eigen::MatrixXd S = H*Pjoint*H.transpose() + R;

//     return {H, Pjoint, S, innov};
// }


// void calc_nis_from_gt_and_compare(const std::string& folder, const jcbb::Hypothesis& h, const jcbb::JCBB& jcbb) {
//         int N = h.num_associations();
//         std::cout << "Num associations: " << N << "\n";
//     int n = jcbb::State::dimension;
//     int m = jcbb::Landmark::RowsAtCompileTime;
//     int d = jcbb::Measurement::RowsAtCompileTime; // Measurement dim

//   MatrixXd S = txt2mat("./xPtests/" + folder + "/S.txt");
//   MatrixXd H = txt2mat("./xPtests/" + folder + "/H.txt");
//   MatrixXd a_mat = txt2mat("./xPtests/" + folder + "/a.txt");
//   MatrixXd P = txt2mat("./xPtests/" + folder + "/P.txt");
//   MatrixXd z_mat = txt2mat("./xPtests/" + folder + "/z.txt");
//   MatrixXd zbar_mat = txt2mat("./xPtests/" + folder + "/zpred.txt");
//   Map<VectorXd> z(z_mat.data(), z_mat.rows());
//   Map<VectorXd> zbar(zbar_mat.data(), zbar_mat.rows());
//   Map<VectorXd> a(a_mat.data(), a_mat.rows());

//   VectorXd innov_gt(N*d);
//   int ki = 0;
//   for (int i = 0; i < a.size(); i++) {
//     if (a(i) == 0) {
//       continue;
//     }
//     int mi = i;
//     int li = a(i) - 1;
//     innov_gt.segment(ki, d) = z.segment(mi, d) - zbar.segment(li, d);
//     ki += d;
//   }

//   // Eigen::MatrixXd Pjoint(3 + N * d, 3 + N * d);
// //   int assos = 0;
// // for (int i = 0; i < a.size(); i++) {
// //   if (a(i) != 0) {
// //     assos++;
// //   }
// // }
// //     Eigen::MatrixXd Pjoint = P.block<3,3>(0,0);
// //     int ki =3;
// //     for (int i = 0; i < a.size(); i++) {
// //       if (a(i) == 0) {
// //         continue;
// //       }
// //       int li = a(i) - 1;
// //         Pjoint.block<3,2>(0, ki) = P.block<3,2>(0,li);
// //         ki++;
// //     }
//   Eigen::MatrixXd Sjoint(N * d, N * d);
//     Eigen::MatrixXd Ha = Eigen::MatrixXd::Zero(N*d, n + N*m);


//     int k = 0, j = 0;
//     for (int i = 0; i < a.size(); i++)
//     {
//       if (a(i) > 0) {
//         int li = a(i) - 1;
//       Ha.block(k, 0, d, n) = H.block<2,3>(i,0);

//       Ha.block(k, n + j, d, m) = H.block<2,2>(i, 3 + li);

//       // Adding R might be done more cleverly
//       // R.block(k, k, d, d) = jcbb.meas_noise_->sigmas().array().square().matrix().asDiagonal();

//       k += d;
//       j += m;
//       }
//     }



//   std::cout << "Size: " << N*d << ", " << N*d << "\n";
// ki = 0;
//     for (int i = 0; i < a.size(); i++)
//     {
// int kj = ki;
//       if (a(i) == 0) {
//         continue;
//       }
//       int li = a(i) - 1;
//       for (int j = i; j < a.size(); j++)
//       {
//         if (a(j) == 0) {
//           continue;
//         }
//         int lj = a(j) - 1;
//         // Pjoint.block(3 + d * ki, 3 + d * kj, d, d) = Pjoint.block<2,2>(li, lj);
//         S.block<2,2>(li, lj);
//         Sjoint.block(d * ki, d * kj, d, d);
//         Sjoint.block(d * ki, d * kj, d, d) = S.block<2,2>(li, lj);
//         kj++;
//       }
//       ki++;
//     }

//     // Pjoint.triangularView<Eigen::Lower>() = Pjoint.transpose();
//     Sjoint.triangularView<Eigen::Lower>() = Sjoint.transpose();

//     auto [Hjcbb, Pjoint_jcbb, Sjcbb, innov_jcbb] = calc_mats(h, jcbb);
//         std::cout << "H size: " << Ha.rows() << ", " << H.cols() << "\n";
//         std::cout << "Hjcbb size: " << Hjcbb.rows() << ", " << Hjcbb.cols() << "\n";
//         std::cout << "Sjoint size: " << Sjoint.rows() << ", " << Sjoint.cols() << "\n";
//         std::cout << "Sjcbb size: " << Sjcbb.rows() << ", " << Sjcbb.cols() << "\n";
//         std::cout << "innov_gt size: " << innov_gt.size() << "\n";
//         std::cout << "innov_jcbb size: " << innov_jcbb.rows() << "\n";
// cout << "Hx:\n";
//   cout << "Truth:\n" << Ha.block<2, 3>(0,0) << "\n\nJCBB:\n" << Hjcbb.block<2,3>(0,0) << "\n";

// cout << "Hl:\n";
//   cout << "Truth:\n" << Ha.block<2, 2>(0,3) << "\n\nJCBB:\n" << Hjcbb.block<2,3>(0,3) << "\n";


//     double Hsimilar = (Ha - Hjcbb).array().abs().sum();
//     double innov_similar = (innov_gt - innov_jcbb).array().abs().mean();
//     double Ssimilar = (Sjoint - Sjcbb).array().abs().mean();

//     std::cout << "Hsimilar: " << Hsimilar; 
// std::cout    << "\ninnov_similar: " << innov_similar
//     << "\nSsimilar: " << Ssimilar
//     << "\n";
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
  // calc_nis_from_gt_and_compare(test_folder, h, jcbb);
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