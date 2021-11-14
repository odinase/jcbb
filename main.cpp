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
#include <glog/logging.h>
#include <algorithm>


using namespace std;
using namespace Eigen;


jcbb::Hypothesis assos2hyp(const Eigen::MatrixXd& assos, const Eigen::MatrixXd& z, const Eigen::MatrixXd& zbar, const MatrixXd& S) {
  std::vector<jcbb::Association::shared_ptr> a;
  for (int i = 0; i < assos.rows(); i++) {
    if (assos(i,0) > 0) { 
    a.push_back(std::make_shared<jcbb::Association>(i, assos(i,0)-1));
    } else {
          a.push_back(std::make_shared<jcbb::Association>(i));
    }
  }
  jcbb::Marginals Sm(S);
  jcbb::Hypothesis h(a, 0);
  double nis = jcbb::jc(h, z, zbar, Sm);
  h.nis = nis;
  return h;
}


void run_test(const std::string& test_folder, double ic_prob, double jc_prob) {
  MatrixXd S = txt2mat("./testing/" + test_folder + "/S.txt");
  MatrixXd z_mat = txt2mat("./testing/"+test_folder+"/z.txt");
  Map<VectorXd> z(z_mat.data(), z_mat.rows());
  MatrixXd zbar_mat = txt2mat("./testing/"+test_folder+"/zbar.txt");
  Map<VectorXd> zbar(zbar_mat.data(), zbar_mat.rows());
  MatrixXd abest = txt2mat("./testing/"+test_folder+"/abest.txt");

  jcbb::Marginals S_marginal(S);

  jcbb::Hypothesis h = jcbb::jcbb(z, zbar, S_marginal, jc_prob, ic_prob);
  cout << "\n********* TEST: " << test_folder << " RESULTS: **************\n";
  cout << "\n************* JCBB ****************\n";
  for (const auto &a : h.associations)
  {
    if (a->associated())
    {
      cout << "associated meas " << a->measurement << " with landmark " << *a->landmark << "\n";
    }
    else
    {
      cout << "meas " << a->measurement << " unassociated\n";
    }
  }
  cout << "nis: " << h.nis << "\n";
  cout << "\n************* GROUND TRUTH ****************\n";
  for (int i = 0; i < abest.rows(); i++) {
    if (abest(i,0) > 0) {
    cout << "Measurement " << i << " associated with landmark " << abest(i,0) -1 << "\n"; 
    } else {
    cout << "Measurement " << i << " unassociated\n"; 
    }
  }
  jcbb::Hypothesis h_gt = assos2hyp(abest, z, zbar, S);
  cout << "nis: " << h_gt.nis << "\n";
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

  std::vector<std::string> test_folders_sim = {
    "test1", "test2","test3"
  };
  cout << "*********** TESTS FOR SIM ***************\n";
  for (const auto& test_folder : test_folders_sim) {
    run_test(test_folder, ic_prob, jc_prob);
  }

  std::vector<std::string> test_folders_vp = {
    "test4","test5"
  };

  MatrixXd alphas = txt2mat("./testing/matrix.txt");
  jc_prob = alphas(0, 0);
  ic_prob = alphas(0, 1);

  cout << "*********** TESTS FOR VP ***************\n";
  for (const auto& test_folder : test_folders_vp) {
    run_test(test_folder, ic_prob, jc_prob);
  }
}