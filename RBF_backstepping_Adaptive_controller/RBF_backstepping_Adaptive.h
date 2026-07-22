#ifndef RBF_BACKSTEPPING_ADAPTIVE_H
#define RBF_BACKSTEPPING_ADAPTIVE_H

#include <iostream>
#include <cmath>
#include <vector>
using namespace std;

class AdaptiveRBFBackstepping {
private:
    double c1, c2, gamma, kw, sigma;   // sigma 为阻尼系数
    int n_rbf;
    double b_rbf;
    vector<double> c_rbf;
    vector<double> W;

    std::vector<double> computePhi(double x);
    std::vector<double> computedPhi(double x);
    double dot(const std::vector<double>& a, const std::vector<double>& b);


public:
    AdaptiveRBFBackstepping(double c1_, double c2_, double gamma_,
                            int nrbf_, double lo, double hi);
    double compute(double x1, double x2,
                   double y_ref, double dy_ref, double ddy_ref, double dt);   
    };  

#endif