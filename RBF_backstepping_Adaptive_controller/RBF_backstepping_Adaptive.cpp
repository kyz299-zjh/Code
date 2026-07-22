#include "RBF_backstepping_Adaptive.h"


std::vector<double> AdaptiveRBFBackstepping::computePhi(double x) {
        vector<double> phi(n_rbf);
        for (int i = 0; i < n_rbf; i++) {
            double diff = x - c_rbf[i];
            phi[i] = exp(-diff * diff / (2.0 * b_rbf * b_rbf));
        }
        return phi;
    }

std::vector<double> AdaptiveRBFBackstepping::computedPhi(double x) {
        vector<double> dphi(n_rbf);
        for (int i = 0; i < n_rbf; i++) {
            double diff = x - c_rbf[i];
            dphi[i] = -diff / (b_rbf * b_rbf)
                     * exp(-diff * diff / (2.0 * b_rbf * b_rbf));
        }
        return dphi;
    }
double AdaptiveRBFBackstepping::dot(const vector<double>& a, const vector<double>& b) {
        double s = 0.0;
        for (size_t i = 0; i < a.size(); i++) s += a[i] * b[i];
        return s;
    }

AdaptiveRBFBackstepping::AdaptiveRBFBackstepping(double c1_, double c2_, double gamma_,
                            int nrbf_, double lo, double hi) {
        c1 = c1_;  c2 = c2_;  gamma = gamma_;
        kw = 5.0;                     // 增大鲁棒补偿增益
        sigma = 0.1;                  // σ-修正系数，防止权重漂移
        n_rbf = nrbf_;
        c_rbf.resize(n_rbf);
        for (int i = 0; i < n_rbf; i++)
            c_rbf[i] = lo + (hi - lo) * i / (n_rbf - 1);

        // RBF 宽度取中心间距的 2 倍，保证充分重叠
        b_rbf = 2.0 * (c_rbf[1] - c_rbf[0]);

        W.resize(n_rbf, 0.01);
    }

double AdaptiveRBFBackstepping::compute(double x1, double x2,
                   double y_ref, double dy_ref, double ddy_ref, double dt) {
        vector<double> phi  = computePhi(x1);
        vector<double> dphi = computedPhi(x1);

        double f_hat      = dot(W, phi);
        double z1         = x1 - y_ref;
        double alpha      = dy_ref - c1 * z1 - f_hat;
        double dalpha_dx1 = -c1 - dot(W, dphi);
        double z2         = x2 - alpha;

        // 正确的自适应调节变量
        double tau = z1 - z2 * dalpha_dx1;

        // 控制律（系统 dx2 = u）
        double phi_norm2 = dot(phi, phi);
        double u = -c2 * z2 - z1
                   + dalpha_dx1 * (x2 + f_hat)
                   + c1 * dy_ref + ddy_ref
                   - kw * z2 * phi_norm2;   // 鲁棒补偿项

        // RBF 权重更新（加入 σ-修正）
        for (int i = 0; i < n_rbf; i++)
            W[i] += dt * (gamma * tau * phi[i] - sigma * W[i]);

        return u;
    }