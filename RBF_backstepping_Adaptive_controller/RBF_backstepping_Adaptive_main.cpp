#include "RBF_backstepping_Adaptive.h"
#ifdef _WIN32
#include <windows.h>
#endif
using namespace std;

int main() {
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
#endif
    double dt = 0.001, T = 12.0;
    int N = (int)(T / dt);

    // 核心参数：低学习率 + σ修正
    AdaptiveRBFBackstepping ctrl.AdaptiveRBFBackstepping(5.0, 5.0, 1.0, 15, -3.0, 3.0);  // gamma=1.0

   
    double x1 = 0.0;
    double x2 = 1.0;              // dy_ref(0)

    double max_err = 0.0, steady = 0.0;

    for (int k = 0; k < N - 1; k++) {
        double t = k * dt;

        double y_ref   = sin(t);
        double dy_ref  = cos(t);
        double ddy_ref = -sin(t);
        
        double u = ctrl.compute(x1, x2, y_ref, dy_ref, ddy_ref, dt);

        // 真实系统：dx1 = x2 + f_true, dx2 = u
        double f_true   = 0.3 * x1 * sin(2.0 * x1);
        double x1_next  = x1 + dt * (x2 + f_true);
        double x2_next  = x2 + dt * u;

        x1 = x1_next;
        x2 = x2_next;

        double e = fabs(y_ref - x1);
        if (e > max_err)  max_err  = e;
        if (k > N - 2000 && e > steady) steady = e;
    }

    printf("瞬态峰值误差 = %.6f\n", max_err);
    printf("稳态误差     < %.6f\n", steady);
    return 0;
}