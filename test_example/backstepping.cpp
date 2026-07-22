#include<iostream>
#include<cmath>
#include<cstdlib>

using namespace std;

class backstepping{
    private:
        double c1,c2;
        double a,b;

    public:
        backstepping(double c1_,double c2_,double a_,double b_){
            c1 = c1_;
            c2 = c2_;
            a = a_;
            b = b_;
        }
        double compute(double x1,double x2,double y_ref,double dy_ref,double ddy_ref){
                double z1 = x1 - y_ref;
                double alpha = -c1*z1 + dy_ref;
                double dalpha = -c1*(x2-dy_ref) + ddy_ref;

                double z2 = x2 - alpha;
                double u = -z1 + a*x1 + b*x2 + dalpha -c2*z2; 
            
                return u;
        }

};

int main(){
    //仿真参数
    double dt = 0.001;
    double T = 10.0;
    int N = (int)(T/dt);

    backstepping back(5.0,5.0,2.0,0.5);
    
    double x1 = 0.0;
    double x2 = 0.0;
    double max_error = 0.0;
    double steady_error = 0.0;

    for(int k = 0;k<N-1;k++){
        double t = k*dt;

        double y_ref = sin(t);
        double dy_ref = cos(t);
        double ddy_ref = -sin(t);

        double u = back.compute(x1,x2,y_ref,dy_ref,ddy_ref);

        double x1_next = x1+dt*x2;
        double x2_next = x2+dt*(-2.0*x1-0.5*x2+u);
        
        x1 = x1_next;
        x2 = x2_next;
        
        double err = fabs(y_ref-x1);
        if(err>max_error) max_error = err;
        if (k > N - 2000) {
              if (err > steady_error) steady_error = err;
          }
    }
    
    printf("瞬态峰值误差 = %.6f\n", max_error);
    printf("稳态误差     < %.6f\n", steady_error);
    
    return 0;
}