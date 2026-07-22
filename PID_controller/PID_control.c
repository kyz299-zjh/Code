#include<stdio.h>
#include<math.h>
#include<stdlib.h>

int main(){
    //仿真参数
    double dt = 0.001;
    double T = 10.0;
    int N = (int)(T/dt);

    //PID参数
    double Kp = 30;
    double Ki = 10;
    double Kd = 5;
    //PID状态
    double integral = 0.0;
    double e_prev = 0.0;
    
    //存储分配
    double* x1 = (double*)malloc(N*sizeof(double));
    double* x2 = (double*)malloc(N*sizeof(double));
    double* u = (double*)malloc(N*sizeof(double));
    
    //初始条件
    x1[0] = 0.0;
    x2[0] = 0.0;

   

    //主循环
    for(int k = 0;k<N-1;k++){
        //参考轨迹
        double y_ref = sin(k*dt);

        //跟踪误差
        double e = y_ref - x1[k];

        //积分累加
        integral += dt*e;

        //微分（差分近似）
        double derivative = (e-e_prev)/dt;
        e_prev = e;

        //PID输出
        u[k] = Kp*e + Ki*integral + Kd*derivative;

        //系统动态
        x1[k+1] = x1[k] + dt*(x2[k]);
        x2[k+1] = x2[k] + dt*(-2*x1[k] - 0.5*x2[k] + u[k]);

    }
    //输出结果
    printf("k*dt     x1      y_ref       error\n");
    printf("----------------------------------\n");
    for(int k = N-500;k<N;k+=50){
        double yr = sin(k*dt);
        printf("%.3f    %.6f    %.6f    %.6f\n",k*dt,x1[k],yr,yr-x1[k]);
    }

    //释放内存
    free(x1);free(x2);free(u);

    printf("\n PID control done.\n");
    return 0;
}