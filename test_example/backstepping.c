#include<stdio.h>
#include<math.h>
#include<stdlib.h>

int main(){
    //仿真参数
    double dt = 0.001;
    double T = 10.0;
    int N = (int)(T/dt);

    //存储分配
    double* x1 = (double*)malloc(N*sizeof(double));
    double* x2 = (double*)malloc(N*sizeof(double));
    double* u = (double*)malloc(N*sizeof(double));
    
    //初始条件
    x1[0] = 0.0;
    x2[0] = 0.0;
    double c1 = 5.0;
    double c2 = 5.0;

    //主循环
    for(int k = 0;k<N-1;k++){
        double t = k*dt;

        //参考轨迹
        double y_ref = sin(t);           
        double dy_ref = cos(t);
        double ddy_ref = -sin(t);

        double z1 = x1[k] - y_ref;
        double alpha = -c1*z1 + dy_ref;
        double dalpha = -c1*(x2[k]-dy_ref) + ddy_ref;
        double z2 = x2[k] - alpha;
        
        u[k] = -z1 + 2*x1[k] + 0.5*x2[k] + dalpha -c2*z2; 
      
        //系统动态
        x1[k+1] = x1[k] + dt*(x2[k]);
        x2[k+1] = x2[k] + dt*(-2*x1[k] - 0.5*x2[k] + u[k]);

    }
    //输出结果
    printf("k*dt     x1      u       error\n");
    printf("----------------------------------\n");
    for(int k = N-500;k<N;k+=50){
        double yr = sin(k*dt);
        printf("%.3f    %.6f    %.6f    %.6f\n",k*dt,x1[k],u[k],sin(k*dt)-x1[k]);
    }

    //释放内存
    free(x1);free(x2);free(u);

    printf("\n PID control done.\n");
    return 0;
}