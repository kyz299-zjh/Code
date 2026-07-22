#include<iostream>
using namespace std;

//类的声明
class Calculator{
private:
    double factor;  //成员变量（只能类内部访问）

public:
    Calculator(double f){//构造函数（创建对象是自动调用）
        factor = f; 
    }
    double multiply(double x){//成员函数
        return x*factor;
    }
    double add(double a,double b){
        return a+b;
    }
};

//主函数
int main(){
    Calculator calc(3.0);

    cout << "5*3=" << calc.multiply(5) << endl;
    cout << "2+7=" << calc.add(2,7) << endl;
    

    return 0;
}