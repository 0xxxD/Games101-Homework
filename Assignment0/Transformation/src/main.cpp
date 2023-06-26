#if __INTELLISENSE__
#undef __ARM_NEON
#undef __ARM_NEON__
#endif

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
using namespace std;

int main(int argc, char** argx)
{
    // // Basic Example of cpp
    std::cout << "Example of cpp \n";
    float a = 1.0, b = 2.0;
    std::cout << a << std::endl;
    std::cout << a/b << std::endl;
    std::cout << std::sqrt(b) << std::endl;
    std::cout << std::acos(-1) << std::endl;
    std::cout << std::asin(-1) << std::endl;
    std::cout << std::asin(0) << std::endl;
    std::cout << std::sin(30.0/180.0*acos(-1)) << std::endl;

    // Example of xector
    std::cout << "Example of xector \n";
    // xector definition
    Eigen::Vector3f x(1.0f,0.0f,0.0f);
    Eigen::Vector3f y(0.0f,1.0f,0.0f);
    // xector output
    std::cout << "Example of output \n";
    std::cout << x << std::endl;
    // xector add
    std::cout << "Example of add \n";
    std::cout << x + y << std::endl;
    // xector scalar multiply
    std::cout << "Example of scalar multiply \n";
    std::cout << x * 3.0f << std::endl;
    std::cout << 2.0f * x << std::endl;
    std::cout << "Here is x.transpose():\n" << x.transpose() << std::endl;
    std::cout << "Here is y*x^T:\n" << y*x.transpose() << std::endl;
    std::cout << "Here is x*y^T:\n" << x*y.transpose() << std::endl;

    std::cout << "Here is y.x:\n" << y.dot(x) << std::endl;
    std::cout << "Here is y x x:\n" << y.cross(x) << std::endl;


    // Example of matrix
    std::cout << "Example of matrix \n";
    // matrix definition
    Eigen::Matrix3f i,j;
    // Eigen::Matrix4f o, k;
    i << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0;
    j << 2.0, 3.0, 1.0, 4.0, 6.0, 5.0, 9.0, 7.0, 8.0;
    // matrix output
    std::cout << "Example of output \n";
    std::cout << "Here is i:\n" << i << std::endl;
    std::cout << "Here is j:\n" << j << std::endl;

    // matrix add i + j
    std::cout << "Here is i + j:\n" << i + j << std::endl;
    // matrix scalar multiply i * 2.0
    std::cout << "scalar multiply i * 2.0:\n" << i * 2.0 << std::endl;
    // matrix multiply i * j
    std::cout << "multiply i * j:\n" << i * j << std::endl;
    
    // matrix multiply vector i * x
    std::cout << "matrix multiply vector i * x:\n" << i * x << std::endl;

    //给定一个点 P =(2,1), 将该点绕原点先逆时针旋转 45◦，再平移 (1,2), 计算出 变换后点的坐标(要求用齐次坐标进行计算)。
    std::float_t angleValue = 45.0f / 180 * M_PI;
    Eigen::Vector3d p(2, 1, 1);
    // Eigen::Matrix3f 
    Eigen::Matrix<double, 3, 3> m {
        {cos(angleValue), -sin(angleValue), 1},
        {sin(angleValue), cos(angleValue), 2},
        {0, 0, 1}
    };             
    std::cout << m * p << std::endl;
    return 0;
}
