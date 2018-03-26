#include <iostream>

#include <ctime>
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;

int main(int argc, char** argv)
{
    Eigen::MatrixXd matrix_A, matrix_b, x, x1;
    matrix_A = Eigen::MatrixXd::Random(100,100);
    matrix_b = Eigen::MatrixXd::Random(100,1);
    // int NN = 0;
    // NN = sizeof(matrix_A);

    x = matrix_A.colPivHouseholderQr().solve(matrix_b);
    x1 = matrix_A.ldlt().solve(matrix_b);
    cout<<"matrix_x: /n"<<matrix_A.rows()<<matrix_A.cols()<<endl;
    // cout<<"x:"<<x<<endl;
    // cout<<"sizeofNN: "<<NN<<endl;
    return 0;
}