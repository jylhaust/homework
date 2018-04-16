#include <iostream>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

int main(int argc, char** argv)
{
    Eigen::Matrix<float,4,1> coes;
    Eigen::Quaternionf q1(0.55, 0.3, 0.2, 0.2), q2(-0.1, 0.3, -0.7, 0.2);
    Eigen::Vector3f t1(0.7, 1.1, 0.2), t2(-0.1, 0.4, 0.8);
    coes = q1.coeffs();
    Eigen::Matrix3f R1_matrix, R2_matrix;
    // cout<<R1_matrix.rows()<<endl<<R1_matrix.cols();
    Eigen::Quaternionf qw1, qw2;
    // cout<<"R1 : "<<endl<<R1_matrix.determinant()<<endl;
    qw1 = q1.normalized(); //正则化
    qw2 = q2.normalized();
    R1_matrix = qw1.toRotationMatrix(); //四元数转为选择矩阵
    R2_matrix = qw2.toRotationMatrix();

    Eigen::Isometry3f T1 = Eigen::Isometry3f::Identity(); //？
    Eigen::Isometry3f T2 = Eigen::Isometry3f::Identity();
    T1.rotate(R1_matrix);
    T2.rotate(R2_matrix);
    T1.pretranslate(t1);
    T2.pretranslate(t2);
    // cout<<"R1 : "<<endl<<R1_matrix.determinant()<<endl;
    // cout<<"T1: \n"<<T1.matrix()<<endl;
    Eigen::Vector3f P1(0.5,-0.1,0.2),Pw, P2;
    Pw = T1.inverse()*P1; 
    // cout<<"Pw: "<<Pw<<endl;
    P2 = T2*Pw;
    cout<<"P2: "<<P2<<endl;
    // cout<<"coes: "<<coes<<endl;
    return 0;
}