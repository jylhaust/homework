//
// Created by 高翔 on 2017/12/19.
// 本程序演示如何从Essential矩阵计算R,t
//

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace Eigen;

#include <sophus/so3.h>
#include <sophus/se3.h>
#include <iostream>

using namespace std;

int main(int argc, char **argv) {

    // 给定Essential矩阵
    Matrix3d E;
    E << -0.0203618550523477, -0.4007110038118445, -0.03324074249824097,
            0.3939270778216369, -0.03506401846698079, 0.5857110303721015,
            -0.006788487241438284, -0.5815434272915686, -0.01438258684486258;

    // 待计算的R,t
    Matrix3d R;
    Vector3d t;
    JacobiSVD<Matrix3d> svd(E,ComputeFullU|ComputeFullV);
    Matrix3d U = svd.matrixU();
    Matrix3d V = svd.matrixV();
    // Matrix3d S = U.inverse() * E * V.transpose().inverse();
    Matrix<double,3,1> SingularValue = svd.singularValues();
    Matrix3d S = SingularValue.asDiagonal();
    // cout<<"U:"<<U<<endl<<"V:"<<V<<endl;
    // cout<<S<<endl;
    // SVD and fix sigular values
    // START YOUR CODE HERE
    // Matrix3d Rz1,Rz2;
    Matrix<double,3,1> n(1,0,0);
    // Rz1 = n*n.transpose()+Sophus::SO3::hat(n);
    // Rz2 = n*n.transpose()-Sophus::SO3::hat(n);
    AngleAxisd Rz1(M_PI/2,Vector3d(0,0,1) );
    AngleAxisd Rz2(-M_PI/2,Vector3d(0,0,1) );
    // cout<<Rz1<<endl;
    // END YOUR CODE HERE

    // set t1, t2, R1, R2 
    // START YOUR CODE HERE
    Matrix3d t_wedge1;
    Matrix3d t_wedge2;

    Matrix3d R1;
    Matrix3d R2;

    t_wedge1 = U*Rz1*S*U.transpose();
    t_wedge2 = U*Rz2*S*U.transpose();
    R1 = U*Rz1*S*V.transpose();
    R2 = U*Rz2*S*V.transpose();
    // END YOUR CODE HERE

    cout << "R1 = " << R1 << endl;
    cout << "R2 = " << R2 << endl;
    cout << "t1 = " << Sophus::SO3::vee(t_wedge1) << endl;
    // cout << "t2 = " << Sophus::SO3d::vee(t_wedge2) << endl;

    // check t^R=E up to scale
    Matrix3d tR = t_wedge1 * R1*(-1.41);
    cout << "t^R = " << tR << endl;

    return 0;
}