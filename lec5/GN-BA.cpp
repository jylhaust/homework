//
// Created by xiang on 12/21/17.
//

#include <Eigen/Core>
#include <Eigen/Dense>

using namespace Eigen;

#include <vector>
#include <fstream>
#include <iostream>
#include <iomanip>

#include "sophus/se3.h"

using namespace std;

typedef vector<Vector3d, Eigen::aligned_allocator<Vector3d>> VecVector3d;
typedef vector<Vector2d, Eigen::aligned_allocator<Vector3d>> VecVector2d;
typedef Matrix<double, 6, 1> Vector6d;

string p3d_file = "../p3d.txt";
string p2d_file = "../p2d.txt";
typedef Matrix<double, 4, 1> Vector4d;
int main(int argc, char **argv) {

    VecVector2d p2d;
    VecVector3d p3d;
    Matrix3d K;
    double fx = 520.9, fy = 521.0, cx = 325.1, cy = 249.7;
    K << fx, 0, cx, 0, fy, cy, 0, 0, 1;
    // load points in to p3d and p2d 
    // START YOUR CODE HERE
    fstream fin1(p3d_file);
    fstream fin2(p2d_file);
    double arr3d[3],arr2d[2];
    while(!fin1.eof())
    {
        for(auto& d:arr3d)
        fin1>>d;
        Vector3d p3v(arr3d[0],arr3d[1],arr3d[2]);
        p3d.push_back(p3v);
        // cout<<"p3v:"<<p3v<<endl;
    }
     while(!fin2.eof())
    {
        for(auto& d:arr2d)
        fin2>>d;
        Vector2d p2v(arr2d[0],arr2d[1]);
        p2d.push_back(p2v);
        // cout<<"p2v:"<<p2v<<endl;
    }
    // for(int i =0;i<p3d.size();i++)  cout<<p3d.at(i)<<endl;
    // END YOUR CODE HERE
    assert(p3d.size() == p2d.size());

    int iterations = 100;
    double cost = 0, lastCost = 0;
    int nPoints = p3d.size();
    // cout << "points: " << nPoints << endl;
    // Matrix3d R;
    // Vector3d t(0,0,0);
    // R << 1,0,0,0,1,0,0,0,1;
    Sophus::SE3 T_esti; // estimated pose
    for (int iter = 0; iter < iterations; iter++) {
        Matrix<double, 6, 6> H = Matrix<double, 6, 6>::Zero();
        Vector6d b = Vector6d::Zero();
        // compute cost
        cost = 0; //每次迭代前必须清零，不然cost>lastcost会退出迭代
        for (int i = 0; i < nPoints; i++) {
            // compute cost for p3d[I] and p2d[I]
            // START YOUR CODE HERE 
             Vector3d Pt = T_esti*p3d[i];
             Vector3d Ut = K*Pt;
             Vector2d e = p2d[i] - Vector2d(Ut(0)/Ut(2),Ut(1)/Ut(2));
	    // END YOUR CODE HERE
           
	    // compute jacobian
            Matrix<double, 2, 6> J;
            // START YOUR CODE HERE 
            J(0,0) = -fx/Pt(2);
            J(0,1) = 0;
            J(0,2) = fx*Pt(0)/pow(Pt(2),2);
            J(0,3) = fx*Pt(0)*Pt(1)/pow(Pt(2),2);
            J(0,4) = -fx-fx*pow(Pt(0),2)/pow(Pt(2),2);
            J(0,5) = fx*Pt(1)/Pt(2);
           
            J(1,0) = 0;
            J(1,1) = -fy/Pt(2);
            J(1,2) = fy*Pt(1)/pow(Pt(2),2);
            J(1,3) = fy+fy*pow(Pt(1),2)/pow(Pt(2),2);
            J(1,4) = -fy*Pt(0)*Pt(1)/pow(Pt(2),2);
            J(1,5) = -fy*Pt(0)/Pt(2);

	    // END YOUR CODE HERE
            H += J.transpose() * J;
            b += -J.transpose() * e;
            cost += e.transpose() * e;
        }
	// solve dx 
        Vector6d dx;
        // START YOUR CODE HERE 
        dx = H.ldlt().solve(b);
        // END YOUR CODE HERE

        if (isnan(dx[0])) {
            cout << "result is nan!" << endl;
            break;
        }

        if (iter > 0 && cost >= lastCost) {
            // cost increase, update is not good
            cout << "cost: " << cost << ", last cost: " << lastCost << endl;
            break;
        }
        // update your estimation
        // START YOUR CODE HERE 
        T_esti = Sophus::SE3::exp(dx)*T_esti;
        // END YOUR CODE HERE
        lastCost = cost;

        cout << "iteration " << iter << " cost=" << cout.precision(12) << cost << endl;
    }

    cout << "estimated pose: \n" << T_esti.matrix() << endl;
    return 0;
}
