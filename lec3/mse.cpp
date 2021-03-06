#include <sophus/se3.h>
#include <string>
#include <iostream>
#include <fstream>

#include <Eigen/Core>
#include <Eigen/Geometry>
// need pangolin for plotting trajectory
#include <pangolin/pangolin.h>

using namespace std;

// path to trajectory file
string estimated_file = "../estimated.txt";
string groundtruth_file = "../groundtruth.txt";
// function for plotting trajectory, don't edit this code
// start point is red and end point is blue
void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>);
vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> ReadPoses(string trajectory_file);
int main(int argc, char **argv) {

    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> truth_poses, est_poses;

    /// implement pose reading code
    typedef Eigen::Matrix<double,6,1> Vector6d;
    double Mse = 0, err = 0, errsq = 0;
    truth_poses = ReadPoses(groundtruth_file);
    est_poses = ReadPoses(estimated_file);
    for(int k = 0;k<truth_poses.size();k++)
    {
        auto Tg = truth_poses[k], Te = est_poses[k];
        auto Tx = Tg.inverse()*Te;
        Vector6d se3 = Tx.log(),se3_hat_vee = Sophus::SE3::vee(Sophus::SE3::hat(se3)).transpose();
        err = se3_hat_vee.norm();
        errsq += pow(err,2);
        // cout<<Sophus::SE3::hat(se3)<<"--"<<endl;
        // cout<<Sophus::SE3::vee(Sophus::SE3::hat(se3)).transpose()<<" hh"<<endl;
    }
    Mse = sqrt(errsq/truth_poses.size());
    cout<<Mse<<endl;
    // cout<<truth_poses.size()<<endl;
    // draw trajectory in pangolin
    // DrawTrajectory(truth_poses);
    // DrawTrajectory(est_poses);
    return 0;
}
vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> ReadPoses(string trajectory_file)
{
    ifstream fin(trajectory_file);
    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses;
    double data[8] = {0};
    while(!fin.eof())
    {
        for ( auto& d:data )
        fin>>d;
        Eigen::Quaterniond q(data[7],data[4],data[5],data[6]);
        Eigen::Vector3d t( data[1],data[2],data[3] );
        Sophus::SE3 SE3_qt(q,t);
        poses.push_back(SE3_qt);
        // cout<<"data: "<<data[1]<<endl;
    }
    return poses;
}
/*******************************************************************************************/
void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses) {
    if (poses.empty()) {
        cerr << "Trajectory is empty!" << endl;
        return;
    }

    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));


    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glLineWidth(2);
        for (size_t i = 0; i < poses.size() - 1; i++) {
            glColor3f(1 - (float) i / poses.size(), 0.0f, (float) i / poses.size());
            glBegin(GL_LINES);
            auto p1 = poses[i], p2 = poses[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }

}