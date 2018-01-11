#include <sophus/se3.h>
#include <string>
#include <iostream>
#include <fstream>

// need pangolin for plotting trajectory
#include <pangolin/pangolin.h>

using namespace std;

// path to trajectory file
string est_traj = "../estimated.txt";
string gt_traj = "../groundtruth.txt";

int main(int argc, char **argv) {

    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> est_poses;
    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> gt_poses;


    /// implement pose reading code
    // start your code here (5~10 lines)
    ifstream est_inFile;
    est_inFile.open(est_traj);
    ifstream gt_inFile;
    gt_inFile.open(gt_traj);
    double count = 0;
    double sum_e = 0;
    double ei_2 = 0;

    while(!est_inFile.eof() && !gt_inFile.eof()){
        if(est_inFile.eof() || gt_inFile.eof()){
            break;
        }
        double est_data[8]={0};
        double gt_data[8]={0};
        for (auto& d:est_data)
            est_inFile >> d;
        for (auto& d:gt_data)
            gt_inFile >> d;
        Eigen::Quaterniond q(est_data[7], est_data[4], est_data[5], est_data[6]);
        Eigen::Vector3d t(est_data[1], est_data[2], est_data[3]);
        Sophus::SE3 Te(q,t);

        Eigen::Quaterniond g_q(gt_data[7], gt_data[4], gt_data[5], gt_data[6]);
        Eigen::Vector3d g_t(gt_data[1], gt_data[2], gt_data[3]);
        Sophus::SE3 Tg(g_q,g_t);
        Sophus::SE3 T_ge(Tg.inverse()*Te);
        ei_2=T_ge.log().transpose()*T_ge.log();
        if (ei_2 <= 10000){
            sum_e = sum_e + ei_2;
            count++;
            est_poses.push_back(Te);
            gt_poses.push_back(Tg);
        }
    }
    est_inFile.close();
    gt_inFile.close();
    cout << "count:" << count << endl;
    cout << "sum_e:" << sum_e << endl;
    cout << "rmse:" << sqrt(sum_e/count) <<endl;
    
    return 0;
}