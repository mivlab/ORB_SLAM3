#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<iomanip>
#include<opencv2/core/core.hpp>
#include"System.h"

using namespace std;

void DrawTrajectory(cv::Mat mShowImg, vector<cv::Point2f> mPts);

int main(int argc, char **argv)
{
    if(argc < 4)
    {
        cerr << endl << "Usage: ./mono_kitti path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

    ofstream outFile("traj.txt");
    cv::Mat showImg(400, 640*2, CV_8UC1);
    showImg.setTo(255);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR,true);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;

    // Main loop
    cv::Mat im, Tcw;
    char name[100];
    vector<cv::Point2f> Pts;
    for(int ni = 0; ni < 500; ni++)
    {
        // Read image from file
        snprintf(name, sizeof(name),"%s/left_%06d.jpg", argv[3], ni);
        im = cv::imread(name, CV_LOAD_IMAGE_GRAYSCALE);
        double tframe = ni; // vTimestamps[ni];

        if(im.empty())
            continue;
    
        // Pass the image to the SLAM system
        Tcw = SLAM.TrackMonocular(im, tframe);

        cv::Point3f pt;
        if (!Tcw.empty()){
            cv::Mat Rwc(3, 3, CV_32F);
            cv::Mat twc(3, 1, CV_32F);
            Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
            twc = -Rwc * Tcw.rowRange(0, 3).col(3);
            pt.x = twc.at<float>(0, 0);
            pt.y = twc.at<float>(1, 0);
            pt.z = twc.at<float>(2, 0);
            Pts.push_back(cv::Point2f(pt.x, pt.z));
        }
        else {
            if (Pts.size() > 0)
                Pts.push_back(Pts[Pts.size() - 1]);
            else
                Pts.push_back(cv::Point2f(0, 0));
            continue;
        }
        
        cout << "frame " << tframe << "\t" << pt.x << "\t" << pt.z << endl;

        im.copyTo(showImg(cv::Rect(0, 0, showImg.cols / 2, showImg.rows)));
        DrawTrajectory(showImg(cv::Rect(showImg.cols / 2, 0, showImg.cols / 2, showImg.rows)), Pts);
        outFile << pt.x << " " << pt.z << endl;
        cv::imshow("traj", showImg);
        cv::waitKey(1);
    }

    // Stop all threads
    SLAM.Shutdown();
    cv::waitKey(0);

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");    

    return 0;
}

void DrawTrajectory(cv::Mat mShowImg, vector<cv::Point2f> mPts)
{
    int width = mShowImg.cols;
    int height = mShowImg.rows;
    static int basex = width / 2;
    static int basey = height * 4 / 5;
    static float showScale = 10;
    int rangePix = (width < height ? width : height);
    int margin = rangePix / 4;

    if (mPts.size() < 1)
        return;
    // 得到最小最大范围.
    float minx = mPts[0].x, maxx = mPts[0].x, miny = mPts[0].y, maxy = mPts[0].y;
    for (unsigned int i = 1; i < mPts.size(); i++)
    {
        if (mPts[i].x < minx)
            minx = mPts[i].x;
        if (mPts[i].x > maxx)
            maxx = mPts[i].x;
        if (mPts[i].y < miny)
            miny = mPts[i].y;
        if (mPts[i].y > maxy)
            maxy = mPts[i].y;
    }

    // 确定显示比例
    float range = maxx - minx;
    if (maxy - miny > range)
        range = maxy - miny;
    if (range < 0.1)
        range = 0.1;
    if (range * showScale > rangePix - margin * 2)
        showScale = (rangePix - margin * 2) / range;
    if (range * showScale < margin * 2)
        showScale = margin * 2 / range;

    // 确定基准点，公式： bx + minx * scale = margin, by - miny * scale = height - margin
    basex = margin - minx * showScale;
    basey = (height - margin) + miny * showScale;

    // 画点
    mShowImg.setTo(255);
    for (unsigned int i = 0; i < mPts.size() - 1; i++)
    {
        circle(mShowImg, cv::Point(mPts[i].x * showScale + basex, -mPts[i].y * showScale + basey), 1, cv::Scalar(120, 120, 120), 1);
        line(mShowImg, cv::Point(mPts[i].x * showScale + basex, -mPts[i].y * showScale + basey),
            cv::Point(mPts[i + 1].x * showScale + basex, -mPts[i + 1].y * showScale + basey), cv::Scalar(120, 120, 120), 1);
    }
}