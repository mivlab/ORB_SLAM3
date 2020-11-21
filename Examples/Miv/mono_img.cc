/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<iomanip>
#include<opencv2/core/core.hpp>
#include"System.h"

using namespace std;

void DrawTrajectory(cv::Mat mShowImg, std::vector<cv::Point2f> mPts);

int main(int argc, char **argv)
{
    if(argc != 4)
    {
        cerr << endl << "Usage: ./mono_kitti path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

    ofstream outFile("traj.txt");
    int start = 1;
    cv::Mat showImg(400, 640*2, CV_8UC1);
    showImg.setTo(255);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR,true);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;

    // Main loop
    cv::Mat im;
    char name[100];
    std::vector<cv::Point2f> Pts, newPts;
    for(int ni=start; ni< 500; ni++)
    {
        // Read image from file
        sprintf(name, "%s/left_%06d.jpg", argv[3], ni);
        im = cv::imread(name, CV_LOAD_IMAGE_GRAYSCALE);
        double tframe = ni;

        if(im.empty())
        {
            continue;
        }

        cv::Point3f pt;
        float quality;
        // Pass the image to the SLAM system
        SLAM.TrackMonocular(im,tframe, pt, quality);
        
        cout << "frame " << ni << "\t" << pt.x << "\t" << pt.z << endl;
        if (quality > 0)
            Pts.push_back(cv::Point2f(pt.x, pt.z));
        else
        {
            if (Pts.size() > 0)
                Pts.push_back(Pts[Pts.size() - 1]);
            else
                //Pts.push_back(cv::Point2f(0, 0));
                continue;
        }

        printf("q%.2f\t%d\n", quality, ni);
        printf("#--------------------------------------#\n");

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

void DrawTrajectory(cv::Mat mShowImg, std::vector<cv::Point2f> mPts)
{
    int width = mShowImg.cols;
    int height = mShowImg.rows;
    static int basex = width / 2;
    static int basey = height * 4 / 5;
    static float showScale = 10;
    int rangePix = (width < height ? width : height);
    int margin = rangePix / 4;
    int x, y;

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