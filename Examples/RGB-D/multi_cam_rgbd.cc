/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <librealsense2/rs.hpp>     // RealSense SDK

#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);

int main(int argc, char** argv)
{
    if(argc != 3)
    {
        cerr << endl << "Usage: ./live_rgbd path_to_vocabulary path_to_settings" << endl;
        return 1;
    }

    // 1) Initialize ORB_SLAM2 in RGBD mode
    string strVocFile = string(argv[1]);
    string strSettingsFile = string(argv[2]);
    ORB_SLAM2::System SLAM_1(strVocFile, strSettingsFile, ORB_SLAM2::System::RGBD, true);
    ORB_SLAM2::System SLAM_2(strVocFile, strSettingsFile, ORB_SLAM2::System::RGBD, true);

    // 2) Set up RealSense pipeline to capture both depth & color
    std::vector<std::string> serials = {
        "102422077153",
        "923322071768"
    };
    
    rs2::pipeline pipe_1;
    rs2::config cfg_1;
    cfg_1.enable_device(serials[0]);
    cfg_1.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    cfg_1.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 30);
    rs2::pipeline_profile profile_1 = pipe_1.start(cfg_1);

    rs2::pipeline pipe_2;
    rs2::config cfg_2;
    cfg_2.enable_device(serials[1]);
    cfg_1.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    cfg_1.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 30);
    rs2::pipeline_profile profile_2 = pipe_2.start(cfg_2);

    // 3) Create an align object so depth→color is registered
    rs2::align align_to_color(RS2_STREAM_COLOR);


    cout << endl << "-------" << endl;
    cout << "Starting live RGB-D capture..." << endl;
    cout << "Press ESC or 'q' in any window to exit." << endl << endl;

    // 6) Main capture + SLAM loop
    while(true)
    {
        // a) Wait for the next set of frames
        rs2::frameset frames_1 = pipe_1.wait_for_frames();
        rs2::frameset frames_2 = pipe_2.wait_for_frames();

        // b) Align depth frame to color frame
        rs2::frameset aligned_1 = align_to_color.process(frames_1);
        rs2::frameset aligned_2 = align_to_color.process(frames_2);

        // c) Extract aligned depth and color
        rs2::depth_frame depth_frame_1 = aligned_1.get_depth_frame();
        rs2::video_frame color_frame_1 = aligned_1.get_color_frame();

        rs2::depth_frame depth_frame_2 = aligned_2.get_depth_frame();
        rs2::video_frame color_frame_2 = aligned_2.get_color_frame();

        if(!depth_frame_1 || !color_frame_1 || !depth_frame_2 || !color_frame_2)
        {
            cerr << "Warning: could not retrieve depth or color frame." << endl;
            continue;
        }

        // d) Convert color_frame → cv::Mat (BGR8)
        int color_w = color_frame_1.get_width();
        int color_h = color_frame_1.get_height();
        cv::Mat imRGB_1(cv::Size(color_w, color_h),
                      CV_8UC3,
                      (void*)color_frame_1.get_data(),
                      cv::Mat::AUTO_STEP);
        cv::Mat imRGB_2(cv::Size(color_w, color_h),
                      CV_8UC3,
                      (void*)color_frame_2.get_data(),
                      cv::Mat::AUTO_STEP);

        // e) Convert depth_frame → cv::Mat (CV_16UC1, depth in millimeters)
        int depth_w = depth_frame_1.get_width();
        int depth_h = depth_frame_1.get_height();
        cv::Mat imDepth_1(cv::Size(depth_w, depth_h),
                        CV_16UC1,
                        (void*)depth_frame_1.get_data(),
                        cv::Mat::AUTO_STEP);
        cv::Mat imDepth_2(cv::Size(depth_w, depth_h),
                        CV_16UC1,
                        (void*)depth_frame_2.get_data(),
                        cv::Mat::AUTO_STEP);
    
        cv::Mat depth_meter_1 = cv::Mat(depth_h, depth_w, CV_32FC1); // one float per pixel
        cv::Mat depth_meter_2 = cv::Mat(depth_h, depth_w, CV_32FC1);
        for (int v = 0; v < depth_h; v++)
        {
            for (int u = 0; u < depth_w; u++)
            {
                // get_distance(u,v) is valid on rs2::depth_frame
                float z = depth_frame_1.get_distance(u, v);
                depth_meter_1.at<float>(v, u) = z;
                z = depth_frame_2.get_distance(u, v);
                depth_meter_2.at<float>(v, u) = z;
                // z is in meters. If no depth, z == 0.
            }
        }

        // h) Compute timestamp for this frame (in seconds)
        //    RealSense's get_timestamp() returns milliseconds since start.
        double tframe_1 = depth_frame_1.get_timestamp() / 1000.0;
        double tframe_2 = depth_frame_2.get_timestamp() / 1000.0;
        // i) Pass the synchronized RGB-D frame to ORB_SLAM2
        SLAM_1.TrackRGBD(imRGB_1, depth_meter_1, tframe_1);
        SLAM_2.TrackRGBD(imRGB_2, depth_meter_2, tframe_2);
        
        // j) Handle exit key
        int key = cv::waitKey(1);
        if( (key & 0xFF) == 27 || (key & 0xFF) == 'q' )
            break;
    }

    // 7) Shutdown SLAM and clean up
    SLAM_1.Shutdown();
    cv::destroyAllWindows();

    // 8) (Optional) Save trajectories if desired
    SLAM_1.SaveTrajectoryTUM("CameraTrajectory.txt");
    SLAM_1.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}
void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps)
{
    ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());
    while(!fAssociation.eof())
    {
        string s;
        getline(fAssociation,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB, sD;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenamesRGB.push_back(sRGB);
            ss >> t;
            ss >> sD;
            vstrImageFilenamesD.push_back(sD);

        }
    }
}
