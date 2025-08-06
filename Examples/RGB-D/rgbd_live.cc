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
#include<MapPoint.h>

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
    ORB_SLAM2::System SLAM(strVocFile, strSettingsFile, ORB_SLAM2::System::RGBD, true);

    // 2) Set up RealSense pipeline to capture both depth & color
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 30);
    rs2::pipeline_profile profile = pipe.start(cfg);

    // 3) Create an align object so depth→color is registered
    rs2::align align_to_color(RS2_STREAM_COLOR);

    // 4) For colorizing the depth (for on‐screen display only)
    // rs2::colorizer color_map;

    // 5) Prepare OpenCV windows (optional, but helpful to debug)
    // const char* color_window = "ORB_SLAM2: RGB";
    // const char* depth_window = "ORB_SLAM2: Depth (Colorized)";
    // cv::namedWindow(color_window, cv::WINDOW_AUTOSIZE);
    // cv::namedWindow(depth_window, cv::WINDOW_AUTOSIZE);

    cout << endl << "-------" << endl;
    cout << "Starting live RGB-D capture..." << endl;
    cout << "Press ESC or 'q' in any window to exit." << endl << endl;

    // 6) Main capture + SLAM loop
    while(true)
    {
        // a) Wait for the next set of frames
        rs2::frameset frames = pipe.wait_for_frames();

        // b) Align depth frame to color frame
        rs2::frameset aligned = align_to_color.process(frames);

        // c) Extract aligned depth and color
        rs2::depth_frame depth_frame = aligned.get_depth_frame();
        rs2::video_frame color_frame = aligned.get_color_frame();

        if(!depth_frame || !color_frame)
        {
            cerr << "Warning: could not retrieve depth or color frame." << endl;
            continue;
        }

        // d) Convert color_frame → cv::Mat (BGR8)
        int color_w = color_frame.get_width();
        int color_h = color_frame.get_height();
        cv::Mat imRGB(cv::Size(color_w, color_h),
                      CV_8UC3,
                      (void*)color_frame.get_data(),
                      cv::Mat::AUTO_STEP);

        // e) Convert depth_frame → cv::Mat (CV_16UC1, depth in millimeters)
        int depth_w = depth_frame.get_width();
        int depth_h = depth_frame.get_height();
        cv::Mat imDepth(cv::Size(depth_w, depth_h),
                        CV_16UC1,
                        (void*)depth_frame.get_data(),
                        cv::Mat::AUTO_STEP);

        // int cx = depth_w / 2; // Assuming depth and color frames are aligned
        // int cy = depth_h / 2;
        // float center_depth = depth_frame.get_distance(cx, cy);
        // float raw_depth = imDepth.at<uint16_t>(cy, cx);
        // std::cout << "Center pixel depth: " << center_depth << " meters, raw depth: " << raw_depth << " mm" << std::endl;
        // Loop over every pixel (u, v):
        cv::Mat depth_meter = cv::Mat(depth_h, depth_w, CV_32FC1); // one float per pixel
        for (int v = 0; v < depth_h; v++)
        {
            for (int u = 0; u < depth_w; u++)
            {
                // get_distance(u,v) is valid on rs2::depth_frame
                float z = depth_frame.get_distance(u, v);
                depth_meter.at<float>(v, u) = z; 
                // z is in meters. If no depth, z == 0.
            }
        }

        // h) Compute timestamp for this frame (in seconds)
        //    RealSense's get_timestamp() returns milliseconds since start.
        double tframe = depth_frame.get_timestamp() / 1000.0;

        // i) Pass the synchronized RGB-D frame to ORB_SLAM2
        SLAM.TrackRGBD(imRGB, depth_meter, tframe);
        
        std::vector<ORB_SLAM2::MapPoint*> vpHighObs = SLAM.GetHighQualityMapPoints();
        std::cout << "High quality map points: " << vpHighObs.size() << std::endl;
        
        // j) Handle exit key
        int key = cv::waitKey(1);
        if( (key & 0xFF) == 27 || (key & 0xFF) == 'q' )
            break;
    }

    // 7) Shutdown SLAM and clean up
    SLAM.Shutdown();
    cv::destroyAllWindows();

    // 8) (Optional) Save trajectories if desired
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

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
