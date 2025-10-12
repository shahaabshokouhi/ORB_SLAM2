// recorder_rgbd_cc11.cpp  (no <filesystem>, OK with C++11)

#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>
#include <csignal>
#include <chrono>
#include <vector>
#include <thread>          // for std::this_thread::sleep_for
#include <cerrno>

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

using clock_steady = std::chrono::steady_clock;
static volatile std::sig_atomic_t g_stop = 0;
void sigint_handler(int){ g_stop = 1; }

static std::string fmt_ts(double t){
    std::ostringstream oss; oss<<std::fixed<<std::setprecision(6)<<t; return oss.str();
}

static bool mkdir_once(const std::string& path) {
    if (path.empty()) return false;
    struct stat st;
    if (::stat(path.c_str(), &st) == 0) return S_ISDIR(st.st_mode);
    if (::mkdir(path.c_str(), 0755) == 0) return true;
    if (errno == EEXIST) return true;
    return false;
}

// Minimal portable "mkdir -p"
static bool mkdir_p(const std::string& dir) {
    if (dir.empty()) return false;
    if (dir[0] == '/') {
        // absolute path
    }
    std::string cur;
    cur.reserve(dir.size());
    for (size_t i = 0; i < dir.size(); ++i) {
        char c = dir[i];
        cur.push_back(c);
        if (c == '/' && cur.size() > 1) { // create each component
            if (!mkdir_once(cur)) return false;
            // collapse multiple slashes
            while (i+1 < dir.size() && dir[i+1] == '/') ++i;
        }
    }
    // final component if not ending with '/'
    if (cur.back() != '/') {
        if (!mkdir_once(cur)) return false;
    }
    return true;
}

static void write_calib_yaml(const rs2::video_stream_profile& color_vsp,
                             const rs2::video_stream_profile& depth_vsp,
                             const std::string& path_yaml)
{
    rs2_intrinsics color_i = color_vsp.get_intrinsics();
    rs2_intrinsics depth_i = depth_vsp.get_intrinsics();
    rs2_extrinsics d2c     = depth_vsp.get_extrinsics_to(color_vsp);

    cv::FileStorage fs(path_yaml, cv::FileStorage::WRITE);
    fs << "camera_model" << "pinhole";
    fs << "image_width" << color_i.width;
    fs << "image_height" << color_i.height;

    fs << "camera_matrix" << "[" << color_i.fx << 0.0 << color_i.ppx
                            << 0.0 << color_i.fy << color_i.ppy
                            << 0.0 << 0.0 << 1.0 << "]";

    cv::Mat dist(1,5,CV_64F, cv::Scalar(0));
    if (color_i.model != RS2_DISTORTION_NONE) {
        dist.at<double>(0,0)=color_i.coeffs[0];
        dist.at<double>(0,1)=color_i.coeffs[1];
        dist.at<double>(0,2)=color_i.coeffs[2];
        dist.at<double>(0,3)=color_i.coeffs[3];
        dist.at<double>(0,4)=color_i.coeffs[4];
    }
    fs << "distortion_coefficients" << dist;

    fs << "depth_image_width" << depth_i.width;
    fs << "depth_image_height" << depth_i.height;
    fs << "depth_camera_matrix" << "[" << depth_i.fx << 0.0 << depth_i.ppx
                                 << 0.0 << depth_i.fy << depth_i.ppy
                                 << 0.0 << 0.0 << 1.0 << "]";
    fs << "depth_unit" << "millimeter";

    cv::Mat R = cv::Mat::eye(3,3,CV_64F);
    cv::Mat t = cv::Mat::zeros(3,1,CV_64F);
    for(int r=0;r<3;++r) for(int c=0;c<3;++c) R.at<double>(r,c)=d2c.rotation[r*3+c];
    for(int r=0;r<3;++r) t.at<double>(r,0)=d2c.translation[r];
    fs << "depth_to_color_rotation" << R;
    fs << "depth_to_color_translation_m" << t;
    fs.release();
}

int main(int argc, char** argv)
{
    if (argc < 2) {
        std::cerr<<"Usage: "<<argv[0]<<" <output_dir> [max_frames|-1] [fps]\n";
        return 1;
    }
    const std::string out_dir  = argv[1];
    const long max_frames      = (argc>=3)? std::stol(argv[2]) : -1;
    const int fps_limit        = (argc>=4)? std::stoi(argv[3]) : 30;
    const int save_every = (argc >= 5) ? std::stoi(argv[4]) : 15; // save every N frames
    std::signal(SIGINT, sigint_handler);

    // Make folders
    if (!mkdir_p(out_dir) ||
        !mkdir_p(out_dir + "/rgb") ||
        !mkdir_p(out_dir + "/depth")) {
        std::cerr << "Failed to create output directories under " << out_dir << "\n";
        return 2;
    }

    std::ofstream assoc(out_dir + "/associations.txt");
    if (!assoc.is_open()) {
        std::cerr << "Cannot open associations.txt for writing\n";
        return 3;
    }

    // RealSense setup
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, fps_limit);
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16,  fps_limit);

    rs2::pipeline pipe;
    rs2::pipeline_profile prof = pipe.start(cfg);
    rs2::align align_to_color(RS2_STREAM_COLOR);

    auto color_sp = prof.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
    auto depth_sp = prof.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    write_calib_yaml(color_sp, depth_sp, out_dir + "/calib.yaml");

    std::cout<<"Recording to "<<out_dir<<"\nCtrl+C to stop.\n";

    const double t0_wall = std::chrono::duration<double>(
        std::chrono::system_clock::now().time_since_epoch()).count();

    long frame_idx = 0;
    clock_steady::time_point t_prev = clock_steady::now();

    while (!g_stop && (max_frames < 0 || frame_idx < max_frames)) {
        rs2::frameset fs = pipe.wait_for_frames();
        rs2::frameset aligned = align_to_color.process(fs);

        rs2::video_frame color = aligned.get_color_frame();
        rs2::depth_frame depth = aligned.get_depth_frame();
        if (!color || !depth) { std::cerr<<"Missing frame\n"; continue; }

        double t_rgb = t0_wall + color.get_timestamp()/1000.0;
        double t_d   = t0_wall + depth.get_timestamp()/1000.0;

        // Color to BGR
        int cw = color.get_width(), ch = color.get_height();
        cv::Mat rgb(ch, cw, CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat bgr; cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);

        // Depth as uint16 (mm)
        int dw = depth.get_width(), dh = depth.get_height();
        cv::Mat depth_mm(dh, dw, CV_16UC1, (void*)depth.get_data(), cv::Mat::AUTO_STEP);


        if (frame_idx % save_every == 0) {
            std::string name_rgb = fmt_ts(t_rgb) + ".png";
            std::string name_d   = fmt_ts(t_d)   + ".png";

            bool ok1 = cv::imwrite(out_dir + "/rgb/"   + name_rgb, bgr);
            bool ok2 = cv::imwrite(out_dir + "/depth/" + name_d,   depth_mm);
            if (!ok1 || !ok2) {
                std::cerr << "Failed to write images\n";
            } else {
                assoc << fmt_ts(t_rgb) << " rgb/"  << name_rgb << " "
                    << fmt_ts(t_d)   << " depth/"<< name_d   << "\n";
            }
        }

        
        // Simple fps throttle
        if (fps_limit > 0) {
            double period = 1.0 / double(fps_limit);
            double elapsed = std::chrono::duration<double>(clock_steady::now() - t_prev).count();
            if (elapsed < period) {
                std::this_thread::sleep_for(std::chrono::milliseconds(int((period - elapsed)*1000.0)));
            }
            t_prev = clock_steady::now();
        }

        // optional: lightweight logging
        if ((frame_idx % 90) == 0) {
            std::cout << "Processed frame " << frame_idx
                        << " (saving every " << save_every << " frames)\n";
        }       

        ++frame_idx;
    }

    assoc.close();
    pipe.stop();
    std::cout<<"Stopped. Wrote "<<frame_idx<<" frames.\n";
    return 0;
}
