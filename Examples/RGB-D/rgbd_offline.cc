// offline_rgbd.cc — Play back recorded RGB-D dataset into ORB-SLAM2

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <chrono>
#include <thread>
#include <iomanip>
#include <algorithm>
// POSIX dir creation
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <errno.h>

#include <opencv2/opencv.hpp>

#include <System.h>
#include <MapPoint.h>

using std::string;
using std::vector;

struct RGBDPair {
    double t_rgb = 0.0;
    double t_d   = 0.0;
    string rgb_path;
    string depth_path;
};

static bool LoadAssociations(const string& dataset_dir, vector<RGBDPair>& out)
{
    std::ifstream f(dataset_dir + "/associations.txt");
    if (!f.is_open()) {
        std::cerr << "Could not open " << dataset_dir << "/associations.txt\n";
        return false;
    }
    out.clear();
    out.reserve(10000);

    // Expected format per line:
    // t_rgb  rgb/<file>  t_depth  depth/<file>
    string line;
    while (std::getline(f, line)) {
        if (line.empty()) continue;
        std::istringstream ss(line);
        double t_rgb, t_d;
        string rgb_rel, d_rel;
        if (!(ss >> t_rgb >> rgb_rel >> t_d >> d_rel)) continue;
        RGBDPair p;
        p.t_rgb = t_rgb;
        p.t_d   = t_d;
        p.rgb_path   = dataset_dir + "/" + rgb_rel;
        p.depth_path = dataset_dir + "/" + d_rel;
        out.push_back(std::move(p));
    }

    // Ensure sorted by time (just in case)
    std::sort(out.begin(), out.end(),
              [](const RGBDPair& a, const RGBDPair& b){ return a.t_rgb < b.t_rgb; });

    std::cout << "Loaded " << out.size() << " associations.\n";
    return !out.empty();
}

static bool mkdir_once(const std::string& path) {
    if (path.empty()) return false;
    struct stat st;
    if (::stat(path.c_str(), &st) == 0) return S_ISDIR(st.st_mode);
    if (::mkdir(path.c_str(), 0755) == 0) return true;
    if (errno == EEXIST) return true;
    return false;
}

static bool mkdir_p(const std::string& dir) {
    if (dir.empty()) return false;
    std::string cur;
    cur.reserve(dir.size());
    for (size_t i = 0; i < dir.size(); ++i) {
        char c = dir[i];
        cur.push_back(c);
        if (c == '/' && cur.size() > 1) {
            if (!mkdir_once(cur)) return false;
            while (i + 1 < dir.size() && dir[i + 1] == '/') ++i;
        }
    }
    if (cur.back() != '/') {
        if (!mkdir_once(cur)) return false;
    }
    return true;
}

int main(int argc, char** argv)
{
    if (argc < 4) {
        std::cerr << "\nUsage: " << argv[0]
                  << " path_to_vocabulary path_to_settings path_to_dataset [--realtime]\n"
                  << "  Example:\n  " << argv[0]
                  << " ORBvoc.txt Examples/RGB-D/RealSense.yaml ./record_data --realtime\n";
        return 1;
    }

    const string voc_path      = argv[1];
    const string settings_path = argv[2];
    const string dataset_dir   = argv[3];
    const bool   replay_realtime = (argc >= 5 && string(argv[4]) == "--realtime");

    // 1) Init ORB-SLAM2 (RGBD)
    ORB_SLAM2::System SLAM(voc_path, settings_path, ORB_SLAM2::System::RGBD, true);

    // 2) Load file list
    vector<RGBDPair> seq;
    if (!LoadAssociations(dataset_dir, seq)) return 2;

    // 3) Main playback loop
    std::cout << "\n-------\nStarting offline RGB-D playback from: " << dataset_dir << "\n";
    std::cout << (replay_realtime ? "Real-time ON (sleeping between frames)\n"
                                  : "Real-time OFF (as fast as possible)\n");

    double t0 = seq.front().t_rgb;
    auto   wall0 = std::chrono::steady_clock::now();

    size_t n_ok = 0;
    for (size_t i = 0; i < seq.size(); ++i) {
        const auto& p = seq[i];

        // (A) Optional real-time sleep to honor timestamps
        if (replay_realtime) {
            double dt_since_start = p.t_rgb - t0; // seconds
            auto tp_due = wall0 + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                        std::chrono::duration<double>(dt_since_start));
            std::this_thread::sleep_until(tp_due);
        }

        // (B) Load color (BGR) and depth (uint16 in millimeters)
        cv::Mat imRGB = cv::imread(p.rgb_path, cv::IMREAD_COLOR);
        cv::Mat imDepthU16 = cv::imread(p.depth_path, cv::IMREAD_UNCHANGED);

        if (imRGB.empty() || imDepthU16.empty()) {
            std::cerr << "Skip: failed to read files:\n  "
                      << p.rgb_path << "\n  " << p.depth_path << "\n";
            continue;
        }
        if (imDepthU16.type() != CV_16UC1) {
            std::cerr << "Warning: depth image is not CV_16UC1, path: "
                      << p.depth_path << " (type=" << imDepthU16.type() << ")\n";
        }
        if (imRGB.size() != imDepthU16.size()) {
            std::cerr << "Warning: size mismatch RGB(" << imRGB.cols << "x" << imRGB.rows
                      << ") vs D(" << imDepthU16.cols << "x" << imDepthU16.rows << ").\n";
        }

        // (C) Convert depth to meters (CV_32F)
        cv::Mat depth_m;
        imDepthU16.convertTo(depth_m, CV_32F, 1.0/1000.0); // mm -> m

        // (D) Feed ORB-SLAM2 (use RGB timestamp as the frame time)
        SLAM.TrackRGBD(imRGB, depth_m, p.t_rgb);

        if ((i % 50) == 0) {
            std::cout << "Processed frame " << i << "/" << seq.size()
                      << "  t=" << std::fixed << std::setprecision(6) << p.t_rgb << "\n";
        }
        ++n_ok;
    }

    // 4) Shutdown SLAM and save outputs like your live app
    SLAM.Shutdown();

    // Save trajectory & any custom exports you use
    const string out_dir = "Results";
    mkdir_p(out_dir);
    SLAM.SaveTrajectoryKITTI(out_dir + "/CameraTrajectory.txt");
    std::cout << "Saving BoW comparison to " << out_dir << "/bow_vs_hqbow_matches.csv\n";
    SLAM.mpHQmanager->ExportBoWTopMatchesCSV(out_dir + "/bow_vs_hqbow_matches.csv", 10);
    SLAM.mpHQmanager->ExportMapPointDescriptorsCSV(out_dir + "/mappoint_descriptors.csv");

    std::cout << "Done. Frames processed: " << n_ok << " / " << seq.size() << "\n";
    return 0;
}
