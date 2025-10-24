#include "HQmanager.h"
#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"

#include <thread>
#include <mutex>
#include <chrono>
#include <map>
#include <fstream>
#include <algorithm>
#include <sstream>
#include <cmath>
#include <iomanip>

using namespace std;
namespace ORB_SLAM2
{
    
HighQualityManager::HighQualityManager(Map* pMap, ORBVocabulary* mpVoc, const std::string& criteria, double period_sec)
:mpMap(pMap), mpVoc(mpVoc), mCriteria(criteria), mPeriodSec(period_sec){}

void HighQualityManager::Run()
{
    using namespace std::chrono;
    while (true) {

        if (mDoScan.exchange(false)) {
            // placeholder
        } else {
            std::this_thread::sleep_for(duration<double>(mPeriodSec));
        }

        if (!mpMap) continue;

        std::vector<MapPoint*> vMPs = mpMap->GetAllMapPoints();
        if (vMPs.empty()) continue;

        std::string crit;
        {
            std::unique_lock<std::mutex> lock(mMutex);
            crit = mCriteria;
        }


        // ----------------------------
        // Thresholds (tune here)
        // ----------------------------
        const int   kMinObs                = 6;      // baseline
        const float kMinFoundRatio         = 0.90f;  // 0.25..0.5 typical
        const int   kMaxScaleLevelDiff     = 1;      // |oct - pred| <= 1
        const float kMinViewCos            = 0.70f;  // cos(60 deg)
        const int   kMinGoodFracPercent    = 60;     // % observations meeting geom checks
        const int   kMinTemporalSpanFrames = 80;     // observation span requirement
        const int   kMaxDescHamMean        = 40;     // ORB Hamming mean
        const int   kMaxDescHamMax         = 60;     // ORB Hamming max
        const double kMaxReprojErrPx       = 0.0;    // 0 disables reprojection check
        // ----------------------------

        for (MapPoint* pMP : vMPs) {
            if (!pMP || pMP->isBad()) continue;

            bool isHQ = false;

            if (crit=="observation") {
                isHQ = (pMP->Observations() >= kMinObs);
            } else if (crit=="foundratio") {
                isHQ = (pMP->GetFoundRatio() >= kMinFoundRatio);
            } else if (crit=="combo")
            {
                isHQ = (pMP->Observations() >= kMinObs) &&
                       (pMP->GetFoundRatio() >= kMinFoundRatio);
            }
            

            ApplyToMapPoint(pMP, isHQ);
            if (isHQ) {
                mpMap->AddHighQualityMapPoints(pMP);
            } else {
                mpMap->RemoveHighQaulityMapPoints(pMP);
            }
        }

        vector<KeyFrame*> vKFs = mpMap->GetAllKeyFrames();
        for (KeyFrame* pKF : vKFs) {
            if (!pKF) continue;

            if (pKF->NeedsHQBoWUpdate()) {
                pKF->ComputeHQBoW();
                pKF->ClearHQBoWUpdateFlag();
            }
        }
    }
}

void HighQualityManager::Notify()
{
    mDoScan = true;
}

void HighQualityManager::SetCriteria(const std::string criteria)
{
    std::lock_guard<std::mutex> lock(mMutex);
    mCriteria = criteria;
}

void HighQualityManager::ApplyToMapPoint(MapPoint* pMP, bool isHQ)
{
    const bool wasHQ = pMP->IsHighQuality();
    if (wasHQ == isHQ) {
        return;
    }

    pMP->SetHighQuality(isHQ);

    // update all keyframes
    const std::map<KeyFrame*, size_t> obs = pMP->GetObservations();
    for (const auto& kv : obs) {
        KeyFrame* pKF = kv.first;
        const size_t idx = kv.second;
        if (!pKF) continue;

        if (isHQ) {
            pKF->AddHighQualityMapPoint(pMP, idx);
        } else {
            pKF->EraseHighQualityMapPoint(idx);
        }
    }
}

static std::string JoinIds(const std::vector<long unsigned int>& ids, const char* sep=",")
{
    std::ostringstream oss;
    for (size_t i=0;i<ids.size();++i){ if(i) oss<<sep; oss<<ids[i]; }
    return oss.str();
}

static std::string DescriptorRowToHex(const cv::Mat& row) {
    if (row.empty()) return std::string();

    // Ensure 1x32 CV_8U
    cv::Mat d = row;

    if (d.rows != 1 && d.cols == 32) d = d.reshape(1, 1); // to 1x32
    if (d.type() != CV_8U) {
        cv::Mat tmp;
        d.convertTo(tmp, CV_8U);
        d = tmp;
    }

    static const char* HEX = "0123456789abcdef";
    std::string out;
    out.resize(d.cols * 2);

    const unsigned char* p = d.ptr<unsigned char>(0);
    for (int c = 0; c < d.cols; ++c) {
        out[2*c]     = HEX[p[c] >> 4];
        out[2*c + 1] = HEX[p[c] & 0x0F];
    }
    return out;
}

void HighQualityManager::ExportBoWTopMatchesCSV(const std::string& csv_path,
                                                int topK, int minFrameGap)
{
    if (!mpMap || !mpVoc) return;

    // 1) Snapshot valid KFs
    std::vector<KeyFrame*> vKFs = mpMap->GetAllKeyFrames();
    vKFs.erase(std::remove_if(vKFs.begin(), vKFs.end(),
                              [](KeyFrame* k){ return !k || k->isBad(); }),
               vKFs.end());
    if (vKFs.empty()) return;

    // 2) Ensure BoWs exist
    for (KeyFrame* kf : vKFs) {
        if (kf->mBowVec.empty()) kf->ComputeBoW();
        kf->ComputeHQBoW();
    }

    // 3) Cache BoWs + frame IDs
    const size_t N = vKFs.size();
    std::vector<DBoW2::BowVector> bows(N), hqbows(N);
    std::vector<long unsigned int> frameIds(N);
    for (size_t i = 0; i < N; ++i) {
        frameIds[i] = vKFs[i]->mnFrameId;      // use FRAME id (not KF id)
        bows[i]     = vKFs[i]->mBowVec;        // normal BoW
        hqbows[i]   = vKFs[i]->GetHQBoWVec();  // HQ BoW
    }

    // 4) Write matches CSV
    std::ofstream ofs(csv_path.c_str());
    ofs << "kf_frame_id,top_bow_frame_ids,top_hqbow_frame_ids\n";

    auto desc = [](const auto& a, const auto& b){ return a.first > b.first; };

    auto join_ids = [](const std::vector<long unsigned int>& v){
        std::ostringstream oss;
        for (size_t i=0;i<v.size();++i){ if(i) oss<<','; oss<<v[i]; }
        return oss.str();
    };

    for (size_t i = 0; i < N; ++i) {
        // Score against ALL others (no gap filter here!)
        std::vector<std::pair<double,size_t>> sc_bow; sc_bow.reserve(N-1);
        std::vector<std::pair<double,size_t>> sc_hq;  sc_hq.reserve(N-1);

        const bool hasHQi = !hqbows[i].empty();

        for (size_t j = 0; j < N; ++j) {
            if (j == i) continue;

            double sN = 0.0;
            if (!bows[i].empty() && !bows[j].empty())
                sN = mpVoc->score(bows[i], bows[j]);
            sc_bow.emplace_back(sN, j);

            if (hasHQi && !hqbows[j].empty()) {
                const double sH = mpVoc->score(hqbows[i], hqbows[j]);
                sc_hq.emplace_back(sH, j);
            }
        }

        // Sort by score (desc), then TAKE TOP-K
        std::sort(sc_bow.begin(), sc_bow.end(), desc);
        std::sort(sc_hq.begin(),  sc_hq.end(),  desc);

        // Now FILTER those K by frame-gap: keep only |Δ| >= minFrameGap
        std::vector<long unsigned int> top_bow_ids;
        std::vector<long unsigned int> top_hq_ids;

        int taken = 0;
        for (size_t k = 0; k < sc_bow.size() && taken < (size_t)topK; ++k) {
            size_t j = sc_bow[k].second;
            long long gap = (long long)frameIds[i] - (long long)frameIds[j];
            if (gap < 0) gap = -gap;
            if (gap >= (long long)minFrameGap) {
                top_bow_ids.push_back(frameIds[j]);
            }
            ++taken; // counted toward "top-K considered"
        }

        taken = 0;
        for (size_t k = 0; k < sc_hq.size() && taken < (size_t)topK; ++k) {
            size_t j = sc_hq[k].second;
            long long gap = (long long)frameIds[i] - (long long)frameIds[j];
            if (gap < 0) gap = -gap;
            if (gap >= (long long)minFrameGap) {
                top_hq_ids.push_back(frameIds[j]);
            }
            ++taken;
        }

        ofs << frameIds[i] << ","
            << "\"" << join_ids(top_bow_ids) << "\","
            << "\"" << join_ids(top_hq_ids)  << "\"\n";
    }
    ofs.close();

    // 5) ALSO write poses.csv in the same directory as csv_path
    std::string poses_path;
    {
        // derive directory of csv_path
        size_t slash = csv_path.find_last_of("/\\");
        std::string dir = (slash == std::string::npos) ? std::string() : csv_path.substr(0, slash+1);
        poses_path = dir + "poses.csv";
    }

    std::ofstream pofs(poses_path.c_str());
    pofs << "frame_id,x,y,z,yaw\n";
    for (KeyFrame* kf : vKFs) {
        if (!kf) continue;
        // Twc = world_T_cam
        cv::Mat Twc = kf->GetPoseInverse();
        const float x = Twc.at<float>(0,3);
        const float y = Twc.at<float>(1,3);
        const float z = Twc.at<float>(2,3);
        cv::Mat Rwc = Twc.rowRange(0,3).colRange(0,3);
        // yaw around Z (atan2(r10, r00))
        const double yaw = std::atan2((double)Rwc.at<float>(1,0), (double)Rwc.at<float>(0,0));

        pofs << kf->mnFrameId << "," << x << "," << y << "," << z << "," << yaw << "\n";
    }
    pofs.close();

    // Map summary csv
    std::string map_summary_path;
    {
        size_t slash = csv_path.find_last_of("/\\");
        std::string dir = (slash == std::string::npos) ? std::string() : csv_path.substr(0, slash+1);
        map_summary_path = dir + "map_summary.csv";
    }

    std::ofstream mofs(map_summary_path.c_str());
    mofs << "num_keyframes,num_mappoints,num_hqmappoints\n";
    mofs << vKFs.size() << "," << mpMap->GetAllMapPoints().size() << "," << mpMap->GetHighQualityMapPoints().size() << "\n";

    std::cout << "[HQManager] Exported BoW comparison CSV to " << csv_path
              << " and poses to " << poses_path
              << " (topK=" << topK << ", minFrameGap=" << minFrameGap << ")\n";
}


void HighQualityManager::ExportMapPointDescriptorsCSV(const std::string& csv_path)
{
    if (!mpMap) return;

    // Use HQ points; switch to GetAllMapPoints() if you want everything
    std::vector<MapPoint*> vMPs = mpMap->GetHighQualityMapPoints();
    vMPs.erase(std::remove_if(vMPs.begin(), vMPs.end(),
                              [](MapPoint* p){ return !p || p->isBad(); }),
               vMPs.end());
    if (vMPs.empty()) return;

    std::ofstream ofs(csv_path.c_str());
    if (!ofs.is_open()) {
        std::cerr << "[HQManager] Failed to open " << csv_path << " for writing.\n";
        return;
    }

    ofs << std::fixed << std::setprecision(6);
    // One row per MapPoint
    // observations = "kfId;kfId;kfId;..."
    ofs << "mp_id,mp_x,mp_y,mp_z,descriptor_hex,observations\n";

    size_t rows_written = 0;

    for (MapPoint* pMP : vMPs) {
        if (!pMP) continue;

        // 3D position
        cv::Mat Xw = pMP->GetWorldPos();  // 3x1, CV_32F
        double X = 0.0, Y = 0.0, Z = 0.0;
        if (!Xw.empty() && Xw.rows >= 3 && Xw.cols >= 1) {
            X = static_cast<double>(Xw.at<float>(0));
            Y = static_cast<double>(Xw.at<float>(1));
            Z = static_cast<double>(Xw.at<float>(2));
        }

        // Descriptor from MapPoint itself
        std::string hex = DescriptorRowToHex(pMP->GetDescriptor()); // ensure forward decl/def exists

        // Gather observing KeyFrame IDs (unique, sorted)
        const std::map<KeyFrame*, size_t> obs = pMP->GetObservations();
        if (obs.empty()) continue;

        std::vector<long unsigned int> kf_ids;
        kf_ids.reserve(obs.size());
        for (const auto& kv : obs) {
            KeyFrame* pKF = kv.first;
            if (!pKF || pKF->isBad()) continue;
            kf_ids.push_back(pKF->mnFrameId);
        }
        if (kf_ids.empty()) continue;
        std::sort(kf_ids.begin(), kf_ids.end());
        kf_ids.erase(std::unique(kf_ids.begin(), kf_ids.end()), kf_ids.end());

        std::ostringstream obss;
        for (size_t i = 0; i < kf_ids.size(); ++i) {
            if (i) obss << ';';
            obss << kf_ids[i];
        }

        ofs << pMP->mnId << ","
            << X << "," << Y << "," << Z << ","
            << "\"" << hex << "\","
            << "\"" << obss.str() << "\"\n";

        ++rows_written;
    }

    ofs.close();
    std::cout << "[HQManager] Exported " << rows_written
              << " MapPoints to " << csv_path << "\n";
}



}