#include "HQmanager.h"
#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"

#include <thread>
#include <mutex>
#include <chrono>
#include <map>
#include <set>
#include <fstream>
#include <algorithm>
#include <sstream>
#include <cmath>
#include <iomanip>
#include <unordered_map>
#include <unordered_set>


using namespace std;
namespace ORB_SLAM2
{

// internal stuff
namespace {

struct SE3 {
    cv::Matx33d R;
    cv::Vec3d   t;
};

struct AgentBuckets {
    std::unordered_map<int, std::vector<cv::Mat>>     kf2descs;
    std::unordered_map<int, std::vector<cv::Point3f>> kf2pts;
    std::unordered_map<int, std::vector<int>>         kf2mpids;
    std::unordered_map<int, DBoW2::BowVector>         kf2bow;
    bool hasTransform = false;
    SE3  T_agent_to_local;
};

struct Candidate { int kf; double score; };
struct Pair      { int kf1; int kf2; double score; };
struct Pairs3D {
    std::vector<cv::Point3f> P1;
    std::vector<cv::Point3f> P2;
    std::vector<cv::DMatch>  matches;
};
struct RansacSE3 {
    SE3 model;
    std::vector<int> inliers;
    bool ok = false;
};
struct EstSE3Weighted { SE3 T; int inliers=0; double score=0.0; };
struct PairEst { Pair pair; RansacSE3 est; };

std::map<std::string, AgentBuckets> gAgentBuckets;
std::unordered_map<int, std::vector<Candidate>> matched_frames;
std::vector<PairEst> pair_ests;

const float ratio = 0.9f;
const int maxHam = 60;
const double ransac_thresh = 0.07; // meters
const int ransac_min_inl = 20;
const int ransac_iters = 1000;

} 



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

bool HighQualityManager::ComputeBowForKF(const ORBVocabulary* voc,
                               const std::vector<cv::Mat>& descs,
                               DBoW2::BowVector& bow,
                               DBoW2::FeatureVector* feat)
{
    bow.clear();
    if (feat) feat->clear();
    if (descs.empty()) return false;

    if (feat) {
        voc->transform(descs, bow, *feat, 4);
    } else {
        voc->transform(descs, bow);
    }

    return !bow.empty();
}

void HighQualityManager::ImportHighQualityMapPoints(
    const std::string &agent_name,
    const std::vector<MapPoint*> &vMPs)
{
    std::unique_lock<std::mutex> lock(mMutexImport);

    // per-agent storage in HQ manager
    std::vector<MapPoint*> &vAgentPoints = mImportedPointsByAgent[agent_name];
    // per-agent “buckets” in this .cc
    AgentBuckets &agentBucket = gAgentBuckets[agent_name];

    std::unordered_set<int> updated_keyframes;

    for (ORB_SLAM2::MapPoint* pSrcMP : vMPs) {
        if (!pSrcMP) {
            std::cerr << "[HQManager] skipping null MapPoint from agent " << agent_name << "\n";
            continue;
        }

        // 1) check observations list
        if (pSrcMP->mvnObservations.empty()) {
            // not fatal, but tell ourselves
            std::cerr << "[HQManager] MapPoint " << pSrcMP->mnId
                      << " from agent " << agent_name
                      << " has no observation KF ids; skipping bucket fill.\n";
        }

        // 2) get world pos safely
        cv::Mat X = pSrcMP->GetWorldPos();
        bool hasPos = (!X.empty() && X.rows >= 3 && X.cols >= 1);

        // 3) get descriptor safely
        cv::Mat desc = pSrcMP->GetDescriptor();
        bool hasDesc = (!desc.empty() && desc.rows == 1 && desc.cols == 32 && desc.type() == CV_8U);
        if (!hasDesc) {
            std::cerr << "[HQManager] MapPoint " << pSrcMP->mnId
                      << " from agent " << agent_name
                      << " has invalid descriptor; rows=" << desc.rows
                      << " cols=" << desc.cols << " type=" << desc.type() << "\n";
        }

        // fill agent buckets (only if we have KF ids)
        for (int kf_id : pSrcMP->mvnObservations) {
            // descriptor
            if (hasDesc) {
                agentBucket.kf2descs[kf_id].push_back(desc.clone());
            } else {
                // keep the three vectors aligned: push placeholders or skip all 3
                // here we'll just skip the whole triplet for this KF
                continue;
            }

            // point
            if (hasPos) {
                cv::Point3f p;
                p.x = X.at<float>(0);
                p.y = X.at<float>(1);
                p.z = X.at<float>(2);
                agentBucket.kf2pts[kf_id].push_back(p);
            } else {
                // still push dummy to maintain parallel sizes
                continue;
            }

            // mp id
            agentBucket.kf2mpids[kf_id].push_back(static_cast<int>(pSrcMP->mnId));

            // 4) quick consistency check: all 3 vectors for this kf_id must have same size
            auto &vD = agentBucket.kf2descs[kf_id];
            auto &vP = agentBucket.kf2pts[kf_id];
            auto &vM = agentBucket.kf2mpids[kf_id];
            if (!(vD.size() == vP.size() && vP.size() == vM.size())) {
                std::cerr << "[HQManager] size mismatch for agent " << agent_name
                          << " KF " << kf_id
                          << " descs=" << vD.size()
                          << " pts="   << vP.size()
                          << " mpids=" << vM.size()
                          << "\n";
            }

            updated_keyframes.insert(kf_id);
        }

        // finally, keep your old behavior: own a copy of the point
        ORB_SLAM2::MapPoint* pCopy = new ORB_SLAM2::MapPoint(*pSrcMP);
        pCopy->ReceivedFromOther(true);
        vAgentPoints.push_back(pCopy);
    }

    size_t numKFs = agentBucket.kf2descs.size();
    size_t totalMPs = 0;
    for (const auto& kv : agentBucket.kf2mpids) {
        totalMPs += kv.second.size();
    }

    // calculating bow vector for updated keyframes
    for (const auto& kf_id : updated_keyframes) {
        const auto& descs = agentBucket.kf2descs[kf_id];

        DBoW2::BowVector bow;
        DBoW2::FeatureVector feat;

        if (ComputeBowForKF(mpVoc, descs, bow, &feat)) {
            agentBucket.kf2bow.emplace(kf_id, std::move(bow));
            std::cout << agent_name << ": KF " << kf_id << " BoW vector updated.\n";
        } else {
            std::cout << "KF " << kf_id << " produced empty BoW.\n";
        }
        
    }
    
    std::cout << "[HQManager] Agent \"" << agent_name << "\" now has "
              << numKFs << " keyframes and "
              << totalMPs << " map points accumulated." << std::endl;

    // number of matched frames
    int n = 0;

    // ransac parameters
    const float ratio = 0.9f;
    const int maxHam = 60;
    const double ransac_thresh = 0.07; // meters
    const int ransac_min_inl = 20;
    const int ransac_iters = 1000;

    std::unordered_map<int, std::vector<Candidate>> matched_frames;


    for (auto& ab : gAgentBuckets) {
        
        if (ab.first == agent_name) continue;
        AgentBuckets &agentBucketi = ab.second;
        
        for (const auto& kf_a : updated_keyframes) {
            double best = 0.0;
            
            for (const auto& kv : agentBucketi.kf2bow) {
                int kf_b = kv.first;
                double sc = mpVoc->score(agentBucket.kf2bow[kf_a], kv.second);
                if (sc > best) best = sc;
                matched_frames[kf_a].push_back({kf_b, sc});
            }

            const double floor_score = 0.03; // empirical floor
            const double rel_cut = 0.70 * best; // relative to best
            const double thresh = std::max(floor_score, rel_cut);

            matched_frames[kf_a].erase(std::remove_if(matched_frames[kf_a].begin(), matched_frames[kf_a].end(),
            [&](const Candidate& c){return c.score < thresh;}),
            matched_frames[kf_a].end());

            // sort descending
            std::sort(matched_frames[kf_a].begin(), matched_frames[kf_a].end(),
                [](const Candidate& a, const Candidate& b){return a.score > b.score;});

            if (!matched_frames[kf_a].empty()){
                std::cout << "Top KF candidate from agent " << ab.first << " for agent " << agent_name << " KF " << kf_a << ": ";
                std::cout << "KF " << matched_frames[kf_a][0].kf << " with score " << matched_frames[kf_a][0].score << "\n";
                ++n;
            }
            
        }
    }
}


}
