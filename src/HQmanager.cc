#include "HQmanager.h"
#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "Frame.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"

#include <opencv2/calib3d.hpp>


#include <thread>
#include <mutex>
#include <chrono>
#include <map>
#include <set>
#include <random>
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
    struct Candidate { KeyFrame* pkf; int kf; double score; };
    struct Pairs3D {
        std::vector<cv::Point3f> P1;
        std::vector<cv::Point3f> P2;
    };
    struct RansacSE3 {
        SE3 model;
        std::vector<int> inliers;
        bool ok = false;
    };
    struct EstSE3Weighted { SE3 T; int inliers=0; double score=0.0; };
    struct PairEst { KeyFrame* pkf; int kf2; RansacSE3 est; double bow_score=0.0; };
    struct AgentBuckets {
        std::unordered_map<int, std::vector<cv::Mat>>     kf2descs;
        std::unordered_map<int, std::vector<cv::Point3f>> kf2pts;
        std::unordered_map<int, std::vector<int>>         kf2mpids;
        std::unordered_map<int, DBoW2::BowVector>         kf2bow;
        // kf_id -> (mpid -> index into kf2descs/kf2pts/kf2mpids), O(1) upsert/erase
        std::unordered_map<int, std::unordered_map<int,int>> kf2mpidx;
        // reverse map: mpid -> set of kf_ids that currently hold it (for stale-entry cleanup)
        std::unordered_map<int, std::unordered_set<int>> mpid2kfids;

        bool hasTransform = false;
        SE3  T;
    };
    // Slim per-cycle snapshot of another agent's bucket: only the fields the
    // SE3 estimation needs, so the copy under gBucketsMx stays cheap.
    struct AgentSnapshot {
        std::unordered_map<int, std::vector<cv::Mat>>     kf2descs; // cv::Mat headers are ref-counted, shallow
        std::unordered_map<int, std::vector<cv::Point3f>> kf2pts;
        std::unordered_map<int, DBoW2::BowVector>         kf2bow;
    };

    std::unordered_map<std::string, SE3> transformsToApply; // guarded by gBucketsMx

    std::map<std::string, AgentBuckets> gAgentBuckets;
    std::unordered_map<std::string, std::unordered_map<int, MapPoint*>> nMpsPerAgent;
    std::mutex gBucketsMx;

    // matching parameters
    // ratio: tightened 0.9->0.75 to reduce false positives when maxHam is relaxed
    // maxHam: relaxed 60->100; cross-agent ORB matches at different viewpoints
    //         routinely land in the 60-100 range and were being discarded
    const float ratio = 0.75f;
    const int maxHam = 100;

    // ----------------------------
    // High Quality Selection Thresholds (tune here)
    // ----------------------------
    const int   kMinObs                = 6;
    const float kMinFoundRatio         = 0.90f;
    const int   kMaxScaleLevelDiff     = 1;
    const float kMinViewCos            = 0.70f;
    const int   kMinGoodFracPercent    = 60;
    const int   kMinTemporalSpanFrames = 80;
    const int   kMaxDescHamMean        = 40;
    const int   kMaxDescHamMax         = 60;
    const double kMaxReprojErrPx       = 0.0;
    // ----------------------------

    // RANSAC + Fusion Thresholds
    // kRansacMinInliers: lowered 20->10; rely on pooled RANSAC for global validation
    // kMinPointsPerPair: lowered 20->8; sparse pairs now feed the pooled stage
    //                    instead of being discarded entirely
    // kMinInlierRatio: slightly relaxed to match lower per-pair expectations
    // kMinPairsForFusion: lowered 3->2; two consistent pairs are enough for pooling
    const double kRansacThresh       = 0.05;  // matches offline
    const int    kRansacIters        = 2000;
    const int    kRansacMinInliers   = 10;
    const int    kMinPointsPerPair   = 8;
    const double kMinInlierRatio     = 0.15;
    const int    kMinPairsForFusion  = 2;

    const double bow_floor_score = 0.02;
    const double bow_rel_cut    = 0.70;
    const bool printLog = false;
    // ----------------------------

    // Foreign relocalization thresholds
    // BoW floor is lower than bow_floor_score: foreign BoWs are built from HQ
    // points only, so absolute scores run lower than full-frame comparisons.
    const double kForeignRelocBowFloor    = 0.015;
    const int    kForeignRelocTopK        = 5;
    const int    kForeignRelocMinMatches  = 20;   // 2D-3D pairs required before PnP
    const int    kForeignRelocMinInliers  = 25;   // PnP RANSAC inliers to accept
    const float  kForeignRelocReprojErr   = 5.0f; // px
    // ----------------------------


} 


static inline bool finite3(const cv::Point3f& p) {
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}

static cv::Mat stack_rows(const std::vector<cv::Mat>& descs) 
{
    CV_Assert(!descs.empty());
    cv::Mat out((int)descs.size(), 32, CV_8U);
    for (int i=0; i<(int)descs.size(); ++i) {
        CV_Assert(descs[i].type()==CV_8U && descs[i].rows==1 && descs[i].cols==32);
        descs[i].copyTo(out.row(i));
    }
    return out;
}

static bool umeyama_rigid(const std::vector<cv::Point3f>& P,
                          const std::vector<cv::Point3f>& Q,
                          SE3& T)
{
    const int N = (int)P.size();
    if (N < 3 || (int)Q.size()!=N) return false;

    // centroids
    cv::Vec3d muP(0,0,0), muQ(0,0,0);
    for (int i=0; i<N; ++i) {
        muP += cv::Vec3d(P[i].x, P[i].y, P[i].z);
        muQ += cv::Vec3d(Q[i].x, Q[i].y, Q[i].z);
    }
    muP *= (1.0/N); muQ *= (1.0/N);

    // covariance H = Σ (p-muP)(q-muQ)^T
    cv::Matx33d H(0,0,0, 0,0,0, 0,0,0);
    for (int i=0;i<N;++i){
        cv::Vec3d p = cv::Vec3d(P[i].x,P[i].y,P[i].z) - muP;
        cv::Vec3d q = cv::Vec3d(Q[i].x,Q[i].y,Q[i].z) - muQ;
        H(0,0)+=p[0]*q[0]; H(0,1)+=p[0]*q[1]; H(0,2)+=p[0]*q[2];
        H(1,0)+=p[1]*q[0]; H(1,1)+=p[1]*q[1]; H(1,2)+=p[1]*q[2];
        H(2,0)+=p[2]*q[0]; H(2,1)+=p[2]*q[1]; H(2,2)+=p[2]*q[2];
    }

    cv::SVD svd(cv::Mat(H), cv::SVD::MODIFY_A);
    cv::Matx33d U = svd.u, Vt = svd.vt;
    cv::Matx33d R = Vt.t() * U.t();

    // reflection fix
    if (cv::determinant(R) < 0) {
        Vt(2,0) *= -1; Vt(2,1) *= -1; Vt(2,2) *= -1;
        R = Vt.t() * U.t();
    }

    cv::Vec3d t = muQ - R * muP;

    T.R = R;
    T.t = t;
    return true;
}

static inline bool non_degenerate_3pt(const cv::Point3f& a,
                                      const cv::Point3f& b,
                                      const cv::Point3f& c,
                                      double eps = 1e-8)
{
    // triangle area ~ |(b-a) x (c-a)|; if near zero => collinear/degenerate
    cv::Vec3d ab(b.x - a.x, b.y - a.y, b.z - a.z);
    cv::Vec3d ac(c.x - a.x, c.y - a.y, c.z - a.z);
    cv::Vec3d cr = ab.cross(ac);
    return (cr.dot(cr) > eps);   // squared magnitude
}

static RansacSE3 estimate_se3_ransac(
    const std::vector<cv::Point3f>& P1,
    const std::vector<cv::Point3f>& P2,
    double thresh = 0.07,
    int maxIters = 1000,
    int minInliers = 20,
    unsigned seed = 12345)
{
    RansacSE3 out;
    const int N = (int)P1.size();
    if (N < 3 || (int)P2.size() != N) {
        if (printLog) std::cout << "[RANSAC] early exit: N=" << N
                                << " P2.size=" << P2.size() << "\n";
        return out;
    }

    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> uni(0, N-1);

    int idx[3];  // always 3 points, matches offline
    const double th2 = thresh * thresh;

    int best_inl = -1;
    SE3 best_T;
    std::vector<int> best_set;

    // buffers reused across iterations
    std::vector<cv::Point3f> sP(3), sQ(3);
    std::vector<int> inl;
    inl.reserve(N);

    for (int it = 0; it < maxIters; ++it) {
        // sample 3 distinct indices
        for (;;) {
            idx[0]=uni(rng); idx[1]=uni(rng); idx[2]=uni(rng);
            if (idx[0]!=idx[1] && idx[0]!=idx[2] && idx[1]!=idx[2]) break;
        }

        // skip collinear/degenerate samples before the SVD
        if (!non_degenerate_3pt(P1[idx[0]], P1[idx[1]], P1[idx[2]])) continue;

        for (int k=0; k<3; ++k) {sP[k]=P1[idx[k]]; sQ[k]=P2[idx[k]];}

        SE3 Tm;
        if (!umeyama_rigid(sP, sQ, Tm)) continue;

        // count inliers (squared error, no sqrt per point)
        inl.clear();
        for (int i=0; i<N; ++i) {
            cv::Vec3d x(P1[i].x, P1[i].y, P1[i].z);
            cv::Vec3d d = Tm.R * x + Tm.t - cv::Vec3d(P2[i].x, P2[i].y, P2[i].z);
            if (d.dot(d) < th2) inl.push_back(i);
        }

        if ((int)inl.size() > best_inl) {
            best_inl = (int)inl.size();
            best_T = Tm;
            best_set = inl;
            if (best_inl == N) break;  // perfect consensus, no better model exists
        }
    }

    // std::cout << "[RANSAC] best_inl=" << best_inl 
    //           << " required=" << std::max(3, minInliers) 
    //           << " N=" << N << "\n";

    if (best_inl < std::max(3, minInliers)) return out;

    // refit with all inliers
    std::vector<cv::Point3f> iP, iQ;
    iP.reserve(best_set.size()); iQ.reserve(best_set.size());
    for (int id : best_set) {iP.push_back(P1[id]); iQ.push_back(P2[id]);}

    SE3 refit;

    if (!umeyama_rigid(iP, iQ, refit)) {
        if (printLog) std::cout << "[RANSAC] refit failed in Umeyama\n";
        return out;
    }

    out.model = refit;
    out.inliers = std::move(best_set);
    out.ok = true;
    return out;
}

static cv::Vec4d matR_to_quat(const cv::Matx33d& Rm) 
{
    // Returns quaternion as (w, x, y, z)
    const double m00 = Rm(0,0), m01 = Rm(0,1), m02 = Rm(0,2);
    const double m10 = Rm(1,0), m11 = Rm(1,1), m12 = Rm(1,2);
    const double m20 = Rm(2,0), m21 = Rm(2,1), m22 = Rm(2,2);

    double trace = m00 + m11 + m22;
    double w, x, y, z;

    if (trace > 0.0) {
        double s = std::sqrt(trace + 1.0) * 2.0; // s = 4*w
        w = 0.25 * s;
        x = (m21 - m12) / s;
        y = (m02 - m20) / s;
        z = (m10 - m01) / s;
    } else {
        // find largest diagonal
        if (m00 > m11 && m00 > m22) {
            double s = std::sqrt(1.0 + m00 - m11 - m22) * 2.0; // s = 4*x
            w = (m21 - m12) / s;
            x = 0.25 * s;
            y = (m01 + m10) / s;
            z = (m02 + m20) / s;
        } else if (m11 > m22) {
            double s = std::sqrt(1.0 + m11 - m00 - m22) * 2.0; // s = 4*y
            w = (m02 - m20) / s;
            x = (m01 + m10) / s;
            y = 0.25 * s;
            z = (m12 + m21) / s;
        } else {
            double s = std::sqrt(1.0 + m22 - m00 - m11) * 2.0; // s = 4*z
            w = (m10 - m01) / s;
            x = (m02 + m20) / s;
            y = (m12 + m21) / s;
            z = 0.25 * s;
        }
    }

    // Normalize to be safe
    double n = std::sqrt(w*w + x*x + y*y + z*z);
    if (n > 0) { w/=n; x/=n; y/=n; z/=n; }
    return cv::Vec4d(w, x, y, z);
}

static cv::Matx33d quat_to_matR(const cv::Vec4d& qin) 
{
    // q = (w, x, y, z)
    double w = qin[0], x = qin[1], y = qin[2], z = qin[3];
    double n = std::sqrt(w*w + x*x + y*y + z*z);
    if (n == 0.0) return cv::Matx33d::eye();
    w/=n; x/=n; y/=n; z/=n;

    cv::Matx33d R;
    R(0,0) = 1 - 2*(y*y + z*z);
    R(0,1) = 2*(x*y - z*w);
    R(0,2) = 2*(x*z + y*w);

    R(1,0) = 2*(x*y + z*w);
    R(1,1) = 1 - 2*(x*x + z*z);
    R(1,2) = 2*(y*z - x*w);

    R(2,0) = 2*(x*z - y*w);
    R(2,1) = 2*(y*z + x*w);
    R(2,2) = 1 - 2*(x*x + y*y);
    return R;
}


static SE3 fuse_transforms_weighted(const std::vector<EstSE3Weighted>& ests)
{
    // weights = inliers (or inliers * score)
    cv::Vec4d qsum(0,0,0,0);
    cv::Vec3d tsum(0,0,0);
    double W = 0.0;

    for (const auto& e : ests) {
        double w = std::max(1, e.inliers);
        cv::Vec4d q = matR_to_quat(e.T.R);
        
        // ensure same hemisphere to avoid cancellation
        if (qsum[0]*q[0]+qsum[1]*q[1]+qsum[2]*q[2]+qsum[3]*q[3] < 0) {
            for (int i=0;i<4;++i) q[i] = -q[i];
        }

        for (int i=0;i<4;++i) qsum[i] += w * q[i];
        tsum += w * e.T.t;
        W +=w;
    }

    for (int i=0; i<4; ++i) qsum[i] /= std::max(1e-9, W);
    tsum /= std::max(1e-9, W);

    SE3 T;
    T.R = quat_to_matR(qsum);
    T.t = tsum;
    return T;
}

static inline cv::Point3f apply_se3(const SE3& T, const cv::Point3f& p)
{
    cv::Vec3d x(p.x, p.y, p.z);
    cv::Vec3d y = T.R * x + T.t;
    return cv::Point3f((float)y[0], (float)y[1], (float)y[2]);
}

static SE3 invert(const SE3& T)
{
    SE3 Ti;
    Ti.R = T.R.t();
    Ti.t = -(Ti.R * T.t);
    return Ti;
}

// Descriptor matrix + aligned 3D points for one keyframe's HQ map points.
struct KFPointData {
    cv::Mat A;                       // Nx32 CV_8U, one descriptor per row
    std::vector<cv::Point3f> pts;    // aligned with rows of A
};

static KFPointData extract_host_kf_data(KeyFrame* pkf)
{
    KFPointData out;
    if (!pkf) return out;

    vector<MapPoint*> vpMp = pkf->GetHighQualityMapPoints();

    std::vector<cv::Mat> D1v;
    D1v.reserve(vpMp.size());
    out.pts.reserve(vpMp.size());

    for (auto& pMp: vpMp) {
        if (!pMp || pMp->isBad()) continue;

        cv::Mat d = pMp->GetDescriptor();  // locks mMutexFeatures, returns clone

        if (d.empty()) continue;
        if (d.rows != 1) d = d.reshape(1, 1);           // force one row
        if (d.type() != CV_8U) d.convertTo(d, CV_8U);   // should already be CV_8U

        cv::Mat Xw = pMp->GetWorldPos();
        if (Xw.empty()) continue;
        Xw = Xw.reshape(1, 3);

        cv::Point3f p;
        p.x = Xw.at<float>(0);
        p.y = Xw.at<float>(1);
        p.z = Xw.at<float>(2);
        if (!finite3(p)) continue;

        D1v.push_back(d);
        out.pts.push_back(p);
    }

    if (!D1v.empty()) out.A = stack_rows(D1v);
    return out;
}

static Pairs3D match_3d_pairs(
    const KFPointData& host,
    const cv::Mat& B, const std::vector<cv::Point3f>& P2v,
    float ratio, int maxHamming,
    long unsigned int kf_me_id, int kf2)
{
    Pairs3D out;

    const cv::Mat& A = host.A;
    const std::vector<cv::Point3f>& P1v = host.pts;

    if (A.empty() || B.empty()) return out;
    if (A.rows != (int)P1v.size() || B.rows != (int)P2v.size()) return out;

    cv::BFMatcher matcher(cv::NORM_HAMMING, false);
    std::vector<std::vector<cv::DMatch>> knn;
    matcher.knnMatch(A, B, knn, 2);

    int raw_matches = 0;
    out.P1.reserve(knn.size());
    out.P2.reserve(knn.size());

    for (auto& v : knn) {
        if (v.empty()) continue;
        ++raw_matches;

        if (v.size() < 2) continue;

        const auto& m1 = v[0];
        const auto& m2 = v[1];
        if (m1.distance > maxHamming || m1.distance > ratio*m2.distance) continue;

        if (m1.queryIdx < 0 || m1.queryIdx >= (int)P1v.size()) continue;
        if (m1.trainIdx < 0 || m1.trainIdx >= (int)P2v.size()) continue;

        const cv::Point3f& X1 = P1v[m1.queryIdx];
        const cv::Point3f& X2 = P2v[m1.trainIdx];
        if (!finite3(X1) || !finite3(X2)) continue;

        out.P1.push_back(X1);
        out.P2.push_back(X2);
    }

    if (printLog) {
        std::cout << "[MATCH DEBUG] KF_me=" << kf_me_id
            << " KF_other=" << kf2
            << " raw_knn=" << raw_matches
            << " good=" << out.P1.size() << std::endl;
    }

    return out;
}


HighQualityManager::HighQualityManager(Map* pMap, ORBVocabulary* mpVoc,
    const std::string& criteria, double period_sec, string agentName)
:msAgentName(agentName), mpMap(pMap), mCriteria(criteria), mPeriodSec(period_sec), mpVoc(mpVoc) {}


void HighQualityManager::Run()
{
    using namespace std::chrono;
    while (true) {
      try {

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

        // resolve criteria once instead of comparing strings per map point
        enum class Crit { Observation, FoundRatio, Combo, None };
        const Crit critMode = (crit == "observation") ? Crit::Observation
                            : (crit == "foundratio")  ? Crit::FoundRatio
                            : (crit == "combo")       ? Crit::Combo
                                                      : Crit::None;

        for (MapPoint* pMP : vMPs) {
            if (!pMP) continue;
            bool isHQ = false;

            if (pMP->isBad()) {

                ApplyToMapPoint(pMP, isHQ);
                mpMap->RemoveHighQaulityMapPoints(pMP);

            } else {

                switch (critMode) {
                    case Crit::Observation:
                        isHQ = (pMP->Observations() >= kMinObs);
                        break;
                    case Crit::FoundRatio:
                        isHQ = (pMP->GetFoundRatio() >= kMinFoundRatio);
                        break;
                    case Crit::Combo:
                        isHQ = (pMP->Observations() >= kMinObs) &&
                               (pMP->GetFoundRatio() >= kMinFoundRatio);
                        break;
                    case Crit::None:
                        break;
                }

                ApplyToMapPoint(pMP, isHQ);
                if (isHQ) {
                    mpMap->AddHighQualityMapPoints(pMP);
                } else {
                    mpMap->RemoveHighQaulityMapPoints(pMP);
                }
            }
        }
        vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
        for (KeyFrame* pKF : vpKFs) {
            if (!pKF) continue;

            if (pKF->NeedsHQBoWUpdate()) {
                pKF->ComputeHQBoW();
                pKF->ClearHQBoWUpdateFlag();
            }
        }
        // Debugging
        vector<MapPoint*> mapPoints = mpMap->GetHighQualityMapPoints();
        std::cout << "\nHost map point size: " << mapPoints.size() << std::endl;
        mapPoints.clear();

        {
            std::unique_lock<std::mutex> glock(gBucketsMx);
            for (auto& ka: nMpsPerAgent) {

                std::cout << "Agent " << ka.first << " has " << ka.second.size() << " Map Points. \n";
            }
        }

        // 1) Snapshot only the fields the SE3 estimation reads, and only for
        //    OTHER agents with data — keeps the time under gBucketsMx short.
        std::map<std::string, AgentSnapshot> bucketsCopy;
        {
            std::unique_lock<std::mutex> glock(gBucketsMx);
            for (const auto &ka : gAgentBuckets) {
                if (ka.first == msAgentName) continue;
                if (ka.second.kf2bow.empty()) continue;
                AgentSnapshot &s = bucketsCopy[ka.first];
                s.kf2descs = ka.second.kf2descs;
                s.kf2pts   = ka.second.kf2pts;
                s.kf2bow   = ka.second.kf2bow;
            }
        }

        // thresholds for the pooled refinement
        const int kMinMatchedFramesForPool = 2;
        const int kMinPoolSizeForRansac    = 20;

        auto make_seed = [&](int a, int b, unsigned base=1337u){
            // stable-ish seed per pair
            return base ^ (unsigned)(a * 73856093) ^ (unsigned)(b * 19349663);
        };

        auto rad2deg = [](double r){ return r * 180.0 / M_PI; };

        if (bucketsCopy.empty()) {
            std::cerr << "No information received from other agents, waiting ...\n";
        } else {

            // Host-KF descriptor matrices are reused across all other agents;
            // build lazily and cache for this cycle.
            std::unordered_map<KeyFrame*, KFPointData> hostKFCache;
            auto getHostData = [&](KeyFrame* pkf) -> const KFPointData& {
                auto it = hostKFCache.find(pkf);
                if (it == hostKFCache.end())
                    it = hostKFCache.emplace(pkf, extract_host_kf_data(pkf)).first;
                return it->second;
            };

            // Transforms computed this cycle; applied under one lock afterwards
            std::unordered_map<std::string, SE3> newTransforms;

            // For each OTHER agent, compute SE3 from snapshot (no locks here)
            for (auto &ka : bucketsCopy) {
                const std::string &otherName = ka.first;
                AgentSnapshot &otherBucketSnap = ka.second;

                // Several host KFs can match the same other-agent KF; stack its
                // descriptor matrix once per cycle.
                std::unordered_map<int, cv::Mat> stackedOtherDescs;

                std::vector<Candidate> matchedCandidates;
                matchedCandidates.reserve(vpKFs.size());

                // kf pairing: keep only the single best-scoring other-KF per host KF
                for (KeyFrame* pkf_me : vpKFs) {
                    const auto &bow_me = pkf_me->mHQBowVec;

                    Candidate best{pkf_me, -1, 0.0};
                    for (const auto &kv_other : otherBucketSnap.kf2bow) {
                        double sc = mpVoc->score(bow_me, kv_other.second);
                        if (sc > best.score) { best.kf = kv_other.first; best.score = sc; }
                    }

                    if (best.kf >= 0 && best.score >= bow_floor_score)
                        matchedCandidates.push_back(best);
                }


                if (matchedCandidates.size() < (size_t)kMinPairsForFusion) {

                    if (printLog) {
                        std::cout << "[SE3] Skipping agent " << otherName
                                << ": only " << matchedCandidates.size()
                                << " KF matches.\n";
                    }

                    continue;
                }

                // --- stats you asked for ---
                int matchedFramesAfterRansac = 0;      // how many KF pairs survived pair-RANSAC
                size_t pooledInliersCount    = 0;      // total pooled correspondences (all inliers across pairs)
                int pooledRansacInliers      = -1;     // inliers from the final pooled RANSAC (if run)

                // pool of inlier correspondences across KF pairs
                std::vector<cv::Point3f> poolP1, poolP2;
                poolP1.reserve(1024);
                poolP2.reserve(1024);

                // also keep per-pair estimates for fallback fusion (your old approach)
                std::vector<PairEst> pair_ests;
                pair_ests.reserve(matchedCandidates.size());

                for (auto &candidate : matchedCandidates) {
                    KeyFrame* pkf = candidate.pkf;
                    const int kf_other = candidate.kf;

                    auto itP2 = otherBucketSnap.kf2pts.find(kf_other);
                    if (itP2 == otherBucketSnap.kf2pts.end()) continue;

                    auto itB = stackedOtherDescs.find(kf_other);
                    if (itB == stackedOtherDescs.end()) {
                        auto itD2 = otherBucketSnap.kf2descs.find(kf_other);
                        if (itD2 == otherBucketSnap.kf2descs.end() || itD2->second.empty()) continue;
                        itB = stackedOtherDescs.emplace(kf_other, stack_rows(itD2->second)).first;
                    }

                    Pairs3D pairs = match_3d_pairs(
                        getHostData(pkf),
                        itB->second, itP2->second,
                        ratio, maxHam,
                        pkf->mnId, kf_other
                    );

                    const int N = (int)pairs.P1.size();
                    if (N < kMinPointsPerPair) {

                        if (printLog) {
                            std::cout << "[SE3] Pair " << pkf->mnId << " - " << kf_other
                                  << " rejected: N = " << N << " < " << kMinPointsPerPair << "\n";
                        }

                        continue;
                    }

                    RansacSE3 r = estimate_se3_ransac(
                        pairs.P1,
                        pairs.P2,
                        kRansacThresh,
                        kRansacIters,
                        kRansacMinInliers,
                        make_seed(pkf->mnId, kf_other)
                    );

                    if (!r.ok) {

                        if (printLog) {
                            std::cout << "[SE3] Pair " << pkf->mnId << " - " << kf_other
                                  << " RANSAC failed.\n";
                        }

                        continue;
                    }

                    const int inl = (int)r.inliers.size();
                    const double ratio_inl = (N > 0) ? (double)inl / (double)N : 0.0;

                    if (inl < kRansacMinInliers || ratio_inl < kMinInlierRatio) {

                        if (printLog) {
                            std::cout << "[SE3] Pair " << pkf->mnId << " - " << kf_other
                                    << " rejected: inliers=" << inl
                                    << " N=" << N
                                    << " ratio=" << ratio_inl << "\n";
                        }

                        continue;
                    }

                    // ACCEPTED pair
                    matchedFramesAfterRansac++;
                    pair_ests.push_back({pkf, kf_other, r, candidate.score});

                    // pool the INLIER correspondences for the second-stage RANSAC
                    poolP1.reserve(poolP1.size() + r.inliers.size());
                    poolP2.reserve(poolP2.size() + r.inliers.size());

                    for (int idx : r.inliers) {
                        poolP1.push_back(pairs.P1[idx]);
                        poolP2.push_back(pairs.P2[idx]);
                    }
                }

                pooledInliersCount = poolP1.size();

                // Print per-agent summary at this time step
                if (printLog) {
                    std::cout << "[SE3] Agent " << otherName
                            << " matchedFrames(after pair-RANSAC)=" << matchedFramesAfterRansac
                            << " pooledInliers=" << pooledInliersCount
                            << "\n";
                }


                if (matchedFramesAfterRansac < kMinPairsForFusion) {
                    if (printLog) {
                        std::cout << "[SE3] Agent " << otherName
                                << " has only " << matchedFramesAfterRansac
                                << " good pairs; skipping.\n";
                    }

                    continue;
                }

                // ---- second-stage pooled RANSAC ----
                SE3 T_other_from_me;
                bool usedPooled = false;

                if (matchedFramesAfterRansac >= kMinMatchedFramesForPool &&
                    (int)pooledInliersCount >= kMinPoolSizeForRansac)
                {
                    RansacSE3 rp = estimate_se3_ransac(
                        poolP1,
                        poolP2,
                        kRansacThresh,
                        kRansacIters,
                        kRansacMinInliers,
                        99991u
                    );

                    if (rp.ok) {
                        usedPooled = true;
                        T_other_from_me = rp.model;
                        pooledRansacInliers = (int)rp.inliers.size();
                        if (printLog) {
                            std::cout << "[SE3] Agent " << otherName
                                    << " pooled-RANSAC inliers=" << pooledRansacInliers
                                    << " / " << pooledInliersCount
                                    << " (ratio=" << (pooledInliersCount ? (double)pooledRansacInliers / pooledInliersCount : 0.0)
                                    << ")\n";
                        }

                    } else {
                        if (printLog) {
                            std::cout << "[SE3] Agent " << otherName
                                    << " pooled-RANSAC failed; falling back to fusion.\n";
                        }
                    }
                }

                // ---- fallback: fuse per-pair SE3s (only when pooling failed) ----
                if (!usedPooled) {
                    std::vector<EstSE3Weighted> ests;
                    ests.reserve(pair_ests.size());
                    for (const auto &pe : pair_ests) {
                        EstSE3Weighted e;
                        e.T       = pe.est.model;
                        e.inliers = (int)pe.est.inliers.size();
                        e.score   = pe.bow_score;
                        ests.push_back(e);
                    }
                    T_other_from_me = fuse_transforms_weighted(ests);
                }

                // We want transform from other agent -> this agent
                const SE3 T_me_from_other = invert(T_other_from_me);
                newTransforms[otherName] = T_me_from_other;

                // Debug: print Euler angles (in degrees)
                const cv::Matx33d &R = T_me_from_other.R;
                double yaw   = std::atan2(R(1,0), R(0,0));
                double pitch = std::asin(-R(2,0));
                double roll  = std::atan2(R(2,1), R(2,2));

                std::cout << "\nTransform (" << (usedPooled ? "pooled RANSAC" : "weighted fusion")
                        << ") " << otherName
                        << " --> " << msAgentName << ":\n"
                        << "R = \n" << cv::Mat(T_me_from_other.R) << "\n"
                        << "t = " << T_me_from_other.t << "\n"
                        << "angles (deg): roll="  << rad2deg(roll)
                        << " pitch=" << rad2deg(pitch)
                        << " yaw="   << rad2deg(yaw) << "\n\n";
            }

            // Publish new transforms and update live buckets under one short lock
            if (!newTransforms.empty()) {
                std::unique_lock<std::mutex> glock(gBucketsMx);
                for (const auto &kv : newTransforms) {
                    transformsToApply[kv.first] = kv.second;

                    auto itLive = gAgentBuckets.find(kv.first);
                    if (itLive == gAgentBuckets.end()) continue;
                    itLive->second.T = kv.second;
                    itLive->second.hasTransform = true;
                }
            }
        }
      } catch (const cv::Exception& e) {
          std::cerr << "[HQmanager::Run] cv::Exception: " << e.what() << "\n";
      } catch (const std::exception& e) {
          std::cerr << "[HQmanager::Run] exception: " << e.what() << "\n";
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
    // if (wasHQ && !isHQ) std::cout << "\nturned to not hq ...." << std::endl;
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

        size_t taken = 0;
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

    std::ofstream ofs(csv_path.c_str());
    if (!ofs.is_open()) {
        std::cerr << "[HQManager] Failed to open " << csv_path << " for writing.\n";
        return;
    }

    ofs << std::fixed << std::setprecision(6);
    ofs << "mp_id,mp_x,mp_y,mp_z,descriptor_hex,observations\n";

    size_t rows_written = 0;

    // Write one MapPoint row.
    // For host points, obs come from GetObservations() (KeyFrame* map).
    // For received points, obs come from mvnObservations (vector<int>).
    // Both are written as semicolon-separated integers in the observations column.
    auto writeMP = [](std::ofstream& os, MapPoint* pMP,
                      const std::vector<int>& obs_ids) {
        cv::Mat Xw = pMP->GetWorldPos();
        double X = 0.0, Y = 0.0, Z = 0.0;
        if (!Xw.empty() && Xw.rows >= 3 && Xw.cols >= 1) {
            X = static_cast<double>(Xw.at<float>(0));
            Y = static_cast<double>(Xw.at<float>(1));
            Z = static_cast<double>(Xw.at<float>(2));
        }
        std::string hex = DescriptorRowToHex(pMP->GetDescriptor());

        std::vector<int> sorted_ids = obs_ids;
        std::sort(sorted_ids.begin(), sorted_ids.end());
        sorted_ids.erase(std::unique(sorted_ids.begin(), sorted_ids.end()), sorted_ids.end());

        os << pMP->mnId << ","
           << X << "," << Y << "," << Z << ","
           << "\"" << hex << "\",\"";
        for (size_t i = 0; i < sorted_ids.size(); ++i) {
            if (i) os << ';';
            os << sorted_ids[i];
        }
        os << "\"\n";
    };

    // --- Host HQ map points ---
    std::vector<MapPoint*> vMPs = mpMap->GetHighQualityMapPoints();
    vMPs.erase(std::remove_if(vMPs.begin(), vMPs.end(),
                              [](MapPoint* p){ return !p || p->isBad(); }),
               vMPs.end());

    for (MapPoint* pMP : vMPs) {
        if (!pMP) continue;

        const std::map<KeyFrame*, size_t> obs = pMP->GetObservations();
        if (obs.empty()) continue;

        std::vector<int> obs_ids;
        obs_ids.reserve(obs.size());
        for (const auto& kv : obs) {
            KeyFrame* pKF = kv.first;
            if (!pKF || pKF->isBad()) continue;
            obs_ids.push_back(static_cast<int>(pKF->mnFrameId));
        }
        if (obs_ids.empty()) continue;

        writeMP(ofs, pMP, obs_ids);
        ++rows_written;
    }

    ofs.close();
    std::cout << "[HQManager] Exported " << rows_written
              << " host MapPoints to " << csv_path << "\n";

    // --- Received agent map points (each agent gets its own file) ---
    // Derive directory from csv_path (e.g. "Results/mappoint_descriptors.csv" → "Results/")
    std::string dir;
    {
        size_t slash = csv_path.rfind('/');
        if (slash != std::string::npos)
            dir = csv_path.substr(0, slash + 1);
    }

    std::unique_lock<std::mutex> glock(gBucketsMx);
    for (const auto& ka : nMpsPerAgent) {
        const std::string& agentName = ka.first;
        const std::string agent_csv = dir + agentName + "_mappoint_descriptors.csv";

        std::ofstream aofs(agent_csv.c_str());
        if (!aofs.is_open()) {
            std::cerr << "[HQManager] Failed to open " << agent_csv << " for writing.\n";
            continue;
        }
        aofs << std::fixed << std::setprecision(6);
        aofs << "mp_id,mp_x,mp_y,mp_z,descriptor_hex,observations\n";

        size_t agent_rows = 0;
        for (const auto& kv : ka.second) {
            MapPoint* pMP = kv.second;
            if (!pMP || pMP->isBad()) continue;
            if (pMP->mvnObservations.empty()) continue;
            writeMP(aofs, pMP, pMP->mvnObservations);
            ++agent_rows;
        }

        aofs.close();
        std::cout << "[HQManager] Exported " << agent_rows
                  << " MapPoints for agent " << agentName << " to " << agent_csv << "\n";
    }
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

vector<cv::Point3f> HighQualityManager::ExportMergedMap() {

    vector<cv::Point3f> out;

    // ── Host map points (already in this agent's frame) ──────────────────────
    std::vector<MapPoint*> vMPs = mpMap->GetHighQualityMapPoints();
    out.reserve(vMPs.size());
    for (MapPoint* pMP : vMPs) {
        if (!pMP || pMP->isBad()) continue;
        cv::Mat pos = pMP->GetWorldPos();
        if (pos.empty()) continue;
        cv::Point3f p(pos.at<float>(0), pos.at<float>(1), pos.at<float>(2));
        if (!finite3(p)) continue;
        out.push_back(p);
    }

    // ── Received agent map points (need SE3 transform into this frame) ────────
    // Read positions while holding gBucketsMx: ImportHighQualityMapPoints may
    // delete these MapPoints concurrently, so pointers must not escape the lock.
    {
        std::unique_lock<std::mutex> glock(gBucketsMx);

        for (const auto& agentToPts : nMpsPerAgent) {
            auto tit = transformsToApply.find(agentToPts.first);
            if (tit == transformsToApply.end()) continue;
            const SE3& T = tit->second;                // T_me_from_other
            const auto& idToPts = agentToPts.second;
            if (idToPts.empty()) continue;
            out.reserve(out.size() + idToPts.size());
            for (const auto& idToPt : idToPts) {
                MapPoint* pMP = idToPt.second;
                if (!pMP || pMP->isBad()) continue;
                cv::Mat pos = pMP->GetWorldPos();
                if (pos.empty()) continue;
                cv::Point3f p(pos.at<float>(0), pos.at<float>(1), pos.at<float>(2));
                if (!finite3(p)) continue;
                out.push_back(apply_se3(T, p));        // transform into this agent's frame
            }
        }
    }

    return out;
}
void HighQualityManager::ImportHighQualityMapPoints(
    const std::string &agent_name,
    const std::vector<MapPoint*> &vMPs)
{   
    if (agent_name == msAgentName) return;

    std::unordered_set<int> updated_keyframes;
    // Descriptor snapshots for BoW computation (taken under lock, computed outside)
    std::unordered_map<int, std::vector<cv::Mat>> kf_descs_for_bow;

    // ==== LOCK: all bucket mutations ====
    {
        std::unique_lock<std::mutex> glock(gBucketsMx);

        auto& agentMpMap = nMpsPerAgent[agent_name];
        AgentBuckets &agentBucket = gAgentBuckets[agent_name];

        // Remove one map point's entry from one keyframe's aligned vectors in
        // O(1) via kf2mpidx + swap-and-pop (vector order carries no meaning).
        auto removeMpFromKF = [&](int kf_id, int mpid) {
            auto idxIt = agentBucket.kf2mpidx.find(kf_id);
            if (idxIt == agentBucket.kf2mpidx.end()) return;
            auto &idxMap = idxIt->second;
            auto ii = idxMap.find(mpid);
            if (ii == idxMap.end()) return;
            const int i = ii->second;

            auto &vD = agentBucket.kf2descs[kf_id];
            auto &vP = agentBucket.kf2pts[kf_id];
            auto &vM = agentBucket.kf2mpids[kf_id];

            const int last = (int)vM.size() - 1;
            if (i != last) {
                vM[i] = vM[last];
                vD[i] = vD[last];
                vP[i] = vP[last];
                idxMap[vM[i]] = i;
            }
            vM.pop_back(); vD.pop_back(); vP.pop_back();
            idxMap.erase(ii);

            if (vM.empty()) {
                agentBucket.kf2bow.erase(kf_id);
                agentBucket.kf2descs.erase(kf_id);
                agentBucket.kf2pts.erase(kf_id);
                agentBucket.kf2mpids.erase(kf_id);
                agentBucket.kf2mpidx.erase(kf_id);
                updated_keyframes.erase(kf_id);
            } else {
                updated_keyframes.insert(kf_id);
            }
        };

        // Purge a map point from every keyframe bucket that still holds it.
        auto purgeMp = [&](int mpid) {
            auto prevIt = agentBucket.mpid2kfids.find(mpid);
            if (prevIt == agentBucket.mpid2kfids.end()) return;
            for (int kf_id : prevIt->second)
                removeMpFromKF(kf_id, mpid);
            agentBucket.mpid2kfids.erase(prevIt);
        };

        for (ORB_SLAM2::MapPoint* pSrcMP : vMPs) {
            if (!pSrcMP) continue;
            const int mpid = static_cast<int>(pSrcMP->mnId);

            // --- bad / no-longer-HQ points: drop stored copy AND bucket entries ---
            if (pSrcMP->isBad() || !pSrcMP->mbHighQaulity) {
                auto it = agentMpMap.find(mpid);
                if (it != agentMpMap.end()) {
                    delete it->second;
                    agentMpMap.erase(it);
                }
                purgeMp(mpid);
                continue;
            }

            // --- keep/refresh stored copy ---
            {
                auto it = agentMpMap.find(mpid);
                if (it != agentMpMap.end() && it->second != pSrcMP)
                    delete it->second;
                agentMpMap[mpid] = pSrcMP;
            }

            cv::Mat X = pSrcMP->GetWorldPos();
            const bool hasPos = (!X.empty() && X.rows >= 3 && X.cols >= 1);

            cv::Mat desc = pSrcMP->GetDescriptor();
            const bool hasDesc = (!desc.empty() && desc.rows == 1 && desc.cols == 32 && desc.type() == CV_8U);
            if (!hasDesc) {
                std::cerr << "[HQManager] MapPoint " << pSrcMP->mnId
                        << " from agent " << agent_name
                        << " has invalid descriptor; rows=" << desc.rows
                        << " cols=" << desc.cols << " type=" << desc.type() << "\n";
            }

            std::unordered_set<int> newKfIds(pSrcMP->mvnObservations.begin(),
                                             pSrcMP->mvnObservations.end());

            // remove stale entries: keyframes that no longer observe this point
            auto prevIt = agentBucket.mpid2kfids.find(mpid);
            if (prevIt != agentBucket.mpid2kfids.end()) {
                for (int old_kf : prevIt->second) {
                    if (newKfIds.count(old_kf)) continue;
                    removeMpFromKF(old_kf, mpid);
                }
            }

            agentBucket.mpid2kfids[mpid] = std::move(newKfIds);

            if (!hasDesc || !hasPos) continue;

            const cv::Point3f p{ X.at<float>(0), X.at<float>(1), X.at<float>(2) };

            for (int kf_id : pSrcMP->mvnObservations) {
                auto &vD = agentBucket.kf2descs[kf_id];
                auto &vP = agentBucket.kf2pts[kf_id];
                auto &vM = agentBucket.kf2mpids[kf_id];
                auto &idxMap = agentBucket.kf2mpidx[kf_id];

                auto ii = idxMap.find(mpid);
                if (ii != idxMap.end()) {
                    desc.copyTo(vD[ii->second]);
                    vP[ii->second] = p;
                } else {
                    idxMap[mpid] = (int)vM.size();
                    vD.push_back(desc.clone());
                    vP.push_back(p);
                    vM.push_back(mpid);
                }

                updated_keyframes.insert(kf_id);
            }
        }

        // Snapshot descriptors needed for BoW — shallow cv::Mat copies are safe
        // (ref-counted; data stays alive even if gAgentBuckets is later modified)
        for (int kf_id : updated_keyframes) {
            auto it = agentBucket.kf2descs.find(kf_id);
            if (it != agentBucket.kf2descs.end())
                kf_descs_for_bow[kf_id] = it->second;
        }
    }  // ==== gBucketsMx released ====


    // ==== BoW computation outside the lock ====
    if (!mpVoc || kf_descs_for_bow.empty()) {
        if (!mpVoc) std::cerr << "[HQManager] mpVoc null\n";
        return;
    }

    std::unordered_map<int, DBoW2::BowVector> new_bows;
    for (auto& kv : kf_descs_for_bow) {
        DBoW2::BowVector bow; DBoW2::FeatureVector feat;
        if (ComputeBowForKF(mpVoc, kv.second, bow, &feat) && !bow.empty())
            new_bows[kv.first] = std::move(bow);
    }

    // Write BoW results back under a short lock
    {
        std::unique_lock<std::mutex> glock(gBucketsMx);
        AgentBuckets &agentBucket = gAgentBuckets[agent_name];
        for (auto& kv : new_bows)
            agentBucket.kf2bow[kv.first] = std::move(kv.second);
    }

}

bool HighQualityManager::EstimateSE3_3D3D(const std::vector<cv::Point3f> &Pw,
                                          const std::vector<cv::Point3f> &Pc,
                                          cv::Mat &Tcw, std::vector<int> &vInliers,
                                          double thresh, int iters, int minInliers)
{
    Tcw.release();
    vInliers.clear();

    // run with the 3-point floor so the best consensus is always reported;
    // the caller-supplied acceptance threshold is applied afterwards
    RansacSE3 r = estimate_se3_ransac(Pw, Pc, thresh, iters, 3, 4242u);
    if (r.ok)
        vInliers = r.inliers;

    if (!r.ok || (int)vInliers.size() < minInliers)
        return false;

    Tcw = cv::Mat::eye(4, 4, CV_32F);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j)
            Tcw.at<float>(i, j) = (float)r.model.R(i, j);
        Tcw.at<float>(i, 3) = (float)r.model.t[i];
    }
    return true;
}

bool HighQualityManager::RelocalizeAgainstForeign(Frame &F, ForeignRelocResult &out)
{
    if (!mpVoc || F.mBowVec.empty()) return false;

    // ── 1) Snapshot agents that have an established transform ────────────────
    struct AgentRelocSnap {
        std::string name;
        SE3 T;  // T_me_from_other
        std::unordered_map<int, std::vector<cv::Mat>>     kf2descs;
        std::unordered_map<int, std::vector<cv::Point3f>> kf2pts;
        std::unordered_map<int, DBoW2::BowVector>         kf2bow;
    };
    std::vector<AgentRelocSnap> snaps;
    {
        std::unique_lock<std::mutex> glock(gBucketsMx);
        for (const auto &ka : gAgentBuckets) {
            if (ka.first == msAgentName) continue;
            if (!ka.second.hasTransform || ka.second.kf2bow.empty()) continue;
            AgentRelocSnap s;
            s.name     = ka.first;
            s.T        = ka.second.T;
            s.kf2descs = ka.second.kf2descs;   // shallow cv::Mat headers
            s.kf2pts   = ka.second.kf2pts;
            s.kf2bow   = ka.second.kf2bow;
            snaps.push_back(std::move(s));
        }
    }
    if (snaps.empty()) return false;

    // ── 2) BoW scoring: pick top-K foreign keyframes across all agents ───────
    struct KFCand { int snapIdx; int kf; double score; };
    std::vector<KFCand> cands;
    for (size_t si = 0; si < snaps.size(); ++si) {
        for (const auto &kv : snaps[si].kf2bow) {
            double sc = mpVoc->score(F.mBowVec, kv.second);
            if (sc >= kForeignRelocBowFloor)
                cands.push_back({(int)si, kv.first, sc});
        }
    }
    if (cands.empty()) return false;

    const size_t topK = std::min<size_t>(kForeignRelocTopK, cands.size());
    std::partial_sort(cands.begin(), cands.begin()+topK, cands.end(),
                      [](const KFCand &a, const KFCand &b){ return a.score > b.score; });
    cands.resize(topK);

    // ── 3) Descriptor matching: frame keypoints vs foreign HQ points ─────────
    // Pool 2D-3D correspondences across the candidate KFs; keep the best match
    // (lowest Hamming distance) per frame keypoint.
    struct Corr { cv::Point3f p; float dist; int snapIdx; };
    std::unordered_map<int, Corr> bestByKp;  // frame keypoint idx -> correspondence

    cv::BFMatcher matcher(cv::NORM_HAMMING, false);

    for (const auto &c : cands) {
        const AgentRelocSnap &s = snaps[c.snapIdx];

        auto itD = s.kf2descs.find(c.kf);
        auto itP = s.kf2pts.find(c.kf);
        if (itD == s.kf2descs.end() || itP == s.kf2pts.end()) continue;
        if (itD->second.empty() || itD->second.size() != itP->second.size()) continue;

        cv::Mat B = stack_rows(itD->second);
        const std::vector<cv::Point3f> &P2v = itP->second;

        std::vector<std::vector<cv::DMatch>> knn;
        matcher.knnMatch(F.mDescriptors, B, knn, 2);

        for (const auto &v : knn) {
            if (v.size() < 2) continue;
            const cv::DMatch &m = v[0];
            if (m.distance > maxHam || m.distance > ratio * v[1].distance) continue;
            if (m.trainIdx < 0 || m.trainIdx >= (int)P2v.size()) continue;

            const cv::Point3f &pOther = P2v[m.trainIdx];
            if (!finite3(pOther)) continue;

            auto it = bestByKp.find(m.queryIdx);
            if (it == bestByKp.end() || m.distance < it->second.dist)
                bestByKp[m.queryIdx] = {apply_se3(s.T, pOther), m.distance, c.snapIdx};
        }
    }

    if ((int)bestByKp.size() < kForeignRelocMinMatches) {
        if (printLog)
            std::cout << "[ForeignReloc] only " << bestByKp.size()
                      << " 2D-3D matches (need " << kForeignRelocMinMatches << ")\n";
        return false;
    }

    // ── 4) Pose estimation ────────────────────────────────────────────────────
    // Preferred: metric 3D-3D alignment using frame depth — the same approach
    // that fixed host-map relocalization (2D-3D PnP proved fragile on
    // outlier-heavy matches under viewpoint change). Fallback: PnP when depth
    // is unavailable for enough matches (e.g. monocular).
    std::vector<cv::Point3f> obj;   // matched foreign points, in host world frame
    std::vector<cv::Point2f> img;   // undistorted keypoint positions
    std::vector<int> kpIdx;
    std::vector<int> corrSnap;
    obj.reserve(bestByKp.size());
    img.reserve(bestByKp.size());
    kpIdx.reserve(bestByKp.size());
    corrSnap.reserve(bestByKp.size());

    for (const auto &kv : bestByKp) {
        obj.push_back(kv.second.p);
        img.push_back(F.mvKeysUn[kv.first].pt);
        kpIdx.push_back(kv.first);
        corrSnap.push_back(kv.second.snapIdx);
    }

    cv::Mat rvec, tvec;
    bool havePose = false;

    // -- 4a) 3D-3D: frame keypoints unprojected through depth vs foreign points
    {
        std::vector<cv::Point3f> Pw, Pc;
        Pw.reserve(obj.size());
        Pc.reserve(obj.size());
        for (size_t i = 0; i < obj.size(); ++i) {
            const float z = F.mvDepth.empty() ? -1.0f : F.mvDepth[kpIdx[i]];
            if (z <= 0) continue;
            const cv::KeyPoint &kp = F.mvKeysUn[kpIdx[i]];
            Pc.emplace_back((kp.pt.x - Frame::cx) * z * Frame::invfx,
                            (kp.pt.y - Frame::cy) * z * Frame::invfy,
                            z);
            Pw.push_back(obj[i]);
        }

        cv::Mat Tcw44;
        std::vector<int> d3Inliers;
        if ((int)Pw.size() >= 12 &&
            EstimateSE3_3D3D(Pw, Pc, Tcw44, d3Inliers, 0.10, 500, 12))
        {
            cv::Mat R64;
            Tcw44(cv::Rect(0, 0, 3, 3)).convertTo(R64, CV_64F);
            cv::Rodrigues(R64, rvec);
            tvec = (cv::Mat_<double>(3, 1) << Tcw44.at<float>(0, 3),
                                              Tcw44.at<float>(1, 3),
                                              Tcw44.at<float>(2, 3));
            havePose = true;
            if (printLog)
                std::cout << "[ForeignReloc] 3D-3D alignment: "
                          << d3Inliers.size() << "/" << Pw.size() << " inliers\n";
        }
    }

    // -- 4b) fallback: 2D-3D PnP (depth sparse or 3D-3D found no consensus)
    if (!havePose) {
        std::vector<int> inlierIdx;
        bool ok = cv::solvePnPRansac(obj, img, F.mK, cv::Mat(), rvec, tvec,
                                     false, 300, kForeignRelocReprojErr, 0.99,
                                     inlierIdx, cv::SOLVEPNP_EPNP);
        if (!ok || (int)inlierIdx.size() < kForeignRelocMinInliers) {
            if (printLog)
                std::cout << "[ForeignReloc] no pose (3D-3D failed, PnP inliers "
                          << inlierIdx.size() << "/" << obj.size() << ")\n";
            return false;
        }
        havePose = true;
    }

    // ── 5) Verification + refinement, mirroring the host path ────────────────
    // Reproject ALL pooled correspondences at the candidate pose and keep those
    // landing within the pixel threshold; refine on that set; verify again.
    auto verify = [&](std::vector<int> &vVerified) {
        vVerified.clear();
        std::vector<cv::Point2f> proj;
        cv::projectPoints(obj, rvec, tvec, F.mK, cv::Mat(), proj);
        const float th2 = kForeignRelocReprojErr * kForeignRelocReprojErr;
        cv::Mat R64;
        cv::Rodrigues(rvec, R64);
        for (size_t i = 0; i < obj.size(); ++i) {
            // point must be in front of the camera
            const double zc = R64.at<double>(2,0)*obj[i].x + R64.at<double>(2,1)*obj[i].y
                            + R64.at<double>(2,2)*obj[i].z + tvec.at<double>(2);
            if (zc <= 0) continue;
            const float dx = proj[i].x - img[i].x;
            const float dy = proj[i].y - img[i].y;
            if (dx*dx + dy*dy < th2)
                vVerified.push_back((int)i);
        }
    };

    std::vector<int> verified;
    verify(verified);
    if ((int)verified.size() >= kForeignRelocMinInliers) {
        std::vector<cv::Point3f> objV; objV.reserve(verified.size());
        std::vector<cv::Point2f> imgV; imgV.reserve(verified.size());
        for (int i : verified) { objV.push_back(obj[i]); imgV.push_back(img[i]); }
        cv::solvePnP(objV, imgV, F.mK, cv::Mat(), rvec, tvec,
                     true, cv::SOLVEPNP_ITERATIVE);
        verify(verified);
    }

    if ((int)verified.size() < kForeignRelocMinInliers) {
        if (printLog)
            std::cout << "[ForeignReloc] pose rejected in verification ("
                      << verified.size() << "/" << obj.size() << " reprojection inliers)\n";
        return false;
    }

    // ── 6) Compose Tcw and fill result ───────────────────────────────────────
    cv::Mat R;
    cv::Rodrigues(rvec, R);

    out.Tcw = cv::Mat::eye(4, 4, CV_32F);
    for (int r = 0; r < 3; ++r) {
        for (int col = 0; col < 3; ++col)
            out.Tcw.at<float>(r, col) = (float)R.at<double>(r, col);
        out.Tcw.at<float>(r, 3) = (float)tvec.at<double>(r);
    }

    // majority vote over the verified set decides which agent we credit
    std::unordered_map<int, int> snapVotes;
    for (int i : verified)
        snapVotes[corrSnap[i]]++;
    int majoritySnap = verified.empty() ? 0 : corrSnap[verified[0]];
    int bestVotes = 0;
    for (const auto &kv : snapVotes)
        if (kv.second > bestVotes) { bestVotes = kv.second; majoritySnap = kv.first; }

    out.inliers.clear();
    out.inliers.reserve(verified.size());
    for (int i : verified)
        out.inliers.emplace_back(kpIdx[i], obj[i]);
    out.agentName = snaps[majoritySnap].name;

    std::cout << "[ForeignReloc] relocalized against agent " << out.agentName
              << ": " << verified.size() << " verified inliers from "
              << obj.size() << " matches\n";
    return true;
}


}
