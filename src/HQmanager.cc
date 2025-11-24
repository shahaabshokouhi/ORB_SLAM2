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
struct Candidate { int kf; double score; };
struct Pair      { int kf1; int kf2; double score; };
struct Pairs3D {
    int kf2;
    std::vector<cv::Point3f> P1;
    std::vector<cv::Point3f> P2;
    std::vector<cv::DMatch>  matches;
};
struct RansacSE3 {
    SE3 model;
    std::vector<int> inliers;
    bool ok = false;
};
struct EstSE3Weighted { SE3 T; int inliers=0;};
struct PairEst { int kf1; int kf2; RansacSE3 est; };
struct AgentBuckets {
    std::unordered_map<int, std::vector<cv::Mat>>     kf2descs;
    std::unordered_map<int, std::vector<cv::Point3f>> kf2pts;
    std::unordered_map<int, std::vector<int>>         kf2mpids;
    std::unordered_map<int, DBoW2::BowVector>         kf2bow;
    std::unordered_map<int, Candidate> best_pairs; // ref map kp, second map kp
    std::unordered_map<int, Pairs3D> point_pairs;

    bool hasTransform = false;
    SE3  T_agent_to_local;
};



std::map<std::string, AgentBuckets> gAgentBuckets;
std::mutex gBucketsMx;
std::unordered_map<int, std::vector<Candidate>> matched_frames;
std::vector<PairEst> pair_ests;

const float ratio = 1.0f;
const int maxHam = 80;
const double ransac_thresh = 0.07; // meters
const int ransac_min_inl = 3;
const int ransac_iters = 1000;

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
        // std::cout << "[RANSAC] early exit: N=" << N 
        //           << " P2.size=" << P2.size() << "\n";
        return out;
    }

    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> uni(0, N-1); 

    int best_inl = -1;
    SE3 best_T;
    std::vector<int> best_set;

    std::vector<int> idx(3);

    for (int it=0; it<maxIters; ++it) {
        // sample 3 distinct indices
        for (;;) {
            idx[0]=uni(rng); idx[1]=uni(rng); idx[2]=uni(rng);
            if (idx[0]!=idx[1] && idx[0]!=idx[2] && idx[1]!=idx[2]) break;
        }

        std::vector<cv::Point3f> sP(3), sQ(3);
        for (int k=0; k<3; ++k) {sP[k]=P1[idx[k]]; sQ[k]=P2[idx[k]];}

        SE3 Tm;
        if (!umeyama_rigid(sP, sQ, Tm)) continue;

        // count inliers
        std::vector<int> inl;
        inl.reserve(N);
        for (int i=0; i<N; ++i) {
            cv::Vec3d x(P1[i].x, P1[i].y, P1[i].z);
            cv::Vec3d y_pred = Tm.R * x + Tm.t;
            cv::Vec3d y(P2[i].x, P2[i].y, P2[i].z);
            double err = cv::norm(y_pred - y);
            if (err < thresh) inl.push_back(i);
        }

        if ((int)inl.size() > best_inl) {
            best_inl = (int)inl.size();
            best_T = Tm;
            best_set = std::move(inl);
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
        std::cout << "[RANSAC] refit failed in Umeyama\n";
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

static std::vector<cv::Point3f> unique_points_by_mpid(
    const std::unordered_map<int, std::vector<cv::Point3f>>& kf2pts,
    const std::unordered_map<int, std::vector<int>>& kf2mp,
    bool average_duplicates = false)
{
    struct Acc {cv::Vec3d sum; int cnt=0;};
    std::unordered_map<int, Acc> acc; // mp_id -> accumulator
    acc.reserve(100000);

    for (const auto& kv : kf2pts) {
        int kf = kv.first;
        const auto& pts = kv.second;

        auto it = kf2mp.find(kf);
        if (it == kf2mp.end()) continue;

        const auto& mpids = it->second;
        if (mpids.size() != pts.size()) continue;

        for (size_t i=0; i<pts.size(); ++i) {
            int id = mpids[i];
            auto& a = acc[id];
            if (average_duplicates) {
                a.sum += cv::Vec3d(pts[i].x, pts[i].y, pts[i].z);
                a.cnt += 1;
            } else {
                if (a.cnt == 0) {
                    a.sum = cv::Vec3d(pts[i].x, pts[i].y, pts[i].z);
                    a.cnt = 1;
                }
            }
        }
    }

    std::vector<cv::Point3f> out;
    out.reserve(acc.size());
    for (auto& kv : acc) {
        const auto& a = kv.second;
        if (a.cnt <= 0) continue;
        cv::Vec3d m = (a.cnt > 0) ? (a.sum * (1.0 / a.cnt)) : a.sum;
        out.emplace_back((float)m[0], (float)m[1], (float)m[2]);
    }
    return out;
}

static bool write_points_csv(const std::string& path,
                             const std::vector<cv::Point3f> pts,
                             const char* header = "x,y,z")
{
    std::ofstream ofs(path);
    if (!ofs.is_open()) return false;
    ofs << header << "\n";
    ofs.setf(std::ios::fixed); ofs << std::setprecision(6);
    for (const auto& p : pts) {
        ofs << p.x << "," << p.y << "," << p.z << "\n";
    }
    return true;
}

static SE3 invert(const SE3& T)
{
    SE3 Ti;
    Ti.R = T.R.t();
    Ti.t = -(Ti.R * T.t);
    return Ti;
}

static Pairs3D build_3d_pairs_from_kf(
    int kf1, int kf2,
    const std::unordered_map<int, std::vector<cv::Mat>>& kf2descs_1,
    const std::unordered_map<int, std::vector<cv::Mat>>& kf2descs_2,
    const std::unordered_map<int, std::vector<cv::Point3f>>& kf2pts_1,
    const std::unordered_map<int, std::vector<cv::Point3f>>& kf2pts_2,
    float ratio=0.9f, int maxHamming=60)
{
    Pairs3D out;
    auto itD1 = kf2descs_1.find(kf1);
    auto itD2 = kf2descs_2.find(kf2);
    auto itP1 = kf2pts_1.find(kf1);
    auto itP2 = kf2pts_2.find(kf2);
    if (itD1==kf2descs_1.end() || itD2==kf2descs_2.end()||
        itP1==kf2pts_1.end() || itP2==kf2pts_2.end())
        return out;

    const auto& D1v = itD1->second;
    const auto& D2v = itD2->second;
    const auto& P1v = itP1->second;
    const auto& P2v = itP2->second;

    if (D1v.empty() || D2v.empty()) return out;
    CV_Assert((int)D1v.size()==(int)P1v.size());
    CV_Assert((int)D2v.size()==(int)P2v.size());

    cv::Mat A = stack_rows(D1v);
    cv::Mat B = stack_rows(D2v);

    cv::BFMatcher matcher(cv::NORM_HAMMING, false);
    std::vector<std::vector<cv::DMatch>> knn;
    matcher.knnMatch(A, B, knn, 2);

    int raw_matches = 0;

    std::vector<cv::DMatch> good;
    good.reserve(knn.size());
    for(auto& v : knn) {
        if (!v.empty()) ++raw_matches;
        if (v.size()<2) continue;
        const auto& m1 = v[0];
        const auto& m2 = v[1];
        if (m1.distance <= maxHamming && m1.distance <= ratio*m2.distance) {
            good.push_back(m1);
        }
    }

    std::cout << "[MATCH DEBUG] KF_me=" << kf1
          << " KF_other=" << kf2
          << " raw_knn=" << raw_matches
          << " good=" << good.size() << std::endl;

    for (const auto& m : good) {
        const cv::Point3f& X1 = P1v[m.queryIdx];
        const cv::Point3f& X2 = P2v[m.trainIdx];
        if (!finite3(X1) || !finite3(X2)) continue;
        out.P1.push_back(X1);
        out.P2.push_back(X2);
        out.matches.push_back(m);
    }

    return out;
}


HighQualityManager::HighQualityManager(Map* pMap, ORBVocabulary* mpVoc, 
    const std::string& criteria, double period_sec, string agentName)
:mpMap(pMap), mpVoc(mpVoc), mCriteria(criteria), mPeriodSec(period_sec), msAgentName(agentName) {}


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

        {
            std::unique_lock<std::mutex> glock(gBucketsMx);
            auto& meAgentBucket = gAgentBuckets[msAgentName];
            for (auto& ka : gAgentBuckets){
                if (ka.first == msAgentName) continue;
                auto& otherAgentBucket = ka.second;
                if (otherAgentBucket.best_pairs.size() > 0) {
                    std::cout << msAgentName << " has " << otherAgentBucket.best_pairs.size()
                              << " match(es) with agent " << ka.first << "!\n";                
                } else {
                    continue;
                }

                std::vector<PairEst> pair_ests;
                pair_ests.reserve(otherAgentBucket.best_pairs.size());

                for (const auto& kb : otherAgentBucket.best_pairs) {
                    int kf_me = kb.first;
                    int kf_other = kb.second.kf;
                    auto itD_me    = meAgentBucket.kf2descs.find(kb.first);
                    auto itD_other = otherAgentBucket.kf2descs.find(kb.second.kf);
                    auto itP_me    = meAgentBucket.kf2pts.find(kb.first);
                    auto itP_other = otherAgentBucket.kf2pts.find(kb.second.kf);

                    int nD_me    = (itD_me    != meAgentBucket.kf2descs.end()) ? itD_me->second.size()    : 0;
                    int nD_other = (itD_other != otherAgentBucket.kf2descs.end()) ? itD_other->second.size() : 0;
                    int nP_me    = (itP_me    != meAgentBucket.kf2pts.end()) ? itP_me->second.size()    : 0;
                    int nP_other = (itP_other != otherAgentBucket.kf2pts.end()) ? itP_other->second.size() : 0;

                    // std::cout << "[KF STATS] me=" << msAgentName 
                    //         << " KF_me=" << kb.first 
                    //         << " descs=" << nD_me << " pts=" << nP_me
                    //         << " | other=" << ka.first 
                    //         << " KF_other=" << kb.second.kf
                    //         << " descs=" << nD_other << " pts=" << nP_other
                    //         << "\n";
                    Pairs3D pairs = build_3d_pairs_from_kf(
                        kf_me, kf_other,
                        meAgentBucket.kf2descs,
                        otherAgentBucket.kf2descs,
                        meAgentBucket.kf2pts,
                        otherAgentBucket.kf2pts,
                        ratio, maxHam
                    );

                    if (pairs.P1.size() < 3) {
                        // std::cout << "Low number of 3d pairs for KF "
                        //           << std::to_string(kf_me) << " and "
                        //           << std::to_string(kf_other) << "\n";
                        continue;
                    }

                    std::cout << "[PAIR] KF_me=" << kf_me 
                              << " KF_other=" << kf_other 
                              << " |P1|=" << pairs.P1.size() 
                              << " |P2|=" << pairs.P2.size() << "\n";

                    RansacSE3 r = estimate_se3_ransac(
                        pairs.P1, pairs.P2, ransac_thresh, ransac_iters, ransac_min_inl, 1234);
                    
                    if (r.ok) {
                        std::cout << "KF " << kf_me << " and KF " << kf_other
                                  << ": inliers = " << r.inliers.size()
                                  << " of " << pairs.P1.size() << "\n";
                        pair_ests.push_back({kf_me, kf_other, r});
                    }
                    
                }

                std::vector<EstSE3Weighted> ests;
                ests.reserve(pair_ests.size());
                for (const auto& pe : pair_ests) {
                    EstSE3Weighted e;
                    e.T = pe.est.model;
                    e.inliers = (int)pe.est.inliers.size();
                    ests.push_back(e);
                }

                SE3 T_map2_from_map1 = fuse_transforms_weighted(ests);
                SE3 T_map1_from_map2 = invert(T_map2_from_map1);

                otherAgentBucket.T_agent_to_local = T_map1_from_map2;

                std::cout << "\nFused transform " << ka.first << " -- > this agent (" << msAgentName << "):\n"
                          << "R = \n" << cv::Mat(T_map1_from_map2.R) << "\n"
                          << "t = " << T_map1_from_map2.t << "\n";
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

    // ==== PHASE 1: mutate buckets (WRITE LOCK) ====
    
    std::unique_lock<std::mutex> glock(gBucketsMx);
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

    // calculating bow vector for updated keyframes
    if (!mpVoc) { std::cerr << "[HQManager] mpVoc null\n"; }
    else {
        for (int kf_id : updated_keyframes) {
            const auto &descs = agentBucket.kf2descs[kf_id];
            DBoW2::BowVector bow; DBoW2::FeatureVector feat;
            if (ComputeBowForKF(mpVoc, descs, bow, &feat) && !bow.empty())
                agentBucket.kf2bow[kf_id] = std::move(bow);
        }
    }


    // number of matched frames
    int n = 0;

    std::unordered_map<int, std::vector<Candidate>> matched_frames;
    
    if (agent_name != msAgentName) {
        for (const auto& kv : gAgentBuckets[msAgentName].kf2bow) {
            double best = 0.0;
            int kf_b = kv.first;

            for (const auto& kf_a : updated_keyframes) {
                double sc = mpVoc->score(kv.second, gAgentBuckets[agent_name].kf2bow[kf_a]);
                if (sc > best) best = sc;
                matched_frames[kf_b].push_back({kf_a, sc});
            }

            const double floor_score = 0.03; // empirical floor
            const double rel_cut = 0.70 * best; // relative to best
            const double thresh = std::max(floor_score, rel_cut);

            matched_frames[kf_b].erase(std::remove_if(matched_frames[kf_b].begin(), matched_frames[kf_b].end(),
            [&](const Candidate& c){return c.score < thresh;}),
            matched_frames[kf_b].end());

            // sort descending
            std::sort(matched_frames[kf_b].begin(), matched_frames[kf_b].end(),
                [](const Candidate& a, const Candidate& b){return a.score > b.score;});

            if (!matched_frames[kf_b].empty()){
                Candidate new_best = matched_frames[kf_b][0];
                if (agentBucket.best_pairs.count(kf_b)) {
                    if (new_best.score > agentBucket.best_pairs[kf_b].score) {
                        agentBucket.best_pairs[kf_b] = new_best;
                        ++n;
                    }
                } else {
                    agentBucket.best_pairs[kf_b] =  new_best;
                    ++n;
                }
                // std::cout << "Top KF candidate from agent " << agent_name << " for this agent's KF " << kf_b << ": ";
                // std::cout << "KF " << matched_frames[kf_b][0].kf << " with score " << matched_frames[kf_b][0].score << "\n";
            }
        }

    } else {
        for (auto& ka : gAgentBuckets) {

            if (ka.first == msAgentName) continue;

            for (const auto& kv : ka.second.kf2bow) {
                double best = 0.0;
                int kf_b = kv.first;

                for (const auto& kf_a : updated_keyframes) {
                    double sc = mpVoc->score(kv.second, gAgentBuckets[msAgentName].kf2bow[kf_a]);
                    if (sc > best) best = sc;
                    matched_frames[kf_a].push_back({kf_b, sc});

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
                        Candidate new_best = matched_frames[kf_a][0];
                        auto &otherBucket = ka.second;   // alias for clarity
                        if (otherBucket.best_pairs.count(kf_a)) {
                            if (new_best.score > otherBucket.best_pairs[kf_a].score) {
                                otherBucket.best_pairs[kf_a] = new_best;
                                ++n;
                            }
                        } else {
                            otherBucket.best_pairs[kf_a] = new_best;
                            ++n;
                        }                  
                    }
                }
            }
        }
    }
    


    
}


}
