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
    struct Candidate { KeyFrame* pkf; int kf; double score; };
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
    struct EstSE3Weighted { SE3 T; int inliers=0; double score=0.0; };
    struct PairEst { KeyFrame* pkf; int kf2; RansacSE3 est; double bow_score=0.0; };
    struct AgentBuckets {
        std::unordered_map<int, std::vector<cv::Mat>>     kf2descs;
        std::unordered_map<int, std::vector<cv::Point3f>> kf2pts;
        std::unordered_map<int, std::vector<int>>         kf2mpids;
        std::unordered_map<int, DBoW2::BowVector>         kf2bow;
        std::unordered_map<int, Candidate> best_pairs; // ref map kp, second map kp
        std::unordered_map<int, Pairs3D> point_pairs;
        // reverse map: mpid -> set of kf_ids that currently hold it (for stale-entry cleanup)
        std::unordered_map<int, std::unordered_set<int>> mpid2kfids;

        bool hasTransform = false;
        SE3  T_agent_to_local;
    };



    std::map<std::string, AgentBuckets> gAgentBuckets;
    std::unordered_map<std::string, std::unordered_set<int>> nMpsPerAgent;
    std::mutex gBucketsMx;
    std::unordered_map<int, std::vector<Candidate>> matched_frames;
    std::vector<PairEst> pair_ests;

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
    // kRansacThresh: relaxed 0.07->0.15m; RGB-D drift and depth noise make
    //               7cm per-point precision unreachable in most pairs
    // kRansacMinInliers: lowered 20->10; rely on pooled RANSAC for global validation
    // kMinPointsPerPair: lowered 20->8; sparse pairs now feed the pooled stage
    //                    instead of being discarded entirely
    // kMinInlierRatio: slightly relaxed to match lower per-pair expectations
    // kMinPairsForFusion: lowered 3->2; two consistent pairs are enough for pooling
    const double kRansacThresh       = 0.15;
    const int    kRansacIters        = 2000;
    const int    kRansacMinInliers   = 10;
    const int    kMinPointsPerPair   = 8;
    const double kMinInlierRatio     = 0.15;
    const int    kMinPairsForFusion  = 2;

    const double bow_floor_score = 0.02;
    const double bow_rel_cut    = 0.70;
    const bool printLog = false;
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
    if ((N < 3 || (int)P2.size() != N) && printLog) {
        std::cout << "[RANSAC] early exit: N=" << N 
                  << " P2.size=" << P2.size() << "\n";
        return out;
    }

    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> uni(0, N-1); 

    const int sample_k = (N >= 4) ? 4 : 3;
    std::vector<int> idx(sample_k);

    int best_inl = -1;
    SE3 best_T;
    std::vector<int> best_set;

    const int early_good = std::max(minInliers, (int)(0.7*N));

    for (int it = 0; it < maxIters; ++it) {
        // sample 3 distinct indices        
        for (;;) {
            for(int k = 0; k < sample_k; ++k) idx[k] = uni(rng);

            bool distinct = true;
            for (int a = 0; a < sample_k && distinct; ++a) {
                for (int b = a + 1; b < sample_k; ++b) {
                    if (idx[a] == idx[b]) {
                        distinct = false;
                        break;
                    }
                }
            }
            if (distinct) break;
        }


        std::vector<cv::Point3f> sP(sample_k), sQ(sample_k);
        for (int k=0; k<sample_k; ++k) {sP[k]=P1[idx[k]]; sQ[k]=P2[idx[k]];}
        
        // deneeracy check
        if (!non_degenerate_3pt(sP[0], sP[1], sP[2]) ||
            !non_degenerate_3pt(sQ[0], sQ[1], sQ[2]))
        {
            continue;
        }

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
            best_set = inl;

            if (best_inl >= early_good) break;
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

    if (!umeyama_rigid(iP, iQ, refit) && printLog) {
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
        double w = std::max(1, e.inliers) * std::max(e.score, 0.01);
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
    KeyFrame* pkf, int kf2,
    const std::unordered_map<int, std::vector<cv::Mat>>& kf2descs_2,
    const std::unordered_map<int, std::vector<cv::Point3f>>& kf2pts_2,
    float ratio=0.9f, int maxHamming=60)
{
    Pairs3D out;
    if (!pkf) return out;

    vector<MapPoint*> vpMp = pkf->GetHighQualityMapPoints();

    std::vector<cv::Mat> D1v;
    std::vector<cv::Point3f> P1v;

    D1v.reserve(vpMp.size());
    P1v.reserve(vpMp.size());

    for (auto& pMp: vpMp) {
        if (!pMp || pMp->isBad() || pMp->mDescriptor.empty()) continue;

        cv::Mat d = pMp->mDescriptor;

        if (d.rows != 1) d = d.reshape(1, 1);           // force one row
        if (d.type() != CV_8U) d.convertTo(d, CV_8U);   // should already be CV_8U
        D1v.push_back(d.clone());                       // clone to avoid aliasing

        // 3D point in world
        // ORB-SLAM2 has `cv::Mat GetWorldPos()`
        cv::Mat Xw = pMp->GetWorldPos();
        if (Xw.empty()) { D1v.pop_back(); continue; }
        Xw = Xw.reshape(1, 3);                          // make sure it's 3x1 or 1x3-ish

        cv::Point3f p;
        p.x = Xw.at<float>(0);
        p.y = Xw.at<float>(1);
        p.z = Xw.at<float>(2);

        if (!finite3(p)) { D1v.pop_back(); continue; }  // keep vectors aligned
        P1v.push_back(p);
    }

    auto itD2 = kf2descs_2.find(kf2);
    auto itP2 = kf2pts_2.find(kf2);
    if (itD2==kf2descs_2.end() || itP2==kf2pts_2.end()) return out;

    const auto& D2v = itD2->second;
    const auto& P2v = itP2->second;

    if (D1v.empty() || D2v.empty()) return out;
    if (D1v.size() != P1v.size()) return out;
    if (D2v.size() != P2v.size()) return out;

    cv::Mat A = stack_rows(D1v);
    cv::Mat B = stack_rows(D2v);
    if (A.empty() || B.empty()) return out;

    cv::BFMatcher matcher(cv::NORM_HAMMING, false);
    std::vector<std::vector<cv::DMatch>> knn;
    matcher.knnMatch(A, B, knn, 2);

    int raw_matches = 0;
    std::vector<cv::DMatch> good;
    good.reserve(knn.size());

    for (auto& v : knn) {
        if (v.empty()) continue;
        ++raw_matches;

        if (v.size() < 2) continue;

        const auto& m1 = v[0];
        const auto& m2 = v[1];
        if (m1.distance <= maxHamming && m1.distance <= ratio*m2.distance) {
            good.push_back(m1);
        }
    
    }
    if (printLog) {
        std::cout << "[MATCH DEBUG] KF_me=" << pkf->mnId
            << " KF_other=" << kf2
            << " raw_knn=" << raw_matches
            << " good=" << good.size() << std::endl;
    }


    for (const auto& m : good) {

        if (m.queryIdx < 0 || m.queryIdx >= (int)P1v.size()) continue;
        if (m.trainIdx < 0 || m.trainIdx >= (int)P2v.size()) continue;

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


        for (MapPoint* pMP : vMPs) {
            if (!pMP) continue;
            bool isHQ = false;

            if (pMP->isBad()) {

                ApplyToMapPoint(pMP, isHQ);
                mpMap->RemoveHighQaulityMapPoints(pMP);
            
            } else {

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

        // === OLD BLOCK REPLACED BY THIS ===

        // 1) Take a snapshot of the current buckets under a short lock
        std::map<std::string, AgentBuckets> bucketsCopy;
        {
            std::unique_lock<std::mutex> glock(gBucketsMx);
            bucketsCopy = gAgentBuckets;
        }

        // thresholds for the pooled refinement
        const int kMinMatchedFramesForPool = 2;
        const int kMinPoolSizeForRansac    = 20;

        auto make_seed = [&](int a, int b, unsigned base=1337u){
            // stable-ish seed per pair
            return base ^ (unsigned)(a * 73856093) ^ (unsigned)(b * 19349663);
        };

        auto rad2deg = [](double r){ return r * 180.0 / M_PI; };

        // Make sure we actually have our own agent in the snapshot
        if (bucketsCopy.empty()) {
            std::cerr << "No information received yet, waiting ...\n";
        } else {

            // For each OTHER agent, compute SE3 from snapshot (no locks here)
            std::map<std::string, SE3> transformsToApply;  // otherName -> T_me_from_other

            for (auto &ka : bucketsCopy) {
                const std::string &otherName = ka.first;
                if (otherName == msAgentName) continue;

                AgentBuckets &otherBucketSnap = ka.second;

                std::vector<Candidate> matchedCandidates;

                // kf pairing
                for (KeyFrame* pkf_me : vpKFs) {
                    int kf_me = pkf_me->mnId;
                    const auto &bow_me = pkf_me->mHQBowVec;

                    double best_score = 0.0;
                    std::vector<Candidate> cands;

                    // compare this agent's kf_me with each NEW keyframe from 'agent_name'
                    for (const auto &kv_other : otherBucketSnap.kf2bow) {
                        int kf_other = kv_other.first;
                        const auto &bow_other = kv_other.second;

                        double sc = mpVoc->score(bow_me, bow_other);
                        cands.push_back({pkf_me, kf_other, sc});
                        if (sc > best_score) best_score = sc;
                    }

                    if (cands.empty()) continue;

                    const double thresh = std::max(bow_floor_score, bow_rel_cut * best_score);

                    // keep only strong candidates
                    cands.erase(std::remove_if(cands.begin(), cands.end(),
                                            [&](const Candidate &c) {
                                                return c.score < thresh;
                                            }),
                                cands.end());

                    if (cands.empty()) continue;

                    // sort descending by score
                    std::sort(cands.begin(), cands.end(),
                            [](const Candidate &a, const Candidate &b) {
                                return a.score > b.score;
                            });

                    const Candidate new_best = cands.front();
                    matchedCandidates.push_back(new_best);
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

                    Pairs3D pairs = build_3d_pairs_from_kf(
                        pkf, kf_other,
                        otherBucketSnap.kf2descs,
                        otherBucketSnap.kf2pts,
                        ratio, maxHam
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
                SE3 T_other_from_me_from_fuse;
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

                // ---- fallback: fuse per-pair SE3s (your old approach) ----
                if (true) {
                    std::vector<EstSE3Weighted> ests;
                    ests.reserve(pair_ests.size());
                    for (const auto &pe : pair_ests) {
                        EstSE3Weighted e;
                        e.T       = pe.est.model;
                        e.inliers = (int)pe.est.inliers.size();
                        e.score   = pe.bow_score;
                        ests.push_back(e);
                    }
                    T_other_from_me_from_fuse = fuse_transforms_weighted(ests);
                }

                // We want transform from other agent -> this agent
                SE3 T_me_from_other = invert(T_other_from_me);
                SE3 T_me_from_other_from_fuse = invert(T_other_from_me_from_fuse);
                // Use pooled RANSAC result when available; fall back to weighted fusion
                transformsToApply[otherName] = usedPooled ? T_me_from_other : T_me_from_other_from_fuse;

                // Debug: print Euler angles (in degrees)
                const cv::Matx33d &R = T_me_from_other.R;
                double yaw   = std::atan2(R(1,0), R(0,0));
                double pitch = std::asin(-R(2,0));
                double roll  = std::atan2(R(2,1), R(2,2));

                // Debug: print Euler angles (in degrees)
                const cv::Matx33d &R_fuse = T_me_from_other_from_fuse.R;
                double yaw_fuse   = std::atan2(R_fuse(1,0), R_fuse(0,0));
                double pitch_fuse = std::asin(-R_fuse(2,0));
                double roll_fuse  = std::atan2(R_fuse(2,1), R_fuse(2,2));

                std::cout << "\nFused transform from pool " << otherName
                        << " --> " << msAgentName << ":\n"
                        << "R = \n" << cv::Mat(T_me_from_other.R) << "\n"
                        << "t = " << T_me_from_other.t << "\n"
                        << "angles (deg): roll="  << rad2deg(roll)
                        << " pitch=" << rad2deg(pitch)
                        << " yaw="   << rad2deg(yaw) << "\n\n";

                
                std::cout << "\nFused transform from fuse " << otherName
                        << " --> " << msAgentName << ":\n"
                        << "R = \n" << cv::Mat(T_me_from_other_from_fuse.R) << "\n"
                        << "t = " << T_me_from_other_from_fuse.t << "\n"
                        << "angles (deg): roll="  << rad2deg(roll_fuse)
                        << " pitch=" << rad2deg(pitch_fuse)
                        << " yaw="   << rad2deg(yaw_fuse) << "\n\n";
            }

            // Apply back to live buckets under a short lock
            if (!transformsToApply.empty()) {
                std::unique_lock<std::mutex> glock(gBucketsMx);
                for (const auto &kv : transformsToApply) {
                    const std::string &otherName = kv.first;
                    const SE3 &T_me_from_other   = kv.second;

                    auto itLive = gAgentBuckets.find(otherName);
                    if (itLive == gAgentBuckets.end()) continue;

                    AgentBuckets &otherBucketLive = itLive->second;
                    otherBucketLive.T_agent_to_local = T_me_from_other;
                    otherBucketLive.hasTransform     = true;
                }
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
    if (agent_name == msAgentName) return;

    std::unique_lock<std::mutex> glock(gBucketsMx);
    
    for (auto* MP : vMPs) {
        if (MP->isBad() || !MP->mbHighQaulity) {
            nMpsPerAgent[agent_name].erase(MP->mnId);
        } else {
            nMpsPerAgent[agent_name].insert(MP->mnId);
        }
    }

    std::cout << "Agent " << agent_name << " has "
              << nMpsPerAgent[agent_name].size()
              << " Map Points. \n";

    // ==== PHASE 1: mutate buckets (WRITE LOCK) ====
    
    // per-agent storage in HQ manager
    // std::vector<MapPoint*> &vAgentPoints = mImportedPointsByAgent[agent_name];
    // per-agent “buckets” in this .cc
    AgentBuckets &agentBucket = gAgentBuckets[agent_name];
    // std::cout << "Importing " << vMPs.size() << " points from "
    //           << agent_name << std::endl;
    std::unordered_set<int> updated_keyframes;

    for (ORB_SLAM2::MapPoint* pSrcMP : vMPs) {

        if (!pSrcMP) {
            std::cerr << "[HQManager] skipping null MapPoint from agent " << agent_name << "\n";
            continue;
        }

        // 1) check observations list
        // if (pSrcMP->mvnObservations.empty()) {
        //     // not fatal, but tell ourselves
        //     std::cerr << "[HQManager] MapPoint " << pSrcMP->mnId
        //             << " from agent " << agent_name
        //             << " has no observation KF ids; skipping bucket fill.\n";
        // }

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
        int mpid = static_cast<int>(pSrcMP->mnId);

        // Build the new set of kf_ids for this mpid
        std::unordered_set<int> newKfIds(pSrcMP->mvnObservations.begin(),
                                         pSrcMP->mvnObservations.end());

        // Remove stale entries: kf_ids that previously held this mpid but are no longer in observations
        auto prevIt = agentBucket.mpid2kfids.find(mpid);
        if (prevIt != agentBucket.mpid2kfids.end()) {
            for (int old_kf : prevIt->second) {
                if (newKfIds.count(old_kf)) continue; // still present, no action needed
                // Remove mpid from this stale kf bucket
                auto itD = agentBucket.kf2descs.find(old_kf);
                auto itP = agentBucket.kf2pts.find(old_kf);
                auto itM = agentBucket.kf2mpids.find(old_kf);
                if (itM == agentBucket.kf2mpids.end()) continue;
                auto &vM = itM->second;
                for (size_t i = 0; i < vM.size(); ++i) {
                    if (vM[i] == mpid) {
                        vM.erase(vM.begin() + i);
                        if (itD != agentBucket.kf2descs.end()) itD->second.erase(itD->second.begin() + i);
                        if (itP != agentBucket.kf2pts.end())   itP->second.erase(itP->second.begin() + i);
                        updated_keyframes.insert(old_kf);
                        break;
                    }
                }
            }
        }

        // Update the reverse map for this mpid
        agentBucket.mpid2kfids[mpid] = newKfIds;

        for (int kf_id : pSrcMP->mvnObservations) {

            // we only insert if descriptor is valid; otherwise skip this MP for this KF
            if (!hasDesc || !hasPos) {
                continue;
            }

            auto &vD = agentBucket.kf2descs[kf_id];
            auto &vP = agentBucket.kf2pts[kf_id];
            auto &vM = agentBucket.kf2mpids[kf_id];

            // try to find existing entry with same mpid
            int existing_idx = -1;
            for (size_t i = 0; i < vM.size(); ++i) {
                if (vM[i] == mpid) {
                    existing_idx = static_cast<int>(i);
                    break;
                }
            }

            cv::Point3f p;
            p.x = X.at<float>(0);
            p.y = X.at<float>(1);
            p.z = X.at<float>(2);

            if (existing_idx >= 0) {
                // REPLACE in-place
                desc.clone().copyTo(vD[existing_idx]);
                vP[existing_idx] = p;
                // vM[existing_idx] already equals mpid
            } else {
                // NEW entry
                vD.push_back(desc.clone());
                vP.push_back(p);
                vM.push_back(mpid);
            }

            // quick consistency check: all 3 vectors for this kf_id must have same size
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
        // ORB_SLAM2::MapPoint* pCopy = new ORB_SLAM2::MapPoint(*pSrcMP);
        // pCopy->ReceivedFromOther(true);
        // vAgentPoints.push_back(pCopy);
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


    // // number of matched frames
    // int n = 0;

    // std::unordered_map<int, std::vector<Candidate>> matched_frames;

    // // Stronger BoW thresholds
    // const double bow_floor_score = 0.02;  // was 0.03, too permissive
    // const double bow_rel_cut    = 0.70;

    // if (agent_name != msAgentName)
    // {
    //     // We received new KFs from 'agent_name'
    //     AgentBuckets &otherBucket = agentBucket;               // this is gAgentBuckets[agent_name]
    //     AgentBuckets &meBucket    = gAgentBuckets[msAgentName];

    //     for (const auto &kv_me : meBucket.kf2bow) {
    //         int kf_me = kv_me.first;
    //         const auto &bow_me = kv_me.second;

    //         double best_score = 0.0;
    //         auto &cands = matched_frames[kf_me];
    //         cands.clear();

    //         // compare this agent's kf_me with each NEW keyframe from 'agent_name'
    //         for (int kf_other : updated_keyframes) {
    //             auto itBowOther = otherBucket.kf2bow.find(kf_other);
    //             if (itBowOther == otherBucket.kf2bow.end()) continue;

    //             double sc = mpVoc->score(bow_me, itBowOther->second);
    //             cands.push_back({kf_other, sc});
    //             if (sc > best_score) best_score = sc;
    //         }

    //         if (cands.empty()) continue;

    //         const double thresh = std::max(bow_floor_score, bow_rel_cut * best_score);

    //         // keep only strong candidates
    //         cands.erase(std::remove_if(cands.begin(), cands.end(),
    //                                    [&](const Candidate &c) {
    //                                        return c.score < thresh;
    //                                    }),
    //                     cands.end());

    //         if (cands.empty()) continue;

    //         // sort descending by score
    //         std::sort(cands.begin(), cands.end(),
    //                   [](const Candidate &a, const Candidate &b) {
    //                       return a.score > b.score;
    //                   });

    //         const Candidate new_best = cands.front();

    //         // update only if this is better than the previous stored match
    //         auto itPrev = otherBucket.best_pairs.find(kf_me);
    //         if (itPrev == otherBucket.best_pairs.end() ||
    //             new_best.score > itPrev->second.score)
    //         {
    //             otherBucket.best_pairs[kf_me] = new_best;
    //             ++n;
    //             // optional debug:
    //             // std::cout << "[BoW MATCH] this KF " << kf_me
    //             //           << " best with agent " << agent_name
    //             //           << " KF " << new_best.kf
    //             //           << " score=" << new_best.score << "\n";
    //         }
    //     }
    // }
    // else
    // {
    //     // We just added new KFs for *this* agent; compare them to all other agents
    //     AgentBuckets &meBucket = gAgentBuckets[msAgentName];

    //     for (auto &ka : gAgentBuckets) {
    //         const std::string &other_name = ka.first;
    //         if (other_name == msAgentName) continue;

    //         AgentBuckets &otherBucket = ka.second;

    //         // fresh map per other agent
    //         matched_frames.clear();

    //         for (int kf_me : updated_keyframes) {
    //             auto itBowMe = meBucket.kf2bow.find(kf_me);
    //             if (itBowMe == meBucket.kf2bow.end()) continue;

    //             const auto &bow_me = itBowMe->second;
    //             double best_score = 0.0;
    //             auto &cands = matched_frames[kf_me];
    //             cands.clear();

    //             // compare this new KF with all KFs of the other agent
    //             for (const auto &kv_other : otherBucket.kf2bow) {
    //                 int kf_other = kv_other.first;
    //                 const auto &bow_other = kv_other.second;

    //                 double sc = mpVoc->score(bow_me, bow_other);
    //                 cands.push_back({kf_other, sc});
    //                 if (sc > best_score) best_score = sc;
    //             }

    //             if (cands.empty()) continue;

    //             const double thresh = std::max(bow_floor_score, bow_rel_cut * best_score);

    //             // keep only strong candidates
    //             cands.erase(std::remove_if(cands.begin(), cands.end(),
    //                                        [&](const Candidate &c) {
    //                                            return c.score < thresh;
    //                                        }),
    //                         cands.end());

    //             if (cands.empty()) continue;

    //             std::sort(cands.begin(), cands.end(),
    //                       [](const Candidate &a, const Candidate &b) {
    //                           return a.score > b.score;
    //                       });

    //             const Candidate new_best = cands.front();

    //             // key in best_pairs is ALWAYS "this agent's KF"
    //             auto itPrev = otherBucket.best_pairs.find(kf_me);
    //             if (itPrev == otherBucket.best_pairs.end() ||
    //                 new_best.score > itPrev->second.score)
    //             {
    //                 otherBucket.best_pairs[kf_me] = new_best;
    //                 ++n;
    //                 // optional debug:
    //                 // std::cout << "[BoW MATCH] this KF " << kf_me
    //                 //           << " best with agent " << other_name
    //                 //           << " KF " << new_best.kf
    //                 //           << " score=" << new_best.score << "\n";
    //             }
    //         }
    //     }
    // }
}


}
