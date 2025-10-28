#include <unordered_map>
#include <sstream>
#include <cctype>
#include <fstream>
#include <iostream>
#include <queue>
#include <random>
#include <cmath>

#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/utility.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp> // for cv::SVD
#include <opencv2/core/core.hpp>
#include <opencv2/features2d.hpp>

#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"  

using namespace ORB_SLAM2;

// structs
struct Candidate {int kf; double score;};
struct Pair {int kf1; int kf2; double score;};
struct Pairs3D {
    std::vector<cv::Point3f> P1;
    std::vector<cv::Point3f> P2;
    std::vector<cv::DMatch> matches;
};
struct SE3 {
    cv::Matx33d R;
    cv::Vec3d t;
};
struct RansacSE3{
    SE3 model;
    std::vector<int> inliers;
    bool ok = false;
};
struct EstSE3Weighted {SE3 T; int inliers=0; double score=0.0;};
struct PairEst {Pair pair; RansacSE3 est;};

// helper function to slplit string on a single char
static std::vector<std::string> split(const std::string& s, char delim)
{
    std::vector<std::string> out;
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) out.push_back(item);
    return out;
}

// trim whitespace
static std::string trim(std::string v) 
{
    auto is_sp = [](int ch){return ch==' ' || ch=='\t' || ch=='\n' || ch=='\r';};
    size_t a = 0, b = v.size();
    while (a < b && is_sp(v[a])) ++a;
    while (b > a && is_sp(v[b-1])) --b;
    return v.substr(a, b-a);
}

static std::string unqoute(std::string s) 
{
    if (s.size() >=2 && s.front() == '"' && s.back() == '"') {
        s = s.substr(1, s.size()-2);
    }
    return s;
}

static bool hex2mat32u(const std::string& hex, cv::Mat& out) {
    constexpr size_t EXPECTED_LEN = 64;
    if (hex.size() != EXPECTED_LEN) {
        std::cout << "hex2mat32u: invalid length " << hex.size() << ", expected " << EXPECTED_LEN << std::endl;
        std::cout << "hex: " << hex[0] << std::endl;
        return false;
    }
    
    // FIXED: Clear + resize (no data corruption)
    out = cv::Mat(1, 32, CV_8U);
    
    auto val = [](char c)->int{
        if (c>='0' && c<='9') return c-'0';
        if (c>='a' && c<='f') return 10+(c-'a');
        if (c>='A' && c<='F') return 10+(c-'A');
        return -1;
    };
    for (int i = 0; i < 32; i++) {
        int hi = val(hex[2*i]);
        int lo = val(hex[2*i+1]);
        if (hi < 0 || lo < 0) return false;
        out.at<uchar>(0,i) = static_cast<uchar>((hi << 4) | lo);
    }
    return true;
}

static std::vector<int> parse_kf_ids(const std::string& s)
{
    std::vector<int> out;
    for (auto& tok : split(s, ';')){
        tok = trim(tok);
        if (!tok.empty()) out.push_back(std::stoi(tok));
    }
    return out;
}

bool load_csv_to_buckets(const std::string& csv_path,
                        std::unordered_map<int, std::vector<cv::Mat>>& kf2descs,
                        std::unordered_map<int, std::vector<cv::Point3f>>* kf2points = nullptr,
                        std::unordered_map<int, std::vector<int>>* kf2mpids = nullptr)
{
    std::ifstream fin(csv_path);
    if (!fin.is_open()) {
        std::cerr << "Cannot open " << csv_path << std::endl;
        return false;
    }

    // read the header
    std::string header;
    if(!std::getline(fin, header)) return false;
    auto cols = split(header, ',');
    for (auto& c : cols) c = trim(c);

    // map name -> index
    auto col_index = [&](const std::string& name) -> int{
        for (int i=0; i < (int)cols.size(); ++i) if (cols[i] == name) return i;
        return -1;
    };

    int i_mp_id = col_index("mp_id");
    int i_x = col_index("mp_x");
    int i_y = col_index("mp_y");
    int i_z       = col_index("mp_z");
    int i_desc    = col_index("descriptor_hex");
    int i_obs     = col_index("observations");

    if (i_mp_id < 0 || i_desc < 0 || i_obs < 0) {
        std::cerr << "CSV missing required columns" << std::endl;
        return false;
    }

    // read rows
    std::string line;
    size_t n_rows = 0, n_ok = 0;
    while (std::getline(fin, line)) {
        if (line.empty()) continue;
        auto fields = split(line, ',');

        auto get = [&](int idx) -> std::string {
            return (idx >= 0 && idx < (int)fields.size()) ? unqoute(trim(fields[idx])) : std::string();
        };

        // parse basics
        int mp_id = -1;
        mp_id = std::stoi(get(i_mp_id));
        std::string hex = get(i_desc);
        std::string obs = get(i_obs);
        if (mp_id < 0 || hex.empty() || obs.empty()) {
            std::cout << "Warning: skipping row " << n_rows << " due to missing data\n";
            ++n_rows;
            continue;
        }

        cv::Mat desc;
        if (!hex2mat32u(hex, desc)) {
            std::cout << "Warning: skipping row " << n_rows << " due to invalid descriptor hex\n";
            ++n_rows;
            continue;
        }

        // optional 3D
        cv::Point3f Pw(0,0,0);
        if (i_x >= 0 && i_y >= 0 && i_z >= 0) {
            Pw.x = std::stof(get(i_x));
            Pw.y = std::stof(get(i_y));
            Pw.z = std::stof(get(i_z));
        } else {
            std::cerr << "Warning: missing 3D point columns, setting to (0,0,0)" << std::endl;
        }

        // observations -> vector<int>
        auto kf_ids = parse_kf_ids(obs);
        if (kf_ids.empty()) {
            std::cout << "Warning: skipping row " << n_rows << " due to no observations\n";
            ++n_rows;
            continue;
        }

        // push the descriptor once per listed KF
        for (int kf : kf_ids) {
            kf2descs[kf].push_back(desc.clone());
            if (kf2points) (*kf2points)[kf].push_back(Pw);
            if (kf2mpids)  (*kf2mpids)[kf].push_back(mp_id);
        }

        ++n_rows; ++n_ok;
    }

    std::cout << "Parsed rows: " << n_rows << ", bucketed OK: " << n_ok << "\n";
    std::cout << "Unique keyframes found: " << kf2descs.size() << "\n";

    return !kf2descs.empty();


}

static bool compute_bow_for_kf(const ORBVocabulary& voc,
                               const std::vector<cv::Mat>& descs,
                               DBoW2::BowVector& bow,
                               DBoW2::FeatureVector* feat=nullptr)
{
    bow.clear();
    if (feat) feat->clear();
    if (descs.empty()) return false;

    if (feat) {
        voc.transform(descs, bow, *feat, 4);
    } else {
        voc.transform(descs, bow);
    }

    return !bow.empty();
}

// --------functions for matching and 3d registration--------

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
        itP1==kf2pts_1.end(), itP2==kf2pts_2.end())
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

    std::vector<cv::DMatch> good;
    good.reserve(knn.size());
    for(auto& v : knn) {
        if (v.size()<2) continue;
        const auto& m1 = v[0];
        const auto& m2 = v[1];
        if (m1.distance <= maxHamming && m1.distance <= ratio*m2.distance) {
            good.push_back(m1);
        }
    }

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
    if (N < 3 || (int)P2.size() != N) return out;

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

    if (best_inl < std::max(3, minInliers)) return out;

    // refit with all inliers
    std::vector<cv::Point3f> iP, iQ;
    iP.reserve(best_set.size()); iQ.reserve(best_set.size());
    for (int id : best_set) {iP.push_back(P1[id]); iQ.push_back(P2[id]);}

    SE3 refit;
    if (!umeyama_rigid(iP, iQ, refit)) return out;

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

int main(int argc, char** argv)
{
    // parameters
    const float ratio = 0.9f;
    const int maxHam = 60;
    const double ransac_thresh = 0.07; // meters
    const int ransac_min_inl = 20;
    const int ransac_iters = 1000;
    
    std::unordered_map<int, std::vector<cv::Mat>> kf2descs_1, kf2descs_2;
    std::unordered_map<int, std::vector<cv::Point3f>> kf2pts_1, kf2pts_2;
    std::unordered_map<int, std::vector<int>> kf2mp_1, kf2mp_2;
    std::unordered_map<int, DBoW2::BowVector> kf2bow_1, kf2bow_2;
    std::unordered_map<int, DBoW2::FeatureVector> kf2feat_1, kf2feat_2;
    std::unordered_map<int, std::vector<Candidate>> matched_frames;
    std::vector<PairEst> pair_ests;

    ORBVocabulary voc;
    if (!voc.loadFromTextFile("./Vocabulary/ORBvoc.txt")) {
        std::cerr << "Error loading vocabulary\n";
        return -1;
    }

    std::cout << "Vocabulary loaded: " << voc.size() << " words.\n";

    bool ok1 = load_csv_to_buckets("./Results/results_agent_0/mappoint_descriptors.csv", kf2descs_1, &kf2pts_1, &kf2mp_1);
    bool ok2 = load_csv_to_buckets("./Results/results_agent_1/mappoint_descriptors.csv", kf2descs_2, &kf2pts_2, &kf2mp_2);

    if (!ok1 || !ok2) {
        std::cerr << "Error loading input maps\n";
        return -1;
    }

    for (auto& kv : kf2descs_1) {
        std::cout << "KF " << kv.first << " has " << kv.second.size() << " descriptors\n";
        break;
    }

    for (auto& d : kf2descs_1.begin()->second) {
        assert(d.type() == CV_8U && d.rows == 1 && d.cols == 32);
    }

    for (const auto& kv : kf2descs_1) {
        int kfid = kv.first;
        const auto& descs = kv.second;

        DBoW2::BowVector bow;
        DBoW2::FeatureVector feat;

        if (compute_bow_for_kf(voc, descs, bow, &feat)) {
            kf2bow_1.emplace(kfid, std::move(bow));
            kf2feat_1.emplace(kfid, std::move(feat));
        } else {
            std::cerr << "KF " << kfid << " produced empty BoW.\n";
        }
    }

    for (const auto& kv : kf2descs_2) {
        int kfid = kv.first;
        const auto& descs = kv.second;

        DBoW2::BowVector bow;
        DBoW2::FeatureVector feat;

        if (compute_bow_for_kf(voc, descs, bow, &feat)) {
            kf2bow_2.emplace(kfid, std::move(bow));
            kf2feat_2.emplace(kfid, std::move(feat));
        } else {
            std::cerr << "KF " << kfid << " produced empty BoW.\n";
        }
    }

    std::cout << "Map 1 KFs with BoW: " << kf2bow_1.size() << "\n";
    std::cout << "Map 2 KFs with BoW: " << kf2bow_2.size() << "\n";

    int n = 0;
    for (const auto& kn : kf2bow_1) {
        double best = 0.0;

        int kf_a = kn.first;

        for (const auto& kv : kf2bow_2) {
            int kf_b = kv.first;
            double sc = voc.score(kf2bow_1[kf_a], kv.second);
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
            std::cout << "Top KF candidates for Map 1 KF " << kf_a << ": ";
            std::cout << "KF " << matched_frames[kf_a][0].kf << " with score " << matched_frames[kf_a][0].score << "\n";
            ++n;
        }
    }
    
    std::cout << "\nNumber of matched frames: " << n << std::endl;
    
    std::vector<Pair> best_pairs;
    for (const auto& kv : matched_frames) {
        int kf1 = kv.first;
        if (kv.second.empty()) continue;
        int kf2 = kv.second.front().kf;
        best_pairs.push_back({kf1, kf2, kv.second.front().score});
    }

    std::cout << "Best pair size: " << best_pairs.size() << std::endl;
    
    pair_ests.reserve(best_pairs.size());
    for (const auto& pr : best_pairs) {
        Pairs3D pairs = build_3d_pairs_from_kf(
            pr.kf1, pr.kf2,
            kf2descs_1, kf2descs_2,
            kf2pts_1, kf2pts_2,
            ratio, maxHam);

        if (pairs.P1.size() < 3) continue;

        RansacSE3 r = estimate_se3_ransac(
            pairs.P1, pairs.P2, ransac_thresh, ransac_iters, ransac_min_inl, 1234);

        if (r.ok) {
            std::cout << "KF " << pr.kf1 << " and KF " << pr.kf2
                      << " : inliers = " << r.inliers.size()
                      << " of " << pairs.P1.size() << "\n";
            pair_ests.push_back({pr, r});
        }
    }

    std::vector<EstSE3Weighted> ests;
    ests.reserve(pair_ests.size());
    for (const auto& pe : pair_ests) {
        EstSE3Weighted e;
        e.T = pe.est.model;
        e.inliers = (int)pe.est.inliers.size();
        e.score = pe.pair.score;
        ests.push_back(e);
    }

    SE3 T_map2_from_map1 = fuse_transforms_weighted(ests);
    std::cout << "\nFused transform T_map2 <-- map1:\n"
              << "R = \n" << cv::Mat(T_map2_from_map1.R) << "\n"
              << "t = " << T_map2_from_map1.t << "\n";
    

    return 0;
}