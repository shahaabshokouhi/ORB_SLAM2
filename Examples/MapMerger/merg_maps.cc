#include <unordered_map>
#include <sstream>
#include <cctype>
#include <fstream>
#include <iostream>
#include <queue>

#include <opencv2/core/core.hpp>
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"  

using namespace ORB_SLAM2;


struct Candidate {int kf; double score;};

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

int main(int argc, char** argv)
{
    std::unordered_map<int, std::vector<cv::Mat>> kf2descs_1, kf2descs_2;
    std::unordered_map<int, std::vector<cv::Point3f>> kf2pts_1, kf2pts_2;
    std::unordered_map<int, std::vector<int>> kf2mp_1, kf2mp_2;
    std::unordered_map<int, DBoW2::BowVector> kf2bow_1, kf2bow_2;
    std::unordered_map<int, DBoW2::FeatureVector> kf2feat_1, kf2feat_2;
    std::unordered_map<int, std::vector<Candidate>> matched_frames;

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

    std::vector<Candidate> topN;
    topN.reserve(32);

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
    
    std::cout << "\n Number of matched frames: " << n << std::endl;

    return 0;
}