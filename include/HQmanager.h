#ifndef HQMANAGER_H
#define HQMANAGER_H

#include "MapPoint.h"
#include "KeyFrame.h"
#include "Map.h"
#include "ORBVocabulary.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"


#include <mutex>
#include <vector>
#include <string>
#include <atomic>
#include <unordered_map>
#include <map>


namespace ORB_SLAM2
{

class Map;
class MapPoint;
class KeyFrame;
class Frame;

// Result of relocalizing against another agent's shared map points.
// Tcw is the recovered camera pose; inliers pairs a keypoint index in the
// query frame with the matched 3D point already transformed into THIS
// agent's world frame.
struct ForeignRelocResult
{
    cv::Mat Tcw;
    std::vector<std::pair<int, cv::Point3f>> inliers;
    std::string agentName;
};

class HighQualityManager
{
public:
    HighQualityManager(Map* pMap, ORBVocabulary* mpVoc, 
        const std::string& criteria, double period_sec, string agentName);
    
    void Run();     

    void Notify();
    void SetCriteria(const std::string criteria);
    void ExportBowTopMatches(const std::string& csv_path, int topK = 10);
    void ExportBoWTopMatchesCSV(const std::string& csv_path, int topK = 10, int minFrameGap = 50);
    void ExportMapPointDescriptorsCSV(const std::string& csv_path);
    std::map<std::string, std::vector<MapPoint*>> mImportedPointsByAgent;
    void ImportHighQualityMapPoints(const std::string &agent_name,
                                    const std::vector<MapPoint*> &vMPs);
    bool ComputeBowForKF(const ORBVocabulary* voc,
                     const std::vector<cv::Mat>& descs,
                     DBoW2::BowVector& bow,
                     DBoW2::FeatureVector* feat = nullptr);
    vector<cv::Point3f> ExportMergedMap();

    // Try to relocalize a lost frame against map points received from other
    // agents (only agents whose SE3 transform into this frame is known).
    // Requires F.mBowVec to be computed. Returns true and fills `out` on success.
    bool RelocalizeAgainstForeign(Frame &F, ForeignRelocResult &out);

    // Rigid 3D-3D alignment with RANSAC (Umeyama on 3-point samples).
    // Estimates Tcw such that Pc ~= R*Pw + t. Returns true when the inlier
    // consensus reaches minInliers; vInliers always reports the best consensus
    // found (diagnostic), even on failure.
    static bool EstimateSE3_3D3D(const std::vector<cv::Point3f> &Pw,
                                 const std::vector<cv::Point3f> &Pc,
                                 cv::Mat &Tcw, std::vector<int> &vInliers,
                                 double thresh = 0.10, int iters = 500,
                                 int minInliers = 12);

    std::string msAgentName;
    

private:
    void ApplyToMapPoint(MapPoint* pMP, bool isHQ);

private:
    Map* mpMap = static_cast<Map*>(nullptr);
    std::string mCriteria = "observation"; // or "ba"
    double mPeriodSec = 1.0;
    ORBVocabulary* mpVoc = static_cast<ORBVocabulary*>(nullptr);

    std::mutex mMutex;
    std::mutex mMutexImport;
    std::atomic<bool> mDoScan{false};
};

}
#endif // HQMANAGER_H