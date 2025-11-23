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