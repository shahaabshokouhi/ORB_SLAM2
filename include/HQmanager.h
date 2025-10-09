#ifndef HQMANAGER_H
#define HQMANAGER_H

#include "MapPoint.h"
#include "KeyFrame.h"
#include "Map.h"
#include "ORBVocabulary.h"


#include <mutex>
#include <vector>
#include <string>
#include <atomic>

namespace ORB_SLAM2
{

class Map;
class MapPoint;
class KeyFrame;

class HighQualityManager
{
public:
    HighQualityManager(Map* pMap, ORBVocabulary* mpVoc, 
        const std::string& criteria, double period_sec);
    
    void Run();     

    void Notify();
    void SetCriteria(const std::string criteria);
    void ExportBowTopMatches(const std::string& csv_path, int topK = 10);
    void ExportBoWTopMatchesCSV(const std::string& csv_path, int topK = 10, int minFrameGap = 50);
    void ExportMapPointDescriptorsCSV(const std::string& csv_path);
private:
    void ApplyToMapPoint(MapPoint* pMP, bool isHQ);

private:
    Map* mpMap = static_cast<Map*>(nullptr);
    std::string mCriteria = "observation"; // or "ba"
    double mPeriodSec = 1.0;
    ORBVocabulary* mpVoc = static_cast<ORBVocabulary*>(nullptr);

    std::mutex mMutex;
    std::atomic<bool> mDoScan{false};
};

}
#endif // HQMANAGER_H