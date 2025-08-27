#include "HQmanager.h"
#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"

#include <thread>
#include <mutex>
#include <chrono>
#include <map>

using namespace std;
namespace ORB_SLAM2
{
    
HighQualityManager::HighQualityManager(Map* pMap, const std::string& criteria, double period_sec)
:mpMap(pMap), mCriteria(criteria), mPeriodSec(period_sec){}

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
            if (!pMP || pMP->isBad()) continue;

            bool isHQ = false;

            if (crit=="observation") {
                isHQ = (pMP->Observations() >= 8);
            } else if (crit=="ba") {
                // placeholder
            }

            ApplyToMapPoint(pMP, isHQ);
            if (isHQ) {
                mpMap->AddHighQualityMapPoints(pMP);
            } else {
                mpMap->RemoveHighQaulityMapPoints(pMP);
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
    if (wasHQ != isHQ) {
        pMP->SetHighQuality(isHQ);
    }

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

}