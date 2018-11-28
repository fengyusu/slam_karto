#ifndef _MAPPER_SENSORDATA_MANAGER_H__
#define _MAPPER_SENSORDATA_MANAGER_H__

#include "Karto.h"

namespace karto
{

  
  /**
   * Manages the scan data for a device
   */
  class ScanManager
  {
  public:
    /**
     * Default constructor
     */
    ScanManager(kt_int32u runningBufferMaximumSize, kt_double runningBufferMaximumDistance)
      : m_pLastScan(NULL)
      , m_RunningBufferMaximumSize(runningBufferMaximumSize)
      , m_RunningBufferMaximumDistance(runningBufferMaximumDistance)
    {
    }

    /**
     * Destructor
     */
    virtual ~ScanManager()
    {
      Clear();
    }

  public:
    /**
     * Adds scan to vector of processed scans tagging scan with given unique id
     * @param pScan
     */
    inline void AddScan(LocalizedRangeScan* pScan, kt_int32s uniqueId)
    {
      // assign state id to scan
      pScan->SetStateId(static_cast<kt_int32u>(m_Scans.size()));

      // assign unique id to scan
      pScan->SetUniqueId(uniqueId);

      // add it to scan buffer
      m_Scans.push_back(pScan);
    }

    /**
     * Gets last scan
     * @param deviceId
     * @return last localized range scan
     */
    inline LocalizedRangeScan* GetLastScan()
    {
      return m_pLastScan;
    }

    /**
     * Sets the last scan
     * @param pScan
     */
    inline void SetLastScan(LocalizedRangeScan* pScan)
    {
      m_pLastScan = pScan;
    }

    /**
     * Gets scans
     * @return scans
     */
    inline LocalizedRangeScanVector& GetScans()
    {
      return m_Scans;
    }

    /**
     * Gets running scans
     * @return running scans
     */
    inline LocalizedRangeScanVector& GetRunningScans()
    {
      return m_RunningScans;
    }

    /**
     * Adds scan to vector of running scans
     * @param pScan
     */
    void AddRunningScan(LocalizedRangeScan* pScan)
    {
      m_RunningScans.push_back(pScan);

      // vector has at least one element (first line of this function), so this is valid
      Pose2 frontScanPose = m_RunningScans.front()->GetSensorPose();
      Pose2 backScanPose = m_RunningScans.back()->GetSensorPose();

      // cap vector size and remove all scans from front of vector that are too far from end of vector
      kt_double squaredDistance = frontScanPose.GetPosition().SquaredDistance(backScanPose.GetPosition());
      while (m_RunningScans.size() > m_RunningBufferMaximumSize ||
             squaredDistance > math::Square(m_RunningBufferMaximumDistance) - KT_TOLERANCE)
      {
        // remove front of running scans
        m_RunningScans.erase(m_RunningScans.begin());

        // recompute stats of running scans
        frontScanPose = m_RunningScans.front()->GetSensorPose();
        backScanPose = m_RunningScans.back()->GetSensorPose();
        squaredDistance = frontScanPose.GetPosition().SquaredDistance(backScanPose.GetPosition());
      }
    }

    /**
     * Deletes data of this buffered device
     */
    void Clear()
    {
      m_Scans.clear();
      m_RunningScans.clear();
    }

  private:
    LocalizedRangeScanVector m_Scans;
    LocalizedRangeScanVector m_RunningScans;
    LocalizedRangeScan* m_pLastScan;

    kt_int32u m_RunningBufferMaximumSize;
    kt_double m_RunningBufferMaximumDistance;
  };  // ScanManager


  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////


  /**
   * Manages the devices for the mapper
   */
  class MapperSensorDataManager  // : public SensorManager
  {
    typedef std::map<Name, ScanManager*> ScanManagerMap;

  public:
    /**
     * Constructor
     */
    MapperSensorDataManager(kt_int32u runningBufferMaximumSize, kt_double runningBufferMaximumDistance)
      : m_RunningBufferMaximumSize(runningBufferMaximumSize)
      , m_RunningBufferMaximumDistance(runningBufferMaximumDistance)
      , m_NextScanId(0)
    {
    }

    /**
     * Destructor
     */
    virtual ~MapperSensorDataManager()
    {
      Clear();
    }

  public:
    /**
     * Registers a sensor (with given name); do
     * nothing if device already registered
     * @param rSensorName
     */
    void RegisterSensor(const Name& rSensorName){
        if (GetScanManager(rSensorName) == NULL)
        {
        m_ScanManagers[rSensorName] = new ScanManager(m_RunningBufferMaximumSize, m_RunningBufferMaximumDistance);
        }
    }

    /**
     * Gets scan from given sensor with given ID
     * @param rSensorName
     * @param scanIndex
     * @return localized range scan
     */
    LocalizedRangeScan* GetScan(const Name& rSensorName, kt_int32s scanIndex)
    {
        ScanManager* pScanManager = GetScanManager(rSensorName);
        if (pScanManager != NULL)
        {
        return pScanManager->GetScans().at(scanIndex);
        }

        assert(false);
        return NULL;
    }
    /**
     * Gets names of all sensors
     * @return sensor names
     */
    inline std::vector<Name> GetSensorNames()
    {
      std::vector<Name> deviceNames;
      const_forEach(ScanManagerMap, &m_ScanManagers)
      {
        deviceNames.push_back(iter->first);
      }

      return deviceNames;
    }

    /**
     * Gets last scan of given sensor
     * @param rSensorName
     * @return last localized range scan of sensor
     */
    LocalizedRangeScan* GetLastScan(const Name& rSensorName)
    {
        RegisterSensor(rSensorName);

        return GetScanManager(rSensorName)->GetLastScan();
    }

    /**
     * Sets the last scan of device of given scan
     * @param pScan
     */
    inline void SetLastScan(LocalizedRangeScan* pScan)
    {
        GetScanManager(pScan)->SetLastScan(pScan);
    }


    /**
     * Gets the scan with the given unique id
     * @param id
     * @return scan
     */
    inline LocalizedRangeScan* GetScan(kt_int32s id)
    {
      assert(math::IsUpTo(id, (kt_int32s)m_Scans.size()));

      return m_Scans[id];
    }

    /**
     * Adds scan to scan vector of device that recorded scan
     * @param pScan
     */
    void AddScan(LocalizedRangeScan* pScan)
    {
        GetScanManager(pScan)->AddScan(pScan, m_NextScanId);
        m_Scans.push_back(pScan);
        m_NextScanId++;
    }

    /**
     * Adds scan to running scans of device that recorded scan
     * @param pScan
     */
    void AddRunningScan(LocalizedRangeScan* pScan)
    {
        GetScanManager(pScan)->AddRunningScan(pScan);
    }

    /**
     * Gets scans of device
     * @param rSensorName
     * @return scans of device
     */
    LocalizedRangeScanVector& GetScans(const Name& rSensorName)
    {
        return GetScanManager(rSensorName)->GetScans();
    }


    /**
     * Gets running scans of device
     * @param rSensorName
     * @return running scans of device
     */
    LocalizedRangeScanVector& GetRunningScans(const Name& rSensorName)
    {
        return GetScanManager(rSensorName)->GetRunningScans();
    }

    /**
     * Gets all scans of all devices
     * @return all scans of all devices
     */
    LocalizedRangeScanVector GetAllScans()
    {
        LocalizedRangeScanVector scans;

        forEach(ScanManagerMap, &m_ScanManagers)
        {
        LocalizedRangeScanVector& rScans = iter->second->GetScans();

        scans.insert(scans.end(), rScans.begin(), rScans.end());
        }

        return scans;
    }

    /**
     * Deletes all scan managers of all devices
     */
    void Clear()
    {
    //    SensorManager::Clear();

        forEach(ScanManagerMap, &m_ScanManagers)
        {
        delete iter->second;
        }

        m_ScanManagers.clear();
    }

  private:
    /**
     * Get scan manager for localized range scan
     * @return ScanManager
     */
    inline ScanManager* GetScanManager(LocalizedRangeScan* pScan)
    {
      return GetScanManager(pScan->GetSensorName());
    }

    /**
     * Get scan manager for id
     * @param rSensorName
     * @return ScanManager
     */
    inline ScanManager* GetScanManager(const Name& rSensorName)
    {
      if (m_ScanManagers.find(rSensorName) != m_ScanManagers.end())
      {
        return m_ScanManagers[rSensorName];
      }

      return NULL;
    }

  private:
    // map from device ID to scan data
    ScanManagerMap m_ScanManagers;

    kt_int32u m_RunningBufferMaximumSize;
    kt_double m_RunningBufferMaximumDistance;

    kt_int32s m_NextScanId;

    std::vector<LocalizedRangeScan*> m_Scans;
  };  // MapperSensorDataManager





    
}  //namespace Karto










#endif //_MAPPER_SENSORDATA_MANAGER_H__
