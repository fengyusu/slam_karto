//
// Created by fengyusu on 18-11-20.
//

#ifndef PROJECT_SCANMATCHER_BASED_OPTIMIZE_H
#define PROJECT_SCANMATCHER_BASED_OPTIMIZE_H


#include "Karto.h"
#include "SensorDataManager.h"

#include <Eigen/Core>
#include <Eigen/Geometry>



namespace karto {




/**
 * Scan matcher
 */
    class ScanMatcherBasedOptimize
    {
    public:
        /**
         * Destructor
         */
        virtual ~ScanMatcherBasedOptimize();

    public:
        /**
         * Create a scan matcher with the given parameters
         */
        static ScanMatcherBasedOptimize* Create(MapperSensorDataManager* p_sensordata_manager);

        double MatchScan(LocalizedRangeScan* pScan,
                        OccupancyGrid* occupancy_grid,
                        Pose2& rMean, Matrix3& rCovariance);
  

    private:

        void BuildOptimizationProblem(const std::vector<Eigen::Vector2d>& range_points);


        

    protected:
        /**
         * Default constructor
         */
        ScanMatcherBasedOptimize(MapperSensorDataManager* p_sensordata_manager):
            p_sensordata_manager_(p_sensordata_manager)
        {
        }

        inline bool PointInMap(Eigen::Vector2d point, OccupancyGrid* occupancy_grid){
            BoundingBox2 map_bounding = occupancy_grid->GetBoundingBox();
            double bound_tolerance = occupancy_grid->GetResolution() * 2;
            if(point[0] > (map_bounding.GetMinimum().GetX() + bound_tolerance) && 
               point[1] > (map_bounding.GetMinimum().GetY() + bound_tolerance) && 
               point[0] < (map_bounding.GetMaximum().GetX() - bound_tolerance) && 
               point[1] < (map_bounding.GetMaximum().GetY() - bound_tolerance)){
                   return true;
            }
            return false;
        }

    private:
  
    MapperSensorDataManager* p_sensordata_manager_;
    OccupancyGrid* occupancy_grid_;

    Eigen::Vector3d best_pose_;
    
    double estimate_x_;
    double estimate_y_;
    double estimate_yaw_;

    };  // ScanMatcher


}

#endif //PROJECT_SCANMATCHER_H
