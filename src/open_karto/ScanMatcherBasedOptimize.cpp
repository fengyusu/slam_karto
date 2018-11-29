//
// Created by fengyusu on 18-11-20.
//

#include "ScanMatcherBasedOptimize.h"


namespace karto {



    ScanMatcherBasedOptimize::~ScanMatcherBasedOptimize()
    {

    }

    ScanMatcherBasedOptimize* ScanMatcherBasedOptimize::Create(MapperSensorDataManager* p_sensordata_manager)
    {
        ScanMatcherBasedOptimize* pScanMatcher = new ScanMatcherBasedOptimize(p_sensordata_manager);

        return pScanMatcher;
    }

    void ScanMatcherBasedOptimize::BuildOptimizationProblem(const std::vector<Eigen::Vector2d>& range_points) {

            if (range_points.empty()) {
                std::cout << "No measurements, no problem to optimize." << std::endl;
                return;
            }

            const int iterations = 10;
            double cost, last_cost;

            for(int i = 0; i < iterations; i++){
                Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
                Eigen::Vector3d b = Eigen::Vector3d::Zero();
                cost = 0;

                Eigen::Vector2d translation(estimate_x_, estimate_y_);
                Eigen::Matrix2d rotation;
                rotation << cos(estimate_yaw_), -sin(estimate_yaw_),
                            sin(estimate_yaw_), cos(estimate_yaw_);

                for(const auto range_point : range_points){
                    double error = 0;
                    double scan_match_response = 0;
                    Eigen::Vector2d map_point = rotation * range_point + translation;

                    if(PointInMap(map_point, occupancy_grid_)){

                        double x = map_point[0];
                        double y = map_point[1];

                        double resolution = (occupancy_grid_->GetResolution());
                        Vector2<double> offset = occupancy_grid_->GetCoordinateConverter()->GetOffset();
                        double gridX = (x - offset.GetX()) / resolution;
                        double gridY = (y - offset.GetY()) / resolution;

                        Vector2<kt_int32s> grid00(static_cast<kt_int32s>(gridX), static_cast<kt_int32s>(gridY));
                        Vector2<kt_int32s> grid01(static_cast<kt_int32s>(gridX), static_cast<kt_int32s>(gridY) + 1);
                        Vector2<kt_int32s> grid10(static_cast<kt_int32s>(gridX) + 1, static_cast<kt_int32s>(gridY));
                        Vector2<kt_int32s> grid11(static_cast<kt_int32s>(gridX) + 1, static_cast<kt_int32s>(gridY)+ 1);

                        double m_p00,m_p01,m_p10,m_p11;
                        if(occupancy_grid_->GetValue(grid00) == GridStates_Occupied){
                            m_p00 = 1;
                        }else{
                            m_p00 = 0;
                        }
                        if(occupancy_grid_->GetValue(grid01) == GridStates_Occupied){
                            m_p01 = 1;
                        }else{
                            m_p01 = 0;
                        }
                        if(occupancy_grid_->GetValue(grid10) == GridStates_Occupied){
                            m_p10 = 1;
                        }else{
                            m_p10 = 0;
                        }
                        if(occupancy_grid_->GetValue(grid11) == GridStates_Occupied){
                            m_p11 = 1;
                        }else{
                            m_p11 = 0;
                        }

                        Vector2<double> world00 = occupancy_grid_->GridToWorld(grid00);
                        Vector2<double> world11 = occupancy_grid_->GridToWorld(grid11);
                        double x0 = world00.GetX();
                        double y0 = world00.GetY();
                        double x1 = world11.GetX();
                        double y1 = world11.GetY();

                        
                        scan_match_response = ((y - y0) / (y1 - y0)) * (m_p11 * (x -x0) / (x1 - x0) + m_p01 * (x1 - x) / (x1 - x0)) + 
                                            ((y1 - y) / (y1 - y0)) * (m_p10 * (x -x0) / (x1 - x0) + m_p00 * (x1 - x) / (x1 - x0));
                    

                        // std::cout << "x : " << x << " y : " << y 
                        //           << " x0 : " << x0  << " y0 : " << y0 
                        //           << " x1 : " << x1  << " y1 : " << y1
                        //           << " scan_match_response : " << scan_match_response << std::endl;

                        scan_match_response = (scan_match_response >= 0) ? ((scan_match_response <= 1) ? (scan_match_response) : (1)) : (0);
                        error = 1 - scan_match_response;
                        cost += (error * error);

                        Eigen::Matrix<double,1,3> j;

                        Eigen::Matrix<double,2,3> de_s;
                        Eigen::Matrix<double,1,2> de_m;

                        de_s << 1, 0, (-sin(estimate_yaw_) * range_point[0] - cos(estimate_yaw_) * range_point[1]),
                                0, 1, (cos(estimate_yaw_) * range_point[0] - sin(estimate_yaw_) * range_point[1]);

                        de_m(0,0) = ((y - y0) / (y1 - y0)) * (m_p11 - m_p01) + ((y1 - y) / (y1 - y0)) * (m_p10 - m_p00);
                        de_m(0,1) = ((x -x0) / (x1 - x0)) * (m_p11 - m_p10) + ((x1 - x) / (x1 - x0)) * (m_p01 - m_p00);

                        j = -de_m * de_s;

                        H += j.transpose() * j;
                        b += -j.transpose() * error;

                    }

                }

                Eigen::Vector3d det;
                det = H.ldlt().solve(b);

                if (std::isnan(det[0]) || std::isnan(det[1]) || std::isnan(det[2])){
                    std::cout << "result is nan!" << std::endl;
                    break;
                }

                if (i > 0 && cost > last_cost) {
                    std::cout << "cost increase ! " << "current cost: " << cost << ", last cost: " << last_cost << std::endl;
                    break;
                }

                if(det[2] > 0.2){
                    det[2] = 0.2;
                }else if(det[2] < -0.2){
                    det[2] = -0.2;
                }
                estimate_x_ += det[0];
                estimate_y_ += det[1];
                estimate_yaw_ += det[2];

                last_cost = cost;

                std::cout << "iterators " << i << "  total cost: " << cost << std::endl;



            }


        }

    double ScanMatcherBasedOptimize::MatchScan(LocalizedRangeScan* pScan,
                        OccupancyGrid* occupancy_grid,
                        Pose2& rMean, Matrix3& rCovariance){
            if(occupancy_grid != nullptr){
                occupancy_grid_ = occupancy_grid;
                std::cout << "map width : " << occupancy_grid_->GetWidth() 
                        << "    map height : " << occupancy_grid_->GetHeight() << std::endl;
            }else{
                std::cout << "null map !" << std::endl;
            }         


            estimate_x_ = pScan->GetSensorPose().GetX();
            estimate_y_ = pScan->GetSensorPose().GetY();
            estimate_yaw_ = pScan->GetSensorPose().GetHeading();


            const PointVectorDouble& rPointReadings = pScan->GetPointReadings(true);
            Transform transform(pScan->GetSensorPose());
            Pose2Vector localPoints;
            const_forEach(PointVectorDouble, &rPointReadings)
            {
                Pose2 vec = transform.InverseTransformPose(Pose2(*iter, 0.0));
                localPoints.push_back(vec);
            }

            std::vector<Eigen::Vector2d> range_points;
            for(const auto range_point : localPoints){
                Eigen::Vector2d point(range_point.GetX(), range_point.GetY());
                range_points.push_back(point);
            }

            BuildOptimizationProblem(range_points);

            best_pose_ << estimate_x_, estimate_y_, estimate_yaw_;
            // rMean.SetX(estimate_x_);
            // rMean.SetY(estimate_y_);
            // rMean.SetHeading(estimate_yaw_);
                            
        }

}