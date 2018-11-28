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

}