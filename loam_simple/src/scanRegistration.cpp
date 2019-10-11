#include"scanRegistration.h"
#include <fstream>
#include<sstream>
using namespace std;




MultiScanMapper::MultiScanMapper(const float& lowerBound,
                                 const float& upperBound,
                                 const uint16_t& nScanRings)
    : _lowerBound(lowerBound),
      _upperBound(upperBound),
      _nScanRings(nScanRings),
      _factor((nScanRings - 1) / (upperBound - lowerBound))
{

}

void MultiScanMapper::set(const float &lowerBound,
                          const float &upperBound,
                          const uint16_t &nScanRings)
{
    _lowerBound = lowerBound;
    _upperBound = upperBound;
    _nScanRings = nScanRings;
    _factor = (nScanRings - 1) / (upperBound - lowerBound);
}



int MultiScanMapper::getRingForAngle(const float& angle) {
    return int(((angle * 180 / M_PI) - _lowerBound) * _factor + 0.5);
}





ScanRegistration::ScanRegistration()
{
    _scanMapper = MultiScanMapper::Velodyne_VLP_16();

    // subscribe to input cloud topic
    _subLaserCloud = node.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1, &ScanRegistration::handleCloudMessage, this);

    /*
    // advertise scan registration topics
    _pubLaserCloud = node.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_22", 2);
    _pubCornerPointsSharp = node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_sharp22", 2);
    _pubCornerPointsLessSharp = node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp22", 2);
    _pubSurfPointsFlat = node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_flat22", 2);
    _pubSurfPointsLessFlat = node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_flat22", 2);
    */
}

ScanRegistration::~ScanRegistration()
{

}


void ScanRegistration::handleCloudMessage(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{

    //downsample to upgrade the effiency
    pointCloudCount++;
    if(pointCloudCount%2==0){
        return;
    }
    pcl::PointCloud<pcl::PointXYZ> laserCloudIn;
    pcl::fromROSMsg(*laserCloudMsg, laserCloudIn);
    //ROS_INFO_STREAM("scanRegistration.cpp: received laserCloudMsg "<<laserCloudMsg->header.stamp);
    process(laserCloudIn, laserCloudMsg->header.stamp);
}

void ScanRegistration::process(const pcl::PointCloud<pcl::PointXYZ>& laserCloudIn,
                               const ros::Time& scanTime)
{


    size_t cloudSize = laserCloudIn.size();

    //clear the cloud from last sweep
    reset(scanTime);

    // determine scan start and end orientations
    float startOri = -std::atan2(laserCloudIn[0].y, laserCloudIn[0].x);
    float endOri = -std::atan2(laserCloudIn[cloudSize - 1].y,
            laserCloudIn[cloudSize - 1].x) + 2 * float(M_PI);
    if (endOri - startOri > 3 * M_PI) {
        endOri -= 2 * M_PI;
    } else if (endOri - startOri < M_PI) {
        endOri += 2 * M_PI;
    }

    bool halfPassed = false;
    pcl::PointXYZI point;
    std::vector<pcl::PointCloud<pcl::PointXYZI> > laserCloudScans(_scanMapper.getNumberOfScanRings());

    // extract valid points from input cloud
    for (int i = 0; i < cloudSize; i++) {
        point.x = laserCloudIn[i].x;
        point.y = laserCloudIn[i].y;
        point.z = laserCloudIn[i].z;

        // skip NaN and INF valued points
        if (!pcl_isfinite(point.x) ||
                !pcl_isfinite(point.y) ||
                !pcl_isfinite(point.z)) {
            continue;
        }

        // skip zero valued points
        if (point.x * point.x + point.y * point.y + point.z * point.z < 0.0001) {
            continue;
        }

        // calculate vertical point angle and scan ID
        //float angle = std::atan(point.y / std::sqrt(point.x * point.x + point.z * point.z));
        float angle = std::atan(point.z / std::sqrt(point.x * point.x + point.y * point.y));
        int scanID = _scanMapper.getRingForAngle(angle);
        if (scanID >= _scanMapper.getNumberOfScanRings() || scanID < 0 ){
            continue;
        }

        // calculate horizontal point angle
        //float ori = -std::atan2(point.x, point.z);
        float ori = -std::atan2(point.y, point.x);
        if (!halfPassed) {
            if (ori < startOri - M_PI / 2) {
                ori += 2 * M_PI;
            } else if (ori > startOri + M_PI * 3 / 2) {
                ori -= 2 * M_PI;
            }

            if (ori - startOri > M_PI) {
                halfPassed = true;
            }
        } else {
            ori += 2 * M_PI;

            if (ori < endOri - M_PI * 3 / 2) {
                ori += 2 * M_PI;
            } else if (ori > endOri + M_PI / 2) {
                ori -= 2 * M_PI;
            }
        }

        // calculate relative scan time based on point orientation
        float relTime = scanPeriod * (ori - startOri) / (endOri - startOri);
        point.intensity = scanID + relTime;

        laserCloudScans[scanID].push_back(point);
    }

    // construct sorted full resolution cloud
    cloudSize = 0;
    for (int i = 0; i < _scanMapper.getNumberOfScanRings(); i++) {
        _laserCloud += laserCloudScans[i];

        IndexRange range(cloudSize, 0);
        cloudSize += laserCloudScans[i].size();
        range.second = cloudSize > 0 ? cloudSize - 1 : 0;
        _scanIndices.push_back(range);
    }



    // extract features
    extractFeatures();

    //put data to queue
    ScanRegisCloudTotal tmpPointCloudTotal;
    pcl::copyPointCloud(_cornerPointsSharp, tmpPointCloudTotal._cornerPointsSharp);
    pcl::copyPointCloud(_cornerPointsLessSharp, tmpPointCloudTotal._cornerPointsLessSharp);
    pcl::copyPointCloud(_surfacePointsFlat, tmpPointCloudTotal._surfacePointsFlat);
    pcl::copyPointCloud(_surfacePointsLessFlat, tmpPointCloudTotal._surfacePointsLessFlat);
    pcl::copyPointCloud(_laserCloud, tmpPointCloudTotal._laserCloud);
    point_cloud_lock.lock();
    queuePointCloudTotal.push(tmpPointCloudTotal);
    point_cloud_lock.unlock();
    ROS_INFO_STREAM("scanRegistration.cpp:queuePointCloudTotal.size: "<<queuePointCloudTotal.size());


}



void ScanRegistration::extractFeatures(const uint16_t& beginIdx)
{

    // extract features from individual scans
    size_t nScans = _scanIndices.size();
    for (size_t i = beginIdx; i < nScans; i++) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<pcl::PointXYZI>);
        size_t scanStartIdx = _scanIndices[i].first;
        size_t scanEndIdx = _scanIndices[i].second;

        // skip empty scans
        if (scanEndIdx <= scanStartIdx + 2 * curvatureRegion) {
            continue;
        }

        // Quick&Dirty fix for relative point time calculation without IMU data
        /*float scanSize = scanEndIdx - scanStartIdx + 1;
    for (int j = scanStartIdx; j <= scanEndIdx; j++) {
      _laserCloud[j].intensity = i + _scanPeriod * (j - scanStartIdx) / scanSize;
    }*/

        // reset scan buffers
        setScanBuffersFor(scanStartIdx, scanEndIdx);

        // extract features from equally sized scan regions
        for (int j = 0; j < nFeatureRegions; j++) {
            size_t sp = ((scanStartIdx + curvatureRegion) * (nFeatureRegions - j)
                         + (scanEndIdx - curvatureRegion) * j) / nFeatureRegions;
            size_t ep = ((scanStartIdx + curvatureRegion) * (nFeatureRegions - 1 - j)
                         + (scanEndIdx - curvatureRegion) * (j + 1)) / nFeatureRegions - 1;

            // skip empty regions
            if (ep <= sp) {
                continue;
            }

            size_t regionSize = ep - sp + 1;

            // reset region buffers
            setRegionBuffersFor(sp, ep);


            int largestPickedNum = 0;
            for (size_t k = regionSize; k > 0 && largestPickedNum < maxCornerLessSharp;) {
                size_t idx = _regionSortIndices[--k];
                size_t scanIdx = idx - scanStartIdx;
                size_t regionIdx = idx - sp;

                if (_scanNeighborPicked[scanIdx] == 0 &&
                        _regionCurvature[regionIdx] > surfaceCurvatureThreshold) {

                    largestPickedNum++;
                    if (largestPickedNum <= maxCornerSharp) {
                        _regionLabel[regionIdx] = CORNER_SHARP;
                        _cornerPointsSharp.push_back(_laserCloud[idx]);
                    } else {
                        _regionLabel[regionIdx] = CORNER_LESS_SHARP;
                    }
                    _cornerPointsLessSharp.push_back(_laserCloud[idx]);

                    markAsPicked(idx, scanIdx);
                }
            }

            // extract flat surface features
            int smallestPickedNum = 0;
            for (int k = 0; k < regionSize && smallestPickedNum < maxSurfaceFlat; k++) {
                size_t idx = _regionSortIndices[k];
                size_t scanIdx = idx - scanStartIdx;
                size_t regionIdx = idx - sp;

                if (_scanNeighborPicked[scanIdx] == 0 &&
                        _regionCurvature[regionIdx] < surfaceCurvatureThreshold) {

                    smallestPickedNum++;
                    _regionLabel[regionIdx] = SURFACE_FLAT;
                    _surfacePointsFlat.push_back(_laserCloud[idx]);

                    markAsPicked(idx, scanIdx);
                }
            }

            // extract less flat surface features
            for (int k = 0; k < regionSize; k++) {
                if (_regionLabel[k] <= SURFACE_LESS_FLAT) {
                    surfPointsLessFlatScan->push_back(_laserCloud[sp + k]);
                }
            }
        }

        // down size less flat surface point cloud of current scan
        pcl::PointCloud<pcl::PointXYZI> surfPointsLessFlatScanDS;
        pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;
        downSizeFilter.setInputCloud(surfPointsLessFlatScan);
        downSizeFilter.setLeafSize(lessFlatFilterSize, lessFlatFilterSize, lessFlatFilterSize);
        downSizeFilter.filter(surfPointsLessFlatScanDS);

        _surfacePointsLessFlat += surfPointsLessFlatScanDS;

        /*publishCloudMsg(_pubLaserCloud,            _laserCloud,            _sweepStart, "/camera");
        publishCloudMsg(_pubCornerPointsSharp,     _cornerPointsSharp,     _sweepStart, "/camera");
        publishCloudMsg(_pubCornerPointsLessSharp, _cornerPointsLessSharp, _sweepStart, "/camera");
        publishCloudMsg(_pubSurfPointsFlat,        _surfacePointsFlat,     _sweepStart, "/camera");
        publishCloudMsg(_pubSurfPointsLessFlat,    _surfacePointsLessFlat, _sweepStart, "/camera");*/


    }


}

bool ScanRegistration::getPointCloud(ros::Time&ros_time,pcl::PointCloud<pcl::PointXYZI>& cornerPointsSharp,pcl::PointCloud<pcl::PointXYZI>& cornerPointsLessSharp,
                                     pcl::PointCloud<pcl::PointXYZI>& surfacePointsFlat,pcl::PointCloud<pcl::PointXYZI>& surfacePointsLessFlat,
                                     pcl::PointCloud<pcl::PointXYZI>& laserCloud)
{

    if(queuePointCloudTotal.empty()){
        return false;
    }
    point_cloud_lock.lock();
    ScanRegisCloudTotal unit = queuePointCloudTotal.front();
    queuePointCloudTotal.pop();
    point_cloud_lock.unlock();
    pcl::copyPointCloud(unit._cornerPointsSharp,cornerPointsSharp);
    pcl::copyPointCloud(unit._cornerPointsLessSharp,cornerPointsLessSharp);
    pcl::copyPointCloud(unit._surfacePointsFlat,surfacePointsFlat);
    pcl::copyPointCloud(unit._surfacePointsLessFlat,surfacePointsLessFlat);
    pcl::copyPointCloud(unit._laserCloud,laserCloud);
    ros_time=_scanTime;
    return true;

}

void ScanRegistration::setScanBuffersFor(const size_t& startIdx,
                                         const size_t& endIdx)
{
    // resize buffers
    size_t scanSize = endIdx - startIdx + 1;
    _scanNeighborPicked.assign(scanSize, 0);

    // mark unreliable points as picked
    for (size_t i = startIdx +   curvatureRegion; i < endIdx -   curvatureRegion; i++) {
        const pcl::PointXYZI& previousPoint = (_laserCloud[i - 1]);
        const pcl::PointXYZI& point = (_laserCloud[i]);
        const pcl::PointXYZI& nextPoint = (_laserCloud[i + 1]);


        float diffNext = calcSquaredDiff(nextPoint, point);

        if (diffNext > 0.1) {
            float depth1 = calcPointDistance(point);
            float depth2 = calcPointDistance(nextPoint);

            if (depth1 > depth2) {
                float weighted_distance = std::sqrt(calcSquaredDiff(nextPoint, point, depth2 / depth1)) / depth2;

                if (weighted_distance < 0.1) {
                    std::fill_n(&_scanNeighborPicked[i - startIdx -   curvatureRegion],   curvatureRegion + 1, 1);

                    continue;
                }
            } else {
                float weighted_distance = std::sqrt(calcSquaredDiff(point, nextPoint, depth1 / depth2)) / depth1;

                if (weighted_distance < 0.1) {
                    std::fill_n(&_scanNeighborPicked[i - startIdx + 1],   curvatureRegion + 1, 1);
                }
            }
        }

        float diffPrevious = calcSquaredDiff(point, previousPoint);
        float dis = calcSquaredPointDistance(point);


        if (diffNext > 0.0002 * dis && diffPrevious > 0.0002 * dis) {
            _scanNeighborPicked[i - startIdx] = 1;
        }
    }
}


void ScanRegistration::markAsPicked(const size_t& cloudIdx,
                                    const size_t& scanIdx)
{
    _scanNeighborPicked[scanIdx] = 1;

    for (int i = 1; i <=curvatureRegion; i++) {
        if (calcSquaredDiff(_laserCloud[cloudIdx + i], _laserCloud[cloudIdx + i - 1]) > 0.05) {
            break;
        }

        _scanNeighborPicked[scanIdx + i] = 1;
    }

    for (int i = 1; i <=curvatureRegion; i++) {
        if (calcSquaredDiff(_laserCloud[cloudIdx - i], _laserCloud[cloudIdx - i + 1]) > 0.05) {
            break;
        }

        _scanNeighborPicked[scanIdx - i] = 1;
    }
}



void ScanRegistration::setRegionBuffersFor(const size_t& startIdx,
                                           const size_t& endIdx)
{

    // resize buffers
    size_t regionSize = endIdx - startIdx + 1;

    _regionCurvature.resize(regionSize);
    _regionSortIndices.resize(regionSize);
    _regionLabel.assign(regionSize, SURFACE_LESS_FLAT);

    // calculate point curvatures and reset sort indices
    float pointWeight = -2 *    curvatureRegion;

    for (size_t i = startIdx, regionIdx = 0; i <= endIdx; i++, regionIdx++) {
        float diffX = pointWeight * _laserCloud[i].x;
        float diffY = pointWeight * _laserCloud[i].y;
        float diffZ = pointWeight * _laserCloud[i].z;

        for (int j = 1; j <= curvatureRegion; j++) {
            diffX += _laserCloud[i + j].x + _laserCloud[i - j].x;
            diffY += _laserCloud[i + j].y + _laserCloud[i - j].y;
            diffZ += _laserCloud[i + j].z + _laserCloud[i - j].z;
        }

        _regionCurvature[regionIdx] = diffX * diffX + diffY * diffY + diffZ * diffZ;
        _regionSortIndices[regionIdx] = i;

    }


    // sort point curvatures
    for (size_t i = 1; i < regionSize; i++) {
        for (size_t j = i; j >= 1; j--) {
            if (_regionCurvature[_regionSortIndices[j] - startIdx] < _regionCurvature[_regionSortIndices[j - 1] - startIdx]) {
                std::swap(_regionSortIndices[j], _regionSortIndices[j - 1]);
            }
        }
    }



}

void ScanRegistration::reset(const ros::Time& scanTime,
                             const bool& newSweep)
{

    _scanTime=scanTime;

    // clear internal cloud buffers at the beginning of a sweep
    if (newSweep) {
        _sweepStart = scanTime;

        // clear cloud buffers
        _laserCloud.clear();
        _cornerPointsSharp.clear();
        _cornerPointsLessSharp.clear();
        _surfacePointsFlat.clear();
        _surfacePointsLessFlat.clear();

        // clear scan indices vector
        _scanIndices.clear();
    }
}
