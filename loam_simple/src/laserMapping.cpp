#include"laserMapping.h"
#include <fstream>
#include<sstream>
using namespace std;

LaserMapping::LaserMapping(LaserOdometry*laserOdomObj)
    : _scanPeriod(0.1),
      _maxIterations(10),
      _stackFrameNum(1),
      _mapFrameNum(5),
      _frameCount(0),
      _mapFrameCount(0),
      _deltaTAbort(0.05),
      _deltaRAbort(0.05),
      _laserCloudCenWidth(10),
      _laserCloudCenHeight(5),
      _laserCloudCenDepth(10),
      _laserCloudWidth(21),
      _laserCloudHeight(11),
      _laserCloudDepth(21),
      _laserCloudNum(_laserCloudWidth * _laserCloudHeight * _laserCloudDepth),
      _laserCloudCornerLast(new pcl::PointCloud<pcl::PointXYZI>()),
      _laserCloudSurfLast(new pcl::PointCloud<pcl::PointXYZI>()),
      _laserCloudFullRes(new pcl::PointCloud<pcl::PointXYZI>()),
      _laserCloudCornerStack(new pcl::PointCloud<pcl::PointXYZI>()),
      _laserCloudSurfStack(new pcl::PointCloud<pcl::PointXYZI>()),
      _laserCloudCornerStackDS(new pcl::PointCloud<pcl::PointXYZI>()),
      _laserCloudSurfStackDS(new pcl::PointCloud<pcl::PointXYZI>()),
      _laserCloudSurround(new pcl::PointCloud<pcl::PointXYZI>()),
      _laserCloudSurroundDS(new pcl::PointCloud<pcl::PointXYZI>()),
      _laserCloudCornerFromMap(new pcl::PointCloud<pcl::PointXYZI>()),
      _laserCloudSurfFromMap(new pcl::PointCloud<pcl::PointXYZI>())
{
    laserOdomObject=laserOdomObj;

    // initialize mapping odometry and odometry tf messages
    _odomAftMapped.header.frame_id = "/camera_init";
    _odomAftMapped.child_frame_id = "/aft_mapped";

    _aftMappedTrans.frame_id_ = "/camera_init";
    _aftMappedTrans.child_frame_id_ = "/aft_mapped";

    segMapTrans.frame_id_ = "/world";   //segmap: default is /world
    segMapTrans.child_frame_id_ = "/velodyne";

    // initialize frame counter
    _frameCount = _stackFrameNum - 1;
    _mapFrameCount = _mapFrameNum - 1;

    // setup cloud vectors
    _laserCloudCornerArray.resize(_laserCloudNum);
    _laserCloudSurfArray.resize(_laserCloudNum);
    _laserCloudCornerDSArray.resize(_laserCloudNum);
    _laserCloudSurfDSArray.resize(_laserCloudNum);

    for (size_t i = 0; i < _laserCloudNum; i++) {
        _laserCloudCornerArray[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
        _laserCloudSurfArray[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
        _laserCloudCornerDSArray[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
        _laserCloudSurfDSArray[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
    }

    // setup down size filters
    float scale=1.5;
    _downSizeFilterCorner.setLeafSize(0.2*scale, 0.2*scale, 0.2*scale);
    _downSizeFilterSurf.setLeafSize(0.4*scale, 0.4*scale, 0.4*scale);
    _downSizeFilterMap.setLeafSize(0.6*scale, 0.6*scale, 0.6*scale);
    scale=1.8;//1.8
    _downSizeFilterCorner_secondScale.setLeafSize(0.2*scale, 0.2*scale, 0.2*scale);
    _downSizeFilterSurf_secondScale.setLeafSize(0.4*scale, 0.4*scale, 0.4*scale);
    scale=2.0;//2.0
    _downSizeFilterCorner_thirdScale.setLeafSize(0.2*scale, 0.2*scale, 0.2*scale);
    _downSizeFilterSurf_thirdScale.setLeafSize(0.4*scale, 0.4*scale, 0.4*scale);

    // advertise laser mapping topics
    ros::NodeHandle node;
    _pubLaserCloudSurround = node.advertise<sensor_msgs::PointCloud2> ("/laser_cloud_surround", 1);
    _pubLaserCloudFullRes = node.advertise<sensor_msgs::PointCloud2> ("/velodyne_cloud_registered", 2);
    _pubOdomAftMapped = node.advertise<nav_msgs::Odometry> ("/aft_mapped_to_init", 5);
    _pubOriginData= node.advertise<sensor_msgs::PointCloud2> ("/myPoints", 1);
    _pubBaseLinkData= node.advertise<sensor_msgs::PointCloud2> ("/baselink_Points", 1);

    //open thread to get data from laserOdometry
    getPointCloudContinue=true;
    std::thread t1(&LaserMapping::getPointCloudFromOdometryThread,this);
    t1.detach();
}

LaserMapping::~LaserMapping()
{

}


void LaserMapping::getPointCloudFromOdometryThread()
{

    while (getPointCloudContinue)
    {
        pcl::PointCloud<pcl::PointXYZI>_lastCornerCloud;
        pcl::PointCloud<pcl::PointXYZI>_lastSurfaceCloud;
        pcl::PointCloud<pcl::PointXYZI>_laserCloud;
        nav_msgs::Odometry _initlaserOdometry;
        bool isOK=laserOdomObject->getPointCloudAndInitOdom(_initlaserOdometry,_lastCornerCloud,_lastSurfaceCloud,_laserCloud);
        if(!isOK){
            usleep(1000*50);
            continue;
        }
	//ROS_INFO_STREAM("laserMapping.cpp: received data from odometry ");
        _timeLaserOdometry = _initlaserOdometry.header.stamp;

        double roll, pitch, yaw;
        geometry_msgs::Quaternion geoQuat_ = _initlaserOdometry.pose.pose.orientation;
        tf::Matrix3x3(tf::Quaternion(geoQuat_.z, -geoQuat_.x, -geoQuat_.y, geoQuat_.w)).getRPY(roll, pitch, yaw);
        _transformSum.rot_x = -pitch;
        _transformSum.rot_y = -yaw;
        _transformSum.rot_z = roll;
        _transformSum.pos.x() = float(_initlaserOdometry.pose.pose.position.x);
        _transformSum.pos.y() = float(_initlaserOdometry.pose.pose.position.y);
        _transformSum.pos.z() = float(_initlaserOdometry.pose.pose.position.z);

        _laserCloudCornerLast=_lastCornerCloud.makeShared();
        _laserCloudSurfLast=_lastSurfaceCloud.makeShared();
        _laserCloudFullRes=_laserCloud.makeShared();
        //in order to coorporate with segMap:need to send tf and origin point cloud together
        pcl::toROSMsg(*_laserCloudFullRes,originData);



        clt.start("laserMapping.cpp: process() cost: ");
        process();
        clt.stop();
     
        publishResult();
        usleep(1000*10);
    }
}


void LaserMapping::process()
{
    // skip some frames?!?
    _frameCount++;
    if (_frameCount < _stackFrameNum) {
        return;
    }
    _frameCount = 0;

    pcl::PointXYZI pointSel;

    // relate incoming data to map
    transformAssociateToMap();

    size_t laserCloudCornerLastNum = _laserCloudCornerLast->points.size();
    for (int i = 0; i < laserCloudCornerLastNum; i++) {
        pointAssociateToMap(_laserCloudCornerLast->points[i], pointSel);
        _laserCloudCornerStack->push_back(pointSel);
    }

    size_t laserCloudSurfLastNum = _laserCloudSurfLast->points.size();
    for (int i = 0; i < laserCloudSurfLastNum; i++) {
        pointAssociateToMap(_laserCloudSurfLast->points[i], pointSel);
        _laserCloudSurfStack->push_back(pointSel);
    }


    pcl::PointXYZI pointOnYAxis;
    pointOnYAxis.x = 0.0;
    pointOnYAxis.y = 10.0;
    pointOnYAxis.z = 0.0;
    pointAssociateToMap(pointOnYAxis, pointOnYAxis);

    int centerCubeI = int((_transformTobeMapped.pos.x() + 25.0) / 50.0) + _laserCloudCenWidth;
    int centerCubeJ = int((_transformTobeMapped.pos.y() + 25.0) / 50.0) + _laserCloudCenHeight;
    int centerCubeK = int((_transformTobeMapped.pos.z() + 25.0) / 50.0) + _laserCloudCenDepth;

    if (_transformTobeMapped.pos.x() + 25.0 < 0) centerCubeI--;
    if (_transformTobeMapped.pos.y() + 25.0 < 0) centerCubeJ--;
    if (_transformTobeMapped.pos.z() + 25.0 < 0) centerCubeK--;

    while (centerCubeI < 3) {
        for (int j = 0; j < _laserCloudHeight; j++) {
            for (int k = 0; k < _laserCloudDepth; k++) {
                for (int i = _laserCloudWidth - 1; i >= 1; i--) {
                    const size_t indexA = toIndex(i, j, k);
                    const size_t indexB = toIndex(i-1, j, k);
                    std::swap( _laserCloudCornerArray[indexA], _laserCloudCornerArray[indexB] );
                    std::swap( _laserCloudSurfArray[indexA],   _laserCloudSurfArray[indexB]);
                }
            }
        }
        centerCubeI++;
        _laserCloudCenWidth++;
    }

    while (centerCubeI >= _laserCloudWidth - 3) {
        for (int j = 0; j < _laserCloudHeight; j++) {
            for (int k = 0; k < _laserCloudDepth; k++) {
                for (int i = 0; i < _laserCloudWidth - 1; i++) {
                    const size_t indexA = toIndex(i, j, k);
                    const size_t indexB = toIndex(i+1, j, k);
                    std::swap( _laserCloudCornerArray[indexA], _laserCloudCornerArray[indexB] );
                    std::swap( _laserCloudSurfArray[indexA],   _laserCloudSurfArray[indexB]);
                }
            }
        }
        centerCubeI--;
        _laserCloudCenWidth--;
    }

    while (centerCubeJ < 3) {
        for (int i = 0; i < _laserCloudWidth; i++) {
            for (int k = 0; k < _laserCloudDepth; k++) {
                for (int j = _laserCloudHeight - 1; j >= 1; j--) {
                    const size_t indexA = toIndex(i, j, k);
                    const size_t indexB = toIndex(i, j-1, k);
                    std::swap( _laserCloudCornerArray[indexA], _laserCloudCornerArray[indexB] );
                    std::swap( _laserCloudSurfArray[indexA],   _laserCloudSurfArray[indexB]);
                }
            }
        }
        centerCubeJ++;
        _laserCloudCenHeight++;
    }

    while (centerCubeJ >= _laserCloudHeight - 3) {
        for (int i = 0; i < _laserCloudWidth; i++) {
            for (int k = 0; k < _laserCloudDepth; k++) {
                for (int j = 0; j < _laserCloudHeight - 1; j++) {
                    const size_t indexA = toIndex(i, j, k);
                    const size_t indexB = toIndex(i, j+1, k);
                    std::swap( _laserCloudCornerArray[indexA], _laserCloudCornerArray[indexB] );
                    std::swap( _laserCloudSurfArray[indexA],   _laserCloudSurfArray[indexB]);
                }
            }
        }
        centerCubeJ--;
        _laserCloudCenHeight--;
    }

    while (centerCubeK < 3) {
        for (int i = 0; i < _laserCloudWidth; i++) {
            for (int j = 0; j < _laserCloudHeight; j++) {
                for (int k = _laserCloudDepth - 1; k >= 1; k--) {
                    const size_t indexA = toIndex(i, j, k);
                    const size_t indexB = toIndex(i, j, k-1);
                    std::swap( _laserCloudCornerArray[indexA], _laserCloudCornerArray[indexB] );
                    std::swap( _laserCloudSurfArray[indexA],   _laserCloudSurfArray[indexB]);
                }
            }
        }
        centerCubeK++;
        _laserCloudCenDepth++;
    }

    while (centerCubeK >= _laserCloudDepth - 3) {
        for (int i = 0; i < _laserCloudWidth; i++) {
            for (int j = 0; j < _laserCloudHeight; j++) {
                for (int k = 0; k < _laserCloudDepth - 1; k++) {
                    const size_t indexA = toIndex(i, j, k);
                    const size_t indexB = toIndex(i, j, k+1);
                    std::swap( _laserCloudCornerArray[indexA], _laserCloudCornerArray[indexB] );
                    std::swap( _laserCloudSurfArray[indexA],   _laserCloudSurfArray[indexB]);
                }
            }
        }
        centerCubeK--;
        _laserCloudCenDepth--;
    }

    _laserCloudValidInd.clear();
    _laserCloudSurroundInd.clear();
    for (int i = centerCubeI - 2; i <= centerCubeI + 2; i++) {
        for (int j = centerCubeJ - 2; j <= centerCubeJ + 2; j++) {
            for (int k = centerCubeK - 2; k <= centerCubeK + 2; k++) {
                if (i >= 0 && i < _laserCloudWidth &&
                        j >= 0 && j < _laserCloudHeight &&
                        k >= 0 && k < _laserCloudDepth) {

                    float centerX = 50.0f * (i - _laserCloudCenWidth);
                    float centerY = 50.0f * (j - _laserCloudCenHeight);
                    float centerZ = 50.0f * (k - _laserCloudCenDepth);

                    pcl::PointXYZI transform_pos = (pcl::PointXYZI) _transformTobeMapped.pos;

                    bool isInLaserFOV = false;
                    for (int ii = -1; ii <= 1; ii += 2) {
                        for (int jj = -1; jj <= 1; jj += 2) {
                            for (int kk = -1; kk <= 1; kk += 2) {
                                pcl::PointXYZI corner;
                                corner.x = centerX + 25.0f * ii;
                                corner.y = centerY + 25.0f * jj;
                                corner.z = centerZ + 25.0f * kk;

                                float squaredSide1 = calcSquaredDiff(transform_pos, corner);
                                float squaredSide2 = calcSquaredDiff(pointOnYAxis, corner);

                                float check1 = 100.0f + squaredSide1 - squaredSide2
                                        - 10.0f * sqrt(3.0f) * sqrt(squaredSide1);

                                float check2 = 100.0f + squaredSide1 - squaredSide2
                                        + 10.0f * sqrt(3.0f) * sqrt(squaredSide1);

                                if (check1 < 0 && check2 > 0) {
                                    isInLaserFOV = true;
                                }
                            }
                        }
                    }

                    size_t cubeIdx = i + _laserCloudWidth*j + _laserCloudWidth * _laserCloudHeight * k;
                    if (isInLaserFOV) {
                        _laserCloudValidInd.push_back(cubeIdx);
                    }
                    _laserCloudSurroundInd.push_back(cubeIdx);
                }
            }
        }
    }

    // prepare valid map corner and surface cloud for pose optimization
    _laserCloudCornerFromMap->clear();
    _laserCloudSurfFromMap->clear();
    size_t laserCloudValidNum = _laserCloudValidInd.size();
    for (int i = 0; i < laserCloudValidNum; i++) {
        *_laserCloudCornerFromMap += *_laserCloudCornerArray[_laserCloudValidInd[i]];
        *_laserCloudSurfFromMap += *_laserCloudSurfArray[_laserCloudValidInd[i]];
    }

    // prepare feature stack clouds for pose optimization
    size_t laserCloudCornerStackNum2 = _laserCloudCornerStack->points.size();
    for (int i = 0; i < laserCloudCornerStackNum2; i++) {
        pointAssociateTobeMapped(_laserCloudCornerStack->points[i], _laserCloudCornerStack->points[i]);
    }

    size_t laserCloudSurfStackNum2 = _laserCloudSurfStack->points.size();
    for (int i = 0; i < laserCloudSurfStackNum2; i++) {
        pointAssociateTobeMapped(_laserCloudSurfStack->points[i], _laserCloudSurfStack->points[i]);
    }

    // down sample feature stack clouds
    _laserCloudCornerStackDS->clear();
    _downSizeFilterCorner.setInputCloud(_laserCloudCornerStack);
    _downSizeFilterCorner.filter(*_laserCloudCornerStackDS);
    size_t laserCloudCornerStackNum = _laserCloudCornerStackDS->points.size();

    _laserCloudSurfStackDS->clear();
    _downSizeFilterSurf.setInputCloud(_laserCloudSurfStack);
    _downSizeFilterSurf.filter(*_laserCloudSurfStackDS);
    size_t laserCloudSurfStackNum = _laserCloudSurfStackDS->points.size();

    _laserCloudCornerStack->clear();
    _laserCloudSurfStack->clear();


    //ct.start("optimize");
    // run pose optimization
    optimizeTransformTobeMapped();
    //ct.stop();


    // store down sized corner stack points in corresponding cube clouds
    for (int i = 0; i < laserCloudCornerStackNum; i++) {
        pointAssociateToMap(_laserCloudCornerStackDS->points[i], pointSel);

        int cubeI = int((pointSel.x + 25.0) / 50.0) + _laserCloudCenWidth;
        int cubeJ = int((pointSel.y + 25.0) / 50.0) + _laserCloudCenHeight;
        int cubeK = int((pointSel.z + 25.0) / 50.0) + _laserCloudCenDepth;

        if (pointSel.x + 25.0 < 0) cubeI--;
        if (pointSel.y + 25.0 < 0) cubeJ--;
        if (pointSel.z + 25.0 < 0) cubeK--;

        if (cubeI >= 0 && cubeI < _laserCloudWidth &&
                cubeJ >= 0 && cubeJ < _laserCloudHeight &&
                cubeK >= 0 && cubeK < _laserCloudDepth) {
            size_t cubeInd = cubeI + _laserCloudWidth * cubeJ + _laserCloudWidth * _laserCloudHeight * cubeK;
            if(_laserCloudCornerArray[cubeInd]->size()>23000*2){//23000*1.5
                continue;
            }
            _laserCloudCornerArray[cubeInd]->push_back(pointSel);
        }
    }


    // store down sized surface stack points in corresponding cube clouds
    for (int i = 0; i < laserCloudSurfStackNum; i++) {
        pointAssociateToMap(_laserCloudSurfStackDS->points[i], pointSel);

        int cubeI = int((pointSel.x + 25.0) / 50.0) + _laserCloudCenWidth;
        int cubeJ = int((pointSel.y + 25.0) / 50.0) + _laserCloudCenHeight;
        int cubeK = int((pointSel.z + 25.0) / 50.0) + _laserCloudCenDepth;

        if (pointSel.x + 25.0 < 0) cubeI--;
        if (pointSel.y + 25.0 < 0) cubeJ--;
        if (pointSel.z + 25.0 < 0) cubeK--;

        if (cubeI >= 0 && cubeI < _laserCloudWidth &&
                cubeJ >= 0 && cubeJ < _laserCloudHeight &&
                cubeK >= 0 && cubeK < _laserCloudDepth) {
            size_t cubeInd = cubeI + _laserCloudWidth * cubeJ + _laserCloudWidth * _laserCloudHeight * cubeK;
            if(_laserCloudSurfArray[cubeInd]->size()>12000*2){//12000*1.5
                continue;
            }
            _laserCloudSurfArray[cubeInd]->push_back(pointSel);
        }
    }

    // down size all valid (within field of view) feature cube clouds
    for (int i = 0; i < laserCloudValidNum; i++) {
        size_t ind = _laserCloudValidInd[i];


        _laserCloudCornerDSArray[ind]->clear();
        if(_laserCloudCornerArray[ind]->points.size()<20000){
            _downSizeFilterCorner.setInputCloud(_laserCloudCornerArray[ind]);
            _downSizeFilterCorner.filter(*_laserCloudCornerDSArray[ind]);
        }
        else if(_laserCloudCornerArray[ind]->points.size()>=20000&&_laserCloudCornerArray[ind]->points.size()<=40000){
            _downSizeFilterCorner_secondScale.setInputCloud(_laserCloudCornerArray[ind]);
            _downSizeFilterCorner_secondScale.filter(*_laserCloudCornerDSArray[ind]);

        }
        else if(_laserCloudCornerArray[ind]->points.size()>40000){
            _downSizeFilterCorner_thirdScale.setInputCloud(_laserCloudCornerArray[ind]);
            _downSizeFilterCorner_thirdScale.filter(*_laserCloudCornerDSArray[ind]);
        }


        _laserCloudSurfDSArray[ind]->clear();
        if(_laserCloudSurfArray[ind]->points.size()<10000){
            _downSizeFilterSurf.setInputCloud(_laserCloudSurfArray[ind]);
            _downSizeFilterSurf.filter(*_laserCloudSurfDSArray[ind]);
        }
        else if(_laserCloudSurfArray[ind]->points.size()>=10000&&_laserCloudSurfArray[ind]->points.size()<=20000){

            _downSizeFilterSurf_secondScale.setInputCloud(_laserCloudSurfArray[ind]);
            _downSizeFilterSurf_secondScale.filter(*_laserCloudSurfDSArray[ind]);
        }
        else if(_laserCloudSurfArray[ind]->points.size()>20000){
            _downSizeFilterSurf_thirdScale.setInputCloud(_laserCloudSurfArray[ind]);
            _downSizeFilterSurf_thirdScale.filter(*_laserCloudSurfDSArray[ind]);

        }
   
        // swap cube clouds for next processing
        _laserCloudCornerArray[ind].swap(_laserCloudCornerDSArray[ind]);
        _laserCloudSurfArray[ind].swap(_laserCloudSurfDSArray[ind]);
    }
}


void LaserMapping::publishResult()
{


    // publish new map cloud according to the input output ratio
    /*_mapFrameCount++;
    if (_mapFrameCount >= _mapFrameNum) {
        _mapFrameCount = 0;

        // accumulate map cloud
        _laserCloudSurround->clear();
        size_t laserCloudSurroundNum = _laserCloudSurroundInd.size();
        for (int i = 0; i < laserCloudSurroundNum; i++) {
            size_t ind = _laserCloudSurroundInd[i];
            *_laserCloudSurround += *_laserCloudCornerArray[ind];
            *_laserCloudSurround += *_laserCloudSurfArray[ind];
        }

        // down size map cloud
        _laserCloudSurroundDS->clear();
        _downSizeFilterCorner.setInputCloud(_laserCloudSurround);
        _downSizeFilterCorner.filter(*_laserCloudSurroundDS);

        // publish new map cloud
        publishCloudMsg(_pubLaserCloudSurround, *_laserCloudSurroundDS, _timeLaserOdometry, "/camera_init");
    }
    // transform full resolution input cloud to map
    size_t laserCloudFullResNum = _laserCloudFullRes->points.size();
    for (int i = 0; i < laserCloudFullResNum; i++) {
        pointAssociateToMap(_laserCloudFullRes->points[i], _laserCloudFullRes->points[i]);
    }

    // publish transformed full resolution input cloud
    publishCloudMsg(_pubLaserCloudFullRes, *_laserCloudFullRes, _timeLaserOdometry, "/camera_init");
*/

    // publish odometry after mapped transformations
    geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw
            ( _transformAftMapped.rot_z.rad(),
              -_transformAftMapped.rot_x.rad(),
              -_transformAftMapped.rot_y.rad());

    //rotate the odom(16 rings do not need )
    tf::Quaternion odom_quat=tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w);
    tf::Quaternion q(0.0,0.0,0.0,1.0);//if rotate 90 degree(0.707) via Z axis,q(0.0,0.0,0.707,0.707)
    odom_quat*=q;
    geometry_msgs::Quaternion  quatAfterRotate;
    tf::quaternionTFToMsg(odom_quat, quatAfterRotate);
    _odomAftMapped.header.stamp = _timeLaserOdometry;
    _odomAftMapped.pose.pose.orientation.x = quatAfterRotate.x;
    _odomAftMapped.pose.pose.orientation.y = quatAfterRotate.y;
    _odomAftMapped.pose.pose.orientation.z = quatAfterRotate.z;
    _odomAftMapped.pose.pose.orientation.w = quatAfterRotate.w;
    _odomAftMapped.pose.pose.position.x = _transformAftMapped.pos.x();
    _odomAftMapped.pose.pose.position.y = _transformAftMapped.pos.y();
    _odomAftMapped.pose.pose.position.z = _transformAftMapped.pos.z();
    _odomAftMapped.twist.twist.angular.x = _transformBefMapped.rot_x.rad();
    _odomAftMapped.twist.twist.angular.y = _transformBefMapped.rot_y.rad();
    _odomAftMapped.twist.twist.angular.z = _transformBefMapped.rot_z.rad();
    _odomAftMapped.twist.twist.linear.x = _transformBefMapped.pos.x();
    _odomAftMapped.twist.twist.linear.y = _transformBefMapped.pos.y();
    _odomAftMapped.twist.twist.linear.z = _transformBefMapped.pos.z(); //_transformBefMapped.pos.z()
    _pubOdomAftMapped.publish(_odomAftMapped);

    _aftMappedTrans.stamp_ = _timeLaserOdometry;
    _aftMappedTrans.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
    _aftMappedTrans.setOrigin(tf::Vector3(_transformAftMapped.pos.x(),
                                          _transformAftMapped.pos.y(),
                                          _transformAftMapped.pos.z()));
    _tfBroadcaster_1.sendTransform(_aftMappedTrans);

    //odometry for segMap 2018/05/02
    segMapTrans.stamp_ = _timeLaserOdometry;
    tf::Quaternion quat=tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w);
    segMapTrans.setRotation(quat);
    segMapTrans.setOrigin(tf::Vector3(_transformAftMapped.pos.x(),
                                      _transformAftMapped.pos.y(),
                                      _transformAftMapped.pos.z()));
    _tfBroadcaster_2.sendTransform(segMapTrans);

    //publish origin points
    originData.header.stamp=_timeLaserOdometry;
    originData.header.frame_id="velodyne";
    _pubOriginData.publish(originData);
    usleep(10);
    originData.header.frame_id="base_link";
    _pubBaseLinkData.publish(originData);

   //ROS_INFO_STREAM("laserMapping.cpp: timestamp of topic  " << _timeLaserOdometry);
}

void LaserMapping::optimizeTransformTobeMapped()
{
    if (_laserCloudCornerFromMap->points.size() <= 10 || _laserCloudSurfFromMap->points.size() <= 100) {
        return;
    }

    pcl::PointXYZI pointSel, pointOri, pointProj, coeff;

    std::vector<int> pointSearchInd(5, 0);
    std::vector<float> pointSearchSqDis(5, 0);

    nanoflann::KdTreeFLANN<pcl::PointXYZI> kdtreeCornerFromMap;
    nanoflann::KdTreeFLANN<pcl::PointXYZI> kdtreeSurfFromMap;

    kdtreeCornerFromMap.setInputCloud(_laserCloudCornerFromMap);
    kdtreeSurfFromMap.setInputCloud(_laserCloudSurfFromMap);

    Eigen::Matrix<float, 5, 3> matA0;
    Eigen::Matrix<float, 5, 1> matB0;
    Eigen::Vector3f matX0;
    Eigen::Matrix3f matA1;
    Eigen::Matrix<float, 1, 3> matD1;
    Eigen::Matrix3f matV1;

    matA0.setZero();
    matB0.setConstant(-1);
    matX0.setZero();

    matA1.setZero();
    matD1.setZero();
    matV1.setZero();

    bool isDegenerate = false;
    Eigen::Matrix<float, 6, 6> matP;

    size_t laserCloudCornerStackNum = _laserCloudCornerStackDS->points.size();
    size_t laserCloudSurfStackNum = _laserCloudSurfStackDS->points.size();

    pcl::PointCloud<pcl::PointXYZI> laserCloudOri;
    pcl::PointCloud<pcl::PointXYZI> coeffSel;

    for (size_t iterCount = 0; iterCount < _maxIterations; iterCount++) {
        laserCloudOri.clear();
        coeffSel.clear();

        for (int i = 0; i < laserCloudCornerStackNum; i++) {
            pointOri = _laserCloudCornerStackDS->points[i];
            pointAssociateToMap(pointOri, pointSel);
            kdtreeCornerFromMap.nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis );

            if (pointSearchSqDis[4] < 1.0) {
                Vector3 vc(0,0,0);

                for (int j = 0; j < 5; j++) {
                    vc += Vector3(_laserCloudCornerFromMap->points[pointSearchInd[j]]);
                }
                vc /= 5.0;

                Eigen::Matrix3f mat_a;
                mat_a.setZero();

                for (int j = 0; j < 5; j++) {
                    Vector3 a = Vector3(_laserCloudCornerFromMap->points[pointSearchInd[j]]) - vc;

                    mat_a(0,0) += a.x() * a.x();
                    mat_a(0,1) += a.x() * a.y();
                    mat_a(0,2) += a.x() * a.z();
                    mat_a(1,1) += a.y() * a.y();
                    mat_a(1,2) += a.y() * a.z();
                    mat_a(2,2) += a.z() * a.z();
                }
                matA1 = mat_a / 5.0;

                Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> esolver(matA1);
                matD1 = esolver.eigenvalues().real();
                matV1 = esolver.eigenvectors().real();

                if (matD1(0, 2) > 3 * matD1(0, 1)) {

                    float x0 = pointSel.x;
                    float y0 = pointSel.y;
                    float z0 = pointSel.z;
                    float x1 = vc.x() + 0.1 * matV1(0, 2);
                    float y1 = vc.y() + 0.1 * matV1(1, 2);
                    float z1 = vc.z() + 0.1 * matV1(2, 2);
                    float x2 = vc.x() - 0.1 * matV1(0, 2);
                    float y2 = vc.y() - 0.1 * matV1(1, 2);
                    float z2 = vc.z() - 0.1 * matV1(2, 2);

                    float a012 = sqrt(((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                                      * ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                                      + ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
                                      * ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
                                      + ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))
                                      * ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)));

                    float l12 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));

                    float la = ((y1 - y2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                                + (z1 - z2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))) / a012 / l12;

                    float lb = -((x1 - x2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                                 - (z1 - z2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

                    float lc = -((x1 - x2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
                                 + (y1 - y2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

                    float ld2 = a012 / l12;

                    // TODO: Why writing to a variable that's never read? Maybe it should be used afterwards?
                    pointProj = pointSel;
                    pointProj.x -= la * ld2;
                    pointProj.y -= lb * ld2;
                    pointProj.z -= lc * ld2;

                    float s = 1 - 0.9f * fabs(ld2);

                    coeff.x = s * la;
                    coeff.y = s * lb;
                    coeff.z = s * lc;
                    coeff.intensity = s * ld2;

                    if (s > 0.1) {
                        laserCloudOri.push_back(pointOri);
                        coeffSel.push_back(coeff);
                    }
                }
            }
        }

        for (int i = 0; i < laserCloudSurfStackNum; i++) {
            pointOri = _laserCloudSurfStackDS->points[i];
            pointAssociateToMap(pointOri, pointSel);
            kdtreeSurfFromMap.nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis );

            if (pointSearchSqDis[4] < 1.0) {
                for (int j = 0; j < 5; j++) {
                    matA0(j, 0) = _laserCloudSurfFromMap->points[pointSearchInd[j]].x;
                    matA0(j, 1) = _laserCloudSurfFromMap->points[pointSearchInd[j]].y;
                    matA0(j, 2) = _laserCloudSurfFromMap->points[pointSearchInd[j]].z;
                }
                matX0 = matA0.colPivHouseholderQr().solve(matB0);

                float pa = matX0(0, 0);
                float pb = matX0(1, 0);
                float pc = matX0(2, 0);
                float pd = 1;

                float ps = sqrt(pa * pa + pb * pb + pc * pc);
                pa /= ps;
                pb /= ps;
                pc /= ps;
                pd /= ps;

                bool planeValid = true;
                for (int j = 0; j < 5; j++) {
                    if (fabs(pa * _laserCloudSurfFromMap->points[pointSearchInd[j]].x +
                             pb * _laserCloudSurfFromMap->points[pointSearchInd[j]].y +
                             pc * _laserCloudSurfFromMap->points[pointSearchInd[j]].z + pd) > 0.2) {
                        planeValid = false;
                        break;
                    }
                }

                if (planeValid) {
                    float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

                    // TODO: Why writing to a variable that's never read? Maybe it should be used afterwards?
                    pointProj = pointSel;
                    pointProj.x -= pa * pd2;
                    pointProj.y -= pb * pd2;
                    pointProj.z -= pc * pd2;

                    float s = 1 - 0.9f * fabs(pd2) / sqrt(calcPointDistance(pointSel));

                    coeff.x = s * pa;
                    coeff.y = s * pb;
                    coeff.z = s * pc;
                    coeff.intensity = s * pd2;

                    if (s > 0.1) {
                        laserCloudOri.push_back(pointOri);
                        coeffSel.push_back(coeff);
                    }
                }
            }
        }

        float srx = _transformTobeMapped.rot_x.sin();
        float crx = _transformTobeMapped.rot_x.cos();
        float sry = _transformTobeMapped.rot_y.sin();
        float cry = _transformTobeMapped.rot_y.cos();
        float srz = _transformTobeMapped.rot_z.sin();
        float crz = _transformTobeMapped.rot_z.cos();

        size_t laserCloudSelNum = laserCloudOri.points.size();
        if (laserCloudSelNum < 50) {
            continue;
        }

        Eigen::Matrix<float, Eigen::Dynamic, 6> matA(laserCloudSelNum, 6);
        Eigen::Matrix<float, 6, Eigen::Dynamic> matAt(6, laserCloudSelNum);
        Eigen::Matrix<float, 6, 6> matAtA;
        Eigen::VectorXf matB(laserCloudSelNum);
        Eigen::VectorXf matAtB;
        Eigen::VectorXf matX;

        for (int i = 0; i < laserCloudSelNum; i++) {
            pointOri = laserCloudOri.points[i];
            coeff = coeffSel.points[i];

            float arx = (crx*sry*srz*pointOri.x + crx*crz*sry*pointOri.y - srx*sry*pointOri.z) * coeff.x
                    + (-srx*srz*pointOri.x - crz*srx*pointOri.y - crx*pointOri.z) * coeff.y
                    + (crx*cry*srz*pointOri.x + crx*cry*crz*pointOri.y - cry*srx*pointOri.z) * coeff.z;

            float ary = ((cry*srx*srz - crz*sry)*pointOri.x
                         + (sry*srz + cry*crz*srx)*pointOri.y + crx*cry*pointOri.z) * coeff.x
                    + ((-cry*crz - srx*sry*srz)*pointOri.x
                       + (cry*srz - crz*srx*sry)*pointOri.y - crx*sry*pointOri.z) * coeff.z;

            float arz = ((crz*srx*sry - cry*srz)*pointOri.x + (-cry*crz-srx*sry*srz)*pointOri.y)*coeff.x
                    + (crx*crz*pointOri.x - crx*srz*pointOri.y) * coeff.y
                    + ((sry*srz + cry*crz*srx)*pointOri.x + (crz*sry-cry*srx*srz)*pointOri.y)*coeff.z;

            matA(i, 0) = arx;
            matA(i, 1) = ary;
            matA(i, 2) = arz;
            matA(i, 3) = coeff.x;
            matA(i, 4) = coeff.y;
            matA(i, 5) = coeff.z;
            matB(i, 0) = -coeff.intensity;
        }

        matAt = matA.transpose();
        matAtA = matAt * matA;
        matAtB = matAt * matB;
        matX = matAtA.colPivHouseholderQr().solve(matAtB);

        if (iterCount == 0) {
            Eigen::Matrix<float, 1, 6> matE;
            Eigen::Matrix<float, 6, 6> matV;
            Eigen::Matrix<float, 6, 6> matV2;

            Eigen::SelfAdjointEigenSolver< Eigen::Matrix<float,6, 6> > esolver(matAtA);
            matE = esolver.eigenvalues().real();
            matV = esolver.eigenvectors().real();

            matV2 = matV;

            isDegenerate = false;
            float eignThre[6] = {100, 100, 100, 100, 100, 100};
            for (int i = 0; i < 6; i++) {
                if (matE(0, i) < eignThre[i]) {
                    for (int j = 0; j < 6; j++) {
                        matV2(i, j) = 0;
                    }
                    isDegenerate = true;
                } else {
                    break;
                }
            }
            matP = matV.inverse() * matV2;
        }

        if (isDegenerate) {
            Eigen::Matrix<float,6, 1> matX2(matX);
            matX = matP * matX2;
        }

        _transformTobeMapped.rot_x += matX(0, 0);
        _transformTobeMapped.rot_y += matX(1, 0);
        _transformTobeMapped.rot_z += matX(2, 0);
        _transformTobeMapped.pos.x() += matX(3, 0);
        _transformTobeMapped.pos.y() += matX(4, 0);
        _transformTobeMapped.pos.z() += matX(5, 0);

        float deltaR = sqrt(pow(rad2deg(matX(0, 0)), 2) +
                            pow(rad2deg(matX(1, 0)), 2) +
                            pow(rad2deg(matX(2, 0)), 2));
        float deltaT = sqrt(pow(matX(3, 0) * 100, 2) +
                            pow(matX(4, 0) * 100, 2) +
                            pow(matX(5, 0) * 100, 2));

        if (deltaR < _deltaRAbort && deltaT < _deltaTAbort) {
            break;
        }
    }

    transformUpdate();
}
void LaserMapping::transformAssociateToMap()
{
    _transformIncre.pos = _transformBefMapped.pos - _transformSum.pos;
    rotateYXZ(_transformIncre.pos, -(_transformSum.rot_y), -(_transformSum.rot_x), -(_transformSum.rot_z));

    float sbcx = _transformSum.rot_x.sin();
    float cbcx = _transformSum.rot_x.cos();
    float sbcy = _transformSum.rot_y.sin();
    float cbcy = _transformSum.rot_y.cos();
    float sbcz = _transformSum.rot_z.sin();
    float cbcz = _transformSum.rot_z.cos();

    float sblx = _transformBefMapped.rot_x.sin();
    float cblx = _transformBefMapped.rot_x.cos();
    float sbly = _transformBefMapped.rot_y.sin();
    float cbly = _transformBefMapped.rot_y.cos();
    float sblz = _transformBefMapped.rot_z.sin();
    float cblz = _transformBefMapped.rot_z.cos();

    float salx = _transformAftMapped.rot_x.sin();
    float calx = _transformAftMapped.rot_x.cos();
    float saly = _transformAftMapped.rot_y.sin();
    float caly = _transformAftMapped.rot_y.cos();
    float salz = _transformAftMapped.rot_z.sin();
    float calz = _transformAftMapped.rot_z.cos();

    float srx = -sbcx*(salx*sblx + calx*cblx*salz*sblz + calx*calz*cblx*cblz)
            - cbcx*sbcy*(calx*calz*(cbly*sblz - cblz*sblx*sbly)
                         - calx*salz*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sbly)
            - cbcx*cbcy*(calx*salz*(cblz*sbly - cbly*sblx*sblz)
                         - calx*calz*(sbly*sblz + cbly*cblz*sblx) + cblx*cbly*salx);
    _transformTobeMapped.rot_x = -asin(srx);

    float srycrx = sbcx*(cblx*cblz*(caly*salz - calz*salx*saly)
                         - cblx*sblz*(caly*calz + salx*saly*salz) + calx*saly*sblx)
            - cbcx*cbcy*((caly*calz + salx*saly*salz)*(cblz*sbly - cbly*sblx*sblz)
                         + (caly*salz - calz*salx*saly)*(sbly*sblz + cbly*cblz*sblx) - calx*cblx*cbly*saly)
            + cbcx*sbcy*((caly*calz + salx*saly*salz)*(cbly*cblz + sblx*sbly*sblz)
                         + (caly*salz - calz*salx*saly)*(cbly*sblz - cblz*sblx*sbly) + calx*cblx*saly*sbly);
    float crycrx = sbcx*(cblx*sblz*(calz*saly - caly*salx*salz)
                         - cblx*cblz*(saly*salz + caly*calz*salx) + calx*caly*sblx)
            + cbcx*cbcy*((saly*salz + caly*calz*salx)*(sbly*sblz + cbly*cblz*sblx)
                         + (calz*saly - caly*salx*salz)*(cblz*sbly - cbly*sblx*sblz) + calx*caly*cblx*cbly)
            - cbcx*sbcy*((saly*salz + caly*calz*salx)*(cbly*sblz - cblz*sblx*sbly)
                         + (calz*saly - caly*salx*salz)*(cbly*cblz + sblx*sbly*sblz) - calx*caly*cblx*sbly);
    _transformTobeMapped.rot_y = atan2(srycrx / _transformTobeMapped.rot_x.cos(),
                                       crycrx / _transformTobeMapped.rot_x.cos());

    float srzcrx = (cbcz*sbcy - cbcy*sbcx*sbcz)*(calx*salz*(cblz*sbly - cbly*sblx*sblz)
                                                 - calx*calz*(sbly*sblz + cbly*cblz*sblx) + cblx*cbly*salx)
            - (cbcy*cbcz + sbcx*sbcy*sbcz)*(calx*calz*(cbly*sblz - cblz*sblx*sbly)
                                            - calx*salz*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sbly)
            + cbcx*sbcz*(salx*sblx + calx*cblx*salz*sblz + calx*calz*cblx*cblz);
    float crzcrx = (cbcy*sbcz - cbcz*sbcx*sbcy)*(calx*calz*(cbly*sblz - cblz*sblx*sbly)
                                                 - calx*salz*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sbly)
            - (sbcy*sbcz + cbcy*cbcz*sbcx)*(calx*salz*(cblz*sbly - cbly*sblx*sblz)
                                            - calx*calz*(sbly*sblz + cbly*cblz*sblx) + cblx*cbly*salx)
            + cbcx*cbcz*(salx*sblx + calx*cblx*salz*sblz + calx*calz*cblx*cblz);
    _transformTobeMapped.rot_z = atan2(srzcrx / _transformTobeMapped.rot_x.cos(),
                                       crzcrx / _transformTobeMapped.rot_x.cos());

    Vector3 v = _transformIncre.pos;
    rotateZXY(v, _transformTobeMapped.rot_z, _transformTobeMapped.rot_x, _transformTobeMapped.rot_y);
    _transformTobeMapped.pos = _transformAftMapped.pos - v;
}

void LaserMapping::transformUpdate()
{
    //_transformTobeMapped.rot_x = 0.998 * _transformTobeMapped.rot_x.rad() + 0.002 * imuCur.pitch.rad();
    //_transformTobeMapped.rot_z = 0.998 * _transformTobeMapped.rot_z.rad() + 0.002 * imuCur.roll.rad();

    _transformBefMapped = _transformSum;
    _transformAftMapped = _transformTobeMapped;
}



void LaserMapping::pointAssociateToMap(const pcl::PointXYZI& pi, pcl::PointXYZI& po)
{
    po.x = pi.x;
    po.y = pi.y;
    po.z = pi.z;
    po.intensity = pi.intensity;

    rotateZXY(po, _transformTobeMapped.rot_z, _transformTobeMapped.rot_x, _transformTobeMapped.rot_y);

    po.x += _transformTobeMapped.pos.x();
    po.y += _transformTobeMapped.pos.y();
    po.z += _transformTobeMapped.pos.z();
}



void LaserMapping::pointAssociateTobeMapped(const pcl::PointXYZI& pi, pcl::PointXYZI& po)
{
    po.x = pi.x - _transformTobeMapped.pos.x();
    po.y = pi.y - _transformTobeMapped.pos.y();
    po.z = pi.z - _transformTobeMapped.pos.z();
    po.intensity = pi.intensity;

    rotateYXZ(po, -_transformTobeMapped.rot_y, -_transformTobeMapped.rot_x, -_transformTobeMapped.rot_z);
}
size_t LaserMapping::toIndex(int i, int j, int k)
{
    return i + _laserCloudWidth * j + _laserCloudWidth * _laserCloudHeight * k;
}

void LaserMapping::closeThread()
{
    getPointCloudContinue=false;
}
