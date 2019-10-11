#ifndef LOAM_LASERMAPPYING_H
#define LOAM_LASERMAPPYING_H
#include"laserOdometry.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include "Twist.h"
#include "nanoflann_pcl.h"
#include <pcl/filters/filter.h>
#include <Eigen/Eigenvalues>
#include <Eigen/QR>
#include<thread>
#include <sys/time.h>

//class for counting time costs
class calTime
{
public:
    calTime()
    {
    }
    ~calTime()
    {
    }


    string info;
    long long t1,t2,t3,t4;
    void start(string i)
    {
        struct timeval curTime;
        info=i;
        gettimeofday(&curTime, NULL);
        t1=curTime.tv_usec;
        t2=curTime.tv_sec;
    }
    void stop()
    {
        struct timeval curTime;
        gettimeofday(&curTime, NULL);
        t3=curTime.tv_usec;
        t4=curTime.tv_sec;
        cout<<info<<" : "<<(t4-t2)*1000+(t3-t1)/1000<<"  ms,"<<endl;
    }

};


class LaserMapping
{
public:

    LaserMapping(LaserOdometry*laserOdomObj);
    ~LaserMapping();
    void closeThread();


protected:

    void getPointCloudFromOdometryThread();
    void process();
    void optimizeTransformTobeMapped();

    void transformAssociateToMap();
    void transformUpdate();
    void pointAssociateToMap(const pcl::PointXYZI& pi, pcl::PointXYZI& po);
    void pointAssociateTobeMapped(const pcl::PointXYZI& pi, pcl::PointXYZI& po);
    size_t toIndex(int i, int j, int k) ;
    void publishResult();


private:

    LaserOdometry*laserOdomObject;
    bool getPointCloudContinue{false};

    float _scanPeriod;          ///< time per scan
    const int _stackFrameNum;
    const int _mapFrameNum;
    long _frameCount;
    long _mapFrameCount;

    size_t _maxIterations;  ///< maximum number of iterations
    float _deltaTAbort;     ///< optimization abort threshold for deltaT
    float _deltaRAbort;     ///< optimization abort threshold for deltaR

    int _laserCloudCenWidth;
    int _laserCloudCenHeight;
    int _laserCloudCenDepth;
    const size_t _laserCloudWidth;
    const size_t _laserCloudHeight;
    const size_t _laserCloudDepth;
    const size_t _laserCloudNum;

    pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudCornerLast;   ///< last corner points cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudSurfLast;     ///< last surface points cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudFullRes;      ///< last full resolution cloud

    pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudCornerStack;
    pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudSurfStack;
    pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudCornerStackDS;  ///< down sampled
    pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudSurfStackDS;    ///< down sampled

    pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudSurround;
    pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudSurroundDS;     ///< down sampled
    pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudCornerFromMap;
    pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudSurfFromMap;

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> _laserCloudCornerArray;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> _laserCloudSurfArray;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> _laserCloudCornerDSArray;  ///< down sampled
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> _laserCloudSurfDSArray;    ///< down sampled

    std::vector<size_t> _laserCloudValidInd;
    std::vector<size_t> _laserCloudSurroundInd;

    Twist _transformSum;
    Twist _transformIncre;
    Twist _transformTobeMapped;
    Twist _transformBefMapped;
    Twist _transformAftMapped;
    pcl::VoxelGrid<pcl::PointXYZI> _downSizeFilterCorner;   ///< voxel filter for down sizing corner clouds
    pcl::VoxelGrid<pcl::PointXYZI> _downSizeFilterSurf;     ///< voxel filter for down sizing surface clouds
    pcl::VoxelGrid<pcl::PointXYZI> _downSizeFilterMap;      ///< voxel filter for down sizing accumulated map
    pcl::VoxelGrid<pcl::PointXYZI> _downSizeFilterCorner_secondScale;   ///< voxel filter for down sizing corner clouds (if clouds' size big enough)
    pcl::VoxelGrid<pcl::PointXYZI> _downSizeFilterSurf_secondScale;     ///< voxel filter for down sizing surface clouds (if clouds' size big enough)
    pcl::VoxelGrid<pcl::PointXYZI> _downSizeFilterCorner_thirdScale;   ///< voxel filter for down sizing corner clouds (if clouds' size big enough)
    pcl::VoxelGrid<pcl::PointXYZI> _downSizeFilterSurf_thirdScale;     ///< voxel filter for down sizing surface clouds (if clouds' size big enough)

    nav_msgs::Odometry _odomAftMapped;      ///< mapping odometry message
    tf::StampedTransform _aftMappedTrans;   ///< mapping odometry transformation
    tf::StampedTransform segMapTrans;     ///< odometry transformation for segMap (Lin2018/05/02)
    sensor_msgs::PointCloud2 originData;
    ros::Publisher _pubOriginData;
    ros::Publisher _pubBaseLinkData;    ///<  points publisher in "base_link"
    ros::Time _timeLaserOdometry;    ///< time of current laser odometry

    ros::Publisher _pubLaserCloudSurround;    ///< map cloud message publisher
    ros::Publisher _pubLaserCloudFullRes;     ///< current full resolution cloud message publisher
    ros::Publisher _pubOdomAftMapped;         ///< mapping odometry publisher
    tf::TransformBroadcaster _tfBroadcaster_1;  ///< mapping odometry transform broadcaste
    tf::TransformBroadcaster _tfBroadcaster_2;
    calTime clt;
};






#endif

