#ifndef LOAM_LASERODOMETRY_H
#define LOAM_LASERODOMETRY_H
#include"scanRegistration.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include "Twist.h"
#include "nanoflann_pcl.h"
#include <pcl/filters/filter.h>
#include <Eigen/Eigenvalues>
#include <Eigen/QR>
#include<thread>
#include <stack>
#include <mutex>
using namespace std;

struct OdometryCloudTotal{
    pcl::PointCloud<pcl::PointXYZI> _lastCornerCloud;
    pcl::PointCloud<pcl::PointXYZI> _lastSurfaceCloud;
    pcl::PointCloud<pcl::PointXYZI> _laserCloud;
    nav_msgs::Odometry _laserOdometryMsg;
};
class LaserOdometry
{
public:

    LaserOdometry(ScanRegistration*scanRegsitrationObj);
    ~LaserOdometry();
    bool getPointCloudAndInitOdom(nav_msgs::Odometry&_laserOdometryMsg,
                       pcl::PointCloud<pcl::PointXYZI>& _lastCornerCloud,
                       pcl::PointCloud<pcl::PointXYZI>& _lastSurfaceCloud,
                       pcl::PointCloud<pcl::PointXYZI>& _laserCloud);
    void closeThread();



protected:
    /** \brief Transform the given point to the start of the sweep.
     *
     * @param pi the point to transform
     * @param po the point instance for storing the result
     */
    void transformToStart(const pcl::PointXYZI& pi,
                          pcl::PointXYZI& po);

    /** \brief Transform the given point cloud to the end of the sweep.
     *
     * @param cloud the point cloud to transform
     */
    size_t transformToEnd(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);



    void accumulateRotation(Angle cx, Angle cy, Angle cz,
                            Angle lx, Angle ly, Angle lz,
                            Angle &ox, Angle &oy, Angle &oz);

    void process();


private:





    ros::NodeHandle node;
    ScanRegistration*scanRegsitrationObject;
    void getPointCloudFromScanRegisThd();
    bool getPointCloudContinue{false};


    float _scanPeriod{0.1};       ///< time per scan
    uint16_t _ioRatio{2};       ///< ratio of input to output frames
    bool _systemInited{false};      ///< initialization flag
    long _frameCount{0};        ///< number of processed frames

    size_t _maxIterations{25};   ///< maximum number of iterations
    float _deltaTAbort{0.1};     ///< optimization abort threshold for deltaT
    float _deltaRAbort{0.1};     ///< optimization abort threshold for deltaR



    std::vector<int> _pointSearchCornerInd1;    ///< first corner point search index buffer
    std::vector<int> _pointSearchCornerInd2;    ///< second corner point search index buffer

    std::vector<int> _pointSearchSurfInd1;    ///< first surface point search index buffer
    std::vector<int> _pointSearchSurfInd2;    ///< second surface point search index buffer
    std::vector<int> _pointSearchSurfInd3;    ///< third surface point search index buffer

    Twist _transform;     ///< optimized pose transformation
    Twist _transformSum;  ///< accumulated optimized pose transformation
    pcl::PointCloud<pcl::PointXYZI>::Ptr _cornerPointsSharp;      ///< sharp corner points cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr _cornerPointsLessSharp;  ///< less sharp corner points cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr _surfPointsFlat;         ///< flat surface points cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr _surfPointsLessFlat;     ///< less flat surface points cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloud;             ///< full resolution cloud

    pcl::PointCloud<pcl::PointXYZI>::Ptr _lastCornerCloud;    ///< last corner points cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr _lastSurfaceCloud;   ///< last surface points cloud

    pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudOri;      ///< point selection
    pcl::PointCloud<pcl::PointXYZI>::Ptr _coeffSel;           ///< point selection coefficients

    nanoflann::KdTreeFLANN<pcl::PointXYZI> _lastCornerKDTree;   ///< last corner cloud KD-tree
    nanoflann::KdTreeFLANN<pcl::PointXYZI> _lastSurfaceKDTree;  ///< last surface cloud KD-tree

    std::stack<OdometryCloudTotal>stackPointCloudTotal;
    std::mutex point_cloud_lock;

    ros::Publisher _pubLaserCloudCornerLast;  ///< last corner cloud message publisher
    ros::Publisher _pubLaserCloudSurfLast;    ///< last surface cloud message publisher
    ros::Publisher _pubLaserCloudFullRes;     ///< full resolution cloud message publisher
    ros::Publisher _pubLaserOdometry;         ///< laser odometry publisher
    tf::TransformBroadcaster _tfBroadcaster;  ///< laser odometry transform broadcaster

    long long preTimeStamp{0};

};






#endif

