#ifndef LOAM_SCANREGISTRATION_H
#define LOAM_SCANREGISTRATION_H
#include <vector>
#include <ros/node_handle.h>
#include <pcl/point_cloud.h>
#include "math_utils.h"
#include "common.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/PointCloud2.h>
#include <queue>
#include <mutex>
using namespace std;


/** \brief A pair describing the start end end index of a range. */
typedef std::pair<size_t, size_t> IndexRange;


/** Point label options. */
enum PointLabel {
    CORNER_SHARP = 2,       ///< sharp corner point
    CORNER_LESS_SHARP = 1,  ///< less sharp corner point
    SURFACE_LESS_FLAT = 0,  ///< less flat surface point
    SURFACE_FLAT = -1       ///< flat surface point
};


/** \brief Class realizing a linear mapping from vertical point angle to the corresponding scan ring.
 *
 */
class MultiScanMapper {
public:
    /** \brief Construct a new multi scan mapper instance.
   *
   * @param lowerBound - the lower vertical bound (degrees)
   * @param upperBound - the upper vertical bound (degrees)
   * @param nScanRings - the number of scan rings
   */
    MultiScanMapper(const float& lowerBound = -15,
                    const float& upperBound = 15,
                    const uint16_t& nScanRings = 16);

    const float& getLowerBound() { return _lowerBound; }
    const float& getUpperBound() { return _upperBound; }
    const uint16_t& getNumberOfScanRings() { return _nScanRings; }

    /** \brief Set mapping parameters.
   *
   * @param lowerBound - the lower vertical bound (degrees)
   * @param upperBound - the upper vertical bound (degrees)
   * @param nScanRings - the number of scan rings
   */
    void set(const float& lowerBound,
             const float& upperBound,
             const uint16_t& nScanRings);

    /** \brief Map the specified vertical point angle to its ring ID.
   *
   * @param angle the vertical point angle (in rad)
   * @return the ring ID
   */
    inline int getRingForAngle(const float& angle);

    /** Multi scan mapper for Velodyne VLP-16 according to data sheet. */
    static inline MultiScanMapper Velodyne_VLP_16() { return MultiScanMapper(-15, 15, 16); };

    /** Multi scan mapper for Velodyne HDL-32 according to data sheet. */
    static inline MultiScanMapper Velodyne_HDL_32() { return MultiScanMapper(-30.67f, 10.67f, 32); };

    /** Multi scan mapper for Velodyne HDL-64E according to data sheet. */
    static inline MultiScanMapper Velodyne_HDL_64E() { return MultiScanMapper(-24.9f, 2, 64); };


private:
    float _lowerBound;      ///< the vertical angle of the first scan ring
    float _upperBound;      ///< the vertical angle of the last scan ring
    uint16_t _nScanRings;   ///< number of scan rings
    float _factor;          ///< linear interpolation factor
};


struct ScanRegisCloudTotal{
    pcl::PointCloud<pcl::PointXYZI> _cornerPointsSharp;      ///< sharp corner points cloud
    pcl::PointCloud<pcl::PointXYZI> _cornerPointsLessSharp;  ///< less sharp corner points cloud
    pcl::PointCloud<pcl::PointXYZI> _surfacePointsFlat;      ///< flat surface points cloud
    pcl::PointCloud<pcl::PointXYZI> _surfacePointsLessFlat;  ///< less flat surface points cloud
    pcl::PointCloud<pcl::PointXYZI> _laserCloud;
};

class ScanRegistration
{
public:

    ScanRegistration();
    ~ScanRegistration();


    void handleCloudMessage(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg);
    void process(const pcl::PointCloud<pcl::PointXYZ>& laserCloudIn,
                 const ros::Time& scanTime);

    //interface for getting processed data
    bool getPointCloud(ros::Time&ros_time,pcl::PointCloud<pcl::PointXYZI>& cornerPointsSharp,pcl::PointCloud<pcl::PointXYZI>& cornerPointsLessSharp,
                       pcl::PointCloud<pcl::PointXYZI>& surfacePointsFlat,pcl::PointCloud<pcl::PointXYZI>& surfacePointsLessFlat,
                       pcl::PointCloud<pcl::PointXYZI>& laserCloud);

private:

    ros::NodeHandle node;

    /** The time per scan. */
    float scanPeriod{0.1};

    /** The size of the IMU history state buffer. */
    int imuHistorySize{200};

    /** The number of (equally sized) regions used to distribute the feature extraction within a scan. */
    int nFeatureRegions{6};

    /** The number of surrounding points (+/- region around a point) used to calculate a point curvature. */
    int curvatureRegion{5};

    /** The maximum number of sharp corner points per feature region. */
    int maxCornerSharp{2};

    /** The maximum number of less sharp corner points per feature region. */
    int maxCornerLessSharp{20};

    /** The maximum number of flat surface points per feature region. */
    int maxSurfaceFlat{4};

    /** The voxel size used for down sizing the remaining less flat surface points. */
    float lessFlatFilterSize{0.2};

    /** The curvature threshold below / above a point is considered a flat / corner point. */
    float surfaceCurvatureThreshold{0.1};


    MultiScanMapper _scanMapper;  ///< mapper for mapping vertical point angles to scan ring IDs

    ros::Subscriber _subLaserCloud;   ///< input cloud message subscriber
    ros::Time _sweepStart;                  ///< time stamp of beginning of current sweep
    ros::Time _scanTime;                    ///< time stamp of most recent scan
    std::vector<IndexRange> _scanIndices;
    pcl::PointCloud<pcl::PointXYZI> _cornerPointsSharp;      ///< sharp corner points cloud
    pcl::PointCloud<pcl::PointXYZI> _cornerPointsLessSharp;  ///< less sharp corner points cloud
    pcl::PointCloud<pcl::PointXYZI> _surfacePointsFlat;      ///< flat surface points cloud
    pcl::PointCloud<pcl::PointXYZI> _surfacePointsLessFlat;  ///< less flat surface points cloud
    pcl::PointCloud<pcl::PointXYZI> _laserCloud;             ///full points cloud

    
    std::queue<ScanRegisCloudTotal>queuePointCloudTotal;     ///
    std::mutex point_cloud_lock;

    std::vector<float> _regionCurvature;      ///< point curvature buffer
    std::vector<PointLabel> _regionLabel;     ///< point label buffer
    std::vector<size_t> _regionSortIndices;   ///< sorted region indices based on point curvature
    std::vector<int> _scanNeighborPicked;     ///< flag if neighboring point was already picked


    void extractFeatures(const uint16_t& beginIdx = 0);
    void setScanBuffersFor(const size_t& startIdx, const size_t& endIdx);
    void markAsPicked(const size_t& cloudIdx,const size_t& scanIdx);
    void setRegionBuffersFor(const size_t& startIdx,const size_t& endIdx);
    void reset(const ros::Time& scanTime,const bool& newSweep = true);

    int pointCloudCount{0};
   /* ros::Publisher _pubLaserCloud;              ///< full resolution cloud message publisher
    ros::Publisher _pubCornerPointsSharp;       ///< sharp corner cloud message publisher
    ros::Publisher _pubCornerPointsLessSharp;   ///< less sharp corner cloud message publisher
    ros::Publisher _pubSurfPointsFlat;          ///< flat surface cloud message publisher
    ros::Publisher _pubSurfPointsLessFlat;      ///< less flat surface cloud message publisher*/

};






#endif

