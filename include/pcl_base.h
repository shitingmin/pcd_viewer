#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

struct PointXYZIT {
    PCL_ADD_POINT4D;
    unsigned char intensity;
    double timestamp;  // height from ground
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointXYZIT,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (unsigned char, intensity, intensity)
    (double, timestamp, timestamp)
)

typedef PointXYZIT Point;
typedef pcl::PointCloud<Point> PointCloud;
typedef pcl::PointCloud<Point>::Ptr PointCloudPtr;