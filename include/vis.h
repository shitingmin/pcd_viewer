#include "pcl/visualization/pcl_visualizer.h"
namespace idg {
namespace perception {
namespace lidar {
using pcl::visualization::PCLVisualizer;
using pcl::visualization::KeyboardEvent;

using std::cout;
using std::endl;

typedef pcl::PointXYZI Point;
typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;
typedef PointCloud::Ptr PointCloudPtr;

struct VisOptions {
    PointCloudPtr pc;
    std::string file_name;
};

class Visualizer {
public:
    Visualizer() = default;
    ~Visualizer() = default;

    bool Init();
    bool Setup();
    bool Run();
    bool Vis(VisOptions& opts);
    void SetKeyCallback(boost::function< void(const pcl::visualization::KeyboardEvent &)>cb);

private:
    std::shared_ptr<PCLVisualizer> pcl_vis_;
};

}
}
}