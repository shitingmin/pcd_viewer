#include "pcl/visualization/pcl_visualizer.h"
#include "pcl_base.h"
namespace idg {
namespace perception {
namespace lidar {
using pcl::visualization::PCLVisualizer;
using pcl::visualization::KeyboardEvent;

using std::cout;
using std::endl;

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
    void PointXYZIT2PointXYZ(PointCloudPtr& src_pc, pcl::PointCloud<pcl::PointXYZ>::Ptr& dst_pc);
    void SetKeyCallback(boost::function< void(const pcl::visualization::KeyboardEvent &)>cb);

private:
    std::shared_ptr<PCLVisualizer> pcl_vis_;
};

}
}
}