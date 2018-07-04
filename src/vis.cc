#include "vis.h"
namespace idg {
namespace perception {
namespace lidar {

bool Visualizer::Init() {
    pcl_vis_.reset(new PCLVisualizer);
    if (pcl_vis_ == nullptr) {
        cout << "Failed to get instance";
        return false;
    }
    return true;
}

bool Visualizer::Setup() {
    return true;
}

bool Visualizer::Run() {
    while (!pcl_vis_->wasStopped()) {
        pcl_vis_->spinOnce(100);  //显示
    }
    return true;
}

bool Visualizer::Vis(VisOptions& opts) {
    pcl_vis_->removeAllPointClouds();
    pcl_vis_->removeText3D("file_name");
    pcl::visualization::PointCloudColorHandlerCustom<Point> pc_sig(opts.pc, 0, 255, 0);
    pcl_vis_->addPointCloud(opts.pc, pc_sig, "pc");
    pcl_vis_->addText(opts.file_name, 10, 10, 12, 1.0, 0, 0, "file_name");
    return true;
}

void Visualizer::SetKeyCallback(boost::function< void(const pcl::visualization::KeyboardEvent &)>cb) {
    pcl_vis_->registerKeyboardCallback(cb);
}

}
}
}