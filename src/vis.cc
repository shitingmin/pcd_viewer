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

void Visualizer::PointXYZIT2PointXYZ(PointCloudPtr& src_pc, pcl::PointCloud<pcl::PointXYZ>::Ptr& dst_pc) {
    dst_pc.reset(new pcl::PointCloud<pcl::PointXYZ>);
    dst_pc->resize(src_pc->size());
    size_t n = 0;
    for (auto &p : src_pc->points) {
        pcl::PointXYZ n_pt(p.x, p.y, p.z);
        dst_pc->points[n++] = n_pt;
    }
}

bool Visualizer::Vis(VisOptions& opts) {
    pcl_vis_->removeAllPointClouds();
    pcl_vis_->removeText3D("file_name");
   
    pcl::PointCloud<pcl::PointXYZ>::Ptr new_pc (new pcl::PointCloud<pcl::PointXYZ>);
    PointXYZIT2PointXYZ(opts.pc, new_pc);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> pc_sig(new_pc, 255, 255, 255);
    pcl_vis_->addPointCloud(new_pc, pc_sig, "pc");

    pcl::ModelCoefficients circle_coeff;
    circle_coeff.values.resize (3);    // We need 3 values
    circle_coeff.values[0] = 0;
    circle_coeff.values[1] = 0;
    for (size_t i = 1; i < 4; ++i) {
        circle_coeff.values[2] = i * 20.0;
        std::stringstream ss;
        ss << circle_coeff.values[2] << "circle";
        pcl_vis_->addCircle(circle_coeff, ss.str());
    }

    pcl_vis_->addText(opts.file_name, 10, 10, 12, 1.0, 0, 0, "file_name");
    return true;
}

void Visualizer::SetKeyCallback(boost::function< void(const pcl::visualization::KeyboardEvent &)>cb) {
    pcl_vis_->registerKeyboardCallback(cb);
}

}
}
}
