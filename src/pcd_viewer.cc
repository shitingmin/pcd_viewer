#include "boost/filesystem/operations.hpp"  
#include "boost/filesystem/path.hpp"
#include <boost/function.hpp>  
#include <pcl/io/pcd_io.h>
#include <gflags/gflags.h>
#include "vis.h"
#include "vector"

namespace idg {
namespace perception {
namespace lidar {
using pcl::visualization::KeyboardEvent;

using std::cout;
using std::endl;
DEFINE_string(pcd_dir, "", "pcd directory");

class PCDViewer {
public:
    PCDViewer() = default;
    ~PCDViewer() = default;

    bool Init() {
        vis_.reset(new Visualizer);
        if (vis_ != nullptr && vis_->Init()) {
            cout << "Init Vis succ \n";
        } else {
            return false;
        }

        if (LoadFileList(FLAGS_pcd_dir)) {
            cout << "Load pcd succ \n";
        } else {
            return false;
        }
        current_frame_ = pcd_list_.size();

        pcd_.reset(new PointCloud);
        return true;
    }

    bool LoadPCD(const size_t index, PointCloudPtr& pc) {
        pc->clear();
        if (index >= 0 && index < pcd_list_.size()) {
            std::string file_name = pcd_list_[index];
            pcl::io::loadPCDFile<Point>(file_name, *pc);
            cout << "Load pcd: " << index << " ,"
                 << file_name  << " pt size: " << pc->size() << endl;
            return true;
        }
        return false;
    }

    bool Run() {
        while(true) {
            vis_->Run();
        }
        return true;
    }

    void Analysis(PointCloudPtr& pc) {
        std::vector<int> beg, end;
        double max_time = -FLT_MAX, min_time = FLT_MAX;
        beg.push_back(0);
        for (size_t i = 1; i < pc->size(); ++i) {
            if (pc->points[i].timestamp > max_time) {
                max_time = pc->points[i].timestamp;
            }

            if (pc->points[i].timestamp < min_time) {
                min_time = pc->points[i].timestamp;
            }
            if (pc->points[i-1].timestamp - pc->points[i].timestamp > 0.05) {
                end.push_back(i-1);
                beg.push_back(i);
            }
        }
        end.push_back(pc->size() - 1);
        cout << "break size: " << beg.size() 
             << " min_time: " << std::fixed << min_time
             << " max_time: " << std::fixed << max_time
             << " time diff: " << std::fixed << max_time - min_time << endl;
        for (size_t i = 0 ; i < beg.size(); ++i) {
            cout << i << " size: " << end[i] - beg[i] + 1
                 << " first timestamp: " << std::fixed << pc->points[beg[i]].timestamp
                 << " last timestamp: " << std::fixed << pc->points[end[i]].timestamp << endl;
        }
    }

    bool LoadFileList(std::string& pcd_dir) {
        namespace fs  =  boost::filesystem;
        fs::path full_path(pcd_dir);
        if (fs::exists(full_path)) {
            fs::directory_iterator item_begin(full_path);
            fs::directory_iterator item_end;
            for (;item_begin != item_end; item_begin++) {
                if (fs::is_directory(*item_begin)) {
                    cout << item_begin->path().native() << "\n";
                    continue;
                } else {
                    std::string file_name = item_begin->path().native();
                    if (file_name.substr(file_name.size() - 3, 3) == "pcd") {
                        pcd_list_.push_back(item_begin->path().native());                        
                    }
                }
            }
            sort(pcd_list_.begin(),pcd_list_.end());
            cout << "PCD directory: " << pcd_dir
                 << " , size: " << pcd_list_.size() << endl;
            return true;
        }
        return false;
    }

    void KeyCallback(const KeyboardEvent &event) {
        if (event.getKeySym() == "a" && event.keyDown()) {
            current_frame_  = (current_frame_ + 1) % pcd_list_.size();
        } else if (event.getKeySym() == "d" && event.keyDown()) {
            current_frame_  = (current_frame_ - 1 + pcd_list_.size()) % pcd_list_.size();            
        }
        if (event.keyDown()) {
            std::string key = event.getKeySym();
            if (key == "a" || key == "d") {
                LoadPCD(current_frame_, pcd_);
                VisOptions opts;
                opts.pc = pcd_;
                std::string file_name = pcd_list_[current_frame_];
                size_t beg = file_name.find_last_of('/', file_name.size() - 1);
                size_t end = file_name.find_last_of('.', file_name.size() - 1);
                opts.file_name = file_name.substr(beg + 1 , end - beg -1);
                Analysis(pcd_);
                vis_->Vis(opts);
            }
        }
    }

    void SetKeyCallback(boost::function< void(const pcl::visualization::KeyboardEvent &)>cb) {
        vis_->SetKeyCallback(cb);
    }

private:
    std::shared_ptr<Visualizer> vis_;    
    std::vector<std::string> pcd_list_;
    size_t current_frame_ = 0;
    PointCloudPtr pcd_;
};

}
}
}

int main(int argc, char** argv) {
    google::ParseCommandLineFlags(&argc, &argv, true);
    idg::perception::lidar::PCDViewer pcd_viewer;
    if (pcd_viewer.Init()) {
        boost::function< void(const pcl::visualization::KeyboardEvent &)> cb =
                boost::bind(&idg::perception::lidar::PCDViewer::KeyCallback, &pcd_viewer, _1);
        pcd_viewer.SetKeyCallback(cb);
        pcd_viewer.Run();
    }
    return 0;
}
