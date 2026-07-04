#pragma once

#include <ctime>
#include <cassert>
#include <cmath>
#include <utility>
#include <vector>
#include <algorithm> 
#include <cstdlib>
#include <memory>
#include <iostream>
#include <fstream>

#include <Eigen/Dense>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include "nanoflann.hpp"
#include "KDTreeVectorOfVectorsAdaptor.h"

#include "tictoc.h"

using namespace Eigen;
using namespace nanoflann;

using std::cout;
using std::endl;
using std::make_pair;

using std::atan2;
using std::cos;
using std::sin;

using SCPointType = pcl::PointXYZI; // using xyz only. but a user can exchange the original bin encoding function (i.e., max hegiht) to max intensity (for detail, refer 20 ICRA Intensity Scan Context)
using KeyMat = std::vector<std::vector<float> >;
using InvKeyTree = KDTreeVectorOfVectorsAdaptor< KeyMat, float >;


// namespace SC2
// {

void coreImportTest ( void );


// sc param-independent helper functions 
float xy2theta( const float & _x, const float & _y );
MatrixXd circshift( MatrixXd &_mat, int _num_shift );
std::vector<float> eig2stdvec( MatrixXd _eigmat );


class SCManager
{
public: 
   
public: 
    // Constructor with con configurable params
    SCManager(
        int num_rings,
        int num_sectors,
        double max_radius,
        double lidar_height,
        int num_exclude_recent,
        int num_candidates_from_tree,
        double search_ratio,
        double sc_dist_thres,
        int tree_making_period
    );

    Eigen::MatrixXd makeScancontext( pcl::PointCloud<SCPointType> & _scan_down );
    Eigen::MatrixXd makeRingkeyFromScancontext( Eigen::MatrixXd &_desc );
    Eigen::MatrixXd makeSectorkeyFromScancontext( Eigen::MatrixXd &_desc );

    int fastAlignUsingVkey ( MatrixXd & _vkey1, MatrixXd & _vkey2 ); 
    double distDirectSC ( MatrixXd &_sc1, MatrixXd &_sc2 ); // "d" (eq 5) in the original paper (IROS 18)
    std::pair<double, int> distanceBtnScanContext ( MatrixXd &_sc1, MatrixXd &_sc2 ); // "D" (eq 6) in the original paper (IROS 18)

    // User-side API
    void makeAndSaveScancontextAndKeys( pcl::PointCloud<SCPointType> & _scan_down, double timestamp );
    std::pair<int, float> detectLoopClosureID( void ); // int: nearest node index, float: relative yaw 
    std::string generateFilename();
    pcl::PointCloud<SCPointType>::Ptr getPointCloud(int idx);
    double getTimestamp(int idx) const;
    int getNumFrames() const;
    ~SCManager();
    

private:
    // hyper parameters ()
    double LIDAR_HEIGHT; // lidar height : add this for simply directly using lidar scan in the lidar local coord (not robot base coord) / if you use robot-coord-transformed lidar scans, just set this as 0.

    int    PC_NUM_RING; // 20 in the original paper (IROS 18)
    int    PC_NUM_SECTOR; // 60 in the original paper (IROS 18)
    double PC_MAX_RADIUS; // 80 meter max in the original paper (IROS 18)
    double PC_UNIT_SECTORANGLE;
    double PC_UNIT_RINGGAP;

    // tree
    int    NUM_EXCLUDE_RECENT; // simply just keyframe gap, but node position distance-based exclusion is ok. 
    int    NUM_CANDIDATES_FROM_TREE; // 10 is enough. (refer the IROS 18 paper)

    // loop thres
    double SEARCH_RATIO; // for fast comparison, no Brute-force, but search 10 % is okay. // not was in the original conf paper, but improved ver.
    double SC_DIST_THRES; // empirically 0.1-0.2 is fine (rare false-alarms) for 20x60 polar context (but for 0.15 <, DCS or ICP fit score check (e.g., in LeGO-LOAM) should be required for robustness)
    // const double SC_DIST_THRES = 0.5; // 0.4-0.6 is good choice for using with robust kernel (e.g., Cauchy, DCS) + icp fitness threshold / if not, recommend 0.1-0.15

    // config 
    int    TREE_MAKING_PERIOD_; // i.e., remaking tree frequency, to avoid non-mandatory every remaking, to save time cost / if you want to find a very recent revisits use small value of it (it is enough fast ~ 5-50ms wrt N.).
    int    tree_making_period_conter;

    // data 
    std::vector<double> polarcontexts_timestamp_; // optional.
    std::vector<Eigen::MatrixXd> polarcontexts_;
    std::vector<Eigen::MatrixXd> polarcontext_invkeys_;
    std::vector<Eigen::MatrixXd> polarcontext_vkeys_;

    KeyMat polarcontext_invkeys_mat_;
    KeyMat polarcontext_invkeys_to_search_;
    std::unique_ptr<InvKeyTree> polarcontext_tree_;

    std::vector<pcl::PointCloud<SCPointType>::Ptr> point_clouds_;


    std::ofstream file;
    std::string filename;

}; // SCManager

// } // namespace SC2
