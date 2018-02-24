#ifndef CSEGMENTERCOLUMNS_H
#define CSEGMENTERCOLUMNS_H

// The next #define avoids deprecation of fopen in favour of fopen_s, and so on (fscanf etc)
#define _CRT_SECURE_NO_DEPRECATE


#include <iostream>
#include <fstream>   // file saving and loading
#include <ctime>    // to measure execution time

// I want to cout with a given number of leading zeros
// http://stackoverflow.com/questions/1714515/how-can-i-pad-an-int-with-leading-zeros-when-using-cout-operator
#include <iomanip>
// ... but it did not work, commented out

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>

#include <pcl/ros/conversions.h>

#include <pcl/filters/project_inliers.h>
#include <pcl/common/transforms.h>

#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>

#include <pcl/filters/uniform_sampling.h>

#include <pcl/visualization/cloud_viewer.h>

// normal estimation...
#include <pcl/features/normal_3d.h>

// Crop box
#include <pcl/filters/impl/crop_box.hpp>

// Colors http://docs.pointclouds.org/1.8.1/classpcl_1_1_glasbey_l_u_t.html
#include <pcl/common/colors.h>

// Opencv  AT PRESENT IT IS USELESS! See code
//#include <opencv2/core/core.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/highgui/highgui.hpp>


// pgm input/output
# include "pgmb_io.h"


typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

struct plane {
    float a;
    float b;
    float c;
    float d;
};







class CSegmenterColumns
{
public:
	CSegmenterColumns::CSegmenterColumns(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud);

	PointCloudT::Ptr                                 getOutputCloud(void);
	std::vector <pcl::PointIndices>                  getOutputClusters(void);

protected:


//    void showHelp(char * program_name);
//    bool loadCloudAndConvertToPointCloud2(int argc, char **argv, pcl::PCLPointCloud2::Ptr cloud_blob);
	bool downsamplePointCloud2(pcl::PCLPointCloud2::Ptr cloud_blob, pcl::PointCloud<PointT>::Ptr cloud_filtered, float leafSize);
	//bool downsamplePointCloud(pcl::PointCloud<PointT>::Ptr cloud_blob, pcl::PointCloud<PointT>::Ptr cloud_filtered, float leafSize);
	bool findPlanes(pcl::PointCloud<PointT>::Ptr cloud_filtered, pcl::PointCloud<PointT>::Ptr cloud_filtered_backup, std::vector<plane> &planes);
    unsigned int findIndexOfGroundPlane(std::vector<plane> planes);
    bool projectCloudToGroundPlane(pcl::PointCloud<PointT>::Ptr cloud_filtered, std::vector<plane> planes, unsigned int idx, pcl::PointCloud<PointT>::Ptr cloud_rotated, pcl::PointCloud<PointT>::Ptr cloud_projected);
    bool buildOctreeAndDoBoxSearchingToCalculatePointDensity(pcl::PointCloud<PointT>::Ptr cloud_projected, PointT minPt, PointT maxPt, float binSide);
    bool applyHoughToPointDensityOfProjectedCloud(unsigned int &nc);
    bool saveColumnBoundingBoxCloudsAndVisualize(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, pcl::PointCloud<PointT>::Ptr cloud_rotated, pcl::PointCloud<PointT>::Ptr cloud_projected, PointT minPt, PointT maxPt, float binSide);
	bool segmentCylinders(unsigned int nc, float binSide);


private:
	PointCloudT::Ptr colored_cloud;
	std::vector <pcl::PointIndices> clusters;  // similarly to CRegionGrowing


};





#endif // CSEGMENTERCOLUMNS_H
