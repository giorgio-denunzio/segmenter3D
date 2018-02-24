#pragma once


// F:\SfM\SOFTWARE\POINT_CLOUD_LIBRARY__CMAKE__VISUALSTUDIO_OPENCV\pcl-pcl-1.8.1\examples\segmentation\example_lccp_segmentation.cpp



// The next #define avoids deprecation of fopen in favour of fopen_s, and so on (fscanf etc)
#define _CRT_SECURE_NO_DEPRECATE


// Stdlib
#include <stdlib.h>
#include <cmath>
#include <limits.h>
#include <map>
#include <vector> 

#include <boost/format.hpp>


// PCL input/output
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>

//PCL other
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/supervoxel_clustering.h>

// The segmentation class this example is for
#include <pcl/segmentation/lccp_segmentation.h>

// VTK
#include <vtkImageReader2Factory.h>
#include <vtkImageReader2.h>
#include <vtkImageData.h>
#include <vtkImageFlip.h>
#include <vtkPolyLine.h>

/// *****  Type Definitions ***** ///

//  typedef pcl::PointXYZRGBA PointT;  // The point type used for input   ORIGINAL!!!!
typedef pcl::PointXYZRGB PointT;  // The point type used for input
typedef pcl::PointCloud<PointT> PointCloudT;

typedef pcl::LCCPSegmentation<PointT>::SupervoxelAdjacencyList SuperVoxelAdjacencyList;


class CSegmenterLCCP
{
public:
	CSegmenterLCCP(pcl::PointCloud<PointT>::Ptr inputCloud,
		float voxel_resolution,
		float seed_resolution,
		float color_importance,
		float spatial_importance,
		float normal_importance,
		bool use_single_cam_transform,
		bool use_supervoxel_refinement,
		float concavity_tolerance_threshold,
		float smoothness_threshold,
		uint32_t min_segment_size,
		bool use_extended_convexity,
		bool use_sanity_criterion
	);
	
	~CSegmenterLCCP();

	PointCloudT::Ptr                                 getOutputCloud(void);
	std::vector <pcl::PointIndices>                  getOutputClusters(void);

private:
	PointCloudT::Ptr colored_cloud;
	std::vector <pcl::PointIndices> clusters;  // similarly to CRegionGrowing

};



