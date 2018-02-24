#ifndef CSEGMENTERREGIONGROWING_H
#define CSEGMENTERREGIONGROWING_H

// The next #define avoids deprecation of fopen in favour of fopen_s, and so on (fscanf etc)
#define _CRT_SECURE_NO_DEPRECATE

#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>


// Check this fucntion:
// bool pcl::RegionGrowing< PointT, NormalT >::prepareForSegmentation	(	)
// (but it is protected)
//
// Moreover, here
// http://docs.pointclouds.org/1.7.0/classpcl_1_1_region_growing.html
// There are many other parameters and flags
//
// Also:
// bool pcl::RegionGrowing< PointT, NormalT >::validatePoint	(	int 	initial_seed, int 	point, int 	nghbr, bool & 	is_a_seed )		const
// This function is checking if the point with index 'nghbr' belongs to the segment.
//
// Also:
// pcl::PointCloud< pcl::PointXYZRGBA >::Ptr pcl::RegionGrowing< PointT, NormalT >::getColoredCloudRGBA	(	)
// can be used to pur a light RED for deleted points

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class CSegmenterRegionGrowing
{
public:
	CSegmenterRegionGrowing(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud,
		int neKSearch,
		int rgMinClusterSize,
		int rgMaxClusterSize,
		int rgNumberOfNeighbours,
		float rgSmoothnessThreshold,
		float rgCurvatureThreshold);
	PointCloudT::Ptr                                 getOutputCloud(void);
    std::vector <pcl::PointIndices>                  getOutputClusters(void);
    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal>   getOutputRegionGrowing(void);

private:
	PointCloudT::Ptr colored_cloud;
    std::vector <pcl::PointIndices> clusters;
    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
};

#endif // CSEGMENTERREGIONGROWING_H
