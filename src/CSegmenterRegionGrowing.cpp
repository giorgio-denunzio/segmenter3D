#include "CSegmenterRegionGrowing.h"

// http://pointclouds.org/documentation/tutorials/region_growing_segmentation.php

CSegmenterRegionGrowing::CSegmenterRegionGrowing(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud,
                                                 int neKSearch,
                                                 int rgMinClusterSize,
                                                 int rgMaxClusterSize,
                                                 int rgNumberOfNeighbours,
                                                 float rgSmoothnessThreshold,
                                                 float rgCurvatureThreshold)
{

	pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloud (new pcl::PointCloud<pcl::PointXYZ>);
	// https://answers.ros.org/question/9515/how-to-convert-between-different-point-cloud-types-using-pcl/
	copyPointCloud(*inputCloud, *xyzCloud);

    pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod (tree);
    normal_estimator.setInputCloud (xyzCloud);
    normal_estimator.setKSearch (neKSearch);    // 50    // 50
    normal_estimator.compute (*normals);

    reg.setMinClusterSize (rgMinClusterSize);   // 50    // 2000
    reg.setMaxClusterSize (rgMaxClusterSize);   // 1000000    // 1000000
    reg.setSearchMethod (tree);
    reg.setNumberOfNeighbours (rgNumberOfNeighbours);   // 30    // 30
    reg.setInputCloud (xyzCloud);
    reg.setInputNormals (normals);
    reg.setSmoothnessThreshold (rgSmoothnessThreshold / 180.0 * M_PI);  // 3.0    // 60
    reg.setCurvatureThreshold (rgCurvatureThreshold);   // 1.0    // 0.5

    reg.extract (clusters);

    std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
    std::cout << "First cluster has " << clusters[0].indices.size () << " points." << endl;



    /*
    std::cout << "These are the indices of the points of the initial" <<
      std::endl << "cloud that belong to the first cluster:" << std::endl;
    int counter = 0;
    while (counter < clusters[0].indices.size ())
    {
      std::cout << clusters[0].indices[counter] << ", ";
      counter++;
      if (counter % 10 == 0)
        std::cout << std::endl;
    }
    std::cout << std::endl;
    */

    // From http://docs.pointclouds.org/1.7.0/classpcl_1_1_region_growing.html#aeb1ae724085424bc60bb4cf27ff27283
    // "If the cloud was successfully segmented, then function returns colored cloud. Otherwise it returns an empty pointer.   <----- !!!!
    // Points that belong to the same segment have the same color. But this function doesn't guarantee that different segments    <----- !!!!
    // will have different color(it all depends on RNG). Points that were not listed in the indices array will have red color"
    // So I need to use indices, not colors!!!!!
    // From the index of the clicked point I can find which is the cluster, so I get the indices of all the points of that cluster, so
    // I can drop all those points.
    // For indices, see
    // http://www.pointclouds.org/documentation/tutorials/extract_indices.php#extract-indices
    // http://www.pointclouds.org/documentation/tutorials/planar_segmentation.php#planar-segmentation


    // Now put dark red into out-of-cluster points
    // http://docs.pointclouds.org/1.7.0/structpcl_1_1_point_x_y_z_r_g_b.html
    // IT DID NOT WORK!?


    colored_cloud = reg.getColoredCloud ();

	// Build up a new cluster containing all the red points, and append it to the clusters variable member
	pcl::PointIndices::Ptr piPtr(new pcl::PointIndices);   // da http://www.pcl-users.org/IndicesPtr-from-PointIndices-td4020356.html
	for (int j = 0; j < colored_cloud->points.size(); j++)
	{
		pcl::PointXYZRGB *p = &colored_cloud->points[j];
		if (p->r == 255 && p->g == 0 && p->b == 0)
			piPtr->indices.push_back(j);
	}
	clusters.push_back(*piPtr);

	for (int c = 0; c < clusters.size(); c++)
	{
		pcl::RGB aColor = pcl::GlasbeyLUT::at(c % pcl::GlasbeyLUT::size());
		for (int k = 0; k < clusters[c].indices.size(); k++)  // for each point in the cluster, color with the original color
		{
			pcl::PointXYZRGB *p = &colored_cloud->points[clusters[c].indices[k]];
			p->r = aColor.r;
			p->g = aColor.g;
			p->b = aColor.b;
		}
	}

}



pcl::PointCloud<pcl::PointXYZRGB>::Ptr CSegmenterRegionGrowing::getOutputCloud(void)
{
    return (colored_cloud);
}

std::vector <pcl::PointIndices> CSegmenterRegionGrowing::getOutputClusters(void)
{
    return (clusters);
}


pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> CSegmenterRegionGrowing::getOutputRegionGrowing(void)
{
    return (reg);
}




/*
uint8_t r, g, b;
r= 255; g = 0; b = 0;    // Red color
uint32_t rgbRed = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
// p.rgb = *reinterpret_cast<float*>(&rgb);
r = 100; g = 0; b = 0;    // Dark red color
uint32_t rgbDarkRed = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
*/


/*
//    for (int j = 0; j < colored_cloud->points.size(); j++)
//        if (colored_cloud->points[j].rgb == *reinterpret_cast<float*>(&rgbRed))
//            colored_cloud->points[j].rgb = *reinterpret_cast<float*>(&rgbDarkRed);
for (int j = 0; j < colored_cloud->points.size(); j++)
{
pcl::PointXYZRGB *p = &colored_cloud->points[j];
if (p->r == 255 && p->g == 0 && p->b == 0)
p->r = 100;
}

*/

