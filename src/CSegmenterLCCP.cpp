#include "CSegmenterLCCP.h"



CSegmenterLCCP::CSegmenterLCCP(pcl::PointCloud<PointT>::Ptr inputCloud,
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
)
{






	/// -----------------------------------|  Preparations  |-----------------------------------

//	bool sv_output_specified = pcl::console::find_switch(argc, argv, "-so");
//	bool show_visualization = (!pcl::console::find_switch(argc, argv, "-novis"));
	bool ignore_provided_normals = false;
	bool add_label_field = false;
	
	/// Create variables needed for preparations
	std::string outputname("");
	pcl::PointCloud<PointT>::Ptr input_cloud_ptr(new pcl::PointCloud<PointT>);
	pcl::PointCloud<pcl::Normal>::Ptr input_normals_ptr(new pcl::PointCloud<pcl::Normal>);
	bool has_normals = false;




	/// Callback and variables


	float normals_scale;

	/*
	///  Default values of parameters before parsing
	// Supervoxel Stuff
	float voxel_resolution = 0.0075f;
	float seed_resolution = 0.03f;
	float color_importance = 0.0f;
	float spatial_importance = 1.0f;
	float normal_importance = 4.0f;
	bool use_single_cam_transform = false;
	bool use_supervoxel_refinement = false;

	// LCCPSegmentation Stuff
	float concavity_tolerance_threshold = 10;
	float smoothness_threshold = 0.1;
	uint32_t min_segment_size = 0;
	bool use_extended_convexity = false;
	bool use_sanity_criterion = false;
	*/
	//Supervoxel Stuff

	normals_scale = seed_resolution / 2.0;

	// Segmentation Stuff
	unsigned int k_factor = 0;
	if (use_extended_convexity)
		k_factor = 1;

	/// Preparation of Input: Supervoxel Oversegmentation

	pcl::SupervoxelClustering<PointT> super(voxel_resolution, seed_resolution);
	super.setUseSingleCameraTransform(use_single_cam_transform);
	super.setInputCloud(inputCloud);
	if (has_normals)
		super.setNormalCloud(input_normals_ptr);
	super.setColorImportance(color_importance);
	super.setSpatialImportance(spatial_importance);
	super.setNormalImportance(normal_importance);
	std::map<uint32_t, pcl::Supervoxel<PointT>::Ptr> supervoxel_clusters;

	PCL_INFO("Extracting supervoxels\n");
	super.extract(supervoxel_clusters);

	if (use_supervoxel_refinement)
	{
		PCL_INFO("Refining supervoxels\n");
		super.refineSupervoxels(2, supervoxel_clusters);
	}
	std::stringstream temp;
	temp << "  Nr. Supervoxels: " << supervoxel_clusters.size() << "\n";
	PCL_INFO(temp.str().c_str());

	PCL_INFO("Getting supervoxel adjacency\n");
	std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
	super.getSupervoxelAdjacency(supervoxel_adjacency);

	/// Get the cloud of supervoxel centroid with normals and the colored cloud with supervoxel coloring (this is used for visualization)
	pcl::PointCloud<pcl::PointNormal>::Ptr sv_centroid_normal_cloud = pcl::SupervoxelClustering<PointT>::makeSupervoxelNormalCloud(supervoxel_clusters);

	/// The Main Step: Perform LCCPSegmentation

	PCL_INFO("Starting Segmentation\n");
	pcl::LCCPSegmentation<PointT> lccp;
	lccp.setConcavityToleranceThreshold(concavity_tolerance_threshold);
	lccp.setSanityCheck(use_sanity_criterion);
	lccp.setSmoothnessCheck(true, voxel_resolution, seed_resolution, smoothness_threshold);
	lccp.setKFactor(k_factor);
	lccp.setInputSupervoxels(supervoxel_clusters, supervoxel_adjacency);
	lccp.setMinSegmentSize(min_segment_size);
	lccp.segment();

	PCL_INFO("Interpolation voxel cloud -> input cloud and relabeling\n");
	pcl::PointCloud<pcl::PointXYZL>::Ptr sv_labeled_cloud = super.getLabeledCloud();
	pcl::PointCloud<pcl::PointXYZL>::Ptr lccp_labeled_cloud = sv_labeled_cloud->makeShared();
	lccp.relabelCloud(*lccp_labeled_cloud);
	SuperVoxelAdjacencyList sv_adjacency_list;
	lccp.getSVAdjacencyList(sv_adjacency_list);  // Needed for visualization

	/// Creating Colored Cloud and clusters
	if (lccp_labeled_cloud->size() == inputCloud->size())
	{
		// OK
		// Transfer the label to the RGB color of colored_cloud, and build a data structure to build on the fly and hold the clusters  
		colored_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);   // initialize the colored cloud
		pcl::copyPointCloud(*inputCloud, *colored_cloud);    // fill it
		
		// The next will be used to build the clusters (associative memory, using the label as the key, and a vector of indices as the value!)
		typedef std::map<uint32_t, std::vector<int>> LabelVectorIntMap;
		LabelVectorIntMap coll;
		
		for (int j = 0; j < colored_cloud->points.size(); j++)
		{
			uint32_t l = lccp_labeled_cloud->points[j].label;
			pcl::RGB aColor = pcl::GlasbeyLUT::at(l % pcl::GlasbeyLUT::size());
			pcl::PointXYZRGB *p = &colored_cloud->points[j];
			colored_cloud->points[j].r = aColor.r;
			colored_cloud->points[j].g = aColor.g;
			colored_cloud->points[j].b = aColor.b;

			// Now append to the vector containing the indices for the current label l, used as map key, the value j og the current index
			// https://stackoverflow.com/questions/1939953/how-to-find-if-a-given-key-exists-in-a-c-stdmap
			// "Unlike map::operator[], map::at will not create a new key in the map if the specified key does not exist."
			// So the following instruction appends the index to the l-th vector if this vector exists, ora creates it before if it doesn't!!! GREAT!
			coll[l].push_back(j);
		}

		// Now transfer to "clusters"

		clusters.clear();

		for (LabelVectorIntMap::iterator it = coll.begin(); it != coll.end(); ++it) {   // https://stackoverflow.com/questions/110157/how-to-retrieve-all-keys-or-values-from-a-stdmap-and-put-them-into-a-vector
			boost::shared_ptr<std::vector<int> > indicesptr(new std::vector<int>(it->second));  // take each vector, comnvert it to an Indices Ptr
		    pcl::PointIndices::Ptr piPtr(new pcl::PointIndices);   // da http://www.pcl-users.org/IndicesPtr-from-PointIndices-td4020356.html
		    piPtr->indices = *indicesptr;
		    clusters.push_back(*piPtr);
		}


	}
	else
	{
		PCL_ERROR("ERROR:: Sizes of input cloud and labeled supervoxel cloud do not match. No output is produced.\n");
		clusters.clear();
		colored_cloud->clear();
	}







}


CSegmenterLCCP::~CSegmenterLCCP()
{
}



PointCloudT::Ptr                CSegmenterLCCP::getOutputCloud(void)
{
	return (colored_cloud);

}

std::vector <pcl::PointIndices> CSegmenterLCCP::getOutputClusters(void)
{
	return (clusters);
}
