#include "CSegmenterColumns.h"

//CSegmenterColumns::CSegmenterColumns(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud, boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
CSegmenterColumns::CSegmenterColumns(pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloud)
{


    time_t t1, t2;   // for execution time measuring

    /*
    // Show help
    if (pcl::console::find_switch(argc, argv, "-h") || pcl::console::find_switch(argc, argv, "--help")) {
        showHelp(argv[0]);
        return 0;
    }
*/
	
    // Now define some point clouds to be used as code works
	pcl::PCLPointCloud2::Ptr cloud_blob(new pcl::PCLPointCloud2);  // point cloud structure for model loading
    pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>),
                                 cloud_filtered_backup(new pcl::PointCloud<PointT>);


    // Load file into cloud_blob; Works with PCD  PLY OBJ files
    //t1 = time(0);
    // if (!loadCloudAndConvertToPointCloud2(argc, argv, cloud_blob)) return -1;


	// I DID DOWNSCALE, NOW I PREFER NOT TO DO BECAUSE THEN I CANNOT RECOGNIZE THE PIXELS IN THE ORIGINAL TEXTURED MESH!

	goto xxxxx;

	
	// Convert to PointCloud2  because later the downsamplePointCloud2 wants this kind of cloud
	pcl::toPCLPointCloud2(*inputCloud, *cloud_blob);    //    pcl::toROSMsg(*inputCloud, *cloud_blob);    // deprecated

    //t2 = time(0);
    //std::cout << "File loaded and converted to pclPointCloud2, ok! Time was " << t2 - t1 << " seconds" << std::endl;
	///////////////////////std::cout << "File converted to pclPointCloud2, ok!\n";

    // Downsample the dataset using a leaf size of 5 cm
    std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points." << std::endl;
    t1 = time(0);
	downsamplePointCloud2(cloud_blob, cloud_filtered, 0.05f);
/////////////////////////////////////////////////////////////////////////////////////////////////////	downsamplePointCloud(inputCloud, cloud_filtered, 0.05f);
	//if (!downsamplePointCloud2(cloud_blob, cloud_filtered, 0.05f)) return -1;
	t2 = time(0);
    std::cout << "Cloud downsampled! Time was " << t2 - t1 << " seconds" << std::endl;



xxxxx:
	pcl::copyPointCloud(*inputCloud, *cloud_filtered);    // Only because the next lines use cloud_filtered, which should be a downscaled cloud but at present is the original one!!
															// The reason for using the original cloud is that I use indices to mark the points in the columns bounding boxes:
															// so I have to maintain the original cloud and the  cloud used for segmentation, aligned.

	// THE DOWNLOADED CLOUD NOW HAS COLOR INFO, SET IT OTHERWISE IT IS BLACK!

	for (int j = 0; j < cloud_filtered->points.size(); j++)
	{
		cloud_filtered->points[j].r = 255;
		cloud_filtered->points[j].g = 255;
		cloud_filtered->points[j].b = 255;
	}

    // Backup
    pcl::copyPointCloud(*cloud_filtered, *cloud_filtered_backup);  // put the downsampled cloud aside, to use it later (after segmenting out planes)
                                                                   // https://github.com/otherlab/pcl/blob/master/examples/common/example_copy_point_cloud.cpp
    // Segmentation: find planes
    std::vector<plane> planes;
    t1 = time(0);
	findPlanes(cloud_filtered, cloud_filtered_backup, planes);
	//if (!findPlanes(cloud_filtered, cloud_filtered_backup, planes)) return -1;
	t2 = time(0);
    std::cout << "Segmentation accomplished! Time was " << t2 - t1 << " seconds" << std::endl;

    // Now take back the downsampled cloud, because the present "filtered" has been deprived of many planes!
    pcl::copyPointCloud(*cloud_filtered_backup, *cloud_filtered);  // we should now drop the real 4 plames that disturb us a lot (walls and floor) but we'll do it in the future

    // Find the ground plane (at present, just a hard-coded value....)
    unsigned int idx = findIndexOfGroundPlane(planes);

    // Project the (original) downsampled cloud to the ground plane
    // Create the rotated and the projected cloud
    pcl::PointCloud<PointT>::Ptr cloud_projected(new pcl::PointCloud<PointT>),
                                 cloud_rotated(new pcl::PointCloud<PointT>);
    t1 = time(0);
	projectCloudToGroundPlane(cloud_filtered, planes, idx, cloud_rotated, cloud_projected);
	//if (!projectCloudToGroundPlane(cloud_filtered, planes, idx, cloud_rotated, cloud_projected)) return -1;
	t2 = time(0);
    std::cout << "Rotation/projection, ok! Time was " << t2 - t1 << " seconds" << std::endl;

    // The next two points define a bounding box for the cloud; they are used by both the next calculation steps,
    // so we calculate them here and then pass them to the two next functions; idem for the binSide variable,
    // used by both functions
    PointT minPt, maxPt;
    pcl::getMinMax3D(*cloud_projected, minPt, maxPt);
    // How many bins? Decide the bin side (i.e. the resolution)
    float binSide = 0.05;  // MAGIC NUMBER  <------------------------------------------------------ 0.05

    // Build an octree out of the cloud projected to the ground plane, so that the octree is a kind of accumulator
    // for point density calculation, then perform box searching so as to measure the occupancy of each cell
    // and build an image containing the point density of the projected cloud; save that image for (matlab) Hough
    // processing for circle search
    t1 = time(0);
	buildOctreeAndDoBoxSearchingToCalculatePointDensity(cloud_projected, minPt, maxPt, binSide);
	//if (!buildOctreeAndDoBoxSearchingToCalculatePointDensity(cloud_projected, minPt, maxPt, binSide)) return -1;
	t2 = time(0);
    std::cout << "Octree building and box searching, ok! Time was " << t2 - t1 << " seconds" << std::endl;

    // Apply the Hough transform to the point density of the projected cloud, then saves centers.txt and radii.txt;
    // use matlab program
    t1 = time(0);
    unsigned int nc;  // number of columns, will be filled in by applyHoughToPointDensityOfProjectedCloud
	applyHoughToPointDensityOfProjectedCloud(nc);
	//if (!applyHoughToPointDensityOfProjectedCloud(nc)) return -1;
	t2 = time(0);
    std::cout << "Hough transform, ok! Time was " << t2 - t1 << " seconds" << std::endl;

    

// Prepare a viewer for subsequent visualizations
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1(new pcl::visualization::PCLVisualizer("3D Viewer"));


    // Visualize cloud, projected cloud, bounding boxes for cylinders
	//////    visualizeClouds(viewer, cloud_rotated, cloud_projected, minPt, maxPt, binSide);
	saveColumnBoundingBoxCloudsAndVisualize(viewer1, cloud_rotated, cloud_projected, minPt, maxPt, binSide);

	
    // segmentCylinders
//	if (!segmentCylinders(nc, binSide)) return -1;
	segmentCylinders(nc, binSide);


	// Now fill the coloured cloud and the cluster indices

	// Remember they are:

	// PointCloudT::Ptr colored_cloud;
	// std::vector <pcl::PointIndices> clusters;

	// The cluster indices have already been set by the saveColumnBoundingBoxCloudsAndVisualize() function

	

	// Prepare a dark red for the default color (no cluster)

	// https://stackoverflow.com/questions/46650052/initialize-a-pcl-pointcloudptr
	colored_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::copyPointCloud(*inputCloud, *colored_cloud);

	
	for (int j = 0; j < colored_cloud->points.size(); j++) {
		colored_cloud->points[j].r = 100;
		colored_cloud->points[j].g = 0;
		colored_cloud->points[j].b = 0;
	}
	



	///// TEST
//	for (int j = 0; j < 20; j++) {
	//	cout << colored_cloud->points[j].x, colored_cloud->points[j].y << 
		//colored_cloud->points[j].z = 0;
	//}



	// Colorize the points belonging to the clusters

	for (int c = 0; c < clusters.size(); c++)
	{
		// Choose color
		pcl::RGB aColor = pcl::GlasbeyLUT::at(c % pcl::GlasbeyLUT::size());
		for (int j = 0; j < clusters[c].indices.size(); j++) {
			colored_cloud->points[clusters[c].indices[j]].r = aColor.r;
			colored_cloud->points[clusters[c].indices[j]].g = aColor.g;
			colored_cloud->points[clusters[c].indices[j]].b = aColor.b;
		}
	}


	viewer1->close();

	/*
    while (!viewer1->wasStopped())
    {
        viewer1->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }

	*/

}





// http://www.pcl-users.org/Downsampling-a-PointCloud-object-by-half-td4027022.html
// iterating on cloud points


// Access the coordinates of a point:

// cloud_in->points[p_idx].x


/*

// This function displays the help
void CSegmenterColumns::showHelp(char * program_name)
{
    std::cout << std::endl;
    std::cout << "Usage: " << program_name << " cloud_filename.[pcd|ply]" << std::endl;
    std::cout << "-h:  Show this help." << std::endl;
    std::cout << "Examples\n";
    //	std::cout << "";
    std::cout << "project J:\\SFM\\gerrard-hall\\GH_High_Res_MeshLab.ply" << std::endl;
    std::cout << "project J:\\SFM\\gerrard-hall\\GH_UltraHigh_Res_photoscan.obj" << std::endl;
    std::cout << "project D:\\_CORVALLIS_\\gerrard-hall\\GH_Medium_Res_MeshLab.ply" << std::endl;

}

*/


/*


bool CSegmenterColumns::loadCloudAndConvertToPointCloud2(int argc, char **argv, pcl::PCLPointCloud2::Ptr cloud_blob)
{
    bool retVal = true;

    // Define a point cloud, used for file loading
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

    // Fetch point cloud filename in arguments | Works with PCD and PLY files, now also obj
    std::vector<int> filenames;

    filenames = pcl::console::parse_file_extension_argument(argc, argv, ".pcd");
    if (filenames.size() == 1) {
        if (pcl::io::loadPCDFile(argv[filenames[0]], *cloud) < 0)
            retVal = false;
    }

    filenames = pcl::console::parse_file_extension_argument(argc, argv, ".ply");
    if (filenames.size() == 1) {
        if (pcl::io::loadPLYFile(argv[filenames[0]], *cloud) < 0)
            retVal = false;
    }

    filenames = pcl::console::parse_file_extension_argument(argc, argv, ".obj");
    if (filenames.size() == 1) {
        if (pcl::io::loadOBJFile(argv[filenames[0]], *cloud) < 0)
            retVal = false;
    }

    if (cloud->empty())	retVal = false;
    else {
        // these instructions were added to convert the loaded cloud to a PointCloud2 object,  cloud_blob
        pcl::toROSMsg(*cloud, *cloud_blob);
    }

    if (!retVal) {
        std::cerr << "Error loading point cloud " << argv[filenames[0]] << std::endl << std::endl;
        showHelp(argv[0]);
    }
    return retVal;
}


*/




// Create the filtering object, then downsample the dataset using a given leaf size


bool CSegmenterColumns::downsamplePointCloud2(pcl::PCLPointCloud2::Ptr cloud_blob, pcl::PointCloud<PointT>::Ptr cloud_filtered, float leafSize)
{

    pcl::PCLPointCloud2::Ptr cloud_filtered_blob(new pcl::PCLPointCloud2); // point cloud structure for downsampled model

    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloud_blob);
    sor.setLeafSize(leafSize, leafSize, leafSize);     // working: 0.1
                                                       //sor.setLeafSize(0.01f, 0.01f, 0.01f);   // original
    sor.filter(*cloud_filtered_blob);      // downsampled cloud


    // Convert from PCLPointCloud2 to the templated PointCloud
    pcl::fromPCLPointCloud2(*cloud_filtered_blob, *cloud_filtered);

    std::cout << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;

    // Write the downsampled version to disk
    pcl::PCDWriter writer;
    writer.write<PointT>("downsampled.pcd", *cloud_filtered, false);

    return true;
}



/*  PASTICCIO DA RISOLVERE O DIMENTICARE */
/*
bool CSegmenterColumns::downsamplePointCloud(const pcl::PointCloud<PointT>::Ptr cloud_blob, pcl::PointCloud<PointT>::Ptr cloud_filtered, float leafSize)
{

	pcl::PointCloud<PointT>::Ptr cloud_filtered_blob(new pcl::PointCloud<PointT>); // point cloud structure for downsampled model

	pcl::VoxelGrid<pcl::PointCloud<PointT>> sor;
	sor.setInputCloud(cloud_blob);
	sor.setLeafSize(leafSize, leafSize, leafSize);     // working: 0.1
													   //sor.setLeafSize(0.01f, 0.01f, 0.01f);   // original
	sor.filter(*cloud_filtered_blob);      // downsampled cloud


										   // Convert from PCLPointCloud2 to the templated PointCloud
	pcl::fromPCLPointCloud2(*cloud_filtered_blob, *cloud_filtered);

	std::cout << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;

	// Write the downsampled version to disk
	pcl::PCDWriter writer;
	writer.write<PointT>("downsampled.pcd", *cloud_filtered, false);

	return true;
}

*/





bool CSegmenterColumns::findPlanes(pcl::PointCloud<PointT>::Ptr cloud_filtered, pcl::PointCloud<PointT>::Ptr cloud_filtered_backup, std::vector<plane> &planes)
{

    pcl::PointCloud<PointT>::Ptr cloud_p(new pcl::PointCloud<PointT>),
                                 cloud_f(new pcl::PointCloud<PointT>);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);  // ha funzionato per tutti i casi in cui si usava la nuvola downsampled
                                 //seg.setMaxIterations(2000);
    seg.setDistanceThreshold(0.1);    // <----------  original: 0.01  <---- MAGIC NUMBER   ha funzionato per tutti i casi in cui si usava la nuvola downsampled
                                      //seg.setDistanceThreshold(0.05);    // <----------  original: 0.01  <---- MAGIC NUMBER

                                      // Create the filtering object
    pcl::ExtractIndices<PointT> extract;

    plane aPlane;

    int i = 0, nr_points = (int)cloud_filtered->points.size();
    // While 30% of the original cloud is still there
    while (cloud_filtered->points.size() > 0.3 * nr_points && i < 50)  // added a limit of 50 planes
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud(cloud_filtered);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0)
        {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the inliers
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cloud_p);
        std::cout << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

        //  Plane parameters, in ax + by + cz + d = 0 form.
        // Remember that a, b, c are the components of a vector normal to the plane, in particular this is already a unit vector so its magnitude is 1
        aPlane.a = coefficients->values[0];
        aPlane.b = coefficients->values[1];
        aPlane.c = coefficients->values[2];
        aPlane.d = coefficients->values[3];
        std::cout << i << " - Plane coefficients: " << aPlane.a << " " << aPlane.b << " " << aPlane.c << " " << aPlane.d << std::endl;
        planes.push_back(aPlane);

        std::stringstream ss;
        ss << "plane_" << i << ".pcd";
        //	ss << "plane_" << setfill('0') << setw(5) << i << ".pcd";    // <---------------- non funziona!
        pcl::PCDWriter writer;
        writer.write<PointT>(ss.str(), *cloud_p, false);

        // Create the filtering object
        extract.setNegative(true);
        extract.filter(*cloud_f);
        cloud_filtered.swap(cloud_f);
        i++;
    }
    return true;

}



unsigned int CSegmenterColumns::findIndexOfGroundPlane(std::vector<plane> planes)
{
    // Hints to be considered: Take the first 5 planes and find which one has normal vector different from
    // all the others (walls are paired, while the floor is unpaired); this works when we have all the
    // walls and a sufficient floor.

    // Now just state the floor is plane_3 (the fourth found plane) or 0, or 1!!!!

    unsigned int idx;
    // leaf is set at:    sor.setLeafSize = ...
    // idx = 0;   // for the High_Res model,  0.0251162 -0.980479 0.195013 -1.54655  leaf 0.1
    // idx = 1;   // for the High_Res model,  0.0251162 -0.980479 0.195013 -1.54655  leaf 0.05
    // idx = 1;   // for the Med_Res model,  0.0251162 -0.980479 0.195013 -1.54655  leaf 0.05
    // idx = 3;   // for the Lowest_Res model, leaf 0.1
    // idx = 1;   // for the UltraHigh_Res_photoscan model,  0.0265256 -0.979325 0.200545 -1.40242  leaf 0.1  (?)
 
	//idx = 1;   // for the UltraHigh_Res_photoscan model,  0.0253401 -0.980082 0.196971 -1.51981  leaf 0.05


	// NEW
	// idx = 1;       // FOR gerrard-hall_High_Res_with_texture_decimated0.5
	//idx = 0;        // FOR apollo_compatta_giorgio_2
	idx = 2;        // FOR apollo_compatta_giorgio  and gerrard-hall_High_Res_with_texture_decimated0.25
	return idx;

}




bool CSegmenterColumns::projectCloudToGroundPlane(pcl::PointCloud<PointT>::Ptr cloud_filtered, std::vector<plane> planes, unsigned int idx, pcl::PointCloud<PointT>::Ptr cloud_rotated, pcl::PointCloud<PointT>::Ptr cloud_projected)
{
    float aGnd = planes[idx].a, bGnd = planes[idx].b, cGnd = planes[idx].c, dGnd = planes[idx].d - 5;

    // Now projection is in order!
    // Perhaps, this paper is also on the subject:
    // Chmelar et al., Projection of Point Cloud for Basic Object Detection (2014)
    // http://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=6923303

    // Now alternative approaches:
    //   (1) project on the plane
    //   (2) rotate everything then drop z coordinate
    // Try approach (1)
    // http://pointclouds.org/documentation/tutorials/project_inliers.php
    // For approach (2), also see
    // http://stackoverflow.com/questions/9423621/3d-rotations-of-a-plane
    // http://www.pcl-users.org/Project-color-cloud-into-rgb-image-td4019433.html
    // http://pointclouds.org/documentation/tutorials/matrix_transform.php
    // http://www.pcl-users.org/2D-projection-kinect-data-in-PCL-td3542621.html <----

    pcl::PCDWriter writer;

#if 0   // directly project to the ground plane

    // Create a set of planar coefficients for the ground plane
    pcl::ModelCoefficients::Ptr projCoefficients(new pcl::ModelCoefficients());
    projCoefficients->values.resize(4);   // 4 coefficients!
    projCoefficients->values[0] = aGnd;
    projCoefficients->values[1] = bGnd;
    projCoefficients->values[2] = cGnd;
    projCoefficients->values[3] = dGnd;

    // Create the filtering object
    pcl::ProjectInliers<PointT> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(cloud_filtered); // Work onm the segmented cloud (cloud_filtered) (can also use the original cloud, cloud)
    proj.setModelCoefficients(projCoefficients);
    proj.filter(*cloud_projected);

    writer.write<PointT>("projected.pcd", *cloud_projected, false);

#else   // rotate then project to the xy plane
    // I want to rotate the cloud so that its ground plane (the found plane) coincides with the xy plane; the plane normal will be the z axis, (0,0,1)
    // http://stackoverflow.com/questions/9423621/3d-rotations-of-a-plane
    // https://en.wikipedia.org/wiki/Rotation_matrix#Axis_and_angle
    // costheta = dot(M, N) / (norm(M)*norm(N));
    float costheta = cGnd;  // simplified because one of the vector is the z unit vector
                            // axis = unitcross(M, N)
                            // http://www.pcl-users.org/is-there-a-pcl-function-or-class-for-the-angle-between-2-vectors-td3561133.html
                            // https://pixinsight.com/developer/pcl/doc/html/group__vector__operators.html
                            // anyway there is no need of doing complicated things, because one of the vectors is the z unit vector:
                            //   v X w = (v2w3 - v3w2) i + (v3w1 - v1w3) j + (v1w2 - v2w1) k
                            // where v = (aGnd, bGnd, cGnd)
                            // and   w = (0, 0, 1)
                            // so:
    float xAxis = bGnd, yAxis = -aGnd, zAxis = 0;
    // c = costheta
    // s = sqrt(1 - c*c)   // sin(theta)
    float sintheta = sqrt(1 - costheta*costheta);
    // C = 1 - c
    float Costheta = 1 - costheta;
    // rmat = matrix([x*x*C + c    x*y*C - z*s  x*z*C + y*s],
    //		         [y*x*C + z*s  y*y*C + c    y*z*C - x*s]
    //               [z*x*C - y*s  z*y*C + x*s  z*z*C + c])
    //
    // http://pointclouds.org/documentation/tutorials/matrix_transform.php#matrix-transform
    // Use the "manual method"
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    // Define the rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
    //    (row, column)
    transform(0, 0) = xAxis * xAxis * Costheta + costheta;
    transform(0, 1) = xAxis * yAxis * Costheta - zAxis * sintheta;
    transform(0, 2) = xAxis * zAxis * Costheta + yAxis * sintheta;
    transform(1, 0) = yAxis * xAxis * Costheta + zAxis * sintheta;
    transform(1, 1) = yAxis * yAxis * Costheta + costheta;
    transform(1, 2) = yAxis * zAxis * Costheta - xAxis * sintheta;
    transform(2, 0) = zAxis * xAxis * Costheta - yAxis * sintheta;
    transform(2, 1) = zAxis * yAxis * Costheta + xAxis * sintheta;
    transform(2, 2) = zAxis * zAxis * Costheta + costheta;
    // Translation is (0,0,0)  (in point of fact, it is already set by the costructor)
    transform(0, 3) = 0.0;
    transform(1, 3) = 0.0;
    transform(2, 3) = 0.0;
    // Print the transformation
    printf("Transformation is (Method #1: using a Matrix4f)\n");
    std::cout << transform << std::endl;
    // Executing the transformation
    // Apply transform (rotate the whole cloud so that the ground plane is orthogonal to the z axis)
    //pcl::transformPointCloud(*cloud_filtered, *cloud_rotated, transform);    // Using the filtered cloud!!!!!!!!!!!!!!!!
    //	pcl::transformPointCloud(*cloud, *cloud_rotated, transform);    // Work on the original cloud (cloud)
    pcl::transformPointCloud(*cloud_filtered, *cloud_rotated, transform);    // Work on the segmented cloud (cloud_filtered)
    writer.write<PointT>("rotated.pcd", *cloud_rotated, false);

    // From now on, equal to approach (1) but the projection plane is normal to (0,0,1)
    // Create a set of planar coefficients with X=Y=0,Z=1
    pcl::ModelCoefficients::Ptr projCoefficients(new pcl::ModelCoefficients());
    projCoefficients->values.resize(4);   // 4 coefficients!
    projCoefficients->values[0] = 0;
    projCoefficients->values[1] = 0;
    projCoefficients->values[2] = 1;
    projCoefficients->values[3] = -5;

    // Create the filtering object
    pcl::ProjectInliers<PointT> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(cloud_rotated);
    proj.setModelCoefficients(projCoefficients);
    proj.filter(*cloud_projected);

    writer.write<PointT>("projected.pcd", *cloud_projected, false);


#endif

    return true;
}



bool CSegmenterColumns::buildOctreeAndDoBoxSearchingToCalculatePointDensity(pcl::PointCloud<PointT>::Ptr cloud_projected, PointT minPt, PointT maxPt, float binSide)
{

    // http://www.pcl-users.org/Saving-Octree-voxel-density-information-to-file-td3840729.html
    // https://github.com/PointCloudLibrary/pcl/blob/master/test/octree/test_octree.cpp
    //
    // http://www.pointclouds.org/assets/icra2012/search.pdf
    // http://www.pcl-users.org/OctreePointCloudDensity-leafnodeiterator-compilation-error-td4028781.html
    // http://pointclouds.org/documentation/tutorials/octree.php
    // http://docs.pointclouds.org/trunk/classpcl_1_1octree_1_1_octree_point_cloud_density.html#details

    // Also:
    // http://www.pcl-users.org/Project-color-cloud-into-rgb-image-td4019433.html
    // https://github.com/daviddoria/Resectioning/blob/master/Resectioning.cpp

#if 0

    // instantiate the OctreePointCloudDensity class
    pcl::octree::OctreePointCloudDensity<PointT> octreeA(1.0f);  // <---- MAGIC NUMBER, 1 for low res, 0.00001 for high res
                                                                 // octreeA.defineBoundingBox(7.0, 7.0, 7.0);

    octreeA.setInputCloud(cloud_projected);
    octreeA.defineBoundingBox();
    double xmin, xmax, ymin, ymax, zmin, zmax;
    octreeA.getBoundingBox(xmin, ymin, zmin, xmax, ymax, zmax);
    std::cout << "Bounding Box for octree is: xmin = " << xmin << ", xmax = " << xmax << ", ymin = " << ymin << ", ymax = " << ymax << ", zmin = " << zmin << ", zmax = " << zmax << std::endl;
    octreeA.addPointsFromInputCloud();   // Add points to the octree
                                         // octreeA.getVoxelDensityAtPoint (PointXYZRGB(x, y, z));   // To query the density at a poin



                                         /*
                                         pcl::octree::OctreePointCloud<PointT>::LeafNodeIterator it(octreeA)
                                         while (*++it) {
                                         OctreeLeafNode* node = *it;
                                         // get data from node..
                                         }
                                         */

#endif

#if 0

                                         // Let's try UniformSampling
                                         // http://docs.pointclouds.org/1.7.1/classpcl_1_1_uniform_sampling.html
                                         // http://pointclouds.org/documentation/tutorials/global_hypothesis_verification.php
                                         // http://pointclouds.org/documentation/tutorials/correspondence_grouping.php
                                         // http://www.pcl-users.org/FPFH-optimising-with-voxel-grid-td3418228.html
                                         // http://www.pcl-users.org/Uniform-Sampling-Compile-Problem-td3657920.html
                                         // http://www.pcl-users.org/problem-with-cluod-uniform-sampling-header-td4040685.html


    pcl::UniformSampling<PointT> uniform_sampling;
    pcl::PointCloud<PointT>::Ptr cloud_projected_sampled(new pcl::PointCloud<PointT>());
    uniform_sampling.setInputCloud(cloud_projected);
    uniform_sampling.setRadiusSearch(1);    // <-----------------------   MAGIC NUMBER
    uniform_sampling.filter(*cloud_projected_sampled);
    std::cout << "Model total points: " << cloud_projected->size() << "; Selected Keypoints: " << cloud_projected_sampled->size() << std::endl;



#endif

#if 1
    std::cout << "Max x: " << maxPt.x << std::endl;
    std::cout << "Max y: " << maxPt.y << std::endl;
    std::cout << "Max z: " << maxPt.z << std::endl;
    std::cout << "Min x: " << minPt.x << std::endl;
    std::cout << "Min y: " << minPt.y << std::endl;
    std::cout << "Min z: " << minPt.z << std::endl;


    // Copy cloud into an OctreePointCloudSearch
    // http://pointclouds.org/documentation/tutorials/octree.php
    float resolution = 1.0f;    // <------------------------------- MAGIC NUMBER
    pcl::octree::OctreePointCloudSearch<PointT> octree(resolution);
    octree.setInputCloud(cloud_projected);
    octree.addPointsFromInputCloud();

    // https://code.sat.qc.ca/redmine/projects/pcl/repository/revisions/70d6897eb6edc8cc0a5b80e4777e8e54440cd595/entry/filters/include/pcl/filters/impl/morphological_filter.hpp
    int maxN = 0, n;    // used to calculate the max cell occupancy, so that we can later normalize
                        // Calculate the number of bins, x and y
    float deltaX = maxPt.x - minPt.x;
    float deltaY = maxPt.y - minPt.y;
    unsigned int iXMax = round(deltaX / binSide);
    unsigned int iYMax = round(deltaY / binSide);
    std::cout << "Image size: iYMax = " << iYMax << ", iXMax = " << iXMax << std::endl;
    Eigen::MatrixXi img(iYMax, iXMax);   // define a dynamic 2D matrix, https://eigen.tuxfamily.org/dox/group__matrixtypedefs.html
    float z1 = minPt.z - binSide, z2 = maxPt.z + binSide; // they are equal; using resolution or anything else to give a thickness
    for (int iX = 0; iX < iXMax; iX++) {
        float x1 = minPt.x + iX * binSide;  // x limits
        float x2 = x1 + binSide;
        for (int iY = 0; iY < iYMax; iY++) {
            float y1 = minPt.y + iY * binSide;  // y limits
            float y2 = y1 + binSide;
            std::vector<int> pt_indices; // will contain the found point vertices
                                         // Prepare bounding box (i.e. the cell) for searching, by two vectors containing the two min and max limits (two 3d points)
            Eigen::Vector3f bbox_min, bbox_max;
            bbox_min = Eigen::Vector3f(x1, y1, z1);
            bbox_max = Eigen::Vector3f(x2, y2, z2);
            n = octree.boxSearch(bbox_min, bbox_max, pt_indices);  // number of pixels in box; http://docs.ros.org/hydro/api/pcl/html/classpcl_1_1octree_1_1OctreePointCloudSearch.html
            if (n > maxN) maxN = n;
            img(iY, iX) = n;
            //		std::cout << n << " " << pt_indices.size() << std::endl;
        }  // for iY
    }  // for iX
    std::vector<unsigned char> imguchar(iXMax*iYMax);
    int imgucharIdx = 0;  // must collect data in row-major order,
    for (int iY = 0; iY < iYMax; iY++)
        for (int iX = 0; iX < iXMax; iX++) {
            imguchar[imgucharIdx] = (unsigned char)round(img(iY, iX) / (float)maxN * 255);
            //		std::cout << (unsigned int)imguchar[imgucharIdx] << " ";
            imgucharIdx++;
        }
    std::cout << std::endl;
    // Now save the buffer, size is iYMax rows by iXMax columns, int; It did not work, replaced by pgm saving!
    //    const char *p = (const char *)img.data();  // should I dealloc this region? is this jus a pointer to an existing memory region or a newly allocated zone?
    //    std::ofstream myFile("data.bin", std::ios::out | std::ios::binary);
    //    myFile.write(p, iYMax * iXMax * sizeof(int)); // unsigned int!
    // Now save the image, size is iYMax rows by iXMax columns, unsigned char
    pgmb_write("data.pgm", iXMax, iYMax, imguchar.data());

#endif

    return true;
}


// Apply the Hough transform to data.pgm so as to find circles, then saves centers.txt and radii.txt

bool CSegmenterColumns::applyHoughToPointDensityOfProjectedCloud(unsigned int &nc)
{

    // The first part is useless at present, and it is replaced by the second part

#if 0   // Unfortunately OpenCV implementation of the Circle Hough transform is not good for my image! Use Matlab!!

    // circle Hough transform
    // The image is in imguchar.data, size iXMax x iYMax, int
    // http://docs.opencv.org/2.4/doc/tutorials/imgproc/imgtrans/hough_circle/hough_circle.html
    // http://opencvexamples.blogspot.com/2013/10/hough-circle-detection.html

    {
        cv::Mat src, src_gray;

        // Read the image
        src = cv::imread("data.pgm", 1);

        if (!src.data)
        {
            return -1;
        }

        // Convert it to gray
        cvtColor(src, src_gray, CV_BGR2GRAY);

        // Reduce the noise so we avoid false circle detection
        //		GaussianBlur(src_gray, src_gray, cv::Size(3, 3), 2, 2);
        //		GaussianBlur(src_gray, src_gray, cv::Size(7, 7), 2, 2, 4);

        std::vector<cv::Vec3f> circles;

        // Apply the Hough Transform to find the circles
        // void HoughCircles(InputArray image, OutputArray circles, int method, double dp, double minDist, double param1=100, double param2=100, int minRadius=0, int maxRadius=0 )
        double dp = 1;
        double minDist = 30;
        double param1 = 200; // 200 First method-specific parameter. In case of CV_HOUGH_GRADIENT , it is the higher threshold of the two passed to the Canny()edge detector (the lower one is twice smaller).
        double param2 = 15; // 15 Second method-specific parameter. In case of CV_HOUGH_GRADIENT , it is the accumulator threshold for the circle centers at the detection stage. The smaller it is, the more false circles may be detected. Circles, corresponding to the larger accumulator values, will be returned first.
        int minRadius = 3;
        int maxRadius = 30;
        HoughCircles(src_gray, circles, CV_HOUGH_GRADIENT, dp, minDist, param1, param2, minRadius, maxRadius);

        // Draw the circles detected
        for (size_t i = 0; i < circles.size(); i++)
        {
            cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
            int radius = cvRound(circles[i][2]);
            // circle center
            //	circle(src, center, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
            // circle outline
            circle(src, center, radius, cv::Scalar(0, 0, 255), 1, 8, 0);
        }

        // Show your results
        cv::namedWindow("Hough Circle Transform Demo", CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO); // CV_WINDOW_AUTOSIZE; http://docs.opencv.org/2.4/doc/tutorials/introduction/display_image/display_image.html
        imshow("Hough Circle Transform Demo", src);

        cv::waitKey(0);
    }


#endif  // OpenCV Circle Hough transform!

    // Call matlab...........!
    // http://stackoverflow.com/questions/486087/how-to-call-an-external-program-with-parameters
    // and other links; some say that "system" is not safe!  CHANGE AND USE the executeCommand member function

    system("findcolumnloop.exe data.pgm");  // reads data.pgm and saves centers.txt and radii.txt

                                   // centers.txt is
                                   // 1.5669058e+02   2.1696284e+02
                                   // 1.3758821e+02   2.6522947e+02
                                   // 1.0007107e+02   3.6237131e+02
                                   // 1.1886819e+02   3.1326580e+02

                                   // radii.txt is
                                   // 5.0000000e+00
                                   // 5.0000000e+00
                                   // 5.0000000e+00
                                   // 6.0000000e+00

    // number of columns found
    std::ifstream radiiFile("radii.txt");
    double cr;
    nc = 0;
    while (radiiFile >> cr) nc++;
    radiiFile.close();
    std::cout << nc << " columns found" << std::endl;

    return nc > 0?  true : false;
}




bool CSegmenterColumns::saveColumnBoundingBoxCloudsAndVisualize(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer, pcl::PointCloud<PointT>::Ptr cloud_rotated, pcl::PointCloud<PointT>::Ptr cloud_projected, PointT minPt, PointT maxPt, float binSide)
{
	// http://pointclouds.org/documentation/tutorials/pcl_visualizer.php
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<PointT>(cloud_rotated, "Segmented cloud");
	viewer->addPointCloud<PointT>(cloud_projected, "Projected cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Segmented cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Projected cloud");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();

	/*

	PointT center(mass_center(0), mass_center(1), mass_center(2));
	PointT x_axis(major_vector(0) + mass_center(0), major_vector(1) + mass_center(1), major_vector(2) + mass_center(2));
	PointT y_axis(middle_vector(0) + mass_center(0), middle_vector(1) + mass_center(1), middle_vector(2) + mass_center(2));
	PointT z_axis(minor_vector(0) + mass_center(0), minor_vector(1) + mass_center(1), minor_vector(2) + mass_center(2));
	viewer->addLine(center, x_axis, 1.0f, 0.0f, 0.0f, "major eigen vector");
	viewer->addLine(center, y_axis, 0.0f, 1.0f, 0.0f, "middle eigen vector");
	viewer->addLine(center, z_axis, 0.0f, 0.0f, 1.0f, "minor eigen vector");



	*/

    // Before anything else, I want to know the cloud_rotated physical limits;
    // I already know x and y (they coincide with those of cloud_projected) but I also want z limits

    PointT minPtRot, maxPtRot;
    pcl::getMinMax3D(*cloud_rotated, minPtRot, maxPtRot);
    std::cout << "Max x: " << maxPtRot.x << std::endl;
    std::cout << "Max y: " << maxPtRot.y << std::endl;
    std::cout << "Max z: " << maxPtRot.z << std::endl;
    std::cout << "Min x: " << minPtRot.x << std::endl;
    std::cout << "Min y: " << minPtRot.y << std::endl;
    std::cout << "Min z: " << minPtRot.z << std::endl;

	// Prepare a dark red for the default color (no cluster)

	for (int j = 0; j < cloud_rotated->points.size(); j++) {
		cloud_rotated->points[j].r = 100;
		cloud_rotated->points[j].g = 0;
		cloud_rotated->points[j].b = 0;
	}


    // Read the two files back (center and radii)
    // http://stackoverflow.com/questions/7868936/read-file-line-by-line
    // and loop on the lines, reading the candidate columns center (cx, cy) and radius (cr) data
    // For each column insert a parallelepiped including the candidate, and save the point cloud to disk

    std::ifstream centersFile("centers.txt");
    std::ifstream radiiFile("radii.txt");
    double cx, cy, cr;
    int nc = 0;
	clusters.clear();
    while ((centersFile >> cx >> cy) && (radiiFile >> cr))
    {
        // now process triad (cx,cy,cr) for each circle
        std::cout << "cx = " << cx << ", cy = " << cy << ", cr = " << cr << std::endl;
        cx--; cy--;  // because they start from 1 in matlab but from 0 in C++; can be omitted if we change the next two instructions to: "- binSide/2"

        std::string str = "cylinderbox_";
        str.append(std::to_string(nc));

        // convert 2d coordinates of the circle center on the projected plane, to 3d
        float x = minPt.x + cx * binSide + binSide / 2;  // center, x
        float y = minPt.y + cy * binSide + binSide / 2;  // center, y
        Eigen::Vector4f minPoint;
        minPoint[0] = x - 3 * cr * binSide;  // define minimum point x
        minPoint[1] = y - 3 * cr * binSide;  // define minimum point y
        minPoint[2] = minPtRot.z;  // define minimum point z
        Eigen::Vector4f maxPoint;
        maxPoint[0] = x + 3 * cr * binSide;  // define max point x
        maxPoint[1] = y + 3 * cr * binSide;  // define max point y
        maxPoint[2] = maxPtRot.z;  // define max point z

								   // http://pointclouds.org/documentation/tutorials/moment_of_inertia.php#moment-of-inertia

		if (0)   // draw bounding boxes 
		{
			viewer->addCube(minPoint[0], maxPoint[0], minPoint[1], maxPoint[1], minPoint[2], maxPoint[2], 0.0, 0.0, 1.0, str);
			// http://www.pcl-users.org/Transparent-sphere-in-PCLVisualizer-td4036346.html
			viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, str);
		}


		
		// Crop and save the column bounding box
        // Alternative approach, use http://www.pcl-users.org/Filtering-objects-in-a-bounding-box-td4037456.html
        pcl::CropBox<PointT> cropFilter;   // extract_removed_indices = true, to get them! see http://docs.pointclouds.org/1.7.1/classpcl_1_1_crop_box_3_01pcl_1_1_p_c_l_point_cloud2_01_4.html#a630765c1f7d44022270fdf27975baebd
        cropFilter.setInputCloud(cloud_rotated);
        cropFilter.setMin(minPoint);
        cropFilter.setMax(maxPoint);
        pcl::PointCloud<PointT>::Ptr cloud_cylinder(new pcl::PointCloud<PointT>);
        cropFilter.filter(*cloud_cylinder);
        pcl::PCDWriter writer;
        writer.write<PointT>(str.append(".pcd"), *cloud_cylinder, false);


		// Now apply again the same technique, but only to get the inidces of the points included in the bbox!
		// Then I can build up some clusters

		// It is better to look at the code:
		// https://github.com/PointCloudLibrary/pcl/blob/master/filters/src/crop_box.cpp
		// to understand how crop box indices work
		//
		// If negative, and the point is outside, I don't want the index (unfortunately it copies the complementary cloud!!)
		// If negative, and the point is inside, I want the index
		//
		// So unfortunately it gives to output the deleted point indices, not the chosen point indices! So in order to get the inside point indices, I must
		// use negative and copy to the poutput cloud all the other points, which are a lot!

		pcl::CropBox<PointT> cropFilter1(true);   // extract_removed_indices = true, to get them! see http://docs.pointclouds.org/1.7.1/classpcl_1_1_crop_box_3_01pcl_1_1_p_c_l_point_cloud2_01_4.html#a630765c1f7d44022270fdf27975baebd
		cropFilter1.setInputCloud(cloud_rotated);
		cropFilter1.setNegative(true);           // <--------------------------------------
		cropFilter1.setMin(minPoint);
		cropFilter1.setMax(maxPoint);
		cropFilter1.filter(*cloud_cylinder);   // use again the cloud, this time to contain the complementary of the bbox, just to later extract the indices...
		
		//		pcl::IndicesPtr	iPtr = cropFilter1.getIndices();
		pcl::IndicesConstPtr riPtr = cropFilter1.getRemovedIndices();

		// Choose color
		pcl::RGB aColor = pcl::GlasbeyLUT::at(nc % pcl::GlasbeyLUT::size());

		// Colorize the points belonging to the bounding box (color from  GlasbeyLUT)
		// There are riPtr->size() points
		for (int j = 0; j < riPtr->size(); j++) {
			cloud_rotated->points[(*riPtr)[j]].r = aColor.r;
			cloud_rotated->points[(*riPtr)[j]].g = aColor.g;
			cloud_rotated->points[(*riPtr)[j]].b = aColor.b;
		}
		
		// http://www.pcl-users.org/from-PointIndices-to-IndicesPtr-td4034763.html : here is how to convert
		// pcl::PointIndices::Ptr fouth_clusterR(new  pcl::PointIndices(clusters[3].indices));
		// I have not used the links now written: I've tried and reasoned... it compiles.... No "make share"? Mistery
		// http://docs.pointclouds.org/trunk/_point_indices_8h_source.html

		pcl::PointIndices::Ptr piPtr(new pcl::PointIndices);   // da http://www.pcl-users.org/IndicesPtr-from-PointIndices-td4020356.html
		piPtr->indices = *riPtr;
		clusters.push_back(*piPtr);

		nc++;
    }  // while (counting the columns)

 
	   
	// Build up a new cluster containing all the red points, and append it to the clusters variable member
	pcl::PointIndices::Ptr piPtr(new pcl::PointIndices);   // da http://www.pcl-users.org/IndicesPtr-from-PointIndices-td4020356.html
	for (int j = 0; j < colored_cloud->points.size(); j++)
	{
		pcl::PointXYZRGB *p = &colored_cloud->points[j];
		if (p->r == 255 && p->g == 0 && p->b == 0)
			piPtr->indices.push_back(j);
	}
	clusters.push_back(*piPtr);


	viewer->updatePointCloud(cloud_rotated, "Segmented cloud");

    centersFile.close();
    radiiFile.close();
    std::cout << nc << " columns found";

 	return true;

	/*
	viewer->addCube(0, 1, 0, 1, 0, 1, 1, 0, 0, "TheCube");
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, "TheCube");
	*/
	/*
	// Draw base grid, built of small squares
	for (int iX = 0; iX < iXMax; iX+=10) {
	float x1 = minPt.x + iX * binSide;  // x limits
	float x2 = x1 + binSide;
	for (int iY = 0; iY < iYMax; iY+=10) {
	float y1 = minPt.y + iY * binSide;  // y limits
	float y2 = y1 + binSide;
	std::string str = "Pt_";
	str.append(std::to_string(iX));
	str.append("_");
	str.append(std::to_string(iY));
	viewer->addCube(x1, x2, y1, y2, 10.0, 10.1, 1, 1, 0, str);
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, str);
	}  // for iY
	}  // for iX
	*/

}






bool CSegmenterColumns::segmentCylinders(unsigned int nc, float binSide)
{
	bool retFlag = true;

    // CYLINDER SEGMENTATION
    for (int j = 1; j <= nc; j++)    // CYLINDERS
    {

        // RANSAC http://pointclouds.org/documentation/tutorials/cylinder_segmentation.php
        // RANSAC http://vml.sakura.ne.jp/koeda/PCL/tutorials/html/cylinder_segmentation.html

        // Load file | Works with PCD  PLY  obj files
        pcl::PointCloud<PointT>::Ptr cloud_cylinder_box(new pcl::PointCloud<PointT>);

        std::string filenameCylinderBox = "cylinderbox_";
        filenameCylinderBox.append(std::to_string(j));
        filenameCylinderBox.append(".pcd");

        if (pcl::io::loadPCDFile(filenameCylinderBox, *cloud_cylinder_box) < 0) {
            std::cerr << "Error loading cylinder point cloud " << filenameCylinderBox << std::endl << std::endl;
			retFlag = false;
        }

        // All the objects needed
        pcl::NormalEstimation<PointT, pcl::Normal> ne;
        pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
        pcl::ExtractIndices<PointT> extract;
        pcl::ExtractIndices<pcl::Normal> extract_normals;
        pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());

        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
        pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);


        // Estimate point normals
        ne.setSearchMethod(tree);
        ne.setInputCloud(cloud_cylinder_box);
        ne.setKSearch(50);
        ne.compute(*cloud_normals);

        // Create the segmentation object for cylinder segmentation and set all the parameters
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_CYLINDER);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setNormalDistanceWeight(0.2);  // surface normals influence weight (0.1); 0.5 is more strict and drops many outliers
        seg.setMaxIterations(10000);
        seg.setDistanceThreshold(0.2);   // distance threshold from each inlier point to the model; 0.05 I loose some points on column top!
        seg.setRadiusLimits(5 * binSide, 30 * binSide);    // RADIUS LIMIT, see Matlab Circle-finding code
        seg.setInputCloud(cloud_cylinder_box);
        seg.setInputNormals(cloud_normals);

        // Obtain the cylinder inliers and coefficients
        seg.segment(*inliers_cylinder, *coefficients_cylinder);
        std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;  // Center.x Center.y Center.z Axis.x Axis.y Axis.z Radius


        // Write the cylinder inliers to disk
        extract.setInputCloud(cloud_cylinder_box);
        extract.setIndices(inliers_cylinder);
        extract.setNegative(false);
        pcl::PointCloud<PointT>::Ptr cloud_cylinder(new pcl::PointCloud<PointT>());
        extract.filter(*cloud_cylinder);
		if (cloud_cylinder->points.empty())
		{
			std::cerr << "Can't find the cylindrical component." << std::endl;
			retFlag = false;
		}
        else
        {
            std::string filenameCylinder = "cylinder_";
            filenameCylinder.append(std::to_string(j));
            filenameCylinder.append(".pcd");

            std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->points.size() << " data points." << std::endl;
            pcl::PCDWriter writer;
            writer.write(filenameCylinder, *cloud_cylinder, false);

			retFlag = true;
        }

    }  // for j (CYLINDERS)

	return retFlag;
}   // segmentCylinders






PointCloudT::Ptr                                 CSegmenterColumns::getOutputCloud(void)
{
	return (colored_cloud);


}


std::vector <pcl::PointIndices>                  CSegmenterColumns::getOutputClusters(void)
{


	return (clusters);

}



/*
int aa = 22;
int bb = 12;
int cc = 10;

cv::namedWindow("picture");
cvCreateTrackbar("X_limit", "picture", &aa, 30, NULL);
cvCreateTrackbar("Y_limit", "picture", &bb, 30, NULL);
cvCreateTrackbar("Z_limit", "picture", &cc, 30, NULL);
*/



// Fill in the cloud data
//	pcl::PCDReader reader;
//	reader.read("table_scene_lms400.pcd", *cloud_blob);




























