#ifndef PCLVIEWER_H
#define PCLVIEWER_H

// The next #define avoids deprecation of fopen in favour of fopen_s, and so on (fscanf etc)
#define _CRT_SECURE_NO_DEPRECATE


#include <iostream>
#include <limits>  // for NaN()
#include <cmath>  // for NaN()   (?)
#include <algorithm>    // std::set_difference, std::sort
#include <iterator>
#include <cstdlib>   // for system (and getline)

using namespace std;

// Qt
#include <QMainWindow>
#include <QFileDialog>
#include <QMessageBox>
#include <QButtonGroup>

#include <QFileInfo>
#include <QTextStream>   // for some misterious reasons, after inserting this, I had to convert all the endl into std::endl otherwise I had errors concerning <<

// Point Cloud Library GDN
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/visualization/point_picking_event.h>
#include <pcl/impl/point_types.hpp>   // for RGB type

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/vtk_lib_io.h>            // load

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>

// Region Growing
#include <pcl/segmentation/region_growing.h>

// indices
#include <pcl/filters/extract_indices.h>

typedef pcl::PointXYZRGB PointT;  // RGBA is not good because there is no A field saved by meshlab
typedef pcl::PointCloud<PointT> PointCloudT;

struct cloudAndIndices
{
	PointCloudT::Ptr pointCloudPtr;
	pcl::IndicesPtr  indicesPtr;
};

namespace Ui
{
  class PCLViewer;
}

class PCLViewer : public QMainWindow
{
  Q_OBJECT

public:
  explicit PCLViewer (QWidget *parent = 0);
  ~PCLViewer ();

protected:
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  PointCloudT::Ptr cloud, labeledCloud; // the original (sparse or dense) and the labeled clouds

  pcl::PolygonMesh mesh;  // GDN
  pcl::TextureMesh textureMesh;   // GDN
  QString CurrentDirectory;   // GDN
  QString FileName;   // GDN

  // next variables set after segmentation in on_pushButton_StartSegmentation_clicked()
 //////////////////// pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;    // Will contain information on RG segmentation
  std::vector <pcl::PointIndices> clusters;             // somewhat redundant because it is in reg, but I am directly using it
//  std::vector <pcl::RGB> clusterColors;               // Contains the color components of the segmented clusters, so that I can restore the clusters if I first delete them (by coloring in red)
  std::vector <float> clusterColors;               // Contains the color components of the segmented clusters, so that I can restore the clusters if I first delete them (by coloring in red)

  // Next variables are used to contain deleted/selected cluster flags.
  // The initial approach concerning cluster deletion, was based on really deleting the points
  //   from the cloud, which was then rejected because it led to point index changes (if you delete
  //   a point from the cloud, all the indices change and the initial clusters do not contain true indices:
  //   there is no alignment between clusters and the cloud!)
  //   In a second approach I transferred the "removed" clusters (i.e. the indices of their points) to
  //   a "deletedClusters" variable, so not really deleting the cluster points from the cloud, so when
  //   for example I am clicking on a point, I could check if this point index was in "clusters" and decide if to select the cluster or not.
  //   Finally I decided to simply mark the selected clusters are selected or deleted.
  std::vector<bool> deletedClustersFlags;    // true/false if flag deleted or not
  std::vector<bool> selectedClustersFlags;    // true/false if flag selected or not


  unsigned int red;
  unsigned int green;
  unsigned int blue;


private slots:
  void on_pushButton_LoadCloud_clicked();
  void on_pushButton_LoadMesh_clicked();
  void on_pushButton_LoadTextureMesh_clicked();
  void on_buttonClicked_LoadSparseOrDenseCloud(int id);  //  I don't use the int id
  void on_buttonClicked_ShowOriginalOrLabelColors(int id);  //  I don't use the int id
  void on_pushButton_StartSegmentation_clicked();
  void on_pushButton_ClearSegmentation_clicked();
  void on_pushButton_DeleteSelected_clicked();
  void on_pushButton_PreserveSelected_clicked();
  void on_pushButton_Save_clicked();
  void on_pushButton_Restore_clicked();
  void on_pushButton_SelectAll_clicked();
  void on_pushButton_DeselectAll_clicked();
  void on_pushButton_InvertSelection_clicked();
  void on_pushButton_SegmResetDefaults_clicked();


private:
  Ui::PCLViewer *ui;

  // GDN
private:
  void resetAll (void);
  bool loadCloud(const string& FileNameStr);
  bool loadMesh(const string& FileNameStr);
  bool loadTextureMesh(const string& FileNameStr);
  string GetFileExtension(const string& FileNameStr);
  void clearViewer();
  bool extractPointClouds(const string &FileNameStr);
  bool executeCommand(const string& command);
  void pp_callback (const pcl::visualization::PointPickingEvent& event, void* viewer_void);
  void selectACluster(int theCluster);
  void deSelectACluster(int theCluster);
  bool filterObjInputFileToOutputFileOnlyPreservingDesiredPoints(const QString inObjFilePath, const QString outObjFilePathset, set<int> indicesToDrop);
  set<int> cumulateDroppedPointIndices(void);
  QString getSaveFileNameFromUser();
  PointCloudT::Ptr extractSelectedClustersFromCloud(void);
  void dropClusterFromClustersWithIndex(int clusterIndex);
  void appendClusterToClusters(pcl::IndicesPtr indicesToAdd);

  void segmResetAllDefaults();
  void segmRGResetDefaults();
  void segmLCCPResetDefaults();
};

#endif // PCLVIEWER_H
