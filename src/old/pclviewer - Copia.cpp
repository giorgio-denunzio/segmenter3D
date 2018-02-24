#include "pclviewer.h"
#include "../build/ui_pclviewer.h"

#include <Windows.h>     // for CreateProcess
#include <tchar.h>

#include "CSegmenterRegionGrowing.h"
#include "CSegmenterLCCP.h"
#include "CSegmenterColumns.h"

// https://stackoverflow.com/questions/2490090/set-label-text-in-qtabwidget-tab

// https://stackoverflow.com/questions/2465413/qt-add-a-hyperlink-to-a-dialog
// http://doc.qt.io/archives/qt-4.8/qlabel.html#openExternalLinks-prop
// (set both OpenExternalLinks and the LinksAccessibleByMouse interaction flag)

// Remark that I am not using any Qt quit like
//               QApplication::quit();
// See https://stackoverflow.com/questions/8026101/correct-way-to-quit-a-qt-program



PCLViewer::PCLViewer(QWidget *parent) :
	QMainWindow(parent),
	ui(new Ui::PCLViewer)
{

	ui->setupUi(this);
	this->setWindowTitle("Segmentation Tool");

	resetAll();

	// Setup the cloud pointers
	cloud.reset(new PointCloudT);   // why not ->?
	labeledCloud.reset(new PointCloudT);

	// The number of points in the cloud
	//cloud->points.resize (200);

	// The default color
	//red   = 128;
	//green = 128;
	//blue  = 128;

	FileName = "";

	// Fill the cloud with some points
	/*

  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
	cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
	cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
	cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);

	cloud->points[i].r = red;
	cloud->points[i].g = green;
	cloud->points[i].b = blue;
  }
*/

// Set up the QVTK window
	viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));   // <------------------------- How is it that I must not use viewer-> ???
	ui->qvtkWidget->SetRenderWindow(viewer->getRenderWindow());
	viewer->setupInteractor(ui->qvtkWidget->GetInteractor(), ui->qvtkWidget->GetRenderWindow());
	ui->qvtkWidget->update();




	// GDN
	// http://www.qtcentre.org/threads/25528-twice-execution-of-click-on-pushbutton
	// I comment this line out because I had the slot called twice (call by name).
	// connect (ui->pushButton_LoadMesh, SIGNAL(clicked(bool)), this, SLOT (on_pushButton_LoadMesh_clicked()));

	// Remember to check but, IIRC, if the slot has the same name as the button (with on_ and _clicked) it is connected automagically so
	// I don't have to explicitly connect. Yes. it works this way.

	connect(ui->buttonGroup_MeshToPointCloud, SIGNAL(buttonClicked(int)), this, SLOT(on_buttonClicked_LoadSparseOrDenseCloud(int)));
	connect(ui->buttonGroup_DisplayPreferences, SIGNAL(buttonClicked(int)), this, SLOT(on_buttonClicked_ShowOriginalOrLabelColors(int)));

	// Tooltips TO BE COMPLETED!!!!
	ui->pushButton_LoadMesh->setToolTip(tr("Load mesh from a file"));
	ui->pushButton_LoadTextureMesh->setToolTip(tr("Load textured mesh from a file"));
	ui->pushButton_LoadCloud->setToolTip(tr("Load point cloud from a file"));

	CurrentDirectory = QDir::currentPath();

	viewer->addPointCloud(cloud, "cloud");
	viewer->addTextureMesh(textureMesh, "textureMesh");
	viewer->addPolygonMesh(mesh, "mesh");   // , 0

	viewer->setBackgroundColor(0, 0, 0);
	// viewer->addCoordinateSystem (1.0);

	//viewer->initCameraParameters ();
	viewer->resetCamera();

	// Events: point clicking, keyboard, mouse:

	//  viewer->registerPointPickingCallback (pp_callback, (void*)&viewer);
	//  viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);
	//  viewer->registerMouseCallback (mouseEventOccurred, (void*)&viewer);

	viewer->registerPointPickingCallback(&PCLViewer::pp_callback, *this);   // Syntax is different when pp_callback is a class member, so I am using &class::.... and *this; see the example in ni_linemod.cpp code

	ui->qvtkWidget->update();
}


void PCLViewer::resetAll()
{
	segmResetAllDefaults();

	ui->pushButton_StartSegmentation->setEnabled(false);
	ui->pushButton_ClearSegmentation->setEnabled(false);
	ui->pushButton_SelectAll->setEnabled(false);
	ui->pushButton_DeselectAll->setEnabled(false);
	ui->pushButton_DeleteSelected->setEnabled(false);
	ui->pushButton_Restore->setEnabled(false);
	ui->pushButton_PreserveSelected->setEnabled(false);
	ui->pushButton_Save->setEnabled(false);
	ui->pushButton_InvertSelection->setEnabled(false);
}



PCLViewer::~PCLViewer()
{
	delete ui;
}

// FROM HERE ON, MY WIDGETS; Contextual Menu, Go to slot, choose the right one and you get here!  GDN

///////////////////////////////////////////////////////////////////////////////////////////////////////


void PCLViewer::on_pushButton_DeleteSelected_clicked()
// I might define soma deleteACluster similarly to selectACluster and simplify the code but perhaps it is useless
// because I delete clusters only in this function (IIRC)
{

	cout << "on_pushButton_DeleteSelected_clicked\n";

	if (clusters.size() > 0)
	{
		// Points belonging to the selected clusters (checked in the selectedClustersFlags vector) get colored in red,
		//   and the corresponding cluster position in deletedClustersFlags are marked as deleted so that after a
		//   mouse click I check if the point belongs to a deleted cluster and I don't risk to select a red cluster.
		// Colors must be rememberd so that later we can restore the deleted clusters.

		for (int c = 0; c < clusters.size(); c++)
		{
			// check if theCluster is selected; if it is,
			//   1) it had not already been deleted, because I cannot select already deleted clusters
			//   2) now mark it as deleted and color its points with red
			if (selectedClustersFlags[c])
			{
				for (int k = 0; k < clusters[c].indices.size(); k++)  // for each point in the cluster, color with red
				{
					labeledCloud->points[clusters[c].indices[k]].r = 100;
					labeledCloud->points[clusters[c].indices[k]].g = 0;
					labeledCloud->points[clusters[c].indices[k]].b = 0;
				} // for int k
				deletedClustersFlags[c] = true;    // logically delete the cluster (set the "selected cluster" flag)
				// selectedClustersFlags[c] = false;  // logically deselect the cluster (clear the "selected cluster" flag) (redundant, see call of on_pushButton_DeselectAll_clicked)
			}     // if the cluster was selected
		}     // for int c

		// Erase cubes and empty selectedClusters
		on_pushButton_DeselectAll_clicked();

		// refresh the point cloud on screen
		viewer->updatePointCloud(labeledCloud, "cloud");
		ui->qvtkWidget->update();
	}
}


///////////////////////////////////////////////////////////////////////////////////////////////////////


void PCLViewer::on_pushButton_DeselectAll_clicked()
{
	cout << "on_pushButton_DeselectAll_clicked\n";

	for (int c = 0; c < clusters.size(); c++)
		if (selectedClustersFlags[c])
			deSelectACluster(c);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////


void PCLViewer::on_pushButton_SelectAll_clicked()
{
	cout << "on_pushButton_SelectAll_clicked\n";

	for (int c = 0; c < clusters.size(); c++)
		if (!selectedClustersFlags[c] && !deletedClustersFlags[c])
			selectACluster(c);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////


void PCLViewer::on_pushButton_InvertSelection_clicked()
{
	cout << "on_pushButton_InvertSelection_clicked\n";

	for (int c = 0; c < clusters.size(); c++)
		if (selectedClustersFlags[c])
			deSelectACluster(c);
		else if (!deletedClustersFlags[c])
			selectACluster(c);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////

void PCLViewer::on_pushButton_PreserveSelected_clicked()
{

	cout << "on_pushButton_PreserveSelected_clicked\n";

	// Invert selection and call on_pushButton_DeleteSelected_clicked
//	for (int c = 0; c < clusters.size(); c++)
//		selectedClustersFlags[c] = !selectedClustersFlags[c];
	on_pushButton_InvertSelection_clicked();
	on_pushButton_DeleteSelected_clicked();

}

///////////////////////////////////////////////////////////////////////////////////////////////////////


void PCLViewer::on_pushButton_Save_clicked()
{


	/*

	int retVal = pcl::io::loadPolygonFileOBJ(FileName.toStdString(),  textureMesh);
	// http://docs.pointclouds.org/1.3.1/namespacepcl_1_1io.html#a97788cd2b1f0623fbcbe5ae42aa3664c
	// http://docs.pointclouds.org/1.7.0/structpcl_1_1_texture_mesh.html
	pcl::io::saveOBJFile ("prova.obj", textureMesh);

	*/






	/*

	// http://en.cppreference.com/w/cpp/types/numeric_limits/quiet_NaN
	double xxx = std::numeric_limits<double>::quiet_NaN();


	std::cout << std::numeric_limits<double>::quiet_NaN() << ' '
			  << std::numeric_limits<double>::signaling_NaN() << ' '
			  << std::acos(2) << ' '
			  << std::tgamma(-1) << ' '
			  << std::log(-1) << ' '
			  << std::sqrt(-1) << ' '
			  << xxx
			  << '\n';

	std::cout << "NaN == NaN? " << std::boolalpha
			  << ( std::numeric_limits<double>::quiet_NaN()
				   == std::numeric_limits<double>::quiet_NaN() ) << '\n';


*/




	cout << "on_pushButton_Save_clicked\n";

	if (clusters.size() == 0)
		return;

	QString saveFileName = getSaveFileNameFromUser();

	if (saveFileName.isNull())
		return;

	// prepare the list of indices to be dropped: later, when scanning the input file and building the output file,
	// when a vertex will have to be dropped (i.e. it will be listed in indicesToDrop), its coordinates will be replaced by nan.
	set<int> indicesToDrop = cumulateDroppedPointIndices();


	// Now call the function which does the job
	// I also need the original .obj file, which will be the input of filtering

	if (filterObjInputFileToOutputFileOnlyPreservingDesiredPoints(FileName, saveFileName, indicesToDrop))
	{
		// ok

	}
	else
	{
		// problems
	}

}






// http://doc.qt.io/qt-5/qtwidgets-tutorials-addressbook-part6-example.html
// https://stackoverflow.com/questions/20928023/how-to-use-qfiledialog-options-and-retrieve-savefilename
// https://stackoverflow.com/questions/4916193/creating-writing-into-a-new-file-in-qt

// Remark: getSaveFileName already check for "file exists"!!!!!! Moreover, it gives a message in italian!!
// http://www.qtcentre.org/threads/19833-QFileDialog-getSaveFileName()-doesn-t-convey-choice-to-NOT-OVERWRITE-existing-file
// I am disabling the feature, so I can check by myself as I prefer (moreover the feature seems not optimal, read the link above)
// http://doc.qt.io/qt-5/qfiledialog.html#getSaveFileName how to insert options; consider that in Qt4 there was one less argument!!!!

QString PCLViewer::getSaveFileNameFromUser()
{

	QString saveFileName = QFileDialog::getSaveFileName(this,
		tr("Save 3D Model to File"),
		CurrentDirectory,
		tr("3D point cloud (*.obj)"),
		Q_NULLPTR,
		QFileDialog::DontConfirmOverwrite);
	if (saveFileName.isNull())   // or isEmpty(), but see https://stackoverflow.com/questions/9724966/how-to-initialize-a-qstring-to-null, null and empty are different!
		return QString();   // null string
	else {
		// Check if file already exists and, if this is the case, ask the user if he/she wants to overwrite
		// https://stackoverflow.com/questions/10273816/how-to-check-whether-file-exists-in-qt-in-c
		QFileInfo check_file(saveFileName);
		// check if file exists and if yes: Is it really a file and no directory?
		if (check_file.exists() && check_file.isFile())
		{
			QMessageBox::StandardButton reply;
			reply = QMessageBox::question(this, "The file already exists", "Overwrite?",
				QMessageBox::Yes | QMessageBox::No);
			if (reply == QMessageBox::No)
				return QString();   // null string
		}
		cout << "Ready to write\n";

		return saveFileName;
	}  // else

}



set<int> PCLViewer::cumulateDroppedPointIndices(void)
{
	// I want to find the indices of the points to be dropped, so that I can use them to later put NaN in the cloud point.
	// I am obliged to reason in the opposite direction: put together all the indices of the points to be preserved  (remember the definition: vector <pcl::PointIndices> clusters;)
	// I'd prefer to directly put together the points to be dropped (instead of gathering the point to be preserved and then to do set difference),
	// but unfortunately the "red" points, not listed in the variable "clusters", are written nowhere (as far as I know), so they must be obtained
	// by difference.
	// So first cumulate the preserved indices.

	std::vector<int> indicesToPreserve;  // vector
	for (int c = 0; c < clusters.size(); c++)
		if (!deletedClustersFlags[c])
			indicesToPreserve.insert(indicesToPreserve.end(), clusters[c].indices.begin(), clusters[c].indices.end());

	// https://stackoverflow.com/questions/20052674/how-to-convert-vector-to-set
	// std::set<int> s(indicesToDrop.begin(), indicesToDrop.end());
	// As an alternative, I leave the vector but I have to sort it first!

	std::sort(indicesToPreserve.begin(), indicesToPreserve.end());

	// https://stackoverflow.com/questions/11017200/efficiently-initialise-stdset-with-a-sequence-of-numbers
	// https://stackoverflow.com/questions/18295333/how-to-efficiently-insert-a-range-of-consecutive-integers-into-a-stdset
	// https://stackoverflow.com/questions/17694579/use-stdfill-to-populate-vector-with-increasing-numbers
	std::set<int> s;   // numbers from 0 to the maximum index
	for (int i = 0; i < cloud->points.size(); ++i)
		s.insert(s.end(), i);

	// Set difference https://stackoverflow.com/questions/283977/c-stl-set-difference
	std::set<int> indicesToDrop;   // set
	std::set_difference(s.begin(), s.end(), indicesToPreserve.begin(), indicesToPreserve.end(), std::inserter(indicesToDrop, indicesToDrop.end()));

	/*
	set<int>::iterator it;
	for (it = indicesToDrop.begin(); it!=indicesToDrop.end(); ++it){
		int ans = *it;
		cout << ans << " ";
	}
	std::cout << std::endl;
	*/

	return indicesToDrop;
}




bool PCLViewer::filterObjInputFileToOutputFileOnlyPreservingDesiredPoints(const QString inObjFilePath, const QString outObjFilePath, set<int> indicesToDrop)
{
	// Only (text) obj files, composed of one .obj, one .mtl, one .jpg (or other image format), in the same directory,
	// with the format shown below; v/vn/vt can be alternated (as in meshlab-saved objs):
	// vn 0.176953 0.979954 0.091527
	// vt 0.093332 0.970970
	// v -1.609500 -0.761175 1.618873
	// ......

	// Open the file, read it and, for each line, if it is v, cumulate, idem for vn and vt; continue till you find the usemtl material_0
	// Then, put NaN into the places we now we hve to drop.
	// Save all the vertices v vn vt , then append the rest of the file
	// Now use meshlab to clean

	//  hope that pcl reads text ply with v vn vt interleaved

	/*
	 mtllib onefile.mtl
	 v -1.59761941 -0.76269394 1.61217582
	 v -1.60731614 -0.76151156 1.62123978
	 v -1.60021901 -0.76124680 1.62986410
	 ...
	 v -10.20813751 1.51792002 5.14335012
	 v -10.21420574 1.52048337 5.14066648
	 vn 0.06178573 0.99799657 -0.01361380
	 vn 0.09502557 0.99503791 -0.02949069
	 vn -0.12893073 0.98445237 -0.11929104
	 ...
	 vn 0.45409283 0.86798728 -0.20099191
	 vt 0.0933786 0.970284
	 vt 0.0933455 0.971181
	 vt 0.0935221 0.971948
	 ...
	 vt 0.814902 0.813738
	 usemtl material

	 f 1/1/1 2/2/2 3/3/3
	 f 1/1/1 4/4/4 2/2/2
	 f 5/5/5 6/6/6 7/7/7
	 ...
	 f 151042/151042/151042 151048/151048/151048 151047/151047/151047
	*/

	/*
	newmtl material
	Ka 0.2 0.2 0.2
	Kd 1 1 1
	Ks 0 0 0
	Tr 1
	illum 1
	Ns 50
	map_Kd onefile.jpg
	*/


	bool abort = false;

	// Open the input obj file
	QFile inObjFile(inObjFilePath);
	if (!inObjFile.open(QFile::ReadOnly | QFile::Text)) {
		QMessageBox::information(this, tr("Unable to open input obj file"), inObjFile.errorString());
		abort = true;
	}

	/*
	// Take the path of the input file, to get the mtl file
	// http://doc.qt.io/qt-5/qfileinfo.html
	// C:\pippo\x.txt.mtl gives:

	// QFileInfo outObjFileInfo(outObjFilePath);
	// QString outObjBaseName         = outObjFileInfo.baseName();   // filename (no path) without extensionS (all after the first .)       x
	// QString outObjfileName         = outObjFileInfo.fileName();   // name of the file, excluding the path (extensions are included)      x.txt.mtl
	// QString outObjAbsolutePath     = outObjFileInfo.absolutePath();   // path without the filename          C:\pippo\
	// QString outObjAbsoluteFilePath = outObjFileInfo.absoluteFilePath();   // path including the filename (i.e. everything?)      C:\pippo\x.txt.mtl
	// QString outObjCompleteBaseName = outObjFileInfo.completeBaseName();   // filename (no path) without the last extension (all after the last .)     x.txt
	// QString outObjCompleteSuffix   = outObjFileInfo.completeSuffix();   // all characters in the file after (but not including) the first '.'   txt.mtl
	*/

	// Open the output obj file
	QFile outObjFile(outObjFilePath);
	if (!outObjFile.open(QFile::WriteOnly | QFile::Text)) {
		QMessageBox::information(this, tr("Unable to open output obj file"), outObjFile.errorString());
		abort = true;
	}

	if (abort)
	{
		if (inObjFile.isOpen())
			inObjFile.close();
		if (outObjFile.isOpen())
			outObjFile.close();
		return false;
	}




	//        folder_name.append(".files");  // "jpg.files"

	//        QDir tmp_dir(fi.dir());// "c:\\myFolder\\"
	//        tmp_dir.mkdir(folder_name); // "c:\\myFolder\\jpg.files\\"

	//    QString inObjFileName = inObjFile.fileName();
	//    QString inObjFileDir = inObjFile.


	// Prepare the output file
	QTextStream outObjStream(&outObjFile);

	// Now scan the input obj file

	int vLine = -1;   // will be the vertex number (during iteration)
	int vnLine = -1;   // will be the vertex number (during iteration)
	int vtLine = -1;   // will be the vertex number (during iteration)

	// Prepare an iterator to scan the indices to be dropped
	set<int>::iterator it = indicesToDrop.begin();

	// Prepare a vector containing vertex translation after vertex dropping
	vector<int> newValuesOfIndices;
	newValuesOfIndices.resize(cloud->points.size());

	// Prepare a counter of the offset to subtract to indexes because of vertex deletion
	unsigned int currentNumberOfDeletedIndices = 0;

	// According to
	// http://myfavoritebug.blogspot.it/2015/05/qt-reading-text-files-with-or-without.html
	// I am reading lines like in "case 1" because it seems to optimize speed, but as then I am converting to string,
	// it should be demonstrated that this is better than directly reading by QTextStream

	// I've mixed Qt stuff with non-Qt (strings, files...) for historical reasons but I really should clean up the code!

	// In what follows, v MUST precede vt and vn because it is used to set "flag", which is also used by nv and vt

	while (!inObjFile.atEnd())
	{
		QByteArray qBA = inObjFile.readLine();
		QString qLine(qBA);
		string line = qLine.toStdString();

		if (!line.empty())
		{
			// get keyword
			string firstWord = line.substr(0, line.find(" "));
			if (firstWord == "v")
			{
				vLine++;
				if (it != indicesToDrop.end() && vLine == *it)  // only if it is not at indicesToDrop.end(), check if this is a vertex to be dropped (and update the indices iterator)
				{
					currentNumberOfDeletedIndices++;
					it++;
					newValuesOfIndices[vLine] = -1;
				}
				else
				{
					outObjStream << qLine;
					newValuesOfIndices[vLine] = vLine - currentNumberOfDeletedIndices + 1;
				}
			}
			else if (firstWord == "vn")
			{
				vnLine++;
				if (newValuesOfIndices[vnLine] != -1)
					outObjStream << qLine;
			}
			else if (firstWord == "vt") {
				vtLine++;
				if (newValuesOfIndices[vtLine] != -1)
					outObjStream << qLine;
			}
			else if (firstWord == "f")
			{

				// la linea letta dal filecontiene, oltre al comando f, tre parole del tipo x/y/z (ossia f a/b/c d/e/f g/h/i)
				// dove a b c etc sono numeri interi positivi (indice v, indice vn, indice vt dei tre vertici del triangolo)
				// f 13499/13499/8753 13483/13483/8741 13505/13505/8756

				istringstream iss(line);
				string w0, w1, w2, w3, w;
				iss >> w0 >> w1 >> w2 >> w3;  // w0 is "f", w1 is "a/b/c", w2 is "d/e/f", w3 is "g/h/i"
				istringstream w1ss(w1), w2ss(w2), w3ss(w3);   // each contains aa/aa/aa
				//                    cout << w1 << " - " << w2 << " - " << w3 << " - " << endl;

				// now split the three strings of type aa/aa/aa
				// https://stackoverflow.com/questions/5757721/use-getline-and-while-loop-to-split-a-string

				char splitChar = '/';
				vector<unsigned int> n0, n1, n2;
				unsigned int n;

				string each;

				// first triple aa/aa/aa
				getline(w1ss, each, splitChar);
				stringstream(each) >> n;
				if (newValuesOfIndices[n - 1] == -1) continue;
				n0.push_back(newValuesOfIndices[n - 1]);

				getline(w1ss, each, splitChar);
				stringstream(each) >> n;
				if (newValuesOfIndices[n - 1] == -1) continue;
				n0.push_back(newValuesOfIndices[n - 1]);

				getline(w1ss, each, splitChar);
				stringstream(each) >> n;
				if (newValuesOfIndices[n - 1] == -1) continue;
				n0.push_back(newValuesOfIndices[n - 1]);

				// second triple aa/aa/aa
				getline(w2ss, each, splitChar);
				stringstream(each) >> n;
				if (newValuesOfIndices[n - 1] == -1) continue;
				n1.push_back(newValuesOfIndices[n - 1]);

				getline(w2ss, each, splitChar);
				stringstream(each) >> n;
				if (newValuesOfIndices[n - 1] == -1) continue;
				n1.push_back(newValuesOfIndices[n - 1]);

				getline(w2ss, each, splitChar);
				stringstream(each) >> n;
				if (newValuesOfIndices[n - 1] == -1) continue;
				n1.push_back(newValuesOfIndices[n - 1]);

				// third triple aa/aa/aa
				getline(w3ss, each, splitChar);
				stringstream(each) >> n;
				if (newValuesOfIndices[n - 1] == -1) continue;
				n2.push_back(newValuesOfIndices[n - 1]);

				getline(w3ss, each, splitChar);
				stringstream(each) >> n;
				if (newValuesOfIndices[n - 1] == -1) continue;
				n2.push_back(newValuesOfIndices[n - 1]);

				getline(w3ss, each, splitChar);
				stringstream(each) >> n;
				if (newValuesOfIndices[n - 1] == -1) continue;
				n2.push_back(newValuesOfIndices[n - 1]);

				line = "f " +
					to_string(n0[0]) + "/" + to_string(n0[1]) + "/" + to_string(n0[2]) + " " +
					to_string(n1[0]) + "/" + to_string(n1[1]) + "/" + to_string(n1[2]) + " " +
					to_string(n2[0]) + "/" + to_string(n2[1]) + "/" + to_string(n2[2]);
				qLine = QString::fromStdString(line);
				outObjStream << qLine << endl;

			}
			else
				//  if (firstWord == "#" || firstWord == "vn" || firstWord == "vt" || firstWord == "g" || firstWord == "o" || firstWord == "mtllib" || firstWord == "usemtl" || firstWord == "f")
				outObjStream << qLine;
		}  // if ! line.empty

	}  // while

	inObjFile.close();
	outObjFile.close();

	/*
	// Now call meshlabserver to get rid of NaNs
	string command = " \"c:\\Program Files\\VCG\\MeshLab\\meshlabserver.exe\" -i \" " + outObjFilePath.toStdString() + "\" -o \"" + outObjFilePath.toStdString() + "-meshlab.obj" + " \" -m vn vt wt";
	//                  "c:\Program Files\VCG\MeshLab\meshlabserver.exe" -i " xxxxxxxxxx" .o "yyyyyyyyy-meshlab.obj " -m vn vt wt
	cout << "Executing: \n" << command << std::endl;
	if (executeCommand(command)) {

	}
*/

	return true;
}








///////////////////////////////////////////////////////////////////////////////////////////////////////


void PCLViewer::on_pushButton_Restore_clicked()
{

	cout << "on_pushButton_Restore_clicked\n";

	for (int c = 0; c < clusters.size(); c++)
		if (deletedClustersFlags[c])   // the cluster had been logically deleted
		{
			for (int k = 0; k < clusters[c].indices.size(); k++)  // for each point in the cluster, color with the original color
				labeledCloud->points[clusters[c].indices[k]].rgb = clusterColors[c];
			deletedClustersFlags[c] = false;  // the cluster is now not logically deleted
		}

	// refresh the point cloud on screen
	viewer->updatePointCloud(labeledCloud, "cloud");
	ui->qvtkWidget->update();

}

///////////////////////////////////////////////////////////////////////////////////////////////////////
void PCLViewer::on_pushButton_LoadCloud_clicked()
{

	FileName = QFileDialog::getOpenFileName(this, tr("Open 3D Model File"),
		CurrentDirectory,
		tr("3D point cloud (*.pcd *.ply *.obj)"));
	if (FileName.isNull())   // or isEmpty()
		return;
	else {
		if (loadCloud(FileName.toStdString())) {
			clearViewer();
			//          viewer->updatePointCloud (cloud, "cloud");  // instead of using this update, I am deleting (in clearViewer) and then adding, because updating is not allowed for texture polygons
			viewer->addPointCloud(cloud, "cloud");
			// viewer->initCameraParameters ();
			viewer->resetCamera();    // Necessary to let the model immediately appear without "touching" with the mouse left button
			ui->qvtkWidget->update();
			CurrentDirectory = FileName;  // update current directory on the latest loaded file
		}
		else
			QMessageBox::information(this, tr("Unable to open file"), "");
	}
}


bool PCLViewer::loadCloud(const string& FileNameStr)
{
	cout << "PCLViewer::loadCloud starting\n";
	bool retValue = true;
	string ext = GetFileExtension(FileNameStr);
	if (ext == "")
		retValue = false;
	if (ext == "pcd")
		if (pcl::io::loadPCDFile(FileNameStr, *cloud) < 0)
			retValue = false;
	if (ext == "ply")
		if (pcl::io::loadPLYFile(FileNameStr, *cloud) < 0)
			retValue = false;
	if (ext == "obj")
		if (pcl::io::loadOBJFile(FileNameStr, *cloud) < 0)
			retValue = false;
	if (cloud->empty())	retValue = false;
	else {
		// these instructions were added to convert the loaded cloud to a PointCloud2 object,  cloud_blob
		//     pcl::toROSMsg(*cloud, *cloud_blob);
	}

	if (!retValue) {
		std::cerr << "Error loading point cloud " << FileNameStr << std::endl << std::endl;
		//   showHelp(argv[0]);
	}
	cout << "PCLViewer::loadCloud ending with retVal = " << retValue << std::endl;
	return retValue;
}

// https://stackoverflow.com/questions/51949/how-to-get-file-extension-from-string-in-c/51993#51993
// this solution will always return the extension even on strings like "this.a.b.c.d.e.s.mp3" if it cannot find the extension it will return "".
string PCLViewer::GetFileExtension(const string& FileNameStr)
{
	if (FileNameStr.find_last_of(".") != string::npos)
		return FileNameStr.substr(FileNameStr.find_last_of(".") + 1);
	return "";
}


///////////////////////////////////////////////////////////////////////////////////////////////////////
void PCLViewer::on_pushButton_LoadTextureMesh_clicked()
{

	FileName = QFileDialog::getOpenFileName(this, tr("Open 3D Model File"),
		CurrentDirectory,
		tr("3D mesh (*.obj)"));
	if (FileName.isNull())   // or isEmpty()
		return;
	else {
		clearViewer();
		if (loadTextureMesh(FileName.toStdString())) {
			//              viewer->updatePolygonMesh(textureMesh, "textureMesh");
			viewer->addTextureMesh(textureMesh, "textureMesh");
			//        viewer->initCameraParameters ();
			viewer->resetCamera();    // Necessary to let the model immediately appear without touching with the mouse
			ui->qvtkWidget->update();
			CurrentDirectory = FileName;  // update current directory on the latest loaded file
		}
		else
			QMessageBox::information(this, tr("Unable to open file"), "");
	}
}



bool PCLViewer::loadTextureMesh(const string& FileNameStr)
{
	cout << "PCLViewer::loadTextureMesh starting\n";
	bool retVal = true;
	// http://docs.pointclouds.org/trunk/group__io.html#ga8382b39bb66ababde45053f67d3c5da3
	//    retVal = pcl::io::load(FileNameStr,  mesh);
	retVal = pcl::io::loadPolygonFileOBJ(FileNameStr, textureMesh);
	// https://github.com/PointCloudLibrary/pcl/issues/1169
	pcl::TextureMesh mesh2;
	pcl::io::loadOBJFile(FileNameStr, mesh2);
	textureMesh.tex_materials = mesh2.tex_materials;

	cout << "PCLViewer::loadTextureMesh ending with retVal = " << retVal << std::endl;
	return retVal;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
void PCLViewer::on_pushButton_LoadMesh_clicked()
{

	FileName = QFileDialog::getOpenFileName(this, tr("Open 3D Model File"),
		CurrentDirectory,
		tr("3D mesh (*.obj)"));
	if (FileName.isNull())   // or isEmpty()
		return;
	else {
		resetAll();   // set some buttons to disabled
		string FileNameStr = FileName.toStdString();   // QString to string
		extractPointClouds(FileNameStr); // create FileNameStr + "_pc_sparse.ply" and FileNameStr + "_pc_dense.ply" point clouds
		clearViewer();
		clusters.clear();  // Initially no clusters exist, so no selected, and no deleted
		selectedClustersFlags.clear();
		deletedClustersFlags.clear();
		on_buttonClicked_LoadSparseOrDenseCloud(0);   // the 0 is fictitious, I don't use it! It is the button id that I don't check (the SLOT has this signature)
		viewer->resetCamera();    // Necessary to let the model immediately appear without "touching" with the mouse left button
	}
}

/*
bool PCLViewer::loadMesh(const string& FileNameStr)
{
	cout << "PCLViewer::loadMesh starting\n";
	bool retVal = true;
	retVal = pcl::io::loadPolygonFileOBJ(FileNameStr,  mesh);
	cout << "PCLViewer::loadMesh ending with retVal = " << retVal << std::endl;
	return retVal;
}
*/

///////////////////////////////////////////////////////////////////////////////////////////////////////
void PCLViewer::clearViewer()
{
	//viewer->removePolygonMesh("textureMesh");  // useless
	//viewer->removePolygonMesh("mesh");  // useless
	viewer->removeAllPointClouds();  // enough to also delete meshes
	viewer->removeAllShapes();  // to remove the selection cubes
	//viewer->resetCamera ();    // Necessary to let the change immediately appear without touching with the mouse
	//ui->qvtkWidget->update ();  // it does not immediately work
}


///////////////////////////////////////////////////////////////////////////////////////////////////////
bool PCLViewer::extractPointClouds(const string& FileNameStr)
{
	//    https://stackoverflow.com/questions/2523765/qstring-to-char-conversion
	//    QByteArray ba = FileName.toLatin1();
	//    const char *FileNameCStr = ba.data();

	string command;  // remember to define a base CloudCompare string so that then I only add the changing part!!!!!!!

	// Convert the mesh to a (sparse) point cloud, with texture baked to vertice color (Meshlab for texture to vertices, then PCL load as point cloud, and save
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	command = "\"c:\\Program Files\\VCG\\MeshLab\\meshlabserver.exe\" -i \"" + FileNameStr + "\" -o \"" + FileNameStr + "_pc_sparse.ply\" -m vc vn -s texture_to_vertex.mlx";
	cout << "Executing: \n" << command << std::endl;
	if (executeCommand(command)) {
		if (pcl::io::loadPLYFile<pcl::PointXYZRGB>(FileNameStr + "_pc_sparse.ply", *cloud) == -1) // load the file
		{
			PCL_ERROR("Couldn't read file\n");
			return (false);
		}
		else
		{
			// Get rid of NaNs, because they can give problems in point picking with mouse (but there were no NaNs!)
			// https://github.com/daviddoria/Examples/blob/master/c%2B%2B/PCL/Filters/RemoveNaNFromPointCloud/RemoveNaNFromPointCloud.cpp
			// http://www.pcl-users.org/removeNaNFromPointCloud-does-not-seem-to-work-td4025886.html
			// http://www.pcl-users.org/Problem-with-picking-points-in-PCLVisualizer-td4019970.html
			// http://www.pcl-users.org/In-AreaPickingEvent-different-selected-points-td4041028.html
			// https://github.com/PointCloudLibrary/pcl/issues/575   see the check for NaNs by if !pcl_isfinite (cloud_in.points[indices[i]].x etc (but NaNs may also be in r g b etc)
			// Here http://www.pcl-users.org/RemoveNaNfromPointCloud-td4040656.html I see that I should use the pcd format!! To be tried!!!!
			std::cout << "size: " << cloud->points.size() << std::endl;
			PointCloudT::Ptr outputCloud(new PointCloudT);
			std::vector<int> indices;
			/*
			pcl::removeNaNFromPointCloud(*cloud, *outputCloud, indices);
			std::cout << "size: " << outputCloud->points.size () << std::endl;

			unsigned char r, g, b;
			float x, y, z;

			PointT p = outputCloud->points[1000];
			r = p.r;
			g = p.g;
			b = p.b;
			x = p.x;
			y = p.y;
			z = p.z;

			cout << "A point: " << x << " " << y << " " << z << " " << (unsigned int) r << " " << (unsigned int)g << " " << (unsigned int)b << std::endl;

			pcl::io::savePLYFileBinary (FileNameStr + "_pc_sparse.ply", *outputCloud); //* save the file (only XYZRGB, so THIS IS A POINT CLOUD!)
			*/
			pcl::io::savePLYFileBinary(FileNameStr + "_pc_sparse.ply", *cloud); //* save the file (only XYZRGB, so THIS IS A POINT CLOUD!)
		}
	}    // if

	// REMIND REMIND REMIND REMIND!!!!!!!!!!!!!!!!!!!!!!!!!!! TO DO!!!!!!!!!!
	// REMIND REMIND REMIND REMIND!!!!!!!!!!!!!!!!!!!!!!!!!!! TO DO!!!!!!!!!!
	// REMIND REMIND REMIND REMIND!!!!!!!!!!!!!!!!!!!!!!!!!!! TO DO!!!!!!!!!!
	// REMIND REMIND REMIND REMIND!!!!!!!!!!!!!!!!!!!!!!!!!!! TO DO!!!!!!!!!!
	// REMIND REMIND REMIND REMIND!!!!!!!!!!!!!!!!!!!!!!!!!!! TO DO!!!!!!!!!!
	// REMIND REMIND REMIND REMIND!!!!!!!!!!!!!!!!!!!!!!!!!!! TO DO!!!!!!!!!!
	// REMIND REMIND REMIND REMIND!!!!!!!!!!!!!!!!!!!!!!!!!!! TO DO!!!!!!!!!!
	// REMIND REMIND REMIND REMIND!!!!!!!!!!!!!!!!!!!!!!!!!!! TO DO!!!!!!!!!!
	// REMEMBER to delete old point clouds if present (I could also leave them there and check if present (then skipping the next lines): but what if I change anything with an external program? allineation problem)
	// I must either delete or check: if I leave them there there is a problem with CC checking and immediately finding the dense cloud


	// NO MORE DENSE CLOUD!!!!!!!!!!!!! COMMENTING OUT THE NEXT PART!!!

	/*

#if 0   // unfortunately this works well from meshlab GUI, but meshlabserver crashes!
	// Convert the mesh to a (dense) point cloud, with texture baked to vertice color (Meshlab for texture to vertices, and texel sampling
	command = "\"c:\\Program Files\\VCG\\MeshLab\\meshlabserver.exe\" -i \"" + FileNameStr + "\" -o \"" + FileNameStr + "_pc_dense.ply\" -m vc vn -s texel_sampling.mlx";
	cout << "Executing: \n" << command << std::endl;
	executeCommand(command);
#else   // Use CloudCompare, see http://www.cloudcompare.org/doc/wiki/index.php?title=Command_line_mode
	// Remember that with the -SILENT option, CC works in background!!! I must understand if the program finished working
	// Remember that the -SILENT option must be the first parameter; also the -NO_TIMESTAMP option has its place (if put later in the command it does not work)
	// Also remember that I can use the -EXTRACT_VERTICES command to build a sparse cloud
	 // Convert the mesh to a (dense) point cloud, with texture baked to vertice color (CloudCompare)
	// "C:\Program Files\CloudCompare\CloudCompare" -SILENT -O onefile.obj -NO_TIMESTAMP -C_EXPORT_FMT PLY -PLY_EXPORT_FMT BINARY_LE  -SAMPLE_MESH DENSITY 100000
	// The created filename is onefile_SAMPLED_POINTS.ply
	command = "\"c:\\Program Files\\CloudCompare\\CloudCompare\" -SILENT -O \"" + FileNameStr + "\" -NO_TIMESTAMP -C_EXPORT_FMT PLY -PLY_EXPORT_FMT BINARY_LE  -SAMPLE_MESH DENSITY 100000";
	cout << "Executing: \n" << command << std::endl;
	executeCommand(command);
	// wait until the file is created
	// https://stackoverflow.com/questions/6417817/easy-way-to-remove-extension-from-a-filename
	string rawPlyFileName = FileNameStr.substr(0, FileNameStr.find_last_of("."));
	string plyFileName = rawPlyFileName + "_SAMPLED_POINTS.ply";
	ifstream myfile;
	do {       // no timeout check!
		myfile.open(plyFileName);
	} while (!myfile.is_open());
	cout << "File created\n";  // Now I am sure that XXX_SAMPLED_POINTS.ply has been created from XX.obj
	myfile.close();
	string newPlyFileName = FileNameStr + "_pc_dense.ply";  // rename with a name we like
	rename((const char *) plyFileName.c_str(), (const char *) newPlyFileName.c_str());

#endif
*/

	return(true);
}



// https://msdn.microsoft.com/en-us/library/windows/desktop/ms682512(v=vs.85).aspx
bool PCLViewer::executeCommand(const string& command)
{
	STARTUPINFO si;
	PROCESS_INFORMATION pi;

	ZeroMemory(&si, sizeof(si));
	si.cb = sizeof(si);
	ZeroMemory(&pi, sizeof(pi));

	// Start the child process.
	if (!CreateProcess(NULL,   // No module name (use command line)
		LPSTR(command.c_str()),        // Command line
		NULL,           // Process handle not inheritable
		NULL,           // Thread handle not inheritable
		FALSE,          // Set handle inheritance to FALSE
		0,              // No creation flags
		NULL,           // Use parent's environment block
		NULL,           // Use parent's starting directory
		&si,            // Pointer to STARTUPINFO structure
		&pi)           // Pointer to PROCESS_INFORMATION structure
		)
	{
		cout << "CreateProcess failed (" << GetLastError() << ")" << std::endl;
		return false;
	}

	// Wait until child process exits.
	WaitForSingleObject(pi.hProcess, INFINITE);

	// Close process and thread handles.
	CloseHandle(pi.hProcess);
	CloseHandle(pi.hThread);

	return true;
}


// http://doc.qt.io/archives/qt-4.8/qbuttongroup.html#signals
void PCLViewer::on_buttonClicked_LoadSparseOrDenseCloud(int id)
{
	cout << "PCLViewer::on_buttonClicked_LoadSparseOrDenseCloud starting\n";

	if (FileName.isNull())   // or isEmpty()
		return;
	else {
		string FileNameStr = FileName.toStdString();
		// https://stackoverflow.com/questions/25280146/do-i-need-to-check-one-by-one-to-know-which-radiobutton-is-checked-in-a-group-in
		// https://wiki.qt.io/How_to_Use_QPushButton
		// http://doc.qt.io/archives/qt-4.8/qbuttongroup.html#checkedButton
		QString checkedButtonName = ui->buttonGroup_MeshToPointCloud->checkedButton()->objectName();
		if (checkedButtonName == "radioButton_Dense")
			FileNameStr = FileNameStr + "_pc_dense.ply";
		else if (checkedButtonName == "radioButton_Sparse")
			FileNameStr = FileNameStr + "_pc_sparse.ply";

		if (loadCloud(FileNameStr))
		{
			//          viewer->updatePointCloud (cloud, "cloud");  // instead of using this update, I am deleting (in clearViewer) and then adding, because updating is not allowed for texture polygons
			viewer->removePointCloud("cloud");  // why? I did a clearViewer!
			viewer->addPointCloud(cloud, "cloud");


			//////////////////////////////////
			// This loop was used to check if the format of the cloud in memory is the same as the obj or ascii ply on disk, as to the order of vertices, and it is.
			//     for (int i = 0; i < 100; i++)
			//    {
			//       cout << cloud->points[i].x << " " << cloud->points[i].y << " " << cloud->points[i].z << std::endl;
			//  }

			//////////////////////////////////


			// viewer->initCameraParameters ();
			//       viewer->resetCamera ();    // Necessary to let the model immediately appear without "touching" with the mouse left button
			ui->qvtkWidget->update();
			CurrentDirectory = FileName;  // update current directory on the latest loaded file

			on_pushButton_ClearSegmentation_clicked();

			// Enable segmentation
			ui->pushButton_StartSegmentation->setEnabled(true);

		}
		else
			QMessageBox::information(this, tr("Unable to open file"), "");
	}
	cout << "PCLViewer::on_buttonClicked_LoadSparseOrDenseCloud ending\n";
}





void PCLViewer::on_buttonClicked_ShowOriginalOrLabelColors(int id)
{
	cout << "PCLViewer::on_buttonClicked_ShowOriginalOrLabelColors starting\n";

	if (FileName.isNull())   // or isEmpty()
		return;
	else {
		// viewer->updatePointCloud (cloud, "cloud");  // instead of using this update, I am deleting (in clearViewer) and then adding, because updating is not allowed for texture polygons
		viewer->removePointCloud("cloud");  // why? I did a clearViewer!
		QString checkedButtonName = ui->buttonGroup_DisplayPreferences->checkedButton()->objectName();
		if (checkedButtonName == "radioButton_ShowOriginalColors")
			viewer->addPointCloud(cloud, "cloud");
		else if (checkedButtonName == "radioButton_ShowClusters")
			viewer->addPointCloud(labeledCloud, "cloud");

		// viewer->initCameraParameters ();
		//       viewer->resetCamera ();    // Necessary to let the model immediately appear without "touching" with the mouse left button
		ui->qvtkWidget->update();
	}
	cout << "PCLViewer::on_buttonClicked_ShowOriginalOrLabelColors ending\n";
}



/*

if (loadMesh(FileNameStr)) {
//              viewer->updatePolygonMesh(mesh, "mesh");
	viewer->addPolygonMesh(mesh, "mesh");
//        viewer->initCameraParameters ();
	viewer->resetCamera ();    // Necessary to let the model immediately appear without touching with the mouse
	ui->qvtkWidget->update ();
	CurrentDirectory = FileName;  // update current directory on the latest loaded file
}
*/




/*
pcl::PolygonMesh mesh;
  pcl::io::loadPolygonFileVTK (mesh_file_vtk_, mesh);
  // Save ascii
  pcl::io::savePLYFile ("test_mesh_ascii.ply", mesh);
  // Save binary
  pcl::io::savePLYFileBinary ("test_mesh_binary.ply", mesh);
  // Load both with vtk ply parser
  pcl::PolygonMesh mesh_ascii_vtk;
  pcl::io::loadPolygonFilePLY ("test_mesh_ascii.ply", mesh_ascii_vtk);
  pcl::PolygonMesh mesh_binary_vtk;
  pcl::io::loadPolygonFilePLY ("test_mesh_binary.ply", mesh_binary_vtk);
  // Load both with pcl ply parser
  pcl::PolygonMesh mesh_ascii_pcl;
  pcl::io::loadPLYFile ("test_mesh_ascii.ply", mesh_ascii_pcl);
  pcl::PolygonMesh mesh_binary_pcl;
  pcl::io::loadPLYFile ("test_mesh_binary.ply", mesh_binary_pcl);
 * */




void PCLViewer::on_pushButton_SegmResetDefaults_clicked()
{
	switch (ui->tabWidget_Segmentation->currentIndex())
	{
	case 0: // CloudCompare connected components
		break;
	case 1:  // Region Growing
		segmRGResetDefaults();
		break;
	case 2: // Hyper Clustering
		break;
	case 3: // LCCP
		segmLCCPResetDefaults();
		break;
	case 4: // Columns
		break;
	}
}

void PCLViewer::segmResetAllDefaults()
{
	segmRGResetDefaults();
	segmLCCPResetDefaults();
}

void PCLViewer::segmRGResetDefaults()
{
	ui->lineEdit_neKSearch->setText("50");
	ui->lineEdit_rgMinClusterSize->setText("200");
	ui->lineEdit_rgMaxClusterSize->setText("10000000");
	ui->lineEdit_rgNumberOfNeighbours->setText("80");
	ui->lineEdit_rgSmoothnessThreshold->setText("150");
	ui->lineEdit_rgCurvatureThreshold->setText("0.1");
}

void PCLViewer::segmLCCPResetDefaults()
{
	ui->lineEdit_LCCPVoxRes->setText("0.08");
	ui->lineEdit_LCCPSeedRes->setText("0.2");
	ui->lineEdit_LCCPcolorImport->setText("0.0");
	ui->lineEdit_LCCPspatialImport->setText("4.0");
	ui->lineEdit_LCCPnormalImport->setText("2.0");
	ui->checkBox_LCCPsingleCam->setChecked(false);
	ui->checkBox_LCCPsupervoxelRef->setChecked(false);
	ui->lineEdit_LCCPconcavToler->setText("1");
	ui->lineEdit_LCCPsmoothThr->setText("0.8");
	ui->lineEdit_LCCPminSegmSz->setText("100");
	ui->checkBox_LCCPextendConvex->setChecked(false);
	ui->checkBox_LCCPsanityCrit->setChecked(false);
}



void PCLViewer::on_pushButton_StartSegmentation_clicked()
{

	//   pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloud (new pcl::PointCloud<pcl::PointXYZ>);
	//   // https://answers.ros.org/question/9515/how-to-convert-between-different-point-cloud-types-using-pcl/
	//   copyPointCloud(*cloud, *xyzCloud);


	// WHAT FOLLOWS IS NOT IMPLEMENTED YET!!!!!!!!!!!!!!!!!!!!!
	// WHAT FOLLOWS IS NOT IMPLEMENTED YET!!!!!!!!!!!!!!!!!!!!!
	// WHAT FOLLOWS IS NOT IMPLEMENTED YET!!!!!!!!!!!!!!!!!!!!!
	// WHAT FOLLOWS IS NOT IMPLEMENTED YET!!!!!!!!!!!!!!!!!!!!!
	// WHAT FOLLOWS IS NOT IMPLEMENTED YET!!!!!!!!!!!!!!!!!!!!!

	// Input filtering: if there is any selected cluster, pass it as the parameter to the segmentation algorithm, not the whole cloud
	// otherwise pass the whole cloud

	// When back from the segmentation function,
	//   0) back-conversion of indices
	//   1) insert the colored cloud into labeledCloud, considering the indices back-conversion (so it gets to the right place)
	//   2) drop the clusters used for segmentation
	//   3) insert the newly created clusters, remembering to correctly convert the indices 
	//   4) don't forget to adapt clusterColors!!

	/////////////////////////////////////////////////////////////////

	PointCloudT::Ptr selectedCloud(new PointCloudT);
	selectedCloud = extractSelectedClustersFromCloud();  // XYZRGB, extracted from the original
	if (selectedCloud->points.size() == 0)
		pcl::copyPointCloud(*cloud, *selectedCloud);  // copy http://www.pcl-users.org/How-to-copy-a-PointCloud-td3718701.html
	
	on_pushButton_DeselectAll_clicked();

	bool badInput = false;   // error checking in data entry

	// https://stackoverflow.com/questions/7663709/convert-string-to-int-c
	switch (ui->tabWidget_Segmentation->currentIndex())
	{
	case 0: // CloudCompare connected components
	{
		QMessageBox::information(this, tr("Not implemented"), tr("Sorry, this feature is not (yet?) implemented!"));
		return;

		cout << "CloudCompare connected components\n";
		string FileNameStr = FileName.toStdString();

		// Connected components; http://www.cloudcompare.org/doc/wiki/index.php?title=Command_line_mode  octree level and minimum number of points per cc!
		// Spostare la segmentazione su uno dei tab della parte specifica di segmentazione.
		// Prima di lanciare tutto il processamento, copiare la mesh in una regione temporanea, cosi' poi tutti i risultati sono li' dentro, alla fine copiare solo il risultato finale nella directory originale!!
		// Credo che le connected components piu' piccole siano gettate via, quindi non devo mettere un numero troppo piccolo!!!!

		// CC puo' fare -C2MD_IST (cloud to mesh distance)  http://www.cloudcompare.org/doc/wiki/index.php?title=Command_line_mode#Cloud-to-mesh_distance
		// http://www.cloudcompare.org/doc/wiki/index.php?title=Cloud-to-Mesh_Distance

		// CC Puo' fare -EXTRACT_VERTICES {method} {parameter}   VEDERE!!!!!

		// CC vedere le funzioni di calcolo densita'

		// CC vedere -SOR (rimuove outlier)

		// Per passare da cloud a mesh usare sicuramente meshlab!

		//            string newPlyFileName = FileNameStr + "_pc_dense.ply";
		string newPlyFileName = FileNameStr + "_pc_sparse.ply";
		string command = "\"c:\\Program Files\\CloudCompare\\CloudCompare\" -SILENT -O \"" + newPlyFileName + "\" -NO_TIMESTAMP -C_EXPORT_FMT PLY -PLY_EXPORT_FMT ASCII  -EXTRACT_CC 8 100";
		//            string command = "\"c:\\Program Files\\CloudCompare\\CloudCompare\" -SILENT -O \"" + newPlyFileName + "\" -NO_TIMESTAMP -C_EXPORT_FMT PLY -PLY_EXPORT_FMT BINARY_LE  -EXTRACT_CC 8 100";
		cout << "Executing: \n" << command << std::endl;
		executeCommand(command);
		ifstream myfile;
		//            string componentPlyFileName = FileNameStr + "_pc_dense_COMPONENT_1.ply";  // try an open one of the components, to know if processing is over (hoping all the files are saved at the end)
		string componentPlyFileName = FileNameStr + "_pc_sparse_COMPONENT_1.ply";  // try an open one of the components, to know if processing is over (hoping all the files are saved at the end)
		do {       // no timeout check!
			myfile.open(componentPlyFileName);
		} while (!myfile.is_open());
		cout << "Components saved\n";
		myfile.close();

		break;
	}
	case 1: // Region Growing
	{
		cout << "Region Growing Segmentation\n";
		int neKSearch, rgMinClusterSize, rgMaxClusterSize, rgNumberOfNeighbours;
		float rgSmoothnessThreshold, rgCurvatureThreshold;

		try
		{
			neKSearch              = stoi(ui->lineEdit_neKSearch->text().toStdString());
			rgMinClusterSize       = stoi(ui->lineEdit_rgMinClusterSize->text().toStdString());
			rgMaxClusterSize       = stoi(ui->lineEdit_rgMaxClusterSize->text().toStdString());
			rgNumberOfNeighbours   = stoi(ui->lineEdit_rgNumberOfNeighbours->text().toStdString());
			rgSmoothnessThreshold  = stof(ui->lineEdit_rgSmoothnessThreshold->text().toStdString());
			rgCurvatureThreshold   = stof(ui->lineEdit_rgCurvatureThreshold->text().toStdString());
		}
		catch (...) {
			badInput = true;
			cout << "Invalid input. Please try again!\n";
			QMessageBox::information(this, tr("Error in input parameters"), tr("Sorry, some parameter is not correct!"));
		}

		if (!badInput)
		{
			CSegmenterRegionGrowing seg(selectedCloud, neKSearch, rgMinClusterSize, rgMaxClusterSize, rgNumberOfNeighbours, rgSmoothnessThreshold, rgCurvatureThreshold);
			labeledCloud = seg.getOutputCloud();
			////////reg = seg.getOutputRegionGrowing();
			clusters = seg.getOutputClusters();
		}
		
		break;
	}
	case 2: // Hyper Clustering
	{
		cout << "Hyper Clustering Segmentation\n";
		QMessageBox::information(this, tr("Not implemented"), tr("Sorry, this feature is not (yet?) implemented!"));
		return;
		break;
	}
	case 3: // LCCP
	{
		cout << "LCCP Segmentation\n";

		float voxel_resolution, seed_resolution, color_importance, spatial_importance, normal_importance, concavity_tolerance_threshold, smoothness_threshold;
		bool use_single_cam_transform, use_supervoxel_refinement, use_extended_convexity, use_sanity_criterion;
		uint32_t min_segment_size;

		try
		{
			// Supervoxel stuff
			voxel_resolution = stof(ui->lineEdit_LCCPVoxRes->text().toStdString());;
			seed_resolution = stof(ui->lineEdit_LCCPSeedRes->text().toStdString());;
			color_importance = stof(ui->lineEdit_LCCPcolorImport->text().toStdString());;
			spatial_importance = stof(ui->lineEdit_LCCPspatialImport->text().toStdString());;
			normal_importance = stof(ui->lineEdit_LCCPnormalImport->text().toStdString());;
			use_single_cam_transform = ui->checkBox_LCCPsingleCam->isChecked();
			use_supervoxel_refinement = ui->checkBox_LCCPsupervoxelRef->isChecked();

			// LCCPSegmentation stuff
			concavity_tolerance_threshold = stof(ui->lineEdit_LCCPconcavToler->text().toStdString());;
			smoothness_threshold = stof(ui->lineEdit_LCCPsmoothThr->text().toStdString());;
			min_segment_size = stoi(ui->lineEdit_LCCPminSegmSz->text().toStdString());;
			use_extended_convexity = ui->checkBox_LCCPextendConvex->isChecked();
			use_sanity_criterion = ui->checkBox_LCCPsanityCrit->isChecked();
		}
		catch (...) {
			badInput = true;
			cout << "Invalid input. Please try again!\n";
			QMessageBox::information(this, tr("Error in input parameters"), tr("Sorry, some parameter is not correct!"));
		}

		if (!badInput)
		{
			CSegmenterLCCP seg(selectedCloud, voxel_resolution, seed_resolution, color_importance, spatial_importance, normal_importance,
				use_single_cam_transform, use_supervoxel_refinement, concavity_tolerance_threshold, smoothness_threshold, 
				min_segment_size, use_extended_convexity, use_sanity_criterion);
			labeledCloud = seg.getOutputCloud();
			clusters = seg.getOutputClusters();
		}

		break;
	}
	case 4: // Columns
	{
		cout << "Column Segmentation\n";
		CSegmenterColumns seg(selectedCloud);
		labeledCloud = seg.getOutputCloud();
		clusters = seg.getOutputClusters();
		break;
	}
	}   // switch


	if (!badInput)
	{

		// Fill the clusterColors structure with the colors of the found clusters
		// http://docs.pointclouds.org/1.7.0/structpcl_1_1_point_x_y_z_r_g_b.html
		// "Due to historical reasons (PCL was first developed as a ROS package), the RGB information is packed into an integer and casted to a float"

		// Moreover, deletedClustersFlags must be reset to all false

		// Fill the selectedClustersFlags and the deletedClustersFlags with "false"
		clusterColors.clear();
		deletedClustersFlags.clear();
		// selectedClustersFlags.clear();
		for (int c = 0; c < clusters.size(); c++)
		{
			PointT aPoint = labeledCloud->points[clusters[c].indices[0]];  // take the 1st point of the c-th cluster
			clusterColors.push_back(aPoint.rgb);
			deletedClustersFlags.push_back(false);
			selectedClustersFlags.push_back(false);
		}

		// Select the "label colors" state
		ui->radioButton_ShowClusters->setChecked(true);
		on_buttonClicked_ShowOriginalOrLabelColors(0);

		// Enable the buttons
		ui->pushButton_SelectAll->setEnabled(true);
		ui->pushButton_DeselectAll->setEnabled(true);
		ui->pushButton_DeleteSelected->setEnabled(true);
		ui->pushButton_Restore->setEnabled(true);
		ui->pushButton_PreserveSelected->setEnabled(true);
		ui->pushButton_Save->setEnabled(true);
		ui->pushButton_InvertSelection->setEnabled(true);

		ui->pushButton_ClearSegmentation->setEnabled(true);
	}
}






void PCLViewer::on_pushButton_ClearSegmentation_clicked()
{

	// initialize the labeled cloud with color
	copyPointCloud(*cloud, *labeledCloud);

	for (size_t i = 0; i < labeledCloud->points.size(); ++i)
	{
		labeledCloud->points[i].r = 255;
		labeledCloud->points[i].g = 255;
		labeledCloud->points[i].b = 255;
	}

	// A new mesh has been converted and the sparse cloud has been loaded. Now:

	// - build up a new cluster containing all the cloud point indices, and initialize the "clusters" variable member 
	// - initialize the "clusterColors" variable member (only white for the only cluster)
	// - initialize the deletedClustersFlags vector to one place, false
	// - initialize the selectedClustersFlags vector to one place, false

	// https://stackoverflow.com/questions/11017200/efficiently-initialise-stdset-with-a-sequence-of-numbers
	// https://stackoverflow.com/questions/18295333/how-to-efficiently-insert-a-range-of-consecutive-integers-into-a-stdset
	// https://stackoverflow.com/questions/17694579/use-stdfill-to-populate-vector-with-increasing-numbers
	pcl::PointIndices::Ptr piPtr(new pcl::PointIndices);   // da http://www.pcl-users.org/IndicesPtr-from-PointIndices-td4020356.html
	std::vector<int> v(cloud->points.size()); // vector with the same number of int as the cloud has points.
	std::iota(std::begin(v), std::end(v), 0); // Fill with 0, 1, ..., 99.
	piPtr->indices = v;
	clusters.push_back(*piPtr);

	clusterColors.clear();
	deletedClustersFlags.clear();
	selectedClustersFlags.clear();

	clusterColors.push_back(labeledCloud->points[0].rgb);    // Take the "white" color
	deletedClustersFlags.push_back(false);
	selectedClustersFlags.push_back(false);

	// Select the initial "original colors" state
	ui->radioButton_ShowOriginalColors->setChecked(true);
	on_buttonClicked_ShowOriginalOrLabelColors(0);

	ui->pushButton_ClearSegmentation->setEnabled(false);

}












/////////////////////////////////////////////////////////////////////////
/** Point picking callback. This gets called when the user selects
  * a 3D point on screen (in the PCLVisualizer window) using Shift+click.
  *
  * Input parameter is: event,  the event that triggered the call
  */
void PCLViewer::pp_callback(const pcl::visualization::PointPickingEvent& event, void* viewer_void)
{
	// Check to see if we got a valid point. Early exit.
	int idx = event.getPointIndex();
	if (idx == -1)
		return;
	float x, y, z;
	event.getPoint(x, y, z);
	PointT p = labeledCloud->points[idx];
	cout << "Picked point # " << idx << " at " << x << " " << y << " " << z;
	cout << " i.e. at " << p.x << " " << p.y << " " << p.z;
	cout << " RGB color is (" << (unsigned int)p.r << ", " << (unsigned int)p.g << ", " << (unsigned int)p.b << ")" << std::endl;

	// Add a sphere to it in the PCLVisualizer window   (from apps\src\ni_linemod.cpp
	//  stringstream ss;
	//  ss << "sphere_" << idx;
	//  viewer->addSphere (p, 0.01, 1.0, 1.0, 1.0, ss.str ());



	// From http://docs.pointclouds.org/1.7.0/classpcl_1_1_region_growing.html#aeb1ae724085424bc60bb4cf27ff27283
	// "If the cloud was successfully segmented, then function returns colored cloud. Otherwise it returns an empty pointer.
	// Points that belong to the same segment have the same color. But this function doesn't guarantee that different segments    <---- !!!!
	// will have different color(it all depends on RNG). Points that were not listed in the indices array will have red color"
	// So I need to use indices, not colors!!!!!
	// From the index of the clicked point I can find which is the cluster, so I get the indices of all the points of that cluster, so
	// I can drop all those points.
	// For indices, see
	// http://www.pointclouds.org/documentation/tutorials/extract_indices.php#extract-indices
	// http://www.pointclouds.org/documentation/tutorials/planar_segmentation.php#planar-segmentation

	// Visualize Bounding Box (GetMinMax3D: http://docs.pointclouds.org/1.7.0/group__common.html)




	// From the index of the clicked point, find the cluster it is in.
	// The variable member "clusters" contains the point indices per cluster
	// The variable member "deletedClustersFlags" tells if the custer was "deleted" so it must not be searched

	// See if this
	// http://docs.pointclouds.org/1.7.0/classpcl_1_1_region_growing.html#a72b71b0e82a8080dcd26142968eb21e3
	// void pcl::RegionGrowing< PointT, NormalT >::getSegmentFromPoint	(int index, pcl::PointIndices & cluster) can be useful:
	// For a given point this function builds a segment to which it belongs and returns this segment.

	bool found = false;
	int theCluster = -1;
	for (int j = 0; j < clusters.size() && !found; j++)  // for each cluster, but getting out when found!! Remember that break exits only the loop it is in!!!! Not the j loop!!!
		if (!deletedClustersFlags[j])
			for (int k = 0; k < clusters[j].indices.size(); k++)  // for each point in the cluster
				if (clusters[j].indices[k] == idx)
				{
					found = true;
					cout << "Cluster # " << j << std::endl;
					theCluster = j;
					break;
				}
	if (!found) return;

	cout << "Found the clicked point in cluster # " << theCluster << std::endl;

	// Check if the object is already selected: if it is, deselect it (and erase the cube) and clear it
	// in the selectedClustersFlags vector, otherwise select it (add a cube) and set true in selectedClustersFlags
	if (selectedClustersFlags[theCluster])    // also if (viewer->contains(ss.str()))
	{
		cout << "Already selected, deselecting the cluster to which the clicked point belongs, cluster size is " << clusters[theCluster].indices.size() << std::endl;
		deSelectACluster(theCluster);
	}
	else
	{
		cout << "Selecting the cluster to which the clicked point belongs, cluster size is " << clusters[theCluster].indices.size() << std::endl;
		selectACluster(theCluster);
	}

	cout << "Set selectedClustersFlags is:";
	for (int c = 0; c < clusters.size(); c++)
		cout << " " << selectedClustersFlags[c];
	cout << '\n';


}



void PCLViewer::selectACluster(int theCluster)
{
	if (theCluster >= 0 && theCluster < clusters.size())
	{
		// I want to create a Cube showing the bounding box of the now-selected cluster
		// Take only the sub-cloud contained in this cluster
		// http://www.pointclouds.org/documentation/tutorials/extract_indices.php#extract-indices

		// Create the filtering object
		pcl::ExtractIndices<PointT> extract;
		// Create the cluster cloud
		PointCloudT::Ptr clusterCloud(new PointCloudT);
		// Prepare to extract the inliers
		// At first, set the input cloud for filtering
		extract.setInputCloud(labeledCloud);

		// Remember:
		// std::vector <pcl::PointIndices> clusters;
		// I must convert a vector of PointIndices to PointIndices::Ptr
		// See http://www.pcl-users.org/from-PointIndices-to-IndicesPtr-td4034763.html

		pcl::PointIndices::Ptr inliers(new  pcl::PointIndices(clusters[theCluster]));

		extract.setIndices(inliers);
		extract.setNegative(false);
		extract.filter(*clusterCloud);

		// Compute the min/max of the object
		PointT min_pt, max_pt;
		getMinMax3D(*clusterCloud, min_pt, max_pt);

		cout << "Cluster bounding box " << min_pt.x << " " << min_pt.y << " " << min_pt.z << " " << max_pt.x << " " << max_pt.y << " " << max_pt.z << " " << std::endl;
		// Visualize the bounding box in 3D...
		stringstream ss;
		ss << "cube_" << theCluster;
		viewer->addCube(min_pt.x, max_pt.x, min_pt.y, max_pt.y, min_pt.z, max_pt.z, 1.0, 1.0, 1.0, ss.str());
		//viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 10, ss.str ());
		//viewer->setPointCloudRenderingProperties  (pcl::visualization::PCL_VISUALIZER_OPACITY, 120, ss.str ());   // https://github.com/PointCloudLibrary/pcl/issues/1111
		viewer->setRepresentationToWireframeForAllActors();

		// Logically mark the cluster as selected
		selectedClustersFlags[theCluster] = true;

		// Refresh (not always needed...)
		ui->qvtkWidget->update();
	}
}


void PCLViewer::deSelectACluster(int theCluster)
{
	if (theCluster >= 0 && theCluster < clusters.size())
	{
		stringstream ss;
		ss << "cube_" << theCluster;
		viewer->removeShape(ss.str());
		// Logically mark the cluster as not selected
		selectedClustersFlags[theCluster] = false;
		// Refresh (not always needed...)
		ui->qvtkWidget->update();
	}
}

// https://github.com/PointCloudLibrary/pcl/blob/master/apps/cloud_composer/tools/euclidean_clustering.cpp
// COMMENT ON EUCLIDEAN CLUSTERING AND indices ("// It's annoying that I have to do this, but Euclidean returns a PointIndices struct")






PointCloudT::Ptr PCLViewer::extractSelectedClustersFromCloud(void)
{
	// Prepare the new cloud to be returned
	PointCloudT::Ptr filteredCloud(new PointCloudT);

	// Prepare the IndicesPtr to contain the indices of the points to be put into the point cloud, i.e. those that 
	// are contained in the selected clusters
	pcl::IndicesPtr indicesToPreserve(new std::vector<int>());

	// Put together all the indices of the points to be preserved and copied to the output cloud (remember the definition: vector <pcl::PointIndices> clusters;)
	// This variable can be later used to restore the indices of the processed clusters
	for (int j = 0; j < selectedClustersFlags.size(); j++)
		if (selectedClustersFlags[j])
		{
			cout << "Preparing to preserve cluster # " << j << "\n";
			indicesToPreserve->insert(indicesToPreserve->end(), clusters[j].indices.begin(), clusters[j].indices.end());
		}

	// Create the filtering object
	pcl::ExtractIndices<PointT> extract;

	// Set the input cloud
	extract.setInputCloud(cloud);

	// Set the index to be preserved
	extract.setIndices(indicesToPreserve);

	// Preserving
	extract.setNegative(false);

	// Execute
	extract.filter(*filteredCloud);

	return filteredCloud;
}





void PCLViewer::dropClusterFromClustersWithIndex(int clusterIndex)
{
	clusters.erase(clusters.begin() + clusterIndex);
	clusterColors.erase(clusterColors.begin() + clusterIndex);	
}


void PCLViewer::appendClusterToClusters(pcl::IndicesPtr indicesToAdd)
{
	pcl::PointIndices::Ptr piPtr(new pcl::PointIndices);   // da http://www.pcl-users.org/IndicesPtr-from-PointIndices-td4020356.html
	piPtr->indices = *indicesToAdd;
	clusters.push_back(*piPtr);
}







#if 0
// I cannot use the following approach because if I effectively delete the unwanted points from the cloud, then the indices are no more valid!!
// I could resegment, but anyway I would have no correspondance with the saved cloud!!!!

// Based on
// https://github.com/PointCloudLibrary/pcl/blob/master/apps/cloud_composer/tools/euclidean_clustering.cpp
// (see the extracted_indices->insert part)
// on
// http://www.pcl-users.org/Removing-a-cluster-Problem-with-pointer-td4023699.html
// and of course on the usual tutorial at
// http://www.pointclouds.org/documentation/tutorials/extract_indices.php

// Prepare the new cleaned cloud
PointCloudT::Ptr cleanedLabeledCloud(new PointCloudT);

pcl::IndicesPtr indicesToDrop(new std::vector<int>());

// Put all the indices of the points to be dropped, together (remember definition: vector <pcl::PointIndices> clusters;)
set<int>::iterator it;
for (it = selectedClusters.begin(); it != selectedClusters.end(); ++it)
{
	int theCluster = *it;
	cout << "Preparing to drop cluster # " << theCluster << "\n";
	indicesToDrop->insert(indicesToDrop->end(), clusters[theCluster].indices.begin(), clusters[theCluster].indices.end());
}

// Create the filtering object
pcl::ExtractIndices<PointT> extract;

// Set the input cloud
extract.setInputCloud(labeledCloud);

// Set the index to be dropped
extract.setIndices(indicesToDrop);

// Dropping
extract.setNegative(true);

// Execute
extract.filter(*cleanedLabeledCloud);

// copy http://www.pcl-users.org/How-to-copy-a-PointCloud-td3718701.html
pcl::copyPointCloud(*cleanedLabeledCloud, *labeledCloud);

#endif








#if 0


vector<int> indices(1);
vector<float> distances(1);

// Use mutices to make sure we get the right cloud
boost::mutex::scoped_lock lock1(cloud_mutex_);

// Get the point that was picked
PointT picked_pt;
event.getPoint(picked_pt.x, picked_pt.y, picked_pt.z);

// Add a sphere to it in the PCLVisualizer window
stringstream ss;
ss << "sphere_" << idx;
viewer.addSphere(picked_pt, 0.01, 1.0, 0.0, 0.0, ss.str());

// Check to see if we have access to the actual cloud data. Use the previously built search object.
if (!search_.isValid())
return;

// Because VTK/OpenGL stores data without NaN, we lose the 1-1 correspondence, so we must search for the real point
search_.nearestKSearch(picked_pt, 1, indices, distances);

// Get the [u, v] in pixel coordinates for the ImageViewer. Remember that 0,0 is bottom left.
uint32_t width = search_.getInputCloud()->width;
//               height = search_.getInputCloud ()->height;
int v = indices[0] / width,
u = indices[0] % width;

// Add some marker to the image
image_viewer_.addCircle(u, v, 5, 1.0, 0.0, 0.0, "circles", 1.0);
image_viewer_.addFilledRectangle(u - 5, u + 5, v - 5, v + 5, 0.0, 1.0, 0.0, "boxes", 0.5);
image_viewer_.markPoint(u, v, visualization::red_color, visualization::blue_color, 10);

// Segment the region that we're interested in, by employing a two step process:
//  * first, segment all the planes in the scene, and find the one closest to our picked point
//  * then, use euclidean clustering to find the object that we clicked on and return it
PlanarRegion<PointT> region;
CloudPtr object;
PointIndices region_indices;
segment(picked_pt, indices[0], region, region_indices, object);

// If no region could be determined, exit
if (region.getContour().empty())
{
	PCL_ERROR("No planar region detected. Please select another point or relax the thresholds and continue.\n");
	return;
}
// Else, draw it on screen
else
{
	//cloud_viewer_.addPolygon (region, 1.0, 0.0, 0.0, "region");
	//cloud_viewer_.setShapeRenderingProperties (visualization::PCL_VISUALIZER_LINE_WIDTH, 10, "region");

	PlanarRegion<PointT> refined_region;
	pcl::approximatePolygon(region, refined_region, 0.01, false, true);
	PCL_INFO("Planar region: %lu points initial, %lu points after refinement.\n", region.getContour().size(), refined_region.getContour().size());
	cloud_viewer_.addPolygon(refined_region, 0.0, 0.0, 1.0, "refined_region");
	cloud_viewer_.setShapeRenderingProperties(visualization::PCL_VISUALIZER_LINE_WIDTH, 10, "refined_region");

	// Draw in image space
	image_viewer_.addPlanarPolygon(search_.getInputCloud(), refined_region, 0.0, 0.0, 1.0, "refined_region", 1.0);
}

// If no object could be determined, exit
if (!object)
{
	PCL_ERROR("No object detected. Please select another point or relax the thresholds and continue.\n");
	return;
}
else
{
	// Visualize the object in 3D...
	visualization::PointCloudColorHandlerCustom<PointT> red(object, 255, 0, 0);
	if (!cloud_viewer_.updatePointCloud(object, red, "object"))
		cloud_viewer_.addPointCloud(object, red, "object");
	// ...and 2D
	image_viewer_.removeLayer("object");
	image_viewer_.addMask(search_.getInputCloud(), *object, "object");

	// Compute the min/max of the object
	PointT min_pt, max_pt;
	getMinMax3D(*object, min_pt, max_pt);
	stringstream ss;
	ss << "cube_" << idx;
	// Visualize the bounding box in 3D...
	cloud_viewer_.addCube(min_pt.x, max_pt.x, min_pt.y, max_pt.y, min_pt.z, max_pt.z, 0.0, 1.0, 0.0, ss.str());
	cloud_viewer_.setShapeRenderingProperties(visualization::PCL_VISUALIZER_LINE_WIDTH, 10, ss.str());

	// ...and 2D
	image_viewer_.addRectangle(search_.getInputCloud(), *object);
}


#endif




