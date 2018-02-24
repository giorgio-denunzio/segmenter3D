/********************************************************************************
** Form generated from reading UI file 'pclviewer.ui'
**
** Created by: Qt User Interface Compiler version 5.8.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_PCLVIEWER_H
#define UI_PCLVIEWER_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QWidget>
#include "QVTKWidget.h"

QT_BEGIN_NAMESPACE

class Ui_PCLViewer
{
public:
    QWidget *centralwidget;
    QHBoxLayout *horizontalLayout;
    QFrame *frame_LeftSection;
    QLabel *label_5;
    QPushButton *pushButton_LoadCloud;
    QGroupBox *groupBox_Segmentation;
    QPushButton *pushButton_StartSegmentation;
    QTabWidget *tabWidget_Segmentation;
    QWidget *tab_ConnectedComponents;
    QLineEdit *lineEdit_octreeLevel;
    QLabel *label_15;
    QLineEdit *lineEdit_minComponentPopul;
    QLabel *label_octreeLevel;
    QLabel *label_17;
    QLabel *label_18;
    QWidget *tab_RegionGrowing;
    QLabel *label_rgMaxClusterSize;
    QLineEdit *lineEdit_rgSmoothnessThreshold;
    QLineEdit *lineEdit_rgMaxClusterSize;
    QLineEdit *lineEdit_rgNumberOfNeighbours;
    QLineEdit *lineEdit_rgMinClusterSize;
    QLabel *label_rgCurvatureThreshold;
    QLineEdit *lineEdit_rgCurvatureThreshold;
    QLabel *label_neKSearch;
    QLabel *label_rgNumberOfNeighbours;
    QLineEdit *lineEdit_neKSearch;
    QLabel *label_rgSmoothnessThreshold;
    QLabel *label_rgMinClusterSize;
    QLabel *label_rgSmoothnessThresholdLimits;
    QLabel *label_RegionGrowingLink;
    QWidget *tab_HyperClustering;
    QLabel *label_16;
    QWidget *tab_LCCP;
    QGroupBox *groupBox;
    QLabel *label_LCCPcolorImport;
    QLabel *label_LCCPspatialImport;
    QLabel *label_LCCPnormalImport;
    QLabel *label_LCCPVoxRes;
    QLabel *label__LCCPSeedRes;
    QLabel *label_LCCPsingleCam;
    QLabel *label_LCCPsupervoxelRef;
    QLineEdit *lineEdit_LCCPVoxRes;
    QLineEdit *lineEdit_LCCPSeedRes;
    QLineEdit *lineEdit_LCCPcolorImport;
    QLineEdit *lineEdit_LCCPspatialImport;
    QLineEdit *lineEdit_LCCPnormalImport;
    QCheckBox *checkBox_LCCPsingleCam;
    QCheckBox *checkBox_LCCPsupervoxelRef;
    QGroupBox *groupBox_2;
    QLabel *label_LCCPconcavToler;
    QLabel *label_LCCPsmoothThr;
    QLabel *label_LCCPminSegmSz;
    QLabel *label_LCCPextendConvex;
    QLabel *label_LCCPsanityCrit;
    QLineEdit *lineEdit_LCCPminSegmSz;
    QLineEdit *lineEdit_LCCPsmoothThr;
    QLineEdit *lineEdit_LCCPconcavToler;
    QCheckBox *checkBox_LCCPsanityCrit;
    QCheckBox *checkBox_LCCPextendConvex;
    QWidget *tab_Columns;
    QPushButton *pushButton_SegmResetDefaults;
    QPushButton *pushButton_ClearSegmentation;
    QPushButton *pushButton_UndoSegmentation;
    QPushButton *pushButton_LoadMesh;
    QPushButton *pushButton_LoadTextureMesh;
    QGroupBox *groupBox_MeshToPointCloud;
    QRadioButton *radioButton_Sparse;
    QRadioButton *radioButton_Dense;
    QGroupBox *groupBox_DisplayPreferences;
    QRadioButton *radioButton_ShowOriginalColors;
    QRadioButton *radioButton_ShowClusters;
    QLabel *label_13;
    QLabel *label_14;
    QPushButton *pushButton_DeleteSelected;
    QPushButton *pushButton_Save;
    QPushButton *pushButton_PreserveSelected;
    QPushButton *pushButton_Restore;
    QPushButton *pushButton_DeselectAll;
    QPushButton *pushButton_SelectAll;
    QPushButton *pushButton_InvertSelection;
    QFrame *frame_qvtkWidget;
    QHBoxLayout *horizontalLayout_2;
    QVTKWidget *qvtkWidget;
    QButtonGroup *buttonGroup_DisplayPreferences;
    QButtonGroup *buttonGroup_MeshToPointCloud;

    void setupUi(QMainWindow *PCLViewer)
    {
        if (PCLViewer->objectName().isEmpty())
            PCLViewer->setObjectName(QStringLiteral("PCLViewer"));
        PCLViewer->resize(1591, 1109);
        PCLViewer->setMinimumSize(QSize(0, 0));
        PCLViewer->setMaximumSize(QSize(5000, 5000));
        centralwidget = new QWidget(PCLViewer);
        centralwidget->setObjectName(QStringLiteral("centralwidget"));
        horizontalLayout = new QHBoxLayout(centralwidget);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        frame_LeftSection = new QFrame(centralwidget);
        frame_LeftSection->setObjectName(QStringLiteral("frame_LeftSection"));
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(frame_LeftSection->sizePolicy().hasHeightForWidth());
        frame_LeftSection->setSizePolicy(sizePolicy);
        frame_LeftSection->setMinimumSize(QSize(360, 0));
        frame_LeftSection->setAutoFillBackground(true);
        frame_LeftSection->setFrameShape(QFrame::WinPanel);
        frame_LeftSection->setFrameShadow(QFrame::Raised);
        frame_LeftSection->setLineWidth(2);
        label_5 = new QLabel(frame_LeftSection);
        label_5->setObjectName(QStringLiteral("label_5"));
        label_5->setGeometry(QRect(30, 20, 271, 16));
        QFont font;
        font.setPointSize(15);
        font.setBold(true);
        font.setWeight(75);
        label_5->setFont(font);
        pushButton_LoadCloud = new QPushButton(frame_LeftSection);
        pushButton_LoadCloud->setObjectName(QStringLiteral("pushButton_LoadCloud"));
        pushButton_LoadCloud->setEnabled(false);
        pushButton_LoadCloud->setGeometry(QRect(30, 100, 81, 23));
        groupBox_Segmentation = new QGroupBox(frame_LeftSection);
        groupBox_Segmentation->setObjectName(QStringLiteral("groupBox_Segmentation"));
        groupBox_Segmentation->setGeometry(QRect(20, 210, 311, 441));
        pushButton_StartSegmentation = new QPushButton(groupBox_Segmentation);
        pushButton_StartSegmentation->setObjectName(QStringLiteral("pushButton_StartSegmentation"));
        pushButton_StartSegmentation->setGeometry(QRect(130, 380, 101, 23));
        tabWidget_Segmentation = new QTabWidget(groupBox_Segmentation);
        tabWidget_Segmentation->setObjectName(QStringLiteral("tabWidget_Segmentation"));
        tabWidget_Segmentation->setGeometry(QRect(10, 20, 291, 351));
        tab_ConnectedComponents = new QWidget();
        tab_ConnectedComponents->setObjectName(QStringLiteral("tab_ConnectedComponents"));
        lineEdit_octreeLevel = new QLineEdit(tab_ConnectedComponents);
        lineEdit_octreeLevel->setObjectName(QStringLiteral("lineEdit_octreeLevel"));
        lineEdit_octreeLevel->setGeometry(QRect(137, 20, 51, 20));
        label_15 = new QLabel(tab_ConnectedComponents);
        label_15->setObjectName(QStringLiteral("label_15"));
        label_15->setGeometry(QRect(10, 50, 111, 20));
        lineEdit_minComponentPopul = new QLineEdit(tab_ConnectedComponents);
        lineEdit_minComponentPopul->setObjectName(QStringLiteral("lineEdit_minComponentPopul"));
        lineEdit_minComponentPopul->setGeometry(QRect(137, 50, 51, 20));
        label_octreeLevel = new QLabel(tab_ConnectedComponents);
        label_octreeLevel->setObjectName(QStringLiteral("label_octreeLevel"));
        label_octreeLevel->setGeometry(QRect(10, 20, 91, 20));
        label_17 = new QLabel(tab_ConnectedComponents);
        label_17->setObjectName(QStringLiteral("label_17"));
        label_17->setGeometry(QRect(200, 20, 91, 20));
        label_18 = new QLabel(tab_ConnectedComponents);
        label_18->setObjectName(QStringLiteral("label_18"));
        label_18->setGeometry(QRect(200, 50, 91, 20));
        tabWidget_Segmentation->addTab(tab_ConnectedComponents, QString());
        tab_RegionGrowing = new QWidget();
        tab_RegionGrowing->setObjectName(QStringLiteral("tab_RegionGrowing"));
        label_rgMaxClusterSize = new QLabel(tab_RegionGrowing);
        label_rgMaxClusterSize->setObjectName(QStringLiteral("label_rgMaxClusterSize"));
        label_rgMaxClusterSize->setGeometry(QRect(3, 100, 91, 20));
        lineEdit_rgSmoothnessThreshold = new QLineEdit(tab_RegionGrowing);
        lineEdit_rgSmoothnessThreshold->setObjectName(QStringLiteral("lineEdit_rgSmoothnessThreshold"));
        lineEdit_rgSmoothnessThreshold->setGeometry(QRect(130, 160, 61, 20));
        lineEdit_rgMaxClusterSize = new QLineEdit(tab_RegionGrowing);
        lineEdit_rgMaxClusterSize->setObjectName(QStringLiteral("lineEdit_rgMaxClusterSize"));
        lineEdit_rgMaxClusterSize->setGeometry(QRect(130, 100, 61, 20));
        lineEdit_rgNumberOfNeighbours = new QLineEdit(tab_RegionGrowing);
        lineEdit_rgNumberOfNeighbours->setObjectName(QStringLiteral("lineEdit_rgNumberOfNeighbours"));
        lineEdit_rgNumberOfNeighbours->setGeometry(QRect(130, 130, 61, 20));
        lineEdit_rgMinClusterSize = new QLineEdit(tab_RegionGrowing);
        lineEdit_rgMinClusterSize->setObjectName(QStringLiteral("lineEdit_rgMinClusterSize"));
        lineEdit_rgMinClusterSize->setGeometry(QRect(130, 70, 61, 20));
        label_rgCurvatureThreshold = new QLabel(tab_RegionGrowing);
        label_rgCurvatureThreshold->setObjectName(QStringLiteral("label_rgCurvatureThreshold"));
        label_rgCurvatureThreshold->setGeometry(QRect(3, 190, 121, 20));
        lineEdit_rgCurvatureThreshold = new QLineEdit(tab_RegionGrowing);
        lineEdit_rgCurvatureThreshold->setObjectName(QStringLiteral("lineEdit_rgCurvatureThreshold"));
        lineEdit_rgCurvatureThreshold->setGeometry(QRect(130, 190, 61, 20));
        label_neKSearch = new QLabel(tab_RegionGrowing);
        label_neKSearch->setObjectName(QStringLiteral("label_neKSearch"));
        label_neKSearch->setGeometry(QRect(3, 40, 91, 20));
        label_rgNumberOfNeighbours = new QLabel(tab_RegionGrowing);
        label_rgNumberOfNeighbours->setObjectName(QStringLiteral("label_rgNumberOfNeighbours"));
        label_rgNumberOfNeighbours->setGeometry(QRect(3, 130, 121, 20));
        lineEdit_neKSearch = new QLineEdit(tab_RegionGrowing);
        lineEdit_neKSearch->setObjectName(QStringLiteral("lineEdit_neKSearch"));
        lineEdit_neKSearch->setGeometry(QRect(130, 40, 61, 20));
        label_rgSmoothnessThreshold = new QLabel(tab_RegionGrowing);
        label_rgSmoothnessThreshold->setObjectName(QStringLiteral("label_rgSmoothnessThreshold"));
        label_rgSmoothnessThreshold->setGeometry(QRect(3, 160, 131, 20));
        label_rgMinClusterSize = new QLabel(tab_RegionGrowing);
        label_rgMinClusterSize->setObjectName(QStringLiteral("label_rgMinClusterSize"));
        label_rgMinClusterSize->setGeometry(QRect(3, 70, 91, 20));
        label_rgSmoothnessThresholdLimits = new QLabel(tab_RegionGrowing);
        label_rgSmoothnessThresholdLimits->setObjectName(QStringLiteral("label_rgSmoothnessThresholdLimits"));
        label_rgSmoothnessThresholdLimits->setGeometry(QRect(200, 165, 61, 16));
        label_RegionGrowingLink = new QLabel(tab_RegionGrowing);
        label_RegionGrowingLink->setObjectName(QStringLiteral("label_RegionGrowingLink"));
        label_RegionGrowingLink->setGeometry(QRect(10, 10, 47, 13));
        label_RegionGrowingLink->setOpenExternalLinks(true);
        label_RegionGrowingLink->setTextInteractionFlags(Qt::LinksAccessibleByMouse);
        tabWidget_Segmentation->addTab(tab_RegionGrowing, QString());
        tab_HyperClustering = new QWidget();
        tab_HyperClustering->setObjectName(QStringLiteral("tab_HyperClustering"));
        label_16 = new QLabel(tab_HyperClustering);
        label_16->setObjectName(QStringLiteral("label_16"));
        label_16->setGeometry(QRect(60, 130, 151, 16));
        tabWidget_Segmentation->addTab(tab_HyperClustering, QString());
        tab_LCCP = new QWidget();
        tab_LCCP->setObjectName(QStringLiteral("tab_LCCP"));
        groupBox = new QGroupBox(tab_LCCP);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        groupBox->setGeometry(QRect(6, 10, 271, 171));
        QFont font1;
        font1.setBold(false);
        font1.setWeight(50);
        groupBox->setFont(font1);
        label_LCCPcolorImport = new QLabel(groupBox);
        label_LCCPcolorImport->setObjectName(QStringLiteral("label_LCCPcolorImport"));
        label_LCCPcolorImport->setGeometry(QRect(10, 63, 91, 16));
        label_LCCPspatialImport = new QLabel(groupBox);
        label_LCCPspatialImport->setObjectName(QStringLiteral("label_LCCPspatialImport"));
        label_LCCPspatialImport->setGeometry(QRect(10, 84, 91, 16));
        label_LCCPnormalImport = new QLabel(groupBox);
        label_LCCPnormalImport->setObjectName(QStringLiteral("label_LCCPnormalImport"));
        label_LCCPnormalImport->setGeometry(QRect(10, 105, 101, 16));
        label_LCCPVoxRes = new QLabel(groupBox);
        label_LCCPVoxRes->setObjectName(QStringLiteral("label_LCCPVoxRes"));
        label_LCCPVoxRes->setGeometry(QRect(10, 20, 91, 16));
        label__LCCPSeedRes = new QLabel(groupBox);
        label__LCCPSeedRes->setObjectName(QStringLiteral("label__LCCPSeedRes"));
        label__LCCPSeedRes->setGeometry(QRect(10, 41, 81, 16));
        label_LCCPsingleCam = new QLabel(groupBox);
        label_LCCPsingleCam->setObjectName(QStringLiteral("label_LCCPsingleCam"));
        label_LCCPsingleCam->setGeometry(QRect(10, 126, 111, 16));
        label_LCCPsupervoxelRef = new QLabel(groupBox);
        label_LCCPsupervoxelRef->setObjectName(QStringLiteral("label_LCCPsupervoxelRef"));
        label_LCCPsupervoxelRef->setGeometry(QRect(10, 147, 111, 16));
        lineEdit_LCCPVoxRes = new QLineEdit(groupBox);
        lineEdit_LCCPVoxRes->setObjectName(QStringLiteral("lineEdit_LCCPVoxRes"));
        lineEdit_LCCPVoxRes->setGeometry(QRect(130, 18, 61, 20));
        lineEdit_LCCPSeedRes = new QLineEdit(groupBox);
        lineEdit_LCCPSeedRes->setObjectName(QStringLiteral("lineEdit_LCCPSeedRes"));
        lineEdit_LCCPSeedRes->setGeometry(QRect(130, 39, 61, 20));
        lineEdit_LCCPcolorImport = new QLineEdit(groupBox);
        lineEdit_LCCPcolorImport->setObjectName(QStringLiteral("lineEdit_LCCPcolorImport"));
        lineEdit_LCCPcolorImport->setGeometry(QRect(130, 60, 61, 20));
        lineEdit_LCCPspatialImport = new QLineEdit(groupBox);
        lineEdit_LCCPspatialImport->setObjectName(QStringLiteral("lineEdit_LCCPspatialImport"));
        lineEdit_LCCPspatialImport->setGeometry(QRect(130, 81, 61, 20));
        lineEdit_LCCPnormalImport = new QLineEdit(groupBox);
        lineEdit_LCCPnormalImport->setObjectName(QStringLiteral("lineEdit_LCCPnormalImport"));
        lineEdit_LCCPnormalImport->setGeometry(QRect(130, 102, 61, 20));
        checkBox_LCCPsingleCam = new QCheckBox(groupBox);
        checkBox_LCCPsingleCam->setObjectName(QStringLiteral("checkBox_LCCPsingleCam"));
        checkBox_LCCPsingleCam->setGeometry(QRect(132, 128, 70, 17));
        checkBox_LCCPsupervoxelRef = new QCheckBox(groupBox);
        checkBox_LCCPsupervoxelRef->setObjectName(QStringLiteral("checkBox_LCCPsupervoxelRef"));
        checkBox_LCCPsupervoxelRef->setGeometry(QRect(132, 146, 70, 17));
        groupBox_2 = new QGroupBox(tab_LCCP);
        groupBox_2->setObjectName(QStringLiteral("groupBox_2"));
        groupBox_2->setGeometry(QRect(10, 190, 271, 121));
        label_LCCPconcavToler = new QLabel(groupBox_2);
        label_LCCPconcavToler->setObjectName(QStringLiteral("label_LCCPconcavToler"));
        label_LCCPconcavToler->setGeometry(QRect(10, 20, 121, 16));
        label_LCCPsmoothThr = new QLabel(groupBox_2);
        label_LCCPsmoothThr->setObjectName(QStringLiteral("label_LCCPsmoothThr"));
        label_LCCPsmoothThr->setGeometry(QRect(10, 40, 121, 16));
        label_LCCPminSegmSz = new QLabel(groupBox_2);
        label_LCCPminSegmSz->setObjectName(QStringLiteral("label_LCCPminSegmSz"));
        label_LCCPminSegmSz->setGeometry(QRect(10, 60, 121, 16));
        label_LCCPextendConvex = new QLabel(groupBox_2);
        label_LCCPextendConvex->setObjectName(QStringLiteral("label_LCCPextendConvex"));
        label_LCCPextendConvex->setGeometry(QRect(10, 80, 121, 16));
        label_LCCPsanityCrit = new QLabel(groupBox_2);
        label_LCCPsanityCrit->setObjectName(QStringLiteral("label_LCCPsanityCrit"));
        label_LCCPsanityCrit->setGeometry(QRect(10, 100, 91, 16));
        lineEdit_LCCPminSegmSz = new QLineEdit(groupBox_2);
        lineEdit_LCCPminSegmSz->setObjectName(QStringLiteral("lineEdit_LCCPminSegmSz"));
        lineEdit_LCCPminSegmSz->setGeometry(QRect(130, 58, 61, 20));
        lineEdit_LCCPsmoothThr = new QLineEdit(groupBox_2);
        lineEdit_LCCPsmoothThr->setObjectName(QStringLiteral("lineEdit_LCCPsmoothThr"));
        lineEdit_LCCPsmoothThr->setGeometry(QRect(130, 37, 61, 20));
        lineEdit_LCCPconcavToler = new QLineEdit(groupBox_2);
        lineEdit_LCCPconcavToler->setObjectName(QStringLiteral("lineEdit_LCCPconcavToler"));
        lineEdit_LCCPconcavToler->setGeometry(QRect(130, 16, 61, 20));
        checkBox_LCCPsanityCrit = new QCheckBox(groupBox_2);
        checkBox_LCCPsanityCrit->setObjectName(QStringLiteral("checkBox_LCCPsanityCrit"));
        checkBox_LCCPsanityCrit->setGeometry(QRect(131, 99, 70, 17));
        checkBox_LCCPextendConvex = new QCheckBox(groupBox_2);
        checkBox_LCCPextendConvex->setObjectName(QStringLiteral("checkBox_LCCPextendConvex"));
        checkBox_LCCPextendConvex->setGeometry(QRect(131, 81, 70, 17));
        tabWidget_Segmentation->addTab(tab_LCCP, QString());
        tab_Columns = new QWidget();
        tab_Columns->setObjectName(QStringLiteral("tab_Columns"));
        tabWidget_Segmentation->addTab(tab_Columns, QString());
        pushButton_SegmResetDefaults = new QPushButton(groupBox_Segmentation);
        pushButton_SegmResetDefaults->setObjectName(QStringLiteral("pushButton_SegmResetDefaults"));
        pushButton_SegmResetDefaults->setEnabled(true);
        pushButton_SegmResetDefaults->setGeometry(QRect(9, 380, 111, 23));
        pushButton_ClearSegmentation = new QPushButton(groupBox_Segmentation);
        pushButton_ClearSegmentation->setObjectName(QStringLiteral("pushButton_ClearSegmentation"));
        pushButton_ClearSegmentation->setGeometry(QRect(130, 410, 102, 23));
        pushButton_UndoSegmentation = new QPushButton(groupBox_Segmentation);
        pushButton_UndoSegmentation->setObjectName(QStringLiteral("pushButton_UndoSegmentation"));
        pushButton_UndoSegmentation->setGeometry(QRect(10, 410, 111, 23));
        pushButton_LoadMesh = new QPushButton(frame_LeftSection);
        pushButton_LoadMesh->setObjectName(QStringLiteral("pushButton_LoadMesh"));
        pushButton_LoadMesh->setGeometry(QRect(120, 100, 75, 23));
        pushButton_LoadTextureMesh = new QPushButton(frame_LeftSection);
        pushButton_LoadTextureMesh->setObjectName(QStringLiteral("pushButton_LoadTextureMesh"));
        pushButton_LoadTextureMesh->setEnabled(false);
        pushButton_LoadTextureMesh->setGeometry(QRect(200, 100, 121, 23));
        groupBox_MeshToPointCloud = new QGroupBox(frame_LeftSection);
        groupBox_MeshToPointCloud->setObjectName(QStringLiteral("groupBox_MeshToPointCloud"));
        groupBox_MeshToPointCloud->setEnabled(false);
        groupBox_MeshToPointCloud->setGeometry(QRect(180, 140, 161, 61));
        radioButton_Sparse = new QRadioButton(groupBox_MeshToPointCloud);
        buttonGroup_MeshToPointCloud = new QButtonGroup(PCLViewer);
        buttonGroup_MeshToPointCloud->setObjectName(QStringLiteral("buttonGroup_MeshToPointCloud"));
        buttonGroup_MeshToPointCloud->addButton(radioButton_Sparse);
        radioButton_Sparse->setObjectName(QStringLiteral("radioButton_Sparse"));
        radioButton_Sparse->setGeometry(QRect(10, 20, 131, 17));
        radioButton_Sparse->setChecked(true);
        radioButton_Dense = new QRadioButton(groupBox_MeshToPointCloud);
        buttonGroup_MeshToPointCloud->addButton(radioButton_Dense);
        radioButton_Dense->setObjectName(QStringLiteral("radioButton_Dense"));
        radioButton_Dense->setGeometry(QRect(10, 40, 131, 17));
        groupBox_DisplayPreferences = new QGroupBox(frame_LeftSection);
        groupBox_DisplayPreferences->setObjectName(QStringLiteral("groupBox_DisplayPreferences"));
        groupBox_DisplayPreferences->setGeometry(QRect(10, 140, 151, 61));
        radioButton_ShowOriginalColors = new QRadioButton(groupBox_DisplayPreferences);
        buttonGroup_DisplayPreferences = new QButtonGroup(PCLViewer);
        buttonGroup_DisplayPreferences->setObjectName(QStringLiteral("buttonGroup_DisplayPreferences"));
        buttonGroup_DisplayPreferences->addButton(radioButton_ShowOriginalColors);
        radioButton_ShowOriginalColors->setObjectName(QStringLiteral("radioButton_ShowOriginalColors"));
        radioButton_ShowOriginalColors->setGeometry(QRect(20, 20, 151, 17));
        radioButton_ShowOriginalColors->setChecked(true);
        radioButton_ShowClusters = new QRadioButton(groupBox_DisplayPreferences);
        buttonGroup_DisplayPreferences->addButton(radioButton_ShowClusters);
        radioButton_ShowClusters->setObjectName(QStringLiteral("radioButton_ShowClusters"));
        radioButton_ShowClusters->setGeometry(QRect(20, 40, 171, 17));
        label_13 = new QLabel(frame_LeftSection);
        label_13->setObjectName(QStringLiteral("label_13"));
        label_13->setGeometry(QRect(30, 50, 301, 16));
        label_14 = new QLabel(frame_LeftSection);
        label_14->setObjectName(QStringLiteral("label_14"));
        label_14->setGeometry(QRect(30, 70, 301, 16));
        pushButton_DeleteSelected = new QPushButton(frame_LeftSection);
        pushButton_DeleteSelected->setObjectName(QStringLiteral("pushButton_DeleteSelected"));
        pushButton_DeleteSelected->setGeometry(QRect(130, 660, 101, 23));
        pushButton_Save = new QPushButton(frame_LeftSection);
        pushButton_Save->setObjectName(QStringLiteral("pushButton_Save"));
        pushButton_Save->setGeometry(QRect(240, 690, 91, 23));
        pushButton_PreserveSelected = new QPushButton(frame_LeftSection);
        pushButton_PreserveSelected->setObjectName(QStringLiteral("pushButton_PreserveSelected"));
        pushButton_PreserveSelected->setGeometry(QRect(130, 690, 101, 23));
        pushButton_Restore = new QPushButton(frame_LeftSection);
        pushButton_Restore->setObjectName(QStringLiteral("pushButton_Restore"));
        pushButton_Restore->setGeometry(QRect(240, 660, 91, 23));
        pushButton_DeselectAll = new QPushButton(frame_LeftSection);
        pushButton_DeselectAll->setObjectName(QStringLiteral("pushButton_DeselectAll"));
        pushButton_DeselectAll->setEnabled(true);
        pushButton_DeselectAll->setGeometry(QRect(20, 690, 101, 23));
        pushButton_SelectAll = new QPushButton(frame_LeftSection);
        pushButton_SelectAll->setObjectName(QStringLiteral("pushButton_SelectAll"));
        pushButton_SelectAll->setEnabled(true);
        pushButton_SelectAll->setGeometry(QRect(20, 660, 101, 23));
        pushButton_InvertSelection = new QPushButton(frame_LeftSection);
        pushButton_InvertSelection->setObjectName(QStringLiteral("pushButton_InvertSelection"));
        pushButton_InvertSelection->setEnabled(true);
        pushButton_InvertSelection->setGeometry(QRect(20, 720, 101, 23));

        horizontalLayout->addWidget(frame_LeftSection);

        frame_qvtkWidget = new QFrame(centralwidget);
        frame_qvtkWidget->setObjectName(QStringLiteral("frame_qvtkWidget"));
        QSizePolicy sizePolicy1(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(frame_qvtkWidget->sizePolicy().hasHeightForWidth());
        frame_qvtkWidget->setSizePolicy(sizePolicy1);
        frame_qvtkWidget->setFrameShape(QFrame::WinPanel);
        frame_qvtkWidget->setFrameShadow(QFrame::Raised);
        horizontalLayout_2 = new QHBoxLayout(frame_qvtkWidget);
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        qvtkWidget = new QVTKWidget(frame_qvtkWidget);
        qvtkWidget->setObjectName(QStringLiteral("qvtkWidget"));
        sizePolicy1.setHeightForWidth(qvtkWidget->sizePolicy().hasHeightForWidth());
        qvtkWidget->setSizePolicy(sizePolicy1);

        horizontalLayout_2->addWidget(qvtkWidget);


        horizontalLayout->addWidget(frame_qvtkWidget);

        PCLViewer->setCentralWidget(centralwidget);

        retranslateUi(PCLViewer);

        tabWidget_Segmentation->setCurrentIndex(1);


        QMetaObject::connectSlotsByName(PCLViewer);
    } // setupUi

    void retranslateUi(QMainWindow *PCLViewer)
    {
        PCLViewer->setWindowTitle(QApplication::translate("PCLViewer", "SegmentationTool", Q_NULLPTR));
        label_5->setText(QApplication::translate("PCLViewer", "SEGMENTATION TOOL v0.1", Q_NULLPTR));
        pushButton_LoadCloud->setText(QApplication::translate("PCLViewer", "Load Cloud", Q_NULLPTR));
        groupBox_Segmentation->setTitle(QApplication::translate("PCLViewer", "Segmentation", Q_NULLPTR));
        pushButton_StartSegmentation->setText(QApplication::translate("PCLViewer", "Start Segmentation", Q_NULLPTR));
        lineEdit_octreeLevel->setText(QApplication::translate("PCLViewer", "8", Q_NULLPTR));
        label_15->setText(QApplication::translate("PCLViewer", "Min. component popul.", Q_NULLPTR));
        lineEdit_minComponentPopul->setText(QApplication::translate("PCLViewer", "500", Q_NULLPTR));
        label_octreeLevel->setText(QApplication::translate("PCLViewer", "Octree level", Q_NULLPTR));
        label_17->setText(QApplication::translate("PCLViewer", "1 ... 21  (def. 8)", Q_NULLPTR));
        label_18->setText(QApplication::translate("PCLViewer", "(def. 500)", Q_NULLPTR));
        tabWidget_Segmentation->setTabText(tabWidget_Segmentation->indexOf(tab_ConnectedComponents), QApplication::translate("PCLViewer", "Connected Components", Q_NULLPTR));
        label_rgMaxClusterSize->setText(QApplication::translate("PCLViewer", "rgMaxClusterSize", Q_NULLPTR));
        lineEdit_rgSmoothnessThreshold->setText(QApplication::translate("PCLViewer", "150", Q_NULLPTR));
        lineEdit_rgMaxClusterSize->setText(QApplication::translate("PCLViewer", "10000000", Q_NULLPTR));
        lineEdit_rgNumberOfNeighbours->setText(QApplication::translate("PCLViewer", "80", Q_NULLPTR));
        lineEdit_rgMinClusterSize->setText(QApplication::translate("PCLViewer", "200", Q_NULLPTR));
        label_rgCurvatureThreshold->setText(QApplication::translate("PCLViewer", "rgCurvatureThreshold", Q_NULLPTR));
        lineEdit_rgCurvatureThreshold->setText(QApplication::translate("PCLViewer", "0.1", Q_NULLPTR));
        label_neKSearch->setText(QApplication::translate("PCLViewer", "neKSearch", Q_NULLPTR));
        label_rgNumberOfNeighbours->setText(QApplication::translate("PCLViewer", "rgNumberOfNeighbours", Q_NULLPTR));
        lineEdit_neKSearch->setText(QApplication::translate("PCLViewer", "50", Q_NULLPTR));
        label_rgSmoothnessThreshold->setText(QApplication::translate("PCLViewer", "rgSmoothnessThreshold", Q_NULLPTR));
        label_rgMinClusterSize->setText(QApplication::translate("PCLViewer", "rgMinClusterSize", Q_NULLPTR));
        label_rgSmoothnessThresholdLimits->setText(QApplication::translate("PCLViewer", "(0 ... 180\302\260)", Q_NULLPTR));
        label_RegionGrowingLink->setText(QApplication::translate("PCLViewer", "<a href=\"http://pointclouds.org/documentation/tutorials/region_growing_segmentation.php\">Link</a>", Q_NULLPTR));
        tabWidget_Segmentation->setTabText(tabWidget_Segmentation->indexOf(tab_RegionGrowing), QApplication::translate("PCLViewer", "Region Growing", Q_NULLPTR));
        label_16->setText(QApplication::translate("PCLViewer", "Not implemented!", Q_NULLPTR));
        tabWidget_Segmentation->setTabText(tabWidget_Segmentation->indexOf(tab_HyperClustering), QApplication::translate("PCLViewer", "Hyper Clustering", Q_NULLPTR));
        groupBox->setTitle(QApplication::translate("PCLViewer", "Supervoxels", Q_NULLPTR));
        label_LCCPcolorImport->setText(QApplication::translate("PCLViewer", "Color importance", Q_NULLPTR));
        label_LCCPspatialImport->setText(QApplication::translate("PCLViewer", "Spatial importance", Q_NULLPTR));
        label_LCCPnormalImport->setText(QApplication::translate("PCLViewer", "Normal importance", Q_NULLPTR));
        label_LCCPVoxRes->setText(QApplication::translate("PCLViewer", "Voxel resolution", Q_NULLPTR));
        label__LCCPSeedRes->setText(QApplication::translate("PCLViewer", "Seed resolution", Q_NULLPTR));
        label_LCCPsingleCam->setText(QApplication::translate("PCLViewer", "Single cam transform", Q_NULLPTR));
        label_LCCPsupervoxelRef->setText(QApplication::translate("PCLViewer", "Supervoxel refinement", Q_NULLPTR));
        lineEdit_LCCPVoxRes->setText(QApplication::translate("PCLViewer", "0.0075", Q_NULLPTR));
        lineEdit_LCCPSeedRes->setText(QApplication::translate("PCLViewer", "0.03", Q_NULLPTR));
        lineEdit_LCCPcolorImport->setText(QApplication::translate("PCLViewer", "0.0", Q_NULLPTR));
        lineEdit_LCCPspatialImport->setText(QApplication::translate("PCLViewer", "1.0", Q_NULLPTR));
        lineEdit_LCCPnormalImport->setText(QApplication::translate("PCLViewer", "4.0", Q_NULLPTR));
        checkBox_LCCPsingleCam->setText(QString());
        checkBox_LCCPsupervoxelRef->setText(QString());
        groupBox_2->setTitle(QApplication::translate("PCLViewer", "LCCP", Q_NULLPTR));
        label_LCCPconcavToler->setText(QApplication::translate("PCLViewer", "Concavity tolerance thr.", Q_NULLPTR));
        label_LCCPsmoothThr->setText(QApplication::translate("PCLViewer", "Smoothness thr.", Q_NULLPTR));
        label_LCCPminSegmSz->setText(QApplication::translate("PCLViewer", "Min segment sz.", Q_NULLPTR));
        label_LCCPextendConvex->setText(QApplication::translate("PCLViewer", "Extended convexity", Q_NULLPTR));
        label_LCCPsanityCrit->setText(QApplication::translate("PCLViewer", "Sanity criterion", Q_NULLPTR));
        lineEdit_LCCPminSegmSz->setText(QApplication::translate("PCLViewer", "0", Q_NULLPTR));
        lineEdit_LCCPsmoothThr->setText(QApplication::translate("PCLViewer", "0.1", Q_NULLPTR));
        lineEdit_LCCPconcavToler->setText(QApplication::translate("PCLViewer", "10", Q_NULLPTR));
        checkBox_LCCPsanityCrit->setText(QString());
        checkBox_LCCPextendConvex->setText(QString());
        tabWidget_Segmentation->setTabText(tabWidget_Segmentation->indexOf(tab_LCCP), QApplication::translate("PCLViewer", "LCCP", Q_NULLPTR));
        tabWidget_Segmentation->setTabText(tabWidget_Segmentation->indexOf(tab_Columns), QApplication::translate("PCLViewer", "Columns", Q_NULLPTR));
        pushButton_SegmResetDefaults->setText(QApplication::translate("PCLViewer", "Reset defaults", Q_NULLPTR));
        pushButton_ClearSegmentation->setText(QApplication::translate("PCLViewer", "Clear Segmentation", Q_NULLPTR));
        pushButton_UndoSegmentation->setText(QApplication::translate("PCLViewer", "Back one step", Q_NULLPTR));
        pushButton_LoadMesh->setText(QApplication::translate("PCLViewer", "Load Mesh", Q_NULLPTR));
        pushButton_LoadTextureMesh->setText(QApplication::translate("PCLViewer", "Load textured Mesh", Q_NULLPTR));
        groupBox_MeshToPointCloud->setTitle(QApplication::translate("PCLViewer", "Mesh to Point Cloud", Q_NULLPTR));
        radioButton_Sparse->setText(QApplication::translate("PCLViewer", "Sparse (mesh vertices)", Q_NULLPTR));
        radioButton_Dense->setText(QApplication::translate("PCLViewer", "Dense (CloudCompare)", Q_NULLPTR));
        groupBox_DisplayPreferences->setTitle(QApplication::translate("PCLViewer", "Display Preferences", Q_NULLPTR));
        radioButton_ShowOriginalColors->setText(QApplication::translate("PCLViewer", "Show Original Colors", Q_NULLPTR));
        radioButton_ShowClusters->setText(QApplication::translate("PCLViewer", "Show Clusters", Q_NULLPTR));
        label_13->setText(QApplication::translate("PCLViewer", "by Giorgio De Nunzio (giorgio.denunzio@unisalento.it)", Q_NULLPTR));
        label_14->setText(QApplication::translate("PCLViewer", "for Corvallis spa", Q_NULLPTR));
        pushButton_DeleteSelected->setText(QApplication::translate("PCLViewer", "Delete selected", Q_NULLPTR));
        pushButton_Save->setText(QApplication::translate("PCLViewer", "Save", Q_NULLPTR));
        pushButton_PreserveSelected->setText(QApplication::translate("PCLViewer", "Preserve selected", Q_NULLPTR));
        pushButton_Restore->setText(QApplication::translate("PCLViewer", "Restore", Q_NULLPTR));
        pushButton_DeselectAll->setText(QApplication::translate("PCLViewer", "Deselect all", Q_NULLPTR));
        pushButton_SelectAll->setText(QApplication::translate("PCLViewer", "Select all", Q_NULLPTR));
        pushButton_InvertSelection->setText(QApplication::translate("PCLViewer", "Invert selection", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class PCLViewer: public Ui_PCLViewer {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PCLVIEWER_H
