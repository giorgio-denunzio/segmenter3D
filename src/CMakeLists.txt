# My changes labeled as GDN

# GDN, changed to 3.10
cmake_minimum_required( VERSION 3.10  FATAL_ERROR )

# Create Project
project( pcl_visualizer )

# vcpkg toolchain   (inserted by GDN)
#set (CMAKE_TOOLCHAIN_FILE "C:/src/vcpkg/scripts/buildsystems/vcpkg.cmake" )

# Set StartUp Project
set_property( DIRECTORY PROPERTY VS_STARTUP_PROJECT "pcl_visualizer" )

# Find PCL Package
# set( PCL_ROOT $ENV{PCL_ROOT} )    # Commented out by GDN
find_package( PCL 1.8.1 REQUIRED )   #  Set to 1.8.1 from 1.8.0

# Find VTK Package
# set( VTK_DIR "$ENV{PCL_ROOT}/3rdParty/VTK" )    # Commented out by GDN
find_package( VTK REQUIRED )

# Find Qt Package
#set( QTDIR $ENV{QTDIR} )    # Commented out by GDN
#set( CMAKE_PREFIX_PATH ${QTDIR} )    # Commented out by GDN
find_package( Qt5Widgets REQUIRED )

# Set Automatic MOC
set( CMAKE_INCLUDE_CURRENT_DIR ON )
set( CMAKE_AUTOMOC ON )

# Generate MOC Code and UI Header
qt5_wrap_cpp( MOC pclviewer.h )
qt5_wrap_ui( UI_HEADER pclviewer.ui )

# Add Executable (REMEMBER TO ADD HERE THE NEW FILES TO BE COMPILED!!)
add_executable( pcl_visualizer main.cpp pclviewer.h pclviewer.cpp pclviewer.ui   CSegmenterRegionGrowing.h CSegmenterRegionGrowing.cpp)

# Additional Include Directories
include_directories( ${PCL_INCLUDE_DIRS} )
include_directories( ${CMAKE_CURRENT_SOURCE_DIR} )
include_directories( ${CMAKE_CURRENT_BINARY_DIR} )

# Load VTK Settings
include( ${VTK_USE_FILE} )

# Preprocessor Definitions
add_definitions( ${PCL_DEFINITIONS} )

# Additional Library Directories
link_directories( ${PCL_LIBRARY_DIRS} )

# Additional Dependencies
target_link_libraries( pcl_visualizer ${PCL_LIBRARIES} ${VTK_LIBRARIES} Qt5::Widgets )
