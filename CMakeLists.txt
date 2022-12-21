cmake_minimum_required(VERSION 2.8)

Project(HeadCV CXX)

set_property( DIRECTORY PROPERTY VS_STARTUP_PROJECT "HeadCV" )

set(CMAKE_CONFIGURATION_TYPES "Release" CACHE STRING "" FORCE)
set(CMAKE_BUILD_TYPE "Release")

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

#set( VTK_DIR "$ENV{PCL_ROOT}/3rdParty/VTK" )

set( QTDIR $ENV{QTDIR} )
set( CMAKE_PREFIX_PATH ${QTDIR} )

#set( PCL_ROOT $ENV{PCL_ROOT} )

find_package(PCL 	REQUIRED)
find_package(VTK	REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(OpenCV	REQUIRED)
find_package(realsense	REQUIRED)
find_package(OpenMP	REQUIRED)

if (CMAKE_COMPILER_IS_GNUCXX)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()

if (MSVC)
	add_definitions(-D_CRT_SECURE_NO_WARNINGS)
endif()

if (OPENMP_FOUND)
	message("OPENMP found")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
	set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

set( CMAKE_INCLUDE_CURRENT_DIR ON )
set( CMAKE_AUTOMOC ON )

qt5_wrap_cpp( MOC mainwindow.h PoseEstimationStep.h)
qt5_wrap_ui( UI_HEADER mainwindow.ui )

set(HeadCV_Processing_Steps
		"PoseEstimationStep.h"
		"PoseEstimationStep.cpp"
		"FilteringCropBoxStep.h"
		"FilteringCropBoxStep.cpp"
	)

set(HeadCV_Grabber_Files
		"SR300Grabber.cpp"
		"SR300Grabber.h"
	)

source_group("Processing steps"	FILES ${HeadCV_Processing_Steps})
source_group("Grabber"			FILES ${HeadCV_Grabber_Files})

add_executable( HeadCV
		main.cpp
		mainwindow.h
		mainwindow.cpp
		mainwindow.ui
		constants.h
		${HeadCV_Processing_Steps}
		${HeadCV_Grabber_Files})

include_directories( ${PCL_INCLUDE_DIRS} )
include_directories( ${CMAKE_CURRENT_SOURCE_DIR} )
include_directories( ${CMAKE_CURRENT_BINARY_DIR} )

include( ${VTK_USE_FILE} )

add_definitions( ${PCL_DEFINITIONS} )

link_directories( ${PCL_LIBRARY_DIRS} )

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

message("CXX flags:" ${CMAKE_CXX_FLAGS})

target_link_libraries( HeadCV 
			${PCL_LIBRARIES} 
			${VTK_LIBRARIES} 
			${realsense_LIBRARY}
			${OpenCV_LIBRARIES}
			Qt5::Widgets)

set_property(TARGET HeadCV PROPERTY FOLDER "App")
