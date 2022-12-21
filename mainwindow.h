#ifndef MAINWINDOW_H
#define MAINWINDOW_H

// Qt
#include <QMainWindow>
#include "ui_mainwindow.h"

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>

// FifthView
#include "SR300Grabber.h"

#include "FilteringCropBoxStep.h"
#include "PoseEstimationStep.h"

namespace Ui
{
	class MainWindow;
}

struct SensorCloudProcessing
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr		cloud;
	boost::shared_ptr<head_cv::SR300Grabber>	grabber;
	boost::mutex								mutex;

	head_cv::PoseEstimationStep				pose_estimation_step;
	head_cv::FilteringCropBoxStep			filtering_crop_box_step;
};

class MainWindow : public QMainWindow
{
	Q_OBJECT
	public:
		explicit MainWindow(QWidget *parent = 0);
		~MainWindow();

		bool wasStopped() const;
		void spinOnce();

	private slots:
		void closeViewer() { _viewer->close(); }
		void sensor0GrabToggleButtonClicked(bool checked_);
		void sensor1GrabToggleButtonClicked(bool checked_);
		void sensor0ToggleCrop(bool checked_);
		void sensor1ToggleCrop(bool checked_);
		void updateViewer(int device_index_);

	private:
		boost::shared_ptr<SensorCloudProcessing> FormSR300CloudInfo(int device_index_);

		bool tryToggleDevice(int device_index_, bool want_to_start_);

	private:
		Ui::MainWindow *ui;

		boost::shared_ptr<pcl::visualization::PCLVisualizer> _viewer;
		std::vector< boost::shared_ptr<SensorCloudProcessing> > _sensors_cloud_processing;
};

#endif // MAINWINDOW_H
