#include "mainwindow.h"
#include <QSignalMapper>

MainWindow::MainWindow(QWidget *parent) :
	QMainWindow(parent),
	ui(new Ui::MainWindow)
{
	ui->setupUi(this);

	_viewer = boost::make_shared<pcl::visualization::PCLVisualizer>("Viewer");
	_viewer->setBackgroundColor(0.25, 0.25, 0.25);
	_viewer->addCoordinateSystem(0.1);
	_viewer->initCameraParameters();
	_viewer->setCameraPosition(0.0, 0.0, -1.5, 0.0, 0.0, 0.0);
	_viewer->setShowFPS(true);
	_viewer->setSize(960, 640);
	_viewer->setPosition(8, 8);
	this->move(880, 16);

	connect(ui->quitButton, SIGNAL(pressed()), this, SLOT(closeViewer()));
	connect(ui->Dev0PushButton, SIGNAL(clicked(bool)), this, SLOT(sensor0GrabToggleButtonClicked(bool)));
	connect(ui->Dev1PushButton, SIGNAL(clicked(bool)), this, SLOT(sensor1GrabToggleButtonClicked(bool)));
	connect(ui->sensor_0_crop_button, SIGNAL(clicked(bool)), this, SLOT(sensor0ToggleCrop(bool)));
	connect(ui->sensor_1_crop_button, SIGNAL(clicked(bool)), this, SLOT(sensor1ToggleCrop(bool)));

	QSignalMapper* signalMapper = new QSignalMapper(this);
	connect(ui->sensor_0_crop_width, SIGNAL(valueChanged(double)), signalMapper, SLOT(map()));
	signalMapper->setMapping(ui->sensor_0_crop_width, 0);
	connect(ui->sensor_0_crop_height, SIGNAL(valueChanged(double)), signalMapper, SLOT(map()));
	signalMapper->setMapping(ui->sensor_0_crop_height, 0);
	connect(ui->sensor_0_crop_depth, SIGNAL(valueChanged(double)), signalMapper, SLOT(map()));
	signalMapper->setMapping(ui->sensor_0_crop_depth, 0);
	connect(ui->sensor_0_crop_translation_x, SIGNAL(valueChanged(double)), signalMapper, SLOT(map()));
	signalMapper->setMapping(ui->sensor_0_crop_translation_x, 0);
	connect(ui->sensor_0_crop_translation_y, SIGNAL(valueChanged(double)), signalMapper, SLOT(map()));
	signalMapper->setMapping(ui->sensor_0_crop_translation_y, 0);
	connect(ui->sensor_0_crop_translation_z, SIGNAL(valueChanged(double)), signalMapper, SLOT(map()));
	signalMapper->setMapping(ui->sensor_0_crop_translation_z, 0);
	connect(ui->sensor_0_crop_roll, SIGNAL(valueChanged(double)), signalMapper, SLOT(map()));
	signalMapper->setMapping(ui->sensor_0_crop_roll, 0);
	connect(ui->sensor_0_crop_yaw, SIGNAL(valueChanged(double)), signalMapper, SLOT(map()));
	signalMapper->setMapping(ui->sensor_0_crop_yaw, 0);
	connect(ui->sensor_0_crop_pitch, SIGNAL(valueChanged(double)), signalMapper, SLOT(map()));
	signalMapper->setMapping(ui->sensor_0_crop_pitch, 0);
	connect(ui->sensor_1_crop_width, SIGNAL(valueChanged(double)), signalMapper, SLOT(map()));
	signalMapper->setMapping(ui->sensor_1_crop_width, 1);
	connect(ui->sensor_1_crop_height, SIGNAL(valueChanged(double)), signalMapper, SLOT(map()));
	signalMapper->setMapping(ui->sensor_1_crop_height, 1);
	connect(ui->sensor_1_crop_depth, SIGNAL(valueChanged(double)), signalMapper, SLOT(map()));
	signalMapper->setMapping(ui->sensor_1_crop_depth, 1);
	connect(ui->sensor_1_crop_translation_x, SIGNAL(valueChanged(double)), signalMapper, SLOT(map()));
	signalMapper->setMapping(ui->sensor_1_crop_translation_x, 1);
	connect(ui->sensor_1_crop_translation_y, SIGNAL(valueChanged(double)), signalMapper, SLOT(map()));
	signalMapper->setMapping(ui->sensor_1_crop_translation_y, 1);
	connect(ui->sensor_1_crop_translation_z, SIGNAL(valueChanged(double)), signalMapper, SLOT(map()));
	signalMapper->setMapping(ui->sensor_1_crop_translation_z, 1);
	connect(ui->sensor_1_crop_roll, SIGNAL(valueChanged(double)), signalMapper, SLOT(map()));
	signalMapper->setMapping(ui->sensor_1_crop_roll, 1);
	connect(ui->sensor_1_crop_yaw, SIGNAL(valueChanged(double)), signalMapper, SLOT(map()));
	signalMapper->setMapping(ui->sensor_1_crop_yaw, 1);
	connect(ui->sensor_1_crop_pitch, SIGNAL(valueChanged(double)), signalMapper, SLOT(map()));
	signalMapper->setMapping(ui->sensor_1_crop_pitch, 1);
	connect(signalMapper, SIGNAL(mapped(int)), this, SLOT(updateViewer(int)));

	for (int dev_index = 0; dev_index < head_cv::SR300Grabber::getAvailableDeviceCount(); ++dev_index)
	{
		boost::shared_ptr<SensorCloudProcessing> cloud_info = FormSR300CloudInfo(dev_index);

		QPushButton* grab_button = ui->centralWidget->findChild<QPushButton*>("Dev" + QString::number(dev_index) + "PushButton");
		grab_button->setEnabled(true);

		cloud_info->pose_estimation_step.init(ui->centralWidget, "sensor_" + QString::number(dev_index) + "_");
		cloud_info->filtering_crop_box_step.init(ui->centralWidget, "sensor_" + QString::number(dev_index) + "_crop_", _viewer);

		_sensors_cloud_processing.push_back(cloud_info);
	}

}

MainWindow::~MainWindow()
{
	for (auto const& device : _sensors_cloud_processing)
	{
		if (!device->grabber->isRunning()) continue;
		device->grabber->stop();
	}
	delete ui;
}

bool MainWindow::wasStopped() const
{
	return _viewer->wasStopped();
}

void MainWindow::spinOnce()
{
	_viewer->spinOnce();

	for (auto const& sensor_cloud_processing : _sensors_cloud_processing)
	{
		if (!sensor_cloud_processing->grabber->isRunning()) continue;

		boost::mutex::scoped_try_lock lock(sensor_cloud_processing->mutex);

		if (!lock.owns_lock() || !sensor_cloud_processing->cloud) continue;

		const std::string& cloud_name = sensor_cloud_processing->grabber->getName();
		_viewer->removePointCloud(cloud_name);
		_viewer->addPointCloud(sensor_cloud_processing->cloud, cloud_name);
		_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloud_name);
	}
}

boost::shared_ptr<SensorCloudProcessing> MainWindow::FormSR300CloudInfo(int device_index_)
{
	boost::shared_ptr<SensorCloudProcessing> sensor_cloud_processing = boost::make_shared<SensorCloudProcessing>();

	sensor_cloud_processing->grabber = boost::make_shared<head_cv::SR300Grabber>(device_index_);

	boost::function<head_cv::SR300Grabber::SignalSR300DataGrabbed> callback_function =
		[sensor_cloud_processing] (const head_cv::GrabbedData grabbed_data_)
	{
		boost::mutex::scoped_lock lock(sensor_cloud_processing->mutex);

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = grabbed_data_.xyz_rgb_cloud->makeShared();

		cloud = sensor_cloud_processing->pose_estimation_step.process(cloud, grabbed_data_);
		cloud = sensor_cloud_processing->filtering_crop_box_step.process(cloud);

		sensor_cloud_processing->cloud = cloud;
	};

	sensor_cloud_processing->grabber->registerCallback(callback_function);

	return sensor_cloud_processing;
}

void MainWindow::sensor0GrabToggleButtonClicked(bool checked_)
{
	ui->Dev0PushButton->setChecked(tryToggleDevice(0, checked_) ? checked_ : !checked_);
}

void MainWindow::sensor1GrabToggleButtonClicked(bool checked_)
{
	ui->Dev1PushButton->setChecked(tryToggleDevice(1, checked_) ? checked_ : !checked_);
}

void MainWindow::sensor0ToggleCrop(bool checked_)
{
	head_cv::FilteringCropBoxStep& filteringCropBox = _sensors_cloud_processing[0]->filtering_crop_box_step;
	filteringCropBox.toggleCropBoxParameters(checked_);
	if (checked_)
		filteringCropBox.updateCropBox();
	else
		_viewer->removeShape(filteringCropBox.getCropBoxName());
}

void MainWindow::sensor1ToggleCrop(bool checked_)
{
	head_cv::FilteringCropBoxStep& filteringCropBox = _sensors_cloud_processing[1]->filtering_crop_box_step;
	filteringCropBox.toggleCropBoxParameters(checked_);
	if (checked_)
		filteringCropBox.updateCropBox();
	else
		_viewer->removeShape(filteringCropBox.getCropBoxName());
}

bool MainWindow::tryToggleDevice(int const device_index_, bool const want_to_start_)
{
	if (!(device_index_ < _sensors_cloud_processing.size())) return false;

	if (want_to_start_)
	{
		if (_sensors_cloud_processing[device_index_]->grabber->isRunning()) return true;

		_sensors_cloud_processing[device_index_]->filtering_crop_box_step.setCropBoxEnabled(true);
		_sensors_cloud_processing[device_index_]->pose_estimation_step.setEstimationPoseEnabled(true);
		_sensors_cloud_processing[device_index_]->grabber->start();
	}
	else
	{
		if (!_sensors_cloud_processing[device_index_]->grabber->isRunning()) return true;

		_sensors_cloud_processing[device_index_]->grabber->stop();
		_sensors_cloud_processing[device_index_]->filtering_crop_box_step.setCropBoxEnabled(false);
		_sensors_cloud_processing[device_index_]->pose_estimation_step.setEstimationPoseEnabled(false);
		_viewer->removePointCloud(_sensors_cloud_processing[device_index_]->grabber->getName());
		_viewer->removeShape(_sensors_cloud_processing[device_index_]->filtering_crop_box_step.getCropBoxName());
	}

	return true;
}

void MainWindow::updateViewer(int device_index_)
{
	_sensors_cloud_processing[device_index_]->filtering_crop_box_step.updateCropBox();
}
