#include "SR300Grabber.h"
#include <iostream>

using namespace head_cv;

boost::shared_ptr<rs::context> SR300Grabber::_rs_context;

int SR300Grabber::getAvailableDeviceCount()
{
	_rs_context = (_rs_context) ? _rs_context : boost::make_shared<rs::context>();

	return _rs_context->get_device_count();
}

SR300Grabber::SR300Grabber(int device_index_) :
	_device_index(device_index_),
	_color_width(1920),
	_color_height(1080),
	_color_fps(30),
	_depth_width(640),
	_depth_height(480),
	_depth_fps(30),
	_infrared_width(640),
	_infrared_height(480),
	_infrared_fps(30),
	_running(false),
	_signal_point_xyz(NULL)
{
	_rs_context = (_rs_context) ? _rs_context : boost::make_shared<rs::context>();

	_sr_300 = _rs_context->get_device(_device_index);

	_sr_300->enable_stream(rs::stream::color, _color_width, _color_height, rs::format::bgr8, _color_fps);
	_sr_300->enable_stream(rs::stream::depth, _depth_width, _depth_height, rs::format::z16, _depth_fps);
	_sr_300->enable_stream(rs::stream::infrared, _infrared_width, _infrared_height, rs::format::y8, _infrared_fps);

	_signal_point_xyz = createSignal<SignalSR300DataGrabbed>();
}

SR300Grabber::~SR300Grabber()
{
	stop();
	disconnect_all_slots<SignalSR300DataGrabbed>();
}

void SR300Grabber::start()
{
	_sr_300->start();
	_running = true;
	_thread = boost::thread(&SR300Grabber::threadFunction, this);
}

void SR300Grabber::stop()
{
	boost::unique_lock<boost::mutex> lock(_mutex);
	{
		_running = false;
	}
	lock.unlock();
	_thread.join();
	_sr_300->stop();
}

bool SR300Grabber::isRunning() const
{
	return _running;
}

std::string SR300Grabber::getName() const
{
	return _sr_300->get_serial();
}

float SR300Grabber::getFramesPerSecond() const
{
	return _depth_fps;
}

void SR300Grabber::threadFunction()
{
	while (_running)
	{
		boost::unique_lock<boost::mutex> lock(_mutex);
		{
			_sr_300->poll_for_frames();

			_depth_intrin			= _sr_300->get_stream_intrinsics(rs::stream::depth);
			_c_a_to_d_intrin		= _sr_300->get_stream_intrinsics(rs::stream::color_aligned_to_depth);
			_color_intrin			= _sr_300->get_stream_intrinsics(rs::stream::color);
			_rectified_color_intrin = _sr_300->get_stream_intrinsics(rs::stream::rectified_color);

			_depth_frame =
				cv::Mat
				(
					cv::Size(_depth_intrin.width, _depth_intrin.height),
					CV_16UC1,
					(void*)_sr_300->get_frame_data(rs::stream::depth),
					cv::Mat::AUTO_STEP
					);

			_color_aligned_to_depth_frame =
				cv::Mat
				(
					cv::Size(_c_a_to_d_intrin.width, _c_a_to_d_intrin.height),
					CV_8UC3,
					(void*)_sr_300->get_frame_data(rs::stream::color_aligned_to_depth),
					cv::Mat::AUTO_STEP
				);

			_color_frame =
				cv::Mat
				(
					cv::Size(_color_intrin.width, _color_intrin.height),
					CV_8UC3,
					(void*)_sr_300->get_frame_data(rs::stream::color),
					cv::Mat::AUTO_STEP
				);

			_rectified_color_frame =
				cv::Mat
				(
					cv::Size(_rectified_color_intrin.width, _rectified_color_intrin.height),
					CV_8UC3,
					(void*)_sr_300->get_frame_data(rs::stream::rectified_color),
					cv::Mat::AUTO_STEP
				);

		}
		lock.unlock();

		if (_signal_point_xyz->num_slots() > 0)
		{
			GrabbedData grabbed_data;

			grabbed_data.sensor_serial	= _sr_300->get_serial();
			grabbed_data.xyz_rgb_cloud	= convertRGBDepthToPointXYZRGB();
			
			grabbed_data.depth_frame			= _depth_frame;
			grabbed_data.color_frame			= _color_frame;
			grabbed_data.rectified_color_frame	= _rectified_color_frame;

			grabbed_data.color_camera_matrix	= FormCameraMatrix(_rectified_color_intrin);
			grabbed_data.color_dist_coeffs		= FormDistortionVector(_rectified_color_intrin);
			
			_signal_point_xyz->operator()(grabbed_data);
		}
		else
		{
			// Do nothing.
		}
	}
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr SR300Grabber::convertRGBDepthToPointXYZRGB()
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

	rs::intrinsics depth_intrin = _sr_300->get_stream_intrinsics(rs::stream::depth);

	float depth_scale = _sr_300->get_depth_scale();

	int depth_width  = depth_intrin.width;
	int depth_height = depth_intrin.height;

	cloud->width  = static_cast<uint32_t>(depth_width);
	cloud->height = static_cast<uint32_t>(depth_height);

	cloud->is_dense = false;
	cloud->points.resize(cloud->height * cloud->width);

	for (int y = 0; y < depth_height; ++y)
	{
		for (int x = 0; x < depth_width; ++x)
		{
			int point_index = y * depth_width + x;

			uint16_t depth_value = _depth_frame.at<uint16_t>(point_index);

			if (depth_value == 0) continue;

			pcl::PointXYZRGB& point = cloud->points[point_index];

			rs::float2 depth_pixel = { (float)x, (float)y };

			float depth_in_meters = depth_value * depth_scale;

			rs::float3 depth_point = depth_intrin.deproject(depth_pixel, depth_in_meters);

			point.x = -1 * depth_point.x;
			point.y = -1 * depth_point.y;
			point.z =      depth_point.z;

			cv::Vec3b color_point = _color_aligned_to_depth_frame.at<cv::Vec3b>(point_index);

			point.r = color_point[2];
			point.g = color_point[1];
			point.b = color_point[0];
		}
	}

	return cloud;
}

cv::Mat SR300Grabber::FormCameraMatrix(const rs::intrinsics &intrinsics_)
{
	cv::Mat camera_matrix(3, 3, CV_32F);

	camera_matrix.at<float>(0, 0) = intrinsics_.fx;
	camera_matrix.at<float>(0, 1) = 0.0f;
	camera_matrix.at<float>(0, 2) = intrinsics_.ppx;

	camera_matrix.at<float>(1, 0) = 0.0f;
	camera_matrix.at<float>(1, 1) = intrinsics_.fy;
	camera_matrix.at<float>(1, 2) = intrinsics_.ppy;

	camera_matrix.at<float>(2, 0) = 0.0f;
	camera_matrix.at<float>(2, 1) = 0.0f;
	camera_matrix.at<float>(2, 2) = 1.0f;

	return camera_matrix;
}

cv::Mat SR300Grabber::FormDistortionVector(const rs::intrinsics &intrinsics_)
{
	cv::Mat dist_coeffs(1, 5, CV_32F);

	for (int i = 0; i < 5; ++i)
	{
		dist_coeffs.at<float>(0, i) = intrinsics_.coeffs[i];
	}

	return dist_coeffs;
}
