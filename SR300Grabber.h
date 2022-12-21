// SR300Grabber.h
#pragma once

#ifndef SR300_GRABBER_H
#define SR300_GRABBER_H

#include <librealsense/rs.hpp>

#include <pcl/io/boost.h>
#include <pcl/io/grabber.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <opencv2/core/core.hpp>

namespace head_cv
{
	struct GrabbedData
	{
		std::string sensor_serial;

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyz_rgb_cloud;

		cv::Mat depth_frame;
		cv::Mat color_frame;
		cv::Mat rectified_color_frame;

		cv::Mat color_camera_matrix;
		cv::Mat color_dist_coeffs;
	};

	struct pcl::PointXYZRGB;
	template <typename T> class pcl::PointCloud;

	template<class Interface>
	inline void SafeRelease(Interface*& i_release_)
	{
		if (i_release_ != NULL)
		{
			i_release_->Release();
			i_release_ = NULL;
		}
	}

	class SR300Grabber : public pcl::Grabber
	{
		public:
			typedef void (SignalSR300DataGrabbed)(const GrabbedData);

		public:
			static int getAvailableDeviceCount();

		public:
			SR300Grabber(int device_index_);
			virtual ~SR300Grabber();
			virtual void start();
			virtual void stop();
			virtual bool isRunning() const;
			virtual std::string getName() const;
			virtual float getFramesPerSecond() const;

		protected:
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertRGBDepthToPointXYZRGB();

			void threadFunction();

		private:
			cv::Mat FormCameraMatrix(const rs::intrinsics &intrinsics_);
			cv::Mat FormDistortionVector(const rs::intrinsics &intrinsics_);

		//==============================================================================================
		// Static part
		//==============================================================================================
		private:

			static boost::shared_ptr<rs::context> _rs_context;

		//==============================================================================================
		// Member part
		//==============================================================================================
		protected:
			int			_device_index;
			rs::device* _sr_300;

			boost::signals2::signal<SignalSR300DataGrabbed>* _signal_point_xyz;

			boost::thread _thread;
			mutable boost::mutex _mutex;

			bool	_running;

			int		_color_width;
			int		_color_height;
			int		_color_fps;

			int		_infrared_width;
			int		_infrared_height;
			int		_infrared_fps;

			int		_depth_width;
			int		_depth_height;
			int		_depth_fps;

			rs::intrinsics _depth_intrin;
			rs::intrinsics _c_a_to_d_intrin;
			rs::intrinsics _color_intrin;
			rs::intrinsics _rectified_color_intrin;

			cv::Mat _depth_frame;
			cv::Mat _color_aligned_to_depth_frame;
			cv::Mat _color_frame;
			cv::Mat _rectified_color_frame;
	};
}

#endif
