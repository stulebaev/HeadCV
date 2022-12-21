#pragma once
#ifndef POSE_ESTIMATION_STEP_H
#define POSE_ESTIMATION_STEP_H

#include <QDoubleSpinBox>
#include <QPushButton>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>

#include "SR300Grabber.h"

namespace head_cv
{
	class PoseEstimationStep : public QObject
	{
		Q_OBJECT
		public:
			PoseEstimationStep();

			void init(QWidget* central_widget_, QString prefix_);

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr process(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_, const head_cv::GrabbedData grabbed_data_);
			bool estimatePosOnNextFrame() const { return _estimate_button->isChecked(); }
			void setEstimationPoseEnabled(bool enabled_) { _estimate_button->setEnabled(enabled_); }

			Eigen::Vector4f		retrieveOrigin() const;
			Eigen::Quaternionf	retrieveOrientation() const;

			void setOriginInUI(float const x_, float const y_, float const z_);
			void setOrientationInUI(float const roll_, float const yaw_, float const pitch_);

		private:
			void CharucoDiamondPoseEstimation(cv::Mat const &frame_, cv::Mat const &camera_matrix_, cv::Mat const &dist_coeffs_, cv::Vec3d &OUT_tvec_, cv::Vec3d &OUT_rvec_);

		private:
			QPushButton*	_estimate_button;

			QDoubleSpinBox* _x;
			QDoubleSpinBox* _y;
			QDoubleSpinBox* _z;

			QDoubleSpinBox* _roll;
			QDoubleSpinBox* _yaw;
			QDoubleSpinBox* _pitch;

			QDoubleSpinBox* _premult_x;
			QDoubleSpinBox* _premult_y;
			QDoubleSpinBox* _premult_z;

			QDoubleSpinBox* _premult_roll;
			QDoubleSpinBox* _premult_yaw;
			QDoubleSpinBox* _premult_pitch;
	};
}

#endif
