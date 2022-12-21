#include "PoseEstimationStep.h"

#include "constants.h"

#include <opencv2/opencv.hpp>

using namespace head_cv;

PoseEstimationStep::PoseEstimationStep()
{
	// Do nothing.
}

void PoseEstimationStep::init(QWidget* central_widget_, QString prefix_)
{
	_x = central_widget_->findChild<QDoubleSpinBox*>(prefix_ + "x");
	_y = central_widget_->findChild<QDoubleSpinBox*>(prefix_ + "y");
	_z = central_widget_->findChild<QDoubleSpinBox*>(prefix_ + "z");

	_roll  = central_widget_->findChild<QDoubleSpinBox*>(prefix_ + "roll");
	_yaw   = central_widget_->findChild<QDoubleSpinBox*>(prefix_ + "yaw");
	_pitch = central_widget_->findChild<QDoubleSpinBox*>(prefix_ + "pitch");

	_estimate_button = central_widget_->findChild<QPushButton*>(prefix_ + "charuco_estimate_button");

	_premult_x = central_widget_->findChild<QDoubleSpinBox*>(prefix_ + "p_e_premult_x");
	_premult_y = central_widget_->findChild<QDoubleSpinBox*>(prefix_ + "p_e_premult_y");
	_premult_z = central_widget_->findChild<QDoubleSpinBox*>(prefix_ + "p_e_premult_z");

	_premult_roll  = central_widget_->findChild<QDoubleSpinBox*>(prefix_ + "p_e_premult_roll");
	_premult_yaw   = central_widget_->findChild<QDoubleSpinBox*>(prefix_ + "p_e_premult_yaw");
	_premult_pitch = central_widget_->findChild<QDoubleSpinBox*>(prefix_ + "p_e_premult_pitch");
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PoseEstimationStep::process(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_, const head_cv::GrabbedData grabbed_data_)
{
	if (estimatePosOnNextFrame())
	{
		cv::Vec3d tvec, rvec;

		CharucoDiamondPoseEstimation
		(
			grabbed_data_.color_frame,
			grabbed_data_.color_camera_matrix,
			cv::Mat(),
			tvec,
			rvec
		);

		tvec /= 10;

		Eigen::Vector3f point = Eigen::Vector3f(tvec[0] - 0.025, tvec[1], -tvec[2]);

		float x_rot = rvec[0] - M_PI;
		float y_rot = rvec[2];
		float z_rot = rvec[1];

		Eigen::Matrix3f rot_m;

		rot_m = Eigen::AngleAxisf(x_rot, Eigen::Vector3f::UnitX()) *
				Eigen::AngleAxisf(y_rot, Eigen::Vector3f::UnitY()) *
				Eigen::AngleAxisf(z_rot, Eigen::Vector3f::UnitZ());

		point = rot_m * point;

		setOriginInUI(point[0], point[1], point[2]);
		setOrientationInUI(x_rot, y_rot, z_rot);

		cloud_->sensor_origin_ = retrieveOrigin();
		cloud_->sensor_orientation_ = retrieveOrientation();
	}
	else
	{
		// Do nothing.
	}

	return cloud_;
}

void PoseEstimationStep::CharucoDiamondPoseEstimation(cv::Mat const &frame_, cv::Mat const &camera_matrix_, cv::Mat const &dist_coeffs_, cv::Vec3d &OUT_tvec_, cv::Vec3d &OUT_rvec_)
{
	const float CHESSBOARD_SQUARE_LENGTH = 0.40f;
	const float ARUCO_MARKER_LENGTH = 0.24f;

	std::vector<int> marker_ids;
	std::vector< std::vector<cv::Point2f> > marker_corners;

	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
	cv::aruco::detectMarkers(frame_, dictionary, marker_corners, marker_ids);

	if (marker_ids.size() <= 0)
	{
		return;
	}

	std::vector< cv::Vec4i > diamond_ids;
	std::vector< std::vector<cv::Point2f> > diamond_corners;

	cv::aruco::detectCharucoDiamond(frame_, marker_corners, marker_ids, CHESSBOARD_SQUARE_LENGTH / ARUCO_MARKER_LENGTH, diamond_corners, diamond_ids, camera_matrix_);

	std::vector<cv::Vec3d> tvecs, rvecs;
	cv::aruco::estimatePoseSingleMarkers(diamond_corners, CHESSBOARD_SQUARE_LENGTH, camera_matrix_, dist_coeffs_, rvecs, tvecs);

	//for (unsigned int i = 0; i < rvecs.size(); i++)
	//{
	//	cv::aruco::drawAxis(frame_, camera_matrix_, dist_coeffs_, rvecs[i], tvecs[i], 1);
	//}

	if (tvecs.size() != 0)
	{
		OUT_tvec_ = tvecs[0];
		OUT_rvec_ = rvecs[0];
	}
	else
	{
		OUT_tvec_ = cv::Vec3d(0.0, 0.0, 0.0);
		OUT_rvec_ = cv::Vec3d(0.0, 0.0, 0.0);
	}
}

Eigen::Vector4f PoseEstimationStep::retrieveOrigin() const
{
	return Eigen::Vector4f(	_x->value() * MM_TO_M,
							_y->value() * MM_TO_M,
							_z->value() * MM_TO_M,
							1.0f);
}

Eigen::Quaternionf PoseEstimationStep::retrieveOrientation() const
{
	return	Eigen::AngleAxisf(_roll->value()  * DEG_TO_RAD, Eigen::Vector3f::UnitX()) *
			Eigen::AngleAxisf(_yaw->value()   * DEG_TO_RAD, Eigen::Vector3f::UnitY()) *
			Eigen::AngleAxisf(_pitch->value() * DEG_TO_RAD, Eigen::Vector3f::UnitZ());
}

void PoseEstimationStep::setOriginInUI(float const x_, float const y_, float const z_)
{
	_x->setValue(x_ * M_TO_MM);
	_y->setValue(y_ * M_TO_MM);
	_z->setValue(z_ * M_TO_MM);
}

void PoseEstimationStep::setOrientationInUI(float const roll_, float const yaw_, float const pitch_)
{
	_roll->setValue(roll_ * RAD_TO_DEG);
	_yaw->setValue(yaw_ * RAD_TO_DEG);
	_pitch->setValue(pitch_ * RAD_TO_DEG);
}
