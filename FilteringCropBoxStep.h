#pragma once
#ifndef FILTERING_CROP_BOX_STEP_H
#define FILTERING_CROP_BOX_STEP_H

#include <QDoubleSpinBox>
#include <QPushButton>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/crop_box.h>

namespace head_cv
{
	class FilteringCropBoxStep
	{
	public:
		void init(QWidget* central_widget_, QString prefix_, pcl::visualization::PCLVisualizer::Ptr viewer_);

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr process(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_);

		bool cropEnabled() const { return crop_button->isChecked(); }
		void setCropBoxEnabled(bool enabled_) { crop_button->setEnabled(enabled_); }
		void toggleCropBoxParameters(bool enabled_);

		Eigen::Vector4f retrieveMinPoint() const;
		Eigen::Vector4f retrieveMaxPoint() const;
		Eigen::Vector3f retrieveTranslation() const;
		Eigen::Vector3f retrieveRotationVector() const;
		Eigen::Quaternionf retrieveRotation() const;

		float retreiveWidth() const;
		float retreiveHeight() const;
		float retreiveDepth() const;
		const std::string& getCropBoxName() { return crop_box_name; }

		void updateCropBox();

	private:
		QPushButton* crop_button;

		QDoubleSpinBox* width;
		QDoubleSpinBox* height;
		QDoubleSpinBox* depth;

		QDoubleSpinBox* translation_x;
		QDoubleSpinBox* translation_y;
		QDoubleSpinBox* translation_z;

		QDoubleSpinBox* roll;
		QDoubleSpinBox* yaw;
		QDoubleSpinBox* pitch;

		std::string crop_box_name;
		pcl::visualization::PCLVisualizer::Ptr viewer;
	};
}

#endif
