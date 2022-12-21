#include "FilteringCropBoxStep.h"
#include "constants.h"

using namespace head_cv;

void FilteringCropBoxStep::init(QWidget* central_widget_, QString prefix_, pcl::visualization::PCLVisualizer::Ptr viewer_)
{
	crop_button = central_widget_->findChild<QPushButton*>(prefix_ + "button");

	width = central_widget_->findChild<QDoubleSpinBox*>(prefix_ + "width");
	height = central_widget_->findChild<QDoubleSpinBox*>(prefix_ + "height");
	depth = central_widget_->findChild<QDoubleSpinBox*>(prefix_ + "depth");

	translation_x = central_widget_->findChild<QDoubleSpinBox*>(prefix_ + "translation_x");
	translation_y = central_widget_->findChild<QDoubleSpinBox*>(prefix_ + "translation_y");
	translation_z = central_widget_->findChild<QDoubleSpinBox*>(prefix_ + "translation_z");

	roll  = central_widget_->findChild<QDoubleSpinBox*>(prefix_ + "roll");
	yaw   = central_widget_->findChild<QDoubleSpinBox*>(prefix_ + "yaw");
	pitch = central_widget_->findChild<QDoubleSpinBox*>(prefix_ + "pitch");

	viewer = viewer_;
	crop_box_name = (prefix_ + "box").toStdString();
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr FilteringCropBoxStep::process(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_)
{
	if (!cropEnabled()) return cloud_;

	pcl::CropBox<pcl::PointXYZRGB> crop_box_filter;

	crop_box_filter.setMin(retrieveMinPoint());
	crop_box_filter.setMax(retrieveMaxPoint());
	crop_box_filter.setTranslation(retrieveTranslation());
	crop_box_filter.setRotation(retrieveRotationVector());

	crop_box_filter.setInputCloud(cloud_);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>());
	crop_box_filter.filter(*cloud_filtered);

	return cloud_filtered;
}

void FilteringCropBoxStep::updateCropBox()
{
	viewer->removeShape(crop_box_name);
	viewer->addCube(
		retrieveTranslation(),
		retrieveRotation(),
		retreiveWidth(),
		retreiveHeight(),
		retreiveDepth(),
		crop_box_name
	);
#if 0
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.1f, crop_box_name);
#else
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.8f, 0.8f, 0.8f, crop_box_name);
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, crop_box_name);
#endif
}

inline Eigen::Vector4f FilteringCropBoxStep::retrieveMinPoint() const
{
	return Eigen::Vector4f(-(width->value()*MM_TO_M)/2, -(height->value()*MM_TO_M)/2, -(depth->value()*MM_TO_M)/2, 1.0f);
}

inline Eigen::Vector4f FilteringCropBoxStep::retrieveMaxPoint() const
{
	return Eigen::Vector4f((width->value()*MM_TO_M)/2, (height->value()*MM_TO_M)/2, (depth->value()*MM_TO_M)/2, 1.0f);
}

inline  Eigen::Vector3f FilteringCropBoxStep::retrieveTranslation() const
{
	return Eigen::Vector3f(	translation_x->value() * MM_TO_M,
							translation_y->value() * MM_TO_M,
							translation_z->value() * MM_TO_M);
}

inline  Eigen::Vector3f FilteringCropBoxStep::retrieveRotationVector() const
{
	return Eigen::Vector3f(	roll->value()  * DEG_TO_RAD,
							yaw->value()   * DEG_TO_RAD,
							pitch->value() * DEG_TO_RAD);
}

inline Eigen::Quaternionf FilteringCropBoxStep::retrieveRotation() const
{
	return	Eigen::AngleAxisf(roll->value()  * DEG_TO_RAD, Eigen::Vector3f::UnitX()) *
			Eigen::AngleAxisf(yaw->value()   * DEG_TO_RAD, Eigen::Vector3f::UnitY()) *
			Eigen::AngleAxisf(pitch->value() * DEG_TO_RAD, Eigen::Vector3f::UnitZ());
}

inline float FilteringCropBoxStep::retreiveWidth()  const { return width->value() * MM_TO_M; }
inline float FilteringCropBoxStep::retreiveHeight() const { return height->value() * MM_TO_M; }
inline float FilteringCropBoxStep::retreiveDepth()  const { return depth->value() * MM_TO_M; }

void FilteringCropBoxStep::toggleCropBoxParameters(bool enabled_)
{
	width->setEnabled(enabled_);
	height->setEnabled(enabled_);
	depth->setEnabled(enabled_);
	translation_x->setEnabled(enabled_);
	translation_y->setEnabled(enabled_);
	translation_z->setEnabled(enabled_);
	roll->setEnabled(enabled_);
	yaw->setEnabled(enabled_);
	pitch->setEnabled(enabled_);
}
