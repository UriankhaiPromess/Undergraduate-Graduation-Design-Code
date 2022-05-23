#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h> 
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/bilateral.h>
#include <pcl/filters/fast_bilateral.h>
#include <pcl/filters/fast_bilateral_omp.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/point_types.h>
#include <pcl/filters/convolution_3d.h>
#include <pcl/filters/convolution.h>
#include <pcl/search/kdtree.h>

int main(int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	if (pcl::io::loadPCDFile("D:\\dianyunchuli\\ro.pcd", *inputCloud) == -1)
	{
		PCL_ERROR("Couldn't read file test_pcd.pcd\n");
		return(-1);
	}
	pcl::filters::Convolution<pcl::PointXYZ, pcl::PointXYZ>  convolution;
	Eigen::ArrayXf gaussian_kernel(5);
	gaussian_kernel << 1.f / 16, 1.f / 4, 3.f / 8, 1.f / 4, 1.f / 16;
	convolution.setBordersPolicy(
		pcl::filters::Convolution<pcl::PointXYZ, pcl::PointXYZ>::BORDERS_POLICY_IGNORE);
	convolution.setDistanceThreshold(static_cast<float> (0.1));
	convolution.setInputCloud(inputCloud);
	convolution.setKernel(gaussian_kernel);
	convolution.convolveRows(*cloud);
	//convolution.convolve(*cloud);
	//显示
	pcl::visualization::PCLVisualizer viewer("passthrough viewer");
	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(inputCloud, "z");//按照z字段进行渲染
	viewer.addPointCloud(inputCloud, fildColor, "origin cloud");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "origin cloud");

	while (!viewer.wasStopped()) {
		viewer.spinOnce();
	}
	return (0);
}