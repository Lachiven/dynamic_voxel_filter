#ifndef __DYNAMIC_VOXEL_FILTER_H
#define __DYNAMIC_VOXEL_FILTER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/search/kdtree.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/common/pca.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/LU"

#include <boost/thread.hpp>
#include <boost/multi_array.hpp>


#define MAX_RANGE_X 100
#define MAX_RANGE_Y 100
#define MAX_RANGE_Z 10

#define VOXEL_NUM_X 500
#define VOXEL_NUM_Y 500
#define VOXEL_NUM_Z 50

#define MEMORY_SIZE 5

typedef pcl::PointXYZI PointI;
typedef pcl::PointCloud<PointI> CloudI;
typedef pcl::PointCloud<PointI>::Ptr CloudIPtr;

typedef pcl::PointXYZINormal PointIVoxel;
typedef pcl::PointCloud<PointIVoxel> CloudIVoxel;
typedef pcl::PointCloud<PointIVoxel>::Ptr CloudIVoxelPtr; // I:Time, Normal:voxel position

struct State{
	int occupatoin;
	Eigen::Vector3f vector;
};


#endif// __DYNAMIC_VOXEL_FILTER_H
