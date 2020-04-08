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

typedef pcl::PointXYZINormal PointINormal;
typedef pcl::PointCloud<PointINormal> CloudINormal;
typedef pcl::PointCloud<PointINormal>::Ptr CloudINormalPtr;

typedef pcl::PointXYZINormal PointIVoxel;
typedef pcl::PointCloud<PointIVoxel> CloudIVoxel;
typedef pcl::PointCloud<PointIVoxel>::Ptr CloudIVoxelPtr; // I:Time, Normal:voxel position

//typedef boost::multi_array<Eigen::Vector3f, 3> MultiArrayEVec3f;
typedef boost::multi_array<State, 3> MultiArrayEVec3f;
MultiArrayEVec3f pca3rd_voxel(boost::extents[VOXEL_NUM_X][VOXEL_NUM_Y][VOXEL_NUM_Z]);


class DynamicVoxelFilter
{
	public:
		DynamicVoxelFilter(void);

		CloudINormal pc_callback(const sensor_msgs::PointCloud2ConstPtr&);
		void odom_callback(const nav_msgs::OdometryConstPtr&);
		void execution(void);
		Eigen::Matrix3f eigen_estimation(CloudIVoxelPtr);
		CloudIVoxelPtr pc_addressing(CloudIVoxelPtr);
		void input_pt2voxel(CloudIVoxelPtr);
		void input_pca3rd2voxel(void);
		void chronological_pca3rd_variance_calculation(void);
	
	private:
		bool pc_callback_flag = false;
		bool odom_callback_flag = false;
		bool tf_listen_flag = false;
		bool first_flag = false;

		const static int X = 0, Y = 1, Z = 2;
		const static int Occupied = 1, Unoccupied = 0, Unknown = -1;
		const float Hz = 100.0;
		const float voxel_size_x = MAX_RANGE_X / VOXEL_NUM_X,
					voxel_size_y = MAX_RANGE_Y / VOXEL_NUM_Y,
					voxel_size_z = MAX_RANGE_Z / VOXEL_NUM_Z;

		ros::Subscriber pc_subscriber;
		ros::Subscriber odom_subscriber;
		ros::Publisher dynamic_pc_publisher;
		ros::Publisher static_pc_publisher;
		
		tf::TransformListener listener;
		tf::StampedTransform transform;

		// PointCloud
        sensor_msgs::PointCloud2 input_pc;
        CloudINormalPtr pcl_input_pc {new CloudINormal};
		CloudIVoxelPtr addressed_pc_ {new CloudIVoxel};

		// Voxel
		CloudIVoxelPtr Voxel[VOXEL_NUM_X][VOXEL_NUM_Y][VOXEL_NUM_Z]; // storage pc_ according to voxel address
		Eigen::Vector3f chronological_variance; // storage PCA 3rd vectors

		// Memory
		std::vector<MultiArrayEVec3f> pca3rd_chronological_memories;

        struct Status{
            int occupation;
            float dynamic_probability;
            CloudINormalPtr pcl_pc;
            Eigen::Vector3f 3rd_main_component;
        };

        std::vector<std::vector<std::vector<Status> > > voxel_grid;
};

#endif// __DYNAMIC_VOXEL_FILTER_H
