#include <ros/ros.h>
#include <iostream>
#include <omp.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
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


typedef pcl::PointXYZI PointI;
typedef pcl::PointCloud<PointI> CloudI;
typedef pcl::PointCloud<PointI>::Ptr CloudIPtr;

typedef pcl::PointXYZINormal PointTEvec;
typedef pcl::PointCloud<PointTEvec> CloudTEvec;
typedef pcl::PointCloud<PointTEvec>::Ptr CloudTEvecPtr; // I:Time, Normal:Eigen vector

typedef pcl::PointXYZINormal PointTPos;
typedef pcl::PointCloud<PointTPos> CloudTPos;
typedef pcl::PointCloud<PointTPos>::Ptr CloudTPosPtr; // I:Time, Normal:voxel position

typedef pcl::PointXYZHSV PointITV; // h:Intensity, s:Time, v:Variance
typedef pcl::PointCloud<PointITV> CloudITV;
typedef pcl::PointCloud<PointITV>::Ptr CloudITVPtr;


class DynamicVoxelFilter
{
	public:
		DynamicVoxelFilter(void);

		void pc_callback(const sensor_msgs::PointCloud2ConstPtr&);
		void odom_callback(const nav_msgs::OdometryConstPtr&);
		void execution(void);
		Eigen::Matrix3f eigen_estimation(CloudTPosPtr);
		CloudTPosPtr pc_addressing(CloudTPosPtr);
		void input_pt2Voxel(CloudTPosPtr);
		void voxel_shooting(void);
		void judge(void);

	private:
		bool pc_callback_flag = false;
		bool odom_callback_flag = false;
		bool tf_listen_flag = false;
		bool first_flag = false;

		const char *str_x = "normal_x";
		const char *str_y = "normal_y";
		const char *str_z = "normal_z";
		const char* filter_str[3] = {str_x, str_y, str_z};

		const static int X = 0, Y = 1, Z = 2;
		const static int voxel_num_x = 500, voxel_num_y = 500, voxel_num_z = 50;
		
		int filter_length[3] = {voxel_num_x/2, voxel_num_y/2, voxel_num_z/2};

		float Hz = 100.0;
		float voxel_size_x = 0.2, voxel_size_y = 0.2, voxel_size_z = 0.2;

		std::vector<CloudITVPtr> shot_voxel_list;

		ros::Subscriber pc_subscriber;
		ros::Subscriber odom_subscriber;
		ros::Publisher dynamic_pc_publisher;
		ros::Publisher static_pc_publisher;

		nav_msgs::Odometry odom;
		sensor_msgs::PointCloud2 tmp_pc;
		sensor_msgs::PointCloud2 transformed_pc;
		
		tf::TransformListener listener;
		tf::StampedTransform transform;

		//PointCloud
		CloudIPtr input_I_pc_ {new CloudI};
		CloudTPosPtr positioning_pc_ {new CloudTPos}; // T:Time, Pos:.normal_x,y,z <=> voxel position x,y,z
		CloudTPosPtr input_TPos_pc_ {new CloudTPos}; // I:Intensity, T:Time, V:Variance
		CloudTPosPtr transformed_TPos_pc_ {new CloudTPos}; // I:Intensity, T:Time, V:Variance
		CloudTPosPtr addressed_TPos_pc_ {new CloudTPos}; // I:Intensity, T:Time, V:Variance

		//Voxel
		CloudTPosPtr Voxel[voxel_num_x][voxel_num_y][voxel_num_z]; // storage pc_Ptr according to voxel address
		CloudTEvecPtr Evec_Voxel[voxel_num_x][voxel_num_y][voxel_num_z]; // storage eigen vectors
		
		//Eigen::Matrix3f eigenVectorsPCA;
};





int main(int argc, char** argv)
{
	ros::init(argc, argv, "pointcloud_lifespan_keeper");

	DynamicVoxelFilter dynamic_voxel_filter;
	dynamic_voxel_filter.execution();

	return 0;
}


DynamicVoxelFilter::DynamicVoxelFilter(void)
{
	ros::NodeHandle n;
	ros::NodeHandle nh("~");

	pc_subscriber = n.subscribe("/velodyne_points", 10, &DynamicVoxelFilter::pc_callback, this);
    odom_subscriber = n.subscribe("/odom", 10, &DynamicVoxelFilter::odom_callback, this);
	
	dynamic_pc_publisher = n.advertise<sensor_msgs::PointCloud2>("/dynamic_pc", 10);
	static_pc_publisher = n.advertise<sensor_msgs::PointCloud2>("/static_pc", 10);
}


void DynamicVoxelFilter::execution(void)
{
	ros::Rate r(Hz);
	while(ros::ok()){
		try{
       		listener.lookupTransform("/odom","/velodyne", ros::Time(0), transform);
			tf_listen_flag = true;
     	}   
     	catch (tf::TransformException ex){
       		ROS_ERROR("%s",ex.what());
       		ros::Duration(1.0).sleep();
    	}

		if(pc_callback_flag && odom_callback_flag && tf_listen_flag){
			pcl::toROSMsg(*input_TPos_pc_, tmp_pc);
			pcl_ros::transformPointCloud("/odom", tmp_pc, transformed_pc, listener);
			pcl::fromROSMsg(transformed_pc, *transformed_TPos_pc_);
			
			addressed_TPos_pc_ = pc_addressing(transformed_TPos_pc_);
			input_pt2Voxel(addressed_TPos_pc_);
			
			for(int xv=0; xv<voxel_num_x; xv++){
				for(int yv=0; yv<voxel_num_y; yv++){
					for(int zv=0; zv<voxel_num_z; zv++){
						if((Voxel[xv][yv][zv])->points[0].intensity > 0){
							
						}else{

						}
					}
				}
			}



			first_flag = true;
			pc_callback_flag = false;
			odom_callback_flag = false;
		}
		r.sleep();
		ros::spinOnce();
	}
}


void DynamicVoxelFilter::pc_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	pcl::fromROSMsg(*msg, *input_I_pc_);
	pcl::copyPointCloud(*input_I_pc_, *input_TPos_pc_);
	pc_callback_flag = true;
}


void DynamicVoxelFilter::odom_callback(const nav_msgs::OdometryConstPtr &msg)
{
	odom = *msg;
	odom_callback_flag = true;
}


Eigen::Matrix3f DynamicVoxelFilter::eigen_estimation(CloudTPosPtr pc_in_voxel_)
{
	/*
	Eigen::Vector4f pcaCentroid;
	pcl::compute3DCentroid(*pc_in_voxel_, pcaCentroid);
	Eigen::Matrix3f covariance;
	computeCovarianceMatrixNormalized(*pc_in_voxel_, pcaCentroid, covariance);
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
	Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
	eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));
	*/

	pcl::PCA<PointTPos> pca;
	pca.setInputCloud(pc_in_voxel_);
	Eigen::Matrix3f eigenVectorsPCA = pca.getEigenVectors();

	return eigenVectorsPCA;
}


CloudTPosPtr DynamicVoxelFilter::pc_addressing(CloudTPosPtr input_TPos_pc_)
{
	for(auto& pt : input_TPos_pc_->points){
		pt.normal_x = 0.5 * (float)voxel_num_x + pt.x / voxel_size_x;
		pt.normal_y = 0.5 * (float)voxel_num_y + pt.x / voxel_size_y;
		pt.normal_z = 0.5 * (float)voxel_num_z + pt.x / voxel_size_z;
	}
	
	return input_TPos_pc_;
}


void DynamicVoxelFilter::input_pt2Voxel(CloudTPosPtr pc_addressed_TPos_)
{
	for(auto& pt : pc_addressed_TPos_->points){
		bool xv_flag = false, yv_flag = false, zv_flag = false;
		
		for(int xv=0; xv<voxel_num_x; xv++){
			for(int yv=0; yv<voxel_num_y; yv++){
				for(int zv=0; zv<voxel_num_z; zv++){
					CloudTPosPtr tmp_pc_ {new CloudTPos};
					tmp_pc_->points.resize(1);
					if(xv==(int)pt.normal_x && yv==(int)pt.normal_y && zv==pt.normal_z){
						tmp_pc_->points[0].x = pt.x;
						tmp_pc_->points[0].y = pt.y;
						tmp_pc_->points[0].z = pt.z;
						tmp_pc_->points[0].intensity = pt.intensity;
						tmp_pc_->points[0].normal_x = pt.normal_x;
						tmp_pc_->points[0].normal_y = pt.normal_y;
						tmp_pc_->points[0].normal_z = pt.normal_z;
						if(first_flag){
							*Voxel[xv][yv][zv] += *tmp_pc_;
						}else{
							*Voxel[xv][yv][zv] = *tmp_pc_;
						}
						xv_flag = true;
						yv_flag = true;
						zv_flag = true;
					}else{
						tmp_pc_->points[0].intensity = -1.0;
						*Voxel[xv][yv][zv] = *tmp_pc_;
					}
					if(zv_flag) break;
				}
				if(yv_flag) break;
			}
			if(zv_flag) break;
		}

	}
}


void DynamicVoxelFilter::voxel_shooting(void)
{
	
}


