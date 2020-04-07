#include "dynamic_voxel_filter/dynamic_voxel_filter"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "dynamic_voxel_filter");

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
			pcl::toROSMsg(*intensity_normal_pc_, tmp_pc);
			pcl_ros::transformPointCloud("/odom", tmp_pc, transformed_pc, listener);
			pcl::fromROSMsg(transformed_pc, *transformed_intensity_normal_pc_);
			
			addressed_pc_ = pc_addressing(transformed_intensity_normal_pc_);
			input_pt2voxel(addressed_pc_);
			input_pca3rd2voxel();	
			chronological_pca3rd_variance_calculation();


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
	pcl::fromROSMsg(*msg, *input_intensity_pc_);
	pcl::copyPointCloud(*input_intensity_pc_, *intensity_normal_pc_);
	pc_callback_flag = true;
}


void DynamicVoxelFilter::odom_callback(const nav_msgs::OdometryConstPtr &msg)
{
	odom = *msg;
	odom_callback_flag = true;
}


Eigen::Matrix3f DynamicVoxelFilter::eigen_estimation(CloudIVoxelPtr pc_in_voxel_)
{
	/*
	Eigen::Vector4f pcaCentroid;
	pcl::compute3DCentroid(*pc_in_voxel_, pcaCentroid);
	Eigen::Matrix3f covariance;
	computeCovarianceMatrixNormalized(*pc_in_voxel_, pcaCentroid, covariance);
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
	Eigen::Matrix3f eigen_vectors = eigen_solver.eigenvectors();
	eigen_vectors.col(2) = eigen_vectors.col(0).cross(eigen_vectors.col(1));
	*/

	pcl::PCA<PointIVoxel> pca;
	pca.setInputCloud(pc_in_voxel_);
	Eigen::Matrix3f eigen_vectors = pca.getEigenVectors();

	return eigen_vectors;
}


CloudIVoxelPtr DynamicVoxelFilter::pc_addressing(CloudIVoxelPtr voxel_pc_)
{
	for(auto& pt : voxel_pc_->points){
		pt.normal_x = 0.5 * (float)VOXEL_NUM_X + pt.x / voxel_size_x;
		pt.normal_y = 0.5 * (float)VOXEL_NUM_Y + pt.x / voxel_size_y;
		pt.normal_z = 0.5 * (float)VOXEL_NUM_Z + pt.x / voxel_size_z;
	}
	
	return voxel_pc_;
}


void DynamicVoxelFilter::input_pt2voxel(CloudIVoxelPtr pc_addressed_TPos_)
{
	for(auto& pt : pc_addressed_TPos_->points){
		bool xv_flag = false, yv_flag = false, zv_flag = false;
		for(int xv = 0; xv < VOXEL_NUM_X; xv++){
			for(int yv = 0; yv < VOXEL_NUM_Y; yv++){
				for(int zv = 0; zv < VOXEL_NUM_Z; zv++){
					CloudIVoxelPtr tmp_pc_ {new CloudIVoxel};
					tmp_pc_->points.resize(1);
					if(xv == (int)pt.normal_x && yv == (int)pt.normal_y && zv == pt.normal_z){
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
					}
					if(zv_flag) break;
				}
				if(yv_flag) break;
			}
			if(zv_flag) break;
		}

	}
}


void DynamicVoxelFilter::input_pca3rd2voxel(void)
{
	for(int xv = 0; xv < VOXEL_NUM_X; xv++){
		for(int yv = 0; yv < VOXEL_NUM_Y; yv++){
			for(int zv = 0; zv < VOXEL_NUM_Z; zv++){
				if((Voxel[xv][yv][zv])->points.size() >= 2){
					Eigen::Matrix3f pca_vectors = eigen_estimation(Voxel[xv][yv][zv]);
					pca3rd_voxel[xv][yv][zv].vector = pca_vectors.block(0, 2, 3, 1);
					pca3rd_voxel[xv][yv][zv].occupatoin = Occupied;
				}else{
					Eigen::Vector3f none_pca_vec = Eigen::Vector3f::Zero();
					pca3rd_voxel[xv][yv][zv].vector = none_pca_vec;
				}
			}
		}
	}
}


void DynamicVoxelFilter::chronological_pca3rd_variance_calculation(void)
{
	pca3rd_chronological_memories.push_back(pca3rd_voxel);
	if(pca3rd_chronological_memories.size() > MEMORY_SIZE){
		pca3rd_chronological_memories.erase(pca3rd_chronological_memories.begin());
	}
	
}







