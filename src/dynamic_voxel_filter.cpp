#include "dynamic_voxel_filter/dynamic_voxel_filter.h"

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

		if(pc_callback_flag && tf_listen_flag){
            sensor_msgs::PointCloud2 odom_transformed_pc;
            CloudINormalPtr pcl_odom_transformed_pc {new CloudINormal};

			pcl_ros::transformPointCloud("/odom", imput_pc, odom_transformed_pc, listener);
			pcl::fromROSMsg(transformed_pc, *pcl_odom_transformed_pc);
			to_voxel_tf();

			pc_addressing(pcl_odom_voxel_transformed_pc);
            eigen_estimation();
			input_pca3rd2voxel();	
			chronological_pca3rd_variance_calculation();


			first_flag = true;
			pc_callback_flag = false;
		}
		r.sleep();
		ros::spinOnce();
	}
}


void DynamicVoxelFilter::pc_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	imput_pc = *msg;
    pc_callback_flag = true;
}


CloudINormalPtr DynamicVoxelFilter::to_voxel_tf(CloudINormalPtr pcl_odom_pc)
{
    for(auto& pt : pcl_odom_pc->points){
        pc.x += 0.5 * MAX_RANGE_X;
        pc.y += 0.5 * MAX_RANGE_Y;
        // z is needless to transform
    }
}


CloudINormalPtr DynamicVoxelFilter::from_voxel_tf(CloudINormalPtr pcl_odom_voxel_pc)
{
    for(auto& pt : pcl_odom_voxel_pc->points){
        pc.x -= 0.5 * MAX_RANGE_X;
        pc.y -= 0.5 * MAX_RANGE_Y;
        // z is needless to transform
    }
}


void DynamicVoxelFilter::pc_addressing(CloudINormalPtr pcl_voxel_pc)
{
	for(auto& pt : pcl_voxel_pc->points){
		bool address_flag = false;
		
        for(int xv = 0; xv < VOXEL_NUM_X; xv++){
            if(pt.x > MAX_RANGE_X) break;
			for(int yv = 0; yv < VOXEL_NUM_Y; yv++){
                if(pt.y > MAX_RANGE_Y) break;
				for(int zv = 0; zv < VOXEL_NUM_Z; zv++){
                    if(pt.z > MAX_RANGE_Z) break;
					
                    CloudINormalPtr pcl_tmp_pt {new CloudINormal};
					pcl_tmp_pt->points.resize(1);
					if(xv == (int)pt.x && yv == (int)pt.y && zv == pt.z){
						pcl_tmp_pt->points[0].x = pt.x;
						pcl_tmp_pt->points[0].y = pt.y;
						pcl_tmp_pt->points[0].z = pt.z;
						pcl_tmp_pt->points[0].intensity = pt.intensity;
						pcl_tmp_pt->points[0].normal_x = pt.normal_x;
						pcl_tmp_pt->points[0].normal_y = pt.normal_y;
						pcl_tmp_pt->points[0].normal_z = pt.normal_z;
                        *voxel_grid[xv][yv][zv].pcl_pc += *pcl_tmp_pt;
					    voxel_grid[xv][yv][zv].occupatoin = Occupied;
						address_flag = true;
					}
					
                    if(address_flag) break;
				}
				if(address_flag) break;
			}
			if(address_flag) break;
		}

	}
}


void DynamicVoxelFilter::3rd_main_component_estimation(void)
{
	for(int xv = 0; xv < VOXEL_NUM_X; xv++){
		for(int yv = 0; yv < VOXEL_NUM_Y; yv++){
			for(int zv = 0; zv < VOXEL_NUM_Z; zv++){
				if((voxel_grid[xv][yv][zv])->points.size() >= 3){
					Eigen::Matrix3f pca_vectors = eigen_estimation(voxel_grid[xv][yv][zv].pcl_pc);
					voxel_grid[xv][yv][zv].3rd_main_component = pca_vectors.block(0, 2, 3, 1);
				}else{
					Eigen::Vector3f none_pca_vec = Eigen::Vector3f::Zero();
					voxel_grid[xv][yv][zv].3rd_main_component = none_pca_vec;
				}
			}
		}
	}
}


Eigen::Matrix3f DynamicVoxelFilter::eigen_estimation(CloudINormalPtr pcl_voxel_pc)
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

	pcl::PCA<PointINormal> pca;
	pca.setInputCloud(pcl_voxel_pc);
	Eigen::Matrix3f eigen_vectors = pca.getEigenVectors();

	return eigen_vectors;
}


void DynamicVoxelFilter::chronological_pca3rd_variance_calculation(void)
{
	pca3rd_chronological_memories.push_back(pca3rd_voxel);
	if(pca3rd_chronological_memories.size() > MEMORY_SIZE){
		pca3rd_chronological_memories.erase(pca3rd_chronological_memories.begin());
	}
	
}







