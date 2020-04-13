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

    nh.param("Hz", Hz, 100);
    nh.param("MAX_LENGTH", MAX_LENGTH, 50); // ->X
    nh.param("MAX_WIDTH", MAX_WIDTH, 50); // ->Y
    nh.param("MAX_HEIGHT", MAX_HEIGHT, 2); // ->Z
    nh.param("VOXEL_NUM_X", VOXEL_NUM_X, 500);
    nh.param("VOXEL_NUM_Y", VOXEL_NUM_Y, 500);
    nh.param("VOXEL_NUM_Z", VOXEL_NUM_X, 20);
    // nh.param("", , );

    voxel_size_x = MAX_LENGTH / VOXEL_NUM_X;
    voxel_size_y = MAX_WIDTH / VOXEL_NUM_Y;
    voxel_size_z = MAX_HEIGHT / VOXEL_NUM_Z;

    pc_subscriber = n.subscribe("/velodyne_points", 10, &DynamicVoxelFilter::pc_callback, this);
    // odom_subscriber = n.subscribe("/odom", 10, &DynamicVoxelFilter::odom_callback, this);
	
	dynamic_pc_publisher = n.advertise<sensor_msgs::PointCloud2>("/dynamic_pc", 10);
	static_pc_publisher = n.advertise<sensor_msgs::PointCloud2>("/static_pc", 10);
}


void DynamicVoxelFilter::execution(void)
{
    formatting();

	ros::Rate r(Hz);
	while(ros::ok()){
        initialization();

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
			pcl::fromROSMsg(odom_transformed_pc, *pcl_odom_transformed_pc);
			to_voxel_tf();

			pc_addressing(pcl_odom_voxel_transformed_pc);
            eigen_estimation();
			3rd_main_component_estimation();	
			chronological_variance_calculation();

			pcl_ros::transformPointCloud("/velodyne", pcl_dynamic_odom_pc, pcl_dynamic_sensor_transformed_pc, listener);
			pcl::toROSMsg(*pcl_dynamic_sensor_transformed_pc, dynamic_pc);

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


void DynamicVoxelFilter::formatting(void)
{
    std::vector<Status> grid_1d;
    std::vector<std::vector<Status> > grid_2d;

    Status initial_status;
    initial_status.occupation = Unknown;
    initial_status.dynamic_probability = 0.0;
    initial_status.pcl_pc->points.resize(0);
    initia_status.3rd_main_component = zero_vector;

    for(int ix = 0; ix < VOXEL_NUM_X; ix++){
        grid_1d.push_back(initial_status);
    }
    for(int iy = 0; iy < VOXEL_NUM_Y; iy++){
        grid_2d.push_back(grid_1d);
    }
    for(int iz = 0; iz < VOXEL_NUM_Z; iz++){
        voxel_grid.push_back(grid_2d);
    }   
}


void DynamicVoxelFilter::initialization(void)
{
    tf_listen_flag = false;
    pc_callback_flag = false;
    voxel_grid.clear();
    // voxel_id_list.clear();
}


CloudINormalPtr DynamicVoxelFilter::to_voxel_tf(CloudINormalPtr pcl_odom_pc)
{
    for(auto& pt : pcl_odom_pc->points){
        pt.x += 0.5 * MAX_RANGE_X;
        pt.y += 0.5 * MAX_RANGE_Y;
        // pt.z is needless to transform
    }
}


CloudINormalPtr DynamicVoxelFilter::from_voxel_tf(CloudINormalPtr pcl_odom_voxel_pc)
{
    for(auto& pt : pcl_odom_voxel_pc->points){
        pt.x -= 0.5 * MAX_RANGE_X;
        pt.y -= 0.5 * MAX_RANGE_Y;
        // z is needless to transform
    }
}


void DynamicVoxelFilter::pc_addressing(CloudINormalPtr pcl_voxel_pc)
{
    Eigen::Vector3d voxel_id;

    CloudINormalPtr pcl_tmp_pt {new CloudINormal};
    pcl_tmp_pt->points.resize(1);
    
    for(auto& pt : pcl_voxel_pc->points){
        // voxel_id.x() = (int)(pt.x/voxel_size_x);
        // voxel_id.y() = (int)(pt.y/voxel_size_y);
        // voxel_id.z() = (int)(pt.z/voxel_size_z);
        // voxel_id_list.push_back(voxel_id);
        ix = (int)(pt.x/voxel_size_x);
        iy = (int)(pt.y/voxel_size_y);
        iz = (int)(pt.z/voxel_size_z);

        pcl_tmp_pt->points[0].x = pt.x;
        pcl_tmp_pt->points[0].y = pt.y;
        pcl_tmp_pt->points[0].z = pt.z;
        pcl_tmp_pt->points[0].intensity = pt.intensity;
        pcl_tmp_pt->points[0].normal_x = pt.normal_x;
        pcl_tmp_pt->points[0].normal_y = pt.normal_y;
        pcl_tmp_pt->points[0].normal_z = pt.normal_z;

        // if(voxel_id.x() < VOXEL_NUM_X && voxel_id.y() < VOXEL_NUM_Y && voxel_id.z() VOXEL_NUM_Z){
        if(ix < VOXEL_NUM_X && iy < VOXEL_NUM_Y && iz < VOXEL_NUM_Z){
            //*voxel_grid[voxel_id.x()][voxel_id.y()][voxel_id.z()].pcl_pc += *pcl_tmp_pt;
            *voxel_grid[ix][iy][iz].pcl_pc += *pcl_tmp_pt;

            //voxel_grid[voxel_id.x()][voxel_id.y()][voxel_id.z()].occupation = Occupied;
            voxel_grid[ix][iy][iz].occupation = Occupied;
        }
    }
}


void DynamicVoxelFilter::3rd_main_component_estimation(void)
{
	for(int ix = 0; ix < VOXEL_NUM_X; ix++){
		for(int iy = 0; yv < VOXEL_NUM_Y; iy++){
			for(int iz = 0; zv < VOXEL_NUM_Z; iz++){
				if((voxel_grid[ix][iy][iz])->points.size() >= 3){
					Eigen::Matrix3f pca_vectors = eigen_estimation(voxel_grid[ix][iy][iz].pcl_pc);
					voxel_grid[ix][iy][iz].3rd_main_component = pca_vectors.block(0, 2, 3, 1);
				}else{
					voxel_grid[ix][iy][iz].3rd_main_component = zero_vector;
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


void DynamicVoxelFilter::chronological_variance_calculation(void)
{
	pca3rd_chronological_memories.push_back(pca3rd_voxel);
	if(pca3rd_chronological_memories.size() > MEMORY_SIZE){
		pca3rd_chronological_memories.erase(pca3rd_chronological_memories.begin());
	}
	
}







