#include "dynamic_voxel_filter/dynamic_voxel_filter.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "dynamic_voxel_filter");

	DynamicVoxelFilter dynamic_voxel_filter;
	dynamic_voxel_filter.execution();

	return 0;
}


DynamicVoxelFilter::DynamicVoxelFilter(void)
: nh("~")
{
    nh.param("Hz", Hz, 100.0);
    nh.param("MAX_LENGTH", MAX_LENGTH, 50.0); // ->X
    nh.param("MAX_WIDTH", MAX_WIDTH, 50.0); // ->Y
    nh.param("MAX_HEIGHT", MAX_HEIGHT, 2.0); // ->Z
    nh.param("VOXEL_NUM_X", VOXEL_NUM_X, 500.0);
    nh.param("VOXEL_NUM_Y", VOXEL_NUM_Y, 500.0);
    nh.param("VOXEL_NUM_Z", VOXEL_NUM_X, 20.0);
    // nh.param("", , );

    voxel_size_x = (float)(MAX_LENGTH / VOXEL_NUM_X);
    voxel_size_y = (float)(MAX_WIDTH / VOXEL_NUM_Y);
    voxel_size_z = (float)(MAX_HEIGHT / VOXEL_NUM_Z);

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
            // sensor_msgs::PointCloud2 odom_transformed_pc;
            sensor_msgs::PointCloud2 dynamic_pc;
            CloudINormalPtr pcl_odom_transformed_pc {new CloudINormal};
            CloudINormalPtr pcl_odom_voxel_transformed_pc {new CloudINormal};
            CloudINormalPtr pcl_dynamic_odom_pc {new CloudINormal};
            CloudINormalPtr pcl_dynamic_sensor_transformed_pc {new CloudINormal};

			pcl_ros::transformPointCloud("/odom", *pcl_input_pc, *pcl_odom_transformed_pc, listener);
			pcl_odom_voxel_transformed_pc = to_voxel_tf(pcl_odom_transformed_pc);

			pc_addressing(pcl_odom_voxel_transformed_pc);
			third_main_component_estimation();	
			chronological_variance_calculation();

			pcl_ros::transformPointCloud("/velodyne", *pcl_dynamic_odom_pc, *pcl_dynamic_sensor_transformed_pc, listener);
			pcl::toROSMsg(*pcl_dynamic_sensor_transformed_pc, dynamic_pc);
		}
		r.sleep();
		ros::spinOnce();
	}
}


void DynamicVoxelFilter::pc_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	input_pc = *msg;
	pcl::fromROSMsg(input_pc, *pcl_input_pc);
    pc_callback_flag = true;
}


void DynamicVoxelFilter::formatting(void)
{
    std::vector<Status> grid_1d;
    std::vector<std::vector<Status> > grid_2d;

    Status initial_status;
    initial_status.pcl_pc->points.resize(0);
    initial_status.step = 0;
    initial_status.occupation = Unknown;
    initial_status.chronological_variance = 0.0;
    initial_status.dynamic_probability = 0.0;
    initial_status.pre_3rd_main_component = zero_vector;
    initial_status.new_3rd_main_component = zero_vector;
    initial_status.pre_3mc_mean = zero_vector;
    initial_status.pre_3mc_var = zero_vector;
    initial_status.pre_3mc_cov = zero_vector;

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
    for(int ix = 0; ix < VOXEL_NUM_X; ix++){
        for(int iy = 0; iy < VOXEL_NUM_Y; iy++){
            for(int iz = 0; iz < VOXEL_NUM_Z; iz++){
                voxel_grid[ix][iy][iz].pcl_pc->points.clear();
            }
        }
    }
    voxel_id_list.clear();
}


CloudINormalPtr DynamicVoxelFilter::to_voxel_tf(CloudINormalPtr pcl_odom_pc)
{
    for(auto& pt : pcl_odom_pc->points){
        pt.x += 0.5 * MAX_LENGTH;
        pt.y += 0.5 * MAX_WIDTH;
        // pt.z is needless to transform
    }

    return pcl_odom_pc;
}


CloudINormalPtr DynamicVoxelFilter::from_voxel_tf(CloudINormalPtr pcl_odom_voxel_pc)
{
    for(auto& pt : pcl_odom_voxel_pc->points){
        pt.x -= 0.5 * MAX_LENGTH;
        pt.y -= 0.5 * MAX_WIDTH;
        // z is needless to transform
    }

    return pcl_odom_voxel_pc;
}


void DynamicVoxelFilter::pc_addressing(CloudINormalPtr pcl_voxel_pc)
{
    Eigen::Vector3d voxel_id;

    CloudINormalPtr pcl_tmp_pt {new CloudINormal};
    pcl_tmp_pt->points.resize(1);
    
    for(auto& pt : pcl_voxel_pc->points){
        voxel_id.x() = (int)(pt.x / voxel_size_x);
        voxel_id.y() = (int)(pt.y / voxel_size_y);
        voxel_id.z() = (int)(pt.z / voxel_size_z);
        voxel_id_list.push_back(voxel_id);

        pcl_tmp_pt->points[0].x = pt.x;
        pcl_tmp_pt->points[0].y = pt.y;
        pcl_tmp_pt->points[0].z = pt.z;
        pcl_tmp_pt->points[0].intensity = pt.intensity;
        pcl_tmp_pt->points[0].normal_x = pt.normal_x;
        pcl_tmp_pt->points[0].normal_y = pt.normal_y;
        pcl_tmp_pt->points[0].normal_z = pt.normal_z;

        if(voxel_id.x() < VOXEL_NUM_X && voxel_id.y() < VOXEL_NUM_Y && voxel_id.z() < VOXEL_NUM_Z){
            *voxel_grid[voxel_id.x()][voxel_id.y()][voxel_id.z()].pcl_pc += *pcl_tmp_pt;
            voxel_grid[voxel_id.x()][voxel_id.y()][voxel_id.z()].occupation = Occupied;
        }
    }
}


void DynamicVoxelFilter::third_main_component_estimation(void)
{
	for(int ix = 0; ix < VOXEL_NUM_X; ix++){
		for(int iy = 0; iy < VOXEL_NUM_Y; iy++){
			for(int iz = 0; iz < VOXEL_NUM_Z; iz++){
				if(voxel_grid[ix][iy][iz].pcl_pc->points.size() >= 3){
					Eigen::Matrix3f pca_vectors = eigen_estimation(voxel_grid[ix][iy][iz].pcl_pc);
					voxel_grid[ix][iy][iz].new_3rd_main_component = pca_vectors.block(0, 2, 3, 1);
				    voxel_grid[ix][iy][iz].step += 1;
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
    for(auto& id : voxel_id_list){
        if(voxel_grid[id.x()][id.y()][id.z()].step == 1){
            voxel_grid[id.x()][id.y()][id.z()].pre_3mc_mean = voxel_grid[id.x()][id.y()][id.z()].new_3rd_main_component;
            voxel_grid[id.x()][id.y()][id.z()].pre_3rd_main_component = voxel_grid[id.x()][id.y()][id.z()].new_3rd_main_component;
        }
        else if(voxel_grid[id.x()][id.y()][id.z()].step > 1){
            Eigen::Vector3f new_3mc_ = voxel_grid[id.x()][id.y()][id.z()].new_3rd_main_component;
            
            int n = voxel_grid[id.x()][id.y()][id.z()].step;
            Eigen::Vector3f pre_3mc_mean_ = voxel_grid[id.x()][id.y()][id.z()].pre_3mc_mean;
            Eigen::Vector3f new_3mc_mean_ = ((n-1)*pre_3mc_mean_ + new_3mc_) / n;
            
            Eigen::Vector3f new_3mc_sqr = hadamard_product(new_3mc_);
            Eigen::Vector3f pre_3mc_mean_sqr = hadamard_product(pre_3mc_mean_);
            Eigen::Vector3f new_3mc_mean_sqr = hadamard_product(new_3mc_mean_);
            Eigen::Vector3f pre_3mc_var_ = voxel_grid[id.x()][id.y()][id.z()].pre_3mc_var;
            Eigen::Vector3f pre_3mc_var_sqr = hadamard_product(pre_3mc_var_);
            Eigen::Vector3f new_3mc_var_ = ((n-1)*(pre_3mc_var_sqr + pre_3mc_mean_sqr) + new_3mc_sqr)/n - new_3mc_mean_sqr;
            
            Eigen::Vector3f pre_3mc_cov_ = voxel_grid[id.x()][id.y()][id.z()].pre_3mc_cov;
            Eigen::Vector3f new_3mc_cov_;
            new_3mc_cov_(xy) = (new_3mc_.x() - pre_3mc_mean_.x())*(new_3mc_.y() - pre_3mc_mean_.y())*(n-1)/(n*n) + pre_3mc_cov_(xy)*(n-1)/n;
            new_3mc_cov_(yz) = (new_3mc_.y() - pre_3mc_mean_.y())*(new_3mc_.z() - pre_3mc_mean_.z())*(n-1)/(n*n) + pre_3mc_cov_(yz)*(n-1)/n;
            new_3mc_cov_(zx) = (new_3mc_.z() - pre_3mc_mean_.z())*(new_3mc_.x() - pre_3mc_mean_.x())*(n-1)/(n*n) + pre_3mc_cov_(zx)*(n-1)/n;

            voxel_grid[id.x()][id.y()][id.z()].chronological_variance = new_3mc_var_.x() + new_3mc_var_.y() + new_3mc_var_.z()
                                                                        + 2*(new_3mc_cov_(xy) + new_3mc_cov_(yz) + new_3mc_cov_(zx));

            voxel_grid[id.x()][id.y()][id.z()].pre_3rd_main_component = new_3mc_;
            voxel_grid[id.x()][id.y()][id.z()].pre_3mc_mean = new_3mc_mean_;
            voxel_grid[id.x()][id.y()][id.z()].pre_3mc_var = new_3mc_var_;
            voxel_grid[id.x()][id.y()][id.z()].pre_3mc_cov = new_3mc_cov_;
        }
    }
}


Eigen::Vector3f DynamicVoxelFilter::hadamard_product(Eigen::Vector3f in){
    Eigen::Vector3f output_vector;
    output_vector << in.x()*in.x(), in.y()*in.y(), in.z()*in.z();
    
    return output_vector;
}




