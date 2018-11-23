/*

remove clustered pointcloud

author:
    Yudai Sadakuni

*/

#include <iostream>

#include <ros/ros.h>
#include <ros/package.h> 

#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/io/pcd_io.h>

#include <sys/stat.h>
#include <sys/types.h>

#define MIN(x, y) ((x) < (y) ? (x) : (y))
#define MAX(x, y) ((x) > (y) ? (x) : (y))

struct Cluster{
    float x; 
    float y; 
    float z;
    float width;
    float height;
    float depth;
    float curvature;
    Eigen::Vector3f min_p;
    Eigen::Vector3f max_p;
};

template<typename T_p, typename T_c, typename T_ptr>
class RmCluster{
    private:
        ros::NodeHandle nh;
        std::string package_path;
        // min max algorithm parameter
        double cell_size;
        int grid_dimentions;
        double threshold;
        // clustering parameter
        double leaf_size;
        double tolerance;
        int min_size, max_size;
        // normal estimation parameter
        double search_radius;

    public:
        RmCluster();
        int file_count_boost(const boost::filesystem::path& root);
        bool loadCloud(T_ptr& cloud, std::string file_name);                     
        void min_max(T_ptr& cloud, T_ptr& ground_cloud, T_ptr& obstacle_cloud);
        void getClusterInfo(T_ptr& pt, Cluster& cluster);
        void normal_estimation(T_ptr& input_cloud, T_ptr& normal_cloud);        
        void remove_cluster(T_ptr& input_cloud, T_ptr& remove_cloud);           
        void saveCloud(T_ptr& cloud, int file_num);                              
        void main();
};

template<typename T_p, typename T_c, typename T_ptr>
RmCluster<T_p, T_c, T_ptr>::RmCluster()
    : nh("~")
{
    package_path = ros::package::getPath("mapping");
    // min max algorithm parameter
    nh.param<double>("cell_size", cell_size, 0.25);
    nh.param<int>("grid_dimentions", grid_dimentions, 500);
    nh.param<double>("threshold", threshold, 0.1);
    // clustering parameter
    nh.param<double>("leaf_size", leaf_size, 0.07);
    nh.param<double>("torelance", tolerance, 0.15);
    nh.param<int>("min_size", min_size, 100);
    nh.param<int>("max_size", max_size, 1500);
    // normal estimation parameter
    nh.param<double>("search_radius", search_radius, 0.30);

}

// path内にデータがいくつがあるか確認
template<typename T_p, typename T_c, typename T_ptr>
int RmCluster<T_p, T_c, T_ptr>::file_count_boost(const boost::filesystem::path& root) {
    namespace fs = boost::filesystem;
    if (!fs::exists(root) || !fs::is_directory(root)) return 0;
    int result = 0;
    fs::directory_iterator last;
    for (fs::directory_iterator pos(root); pos != last; ++pos) {
        ++result;
        if (fs::is_directory(*pos)) result += file_count_boost(pos->path());
    }
    return result;
}


// Cloudデータを読み込み
template<typename T_p, typename T_c, typename T_ptr>
bool RmCluster<T_p, T_c, T_ptr>::loadCloud(T_ptr& cloud, std::string file_name)
{
    if(pcl::io::loadPCDFile<T_p> (file_name, *cloud) == -1){
        std::cout<<file_name<<" is none"<<std::endl;
        return false;
    }else{
        std::cout<<"Load "<<file_name<<" is success"<<std::endl;
        return true;
    }
}

// min max algorithm
template<typename T_p, typename T_c, typename T_ptr>
void RmCluster<T_p, T_c, T_ptr>::min_max(T_ptr& cloud, T_ptr& ground_cloud, T_ptr& obstacle_cloud)
{
    float min[grid_dimentions][grid_dimentions];
    float max[grid_dimentions][grid_dimentions];
    bool init[grid_dimentions][grid_dimentions];

    memset(&min,  0, grid_dimentions*grid_dimentions);
    memset(&max,  0, grid_dimentions*grid_dimentions); 
    memset(&init, 0, grid_dimentions*grid_dimentions);

// #pragma omp parallel for
    for(size_t i=0; i<cloud->points.size(); i++)
    {
        int x = (grid_dimentions/2) + cloud->points[i].x/cell_size;
        int y = (grid_dimentions/2) + cloud->points[i].y/cell_size;

        if(0<=x && x<grid_dimentions && 0<=y && y<grid_dimentions)
        {
            if(!init[x][y]){
                min[x][y] = cloud->points[i].z;
                max[x][y] = cloud->points[i].z;
                init[x][y] = true;
            }
            else{
                min[x][y] = MIN(min[x][y], cloud->points[i].z);
                max[x][y] = MAX(max[x][y], cloud->points[i].z);
            }
        }
    }

    for(size_t i=0;i<cloud->points.size();i++)
    {
        int x = (grid_dimentions/2) + cloud->points[i].x/cell_size;
        int y = (grid_dimentions/2) + cloud->points[i].y/cell_size;

        if(0<=x && x<grid_dimentions && 0<=y && y<grid_dimentions){
            if(init[x][y] && max[x][y]-min[x][y]<threshold)
                ground_cloud->points.push_back(cloud->points[i]);
            else
                obstacle_cloud->points.push_back(cloud->points[i]);
        }else
            obstacle_cloud->points.push_back(cloud->points[i]);
    }
}

// Normal Estimation
template<typename T_p, typename T_c, typename T_ptr>
void RmCluster<T_p, T_c, T_ptr>::normal_estimation(T_ptr& cloud, T_ptr& normal_cloud)
{
    pcl::NormalEstimationOMP<T_p, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    typename pcl::search::KdTree<T_p>::Ptr tree (new pcl::search::KdTree<T_p> ());
    ne.setSearchMethod (tree);
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch (search_radius);
    ne.compute(*normals);

    T_p tmp;
    for(size_t i=0;i<cloud->points.size();i++)
    {
		tmp.x = cloud->points[i].x;
		tmp.y = cloud->points[i].y;
		tmp.z = cloud->points[i].z;
		if(!std::isnan(normals->points[i].normal_x)){
			tmp.normal_x = normals->points[i].normal_x;
		}
		else{
			tmp.normal_x = 0.0;
		}
		if(!std::isnan(normals->points[i].normal_y)){
			tmp.normal_y = normals->points[i].normal_y;
		}
		else{
			tmp.normal_y = 0.0;
		}
		if(!std::isnan(normals->points[i].normal_z)){
			tmp.normal_z = normals->points[i].normal_z;
		}
		else{
			tmp.normal_z = 0.0;
		}
		if(!std::isnan(normals->points[i].curvature)){
			tmp.curvature = normals->points[i].curvature;
		}
		else{
			tmp.curvature = 0.0;
		}
		normal_cloud->points.push_back(tmp);
	}
}

// Remove Cluster
template<typename T_p, typename T_c, typename T_ptr>
void RmCluster<T_p, T_c, T_ptr>::remove_cluster(T_ptr& cloud, T_ptr& cloud_removed)
{
    //Downsample//
    pcl::VoxelGrid<T_p> vg;
    T_ptr ds_cloud (new T_c);
    vg.setInputCloud (cloud);  
    vg.setLeafSize (leaf_size, leaf_size, leaf_size);
    vg.filter (*ds_cloud);

    //downsampled point's z =>0
    std::vector<float> tmp_z;
    tmp_z.resize(ds_cloud->points.size());
	for(int i=0;i<(int)ds_cloud->points.size();i++){
        tmp_z[i]=ds_cloud->points[i].z;
		ds_cloud->points[i].z  = 0.0;
    }

    //Clustering
    typename pcl::search::KdTree<T_p>::Ptr tree (new pcl::search::KdTree<T_p>);
    tree->setInputCloud (ds_cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<T_p> ec;
    ec.setClusterTolerance (tolerance);
    ec.setMinClusterSize (min_size);
    ec.setMaxClusterSize (max_size);
    ec.setSearchMethod (tree);
    ec.setInputCloud(ds_cloud);
    ec.extract (cluster_indices);
    
    //reset z value
	for(int i=0;i<(int)ds_cloud->points.size();i++)
        ds_cloud->points[i].z=tmp_z[i];

    // remove cluster
    std::vector<int> id_rm;
    std::vector<int> id_cluster;

    // check cluster size
    for(const auto& it : cluster_indices){
        T_ptr cluster_cloud(new T_c);
        for(const auto& pit : it.indices){
            cluster_cloud->points.push_back(ds_cloud->points[pit]);
        }
        Cluster cluster;
        getClusterInfo(cluster_cloud, cluster);
        
        // pick up human size cluster
        if(0.3<cluster.width && cluster.width<1.0 && 
           0.3<cluster.depth && cluster.depth<1.0 && 
           1.0<cluster.height && cluster.height<2.0)
        {
            for(const auto& pit : it.indices){
                id_cluster.push_back(pit);
            }
        }
    }

    // for(const auto& it : cluster_indices){
    //     for(const auto& pit : it.indices){
    //         id_cluster.push_back(pit);
    //     }
    // }

    std::sort(id_cluster.begin(), id_cluster.end());

    int npoints = ds_cloud->points.size();
    
    std::vector<int>::iterator it = id_cluster.begin();

    for(int i=0;i<npoints; ++i){
        if(it == id_cluster.end() || i != *it){
            cloud_removed->points.push_back(ds_cloud->points[i]);
        }
        else{
            ++it;
        }
    }
}

// getClusterInfo
template<typename T_p, typename T_c, typename T_ptr>
void RmCluster<T_p, T_c, T_ptr>::getClusterInfo(T_ptr& pt, Cluster& cluster)
{
    Eigen::Vector3f centroid;
    centroid[0]=pt->points[0].x;
    centroid[1]=pt->points[0].y;
    centroid[2]=pt->points[0].z;
    
    Eigen::Vector3f min_p;
    min_p[0]=pt->points[0].x;
    min_p[1]=pt->points[0].y;
    min_p[2]=pt->points[0].z;

    Eigen::Vector3f max_p;
    max_p[0]=pt->points[0].x;
    max_p[1]=pt->points[0].y;
    max_p[2]=pt->points[0].z;

    for(size_t i=1;i<pt->points.size();i++){
        centroid[0]+=pt->points[i].x;
        centroid[1]+=pt->points[i].y;
        centroid[2]+=pt->points[i].z;
        if (pt->points[i].x<min_p[0]) min_p[0]=pt->points[i].x;
        if (pt->points[i].y<min_p[1]) min_p[1]=pt->points[i].y;
        if (pt->points[i].z<min_p[2]) min_p[2]=pt->points[i].z;

        if (pt->points[i].x>max_p[0]) max_p[0]=pt->points[i].x;
        if (pt->points[i].y>max_p[1]) max_p[1]=pt->points[i].y;
        if (pt->points[i].z>max_p[2]) max_p[2]=pt->points[i].z;
    }

    cluster.x=centroid[0]/(float)pt->points.size();
    cluster.y=centroid[1]/(float)pt->points.size();
    cluster.z=centroid[2]/(float)pt->points.size();
    cluster.depth  = max_p[0]-min_p[0];
    cluster.width  = max_p[1]-min_p[1];
    cluster.height = max_p[2]-min_p[2]; 
    cluster.min_p = min_p;
    cluster.max_p = max_p;
}


// save PointCloud
template<typename T_p, typename T_c, typename T_ptr>
void RmCluster<T_p, T_c, T_ptr>::saveCloud(T_ptr& cloud, int file_num)
{
    std::string file_name = std::to_string(file_num);
    std::string save_path = package_path+"/data/remove/"+file_name+".pcd";
    
    cloud->width = 1;
    cloud->height = cloud->points.size();

    pcl::io::savePCDFile(save_path, *cloud);
}

// main Function
template<typename T_p, typename T_c, typename T_ptr>
void RmCluster<T_p, T_c, T_ptr>::main()
{
    std::cout<<"file path:"<<package_path<<std::endl;

    std::string cloud_path = package_path + "/data/cloud";

    int file_size = file_count_boost(cloud_path.c_str());
    std::cout<<"file size:"<<file_size<<std::endl;

    for(int i=0;i<file_size;i++){
        T_ptr cloud(new T_c);
        std::string cloud_name = cloud_path+"/"+std::to_string(i)+".pcd";
        
        // loadPointCloud
        bool cloud_flag = loadCloud(cloud, cloud_name);
        if(!cloud_flag) break;

        // Min-Max Algorithm
        T_ptr ground_cloud(new T_c);
        T_ptr obstacle_cloud(new T_c);
        min_max(cloud, ground_cloud, obstacle_cloud);

        // Remove Cluster
        T_ptr remove_cloud(new T_c);
        remove_cluster(obstacle_cloud, remove_cloud);

        // mergePointCloud
        T_ptr merge_cloud(new T_c);
        *merge_cloud += *ground_cloud;
        *merge_cloud += *remove_cloud;

        // Normal Estimation
        T_ptr normal_cloud(new T_c);
        normal_estimation(merge_cloud, normal_cloud);

        // savePointCloud
        saveCloud(merge_cloud, i);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "remove_cluster");

    RmCluster <pcl::PointXYZINormal, 
               pcl::PointCloud<pcl::PointXYZINormal>, 
               pcl::PointCloud<pcl::PointXYZINormal>::Ptr> rc;

    rc.main();

    return 0;
}
