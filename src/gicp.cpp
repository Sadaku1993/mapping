#include <ros/ros.h>
#include <ros/package.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/tf.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>

#include <mapping/function.h>
#include <mapping/normal_estimation.h>

void print4x4Matrix (const Eigen::Matrix4f & matrix)
{
  printf ("Rotation matrix :\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
  printf ("Translation vector :\n");
  printf ("t = < %6.3f, %6.3f, %6.3f >\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}

struct Transform{
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
};

template<typename T_p>
class Gicp{
    private:
        ros::NodeHandle nh;
        std::string package_path;
        std::string cloud_path;
        std::string tf_path;
        std::string save_path;
    public:
        Gicp();

        Function<T_p> Fc;

        void gicp(typename pcl::PointCloud<T_p>::Ptr& source_cloud, 
                  typename pcl::PointCloud<T_p>::Ptr& target_cloud, 
                  Eigen::Matrix4f& transformation_matrix);
        void accuracy(tf::Transform source_frame,
                      tf::Transform target_frame,
                      Eigen::Matrix4f gicp_matrix,
                      Eigen::Matrix4f& matrix);
        void main();
};


// template<> 
// void Gicp<pcl::PointXYZINormal>::icp(pcl::PointCloud<pcl::PointXYZINormal>::Ptr& source_cloud,
//                                      pcl::PointCloud<pcl::PointXYZINormal>::Ptr& target_cloud,
//                                      Eigen::Matrix4f& transform_matrix)
// {
//     std::cout<<"special"<<std::endl;
// }


template<typename T_p>
Gicp<T_p>::Gicp()
    : nh("~")
{
    package_path = ros::package::getPath("mapping");
    nh.param<std::string>("cloud_path", cloud_path, "/data/remove");
    nh.param<std::string>("tf_path", tf_path, "/data/tf");
    nh.param<std::string>("save_path", save_path, "/data/trans_cloud");

    cloud_path.insert(0, package_path);
    tf_path.insert(0, package_path);
    save_path.insert(0, package_path);
}

template<typename T_p>
void Gicp<T_p>::gicp(typename pcl::PointCloud<T_p>::Ptr& source_cloud, 
                     typename pcl::PointCloud<T_p>::Ptr& target_cloud, 
                     Eigen::Matrix4f& transformation_matrix)
{
	pcl::GeneralizedIterativeClosestPoint<T_p, T_p> gicp; 
	gicp.setMaxCorrespondenceDistance (0.7);
	gicp.setMaximumIterations (100);        
	gicp.setTransformationEpsilon (1e-8);   
	gicp.setEuclideanFitnessEpsilon (1e-8);
    gicp.setInputSource(source_cloud);
    gicp.setInputTarget(target_cloud);
    pcl::PointCloud<T_p> Final;
    gicp.align(Final);

    transformation_matrix = gicp.getFinalTransformation();
    print4x4Matrix(transformation_matrix);
}

template<typename T_p>
void Gicp<T_p>::accuracy(tf::Transform source_transform,
                         tf::Transform target_transform,
                         Eigen::Matrix4f gicp_matrix,
                         Eigen::Matrix4f& matrix)
{
    tf::Vector3 source = source_transform.getOrigin();
    tf::Vector3 target = target_transform.getOrigin();
    
    double s_roll, s_pitch, s_yaw;
    double t_roll, t_pitch, t_yaw;
    tf::Matrix3x3(source_transform.getRotation()).getRPY(s_roll, s_pitch, s_yaw);
    tf::Matrix3x3(target_transform.getRotation()).getRPY(t_roll, t_pitch, t_yaw);
    
    tf::Transform transform;
    tf::Vector3 vector(target.x() - source.x(), 
                       target.y() - source.y(), 
                       target.z() - source.z());
    tf::Quaternion quaternion = tf::createQuaternionFromRPY(t_roll - s_roll,
                                                            t_pitch - s_pitch,
                                                            t_yaw - s_yaw);
    transform.setOrigin(vector);
    transform.setRotation(quaternion);
    
    double x = transform.getOrigin().x();
    double y = transform.getOrigin().y();
    double z = transform.getOrigin().z();
    double roll, pitch, yaw;
    tf::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);
    std::cout<<"x:"<<x<<" y:"<<y<<" z:"<<z
             <<"roll:"<<roll<<" picth:"<<pitch<<" yaw:"<<yaw<<std::endl;
}


template<typename T_p>
void Gicp<T_p>::main()
{
    std::cout<<"main"<<std::endl;

    int file_size = Fc.file_count_boost(cloud_path.c_str());

    std::cout<<"file size"<<file_size<<std::endl;

    for(int i=0;i<file_size-1;i++){
        std::cout<<"node:"<<i<<" --- "<<i+1<<std::endl;
        typename pcl::PointCloud<T_p>::Ptr source_cloud(new pcl::PointCloud<T_p>);
        typename pcl::PointCloud<T_p>::Ptr target_cloud(new pcl::PointCloud<T_p>);
        tf::Transform source_transform;
        tf::Transform target_transform;

        std::string source_cloud_name = cloud_path + "/" + std::to_string(i) + ".pcd";
        std::string target_cloud_name = cloud_path + "/" + std::to_string(i+1) + ".pcd";
        std::string source_tf_name = tf_path + "/" + std::to_string(i) + ".csv";
        std::string target_tf_name = tf_path + "/" + std::to_string(i+1) + ".csv";

        // load 
        bool source_cloud_flag = Fc.loadCloud(source_cloud, source_cloud_name);
        bool target_cloud_flag = Fc.loadCloud(target_cloud, target_cloud_name);
        bool source_tf_flag = Fc.loadTF(source_transform, source_tf_name);
        bool target_tf_flag = Fc.loadTF(target_transform, target_tf_name);

        if(!source_cloud_flag || !target_cloud_flag || !source_tf_flag || !target_tf_flag)
            continue;

        // transform PointCloud
        typename pcl::PointCloud<T_p>::Ptr transform_source_cloud(new pcl::PointCloud<T_p>);
        typename pcl::PointCloud<T_p>::Ptr transform_target_cloud(new pcl::PointCloud<T_p>);
        Fc.transform_pointcloud(source_cloud, transform_source_cloud, source_transform);
        Fc.transform_pointcloud(target_cloud, transform_target_cloud, target_transform);

        // icp
        Eigen::Matrix4f transformation_matrix;
        gicp(target_cloud, source_cloud, transformation_matrix);

        Eigen::Matrix4f matrix;
        accuracy(source_transform, target_transform, transformation_matrix, matrix);
    }
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "gicp");

    Gicp<pcl::PointXYZINormal> gc;

    gc.main();

    return 0;
}
