/*
 *
 * src : loop_detector.cpp
 * Author : Yudai Sadakuni
 *
 * pkg : mapping
 * createrd : 2019.1.8
 * latestupdate : 2019.1.8
 *
 * memo : 再訪判定に使用. 任意のNodeから一定範囲内にある別のNodeを探し、
 *        ２つのNodeの位置情報と点群情報を保存
 */

#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>

#include <sensor_msgs/PointCloud.h>

#include <Eigen/Core>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <mapping/util.h>

#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

// inputをdelimiterでsplit
std::vector<std::string> split(std::string& input, char delimiter)
{
    std::istringstream stream(input);
    std::string field;
    std::vector<std::string> result;
    while (getline(stream, field, delimiter)) {
        result.push_back(field);
    }
    return result;
}

class LoopDetector{
    private:
        ros::NodeHandle nh;

        std::string package_path;
        std::string aft_file;

        double distance;
        double dist;

        std::vector< Eigen::Matrix4f > poses;

        Util ul;

    public:
        LoopDetector();

        void load(std::string file_path);
        void main();
};

LoopDetector::LoopDetector()
    : nh("~")
{
    package_path = ros::package::getPath("mapping");

    nh.param<std::string>("aft_file", aft_file, "aft.csv");
    nh.param<double>("distance", distance, 5.0);
    nh.param<double>("dist", dist, 20);
}

void LoopDetector::load(std::string file_path)
{
    std::ifstream ifs(file_path);

    if(ifs.fail()){
        std::cout<<" File is None "<<std::endl;
        exit(0);
    }

    std::string line;
    while(getline(ifs, line)){
        std::vector<std::string> strvec = split(line, ' ');
        if(strvec.at(0) != "VERTEX_SE3:QUAT")
            continue;
        
        Eigen::Vector3f euler(std::stof(strvec.at(2)),
                              std::stof(strvec.at(3)),
                              std::stof(strvec.at(4)));
        Eigen::Quaternionf quaternion(std::stof(strvec.at(5)),
                                      std::stof(strvec.at(6)),
                                      std::stof(strvec.at(7)),
                                      std::stof(strvec.at(8)));
        
        Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
        matrix(0, 3) = euler(0);
        matrix(1, 3) = euler(1);
        matrix(2, 3) = euler(2);
        matrix.block(0, 0, 3, 3) = ul.quat2mat(quaternion);

        poses.push_back(matrix);
    }
}

void LoopDetector::main()
{

    // bfr.csv and aft.csv are saved at home directory
    struct passwd *pw = getpwuid(getuid());
    const char *homedir = pw->pw_dir;
    std::string bfr_path = std::string(homedir) + "/bfr.csv";
    std::string aft_path = std::string(homedir) + "/aft.csv";

    load(aft_path);

    std::cout << poses.size() << std::endl;
    std::cout << poses.front().size() << std::endl;

    for(size_t i=0;i<poses.size();i++){
        
        size_t id = 0;
        float min_dist = INFINITY;
        for(size_t j=0;j<poses.size();j++){
            if(i==j) continue;
            float delta_x = poses[i](0, 3) - poses[j](0, 3);
            float delta_y = poses[i](1, 3) - poses[j](1, 3);
            float delta_z = poses[i](2, 3) - poses[j](2, 3);
            float delta = sqrt(delta_x*delta_x + delta_y*delta_y + delta_z*delta_z);
            if(delta<min_dist){
                id = j;
                min_dist = delta;
            }
        }

        if(distance<min_dist) continue;

        std::cout<< i <<" "<< id <<" "<< min_dist << std::endl;

        Eigen::Matrix4f T = poses[id].inverse() * poses[i];
        std::cout<<"T = "<< std::endl << T << std::endl;
    }

    {{{
    /*
    std::vector< std::vector<double> > data;
    data.assign(poses.size(), std::vector<double>(poses.size(), INFINITY));

    for(size_t i=0;i<data.size();i++){
        for(size_t j=0;j<data.front().size();j++){
            if(i==j) continue;
            double delta_x = poses[i](0,3) - poses[j](0,3);
            double delta_y = poses[i](1,3) - poses[j](1,3);
            double delta_z = poses[i](2,3) - poses[j](2,3);
            float delta = sqrt(delta_x*delta_x + delta_y*delta_y + delta_z*delta_z);
            data[i][j] = delta;
        }
    }

    std::vector< int > min_node(data.size(), 0);
    for(size_t i=0;i<data.size();i++){
        double min = data[i][0];
        for(size_t j=1;j<data[i].size();j++){
            if(data[i][j]<min)
            {
                min_node[i] = j;
                min = data[i][j];
            }
        }
    }

    for(size_t i=0;i<min_node.size();i++)
        std::cout<<i<<" : "<<min_node[i]<<std::endl;
    */
    }}}

    poses.clear();
    load(aft_path);
    std::cout << poses.size() << std::endl;
    std::cout << poses.front().size() << std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "loop_detector");

    LoopDetector ld;
    
    ld.main();

    return 0;
}
