#include <tf/tf.h>

class Util{
    public:
        void quat2mat(tf::Quaternion quaternion,
                      Eigen::Matrix3f& matrix);
};

void Util::quat2mat(tf::Quaternion quaternion,
                        Eigen::Matrix3f& matrix)
{
    float qx = quaternion.x;
    float qy = quaternion.y;
    float qz = quaternion.z;
    float qw = quaternion.w;

    matrix(0, 0) = 1 - 2*qy*qy - 2*qz*qz;
    matrix(0, 1) = 2*qx*qy - 2*qz*qw;
    matrix(0, 2) = 2*qx*qz + 2*qy*qw;
    matrix(1, 0) = 2*qx*qy + 2*qz*qw;
    matrix(1, 1) = 1 - 2*qx*qx - 2*qz*qz;
    matrix(1, 2) = 2*qy*qz - 2*qx*qw;
    matrix(2, 0) = 2*qx*qz - 2*qy*qw;
    matrix(2, 1) = 2*qx*qz + 2*qx*qw;
    matrix(2, 2) = 1 - 2*qx*qx - 2*qy*qy;
}

