# Graph Based SLAM 

# Requirement

- ROS Kinetic
- PCL 1.8
- [g2o](https://github.com/RainerKuemmerle/g2o)

# Set Up

1. ROS Kinetic
2. PCL 1.8
3. [velodyne package](http://wiki.ros.org/ja/velodyne)
4. [g2o](https://github.com/RainerKuemmerle/g2o)

# How to Use

## Make directory for storing pointcloud etc.

```bash
$ roscd mapping
$ chmod a+x bash.sh
$ ./bash.sh
```

## Save PointCloud and Odometry Data

```bash
$ roslaunch mapping data_save.launch
```

```bash
mapping/
    CMakeLists.txt
    src/
    include/
    package.xml
    README.md
    data/
        cloud/ <--- raw pointcloud data are saved here
        tf/    <--- odometry data are saverd here
```

## Remove Obstacle

```bash
$ roslaunch mapping remove_cluster.launch
```

```bash
mapping/
    CMakeLists.txt
    src/
    include/
    package.xml
    README.md
    data/
        remove/ <-- pointcloud removed obstacle are saved here
```

## Generalised Iterative Closest Point

```bash
$ rosrun mapping gicp_v2
```

gicp results are saved at /data/relative_path and /data/absolute_path

## Generate Graph Optimization(g2o)

```bash
$ roslaunch mapping g2o
```

## Loop Closing

```bash
$ roslaunch mapping g2o.launch
```

## Show Result

```bash
$roslaunch mapping show_map.launch
```
