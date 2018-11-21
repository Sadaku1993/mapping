# Graph Based SLAM 

# Requirement

- ros(kinetic)
- PCL1.8

# Set Up

1. ROS Kinetic
2. PCL1.8
3. velodyne package

# How to Use

## Make data directory for pointcloud ext.

```bash
$ roscd mapping
$ chmod a+x bash.sh
$ ./bash.sh
```

```bash
mapping/
    CMakeLists.txt
    src/
    include/
    package.xml
    README.md
    data/
```

## Save PointCloud and Odometry Data

```bash
$ roslaunch mapping savedata.launch
```

```bash
mapping/
    CMakeLists.txt
    src/
    include/
    package.xml
    README.md
    data/
        pointcloud/ <--- pointcloud data are saved here
        odometry/   <--- odometry data are saverd here
```

## Normal Estimation
In order to gicp we need to calcrate pointcloud's normal information.
Use pcl function.

```bash
$ roslaunch mapping normal_estimation.launch
```

## Generalised Iterative Closest Point

```bash
$ roslaunch mapping gicp.launch
```

## Loop Closing

```bash
$ roslaunch mapping g2o.launch
```

## Show Result

```bash
$roslaunch mapping show_map.launch
```
