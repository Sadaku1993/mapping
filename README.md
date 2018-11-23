# Graph Based SLAM 

# Requirement

- ros(kinetic)
- PCL1.8

# Set Up

1. ROS Kinetic
2. PCL1.8
3. velodyne package

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
$ roslaunch mapping remove_obstacle.launch
```

obstacle removed pointcloud are saved at mapping/data/rm_obstacle.

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
