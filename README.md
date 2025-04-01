<div align="center">
  <h1>LodeStar</h1>
  <a href="/"><img src="https://img.shields.io/badge/-C++-blue?logo=cplusplus" /></a>
  <a href="https://ieeexplore.ieee.org/document/10380692"><img src="https://img.shields.io/badge/Paper-PDF-yellow" alt="Paper" /></a>
  <a href="https://arxiv.org/abs/2403.02773"><img src="https://img.shields.io/badge/arXiv-2410.01325-b31b1b.svg?style=flat-square" alt="Arxiv" /></a>
  <a href="https://youtu.be/YRMNGUgaGSI?feature=shared"><img src="https://badges.aleen42.com/src/youtube.svg" alt="YouTube" /></a>
  <br />
  <br />
</div>
This repository contains the test code for the descriptor in the paper "LodeStar: Maritime Radar Descriptor for Semi-Direct Radar Odometry"

Repo is mainly dealing with the synthesis of descriptor LodeStar and point normal matcher from [CFEAR](https://github.com/dan11003/CFEAR_Radarodometry_code_public.git)

![lodestar_results](https://github.com/hyesu-jang/LodeStar/assets/30336462/08def05c-b2ac-4d4d-aed5-bdd605084c0c)


## Prerequisites
  * Google Ceres solver  http://ceres-solver.org/installation.html
  * ROS [Noetic](http://wiki.ros.org/noetic) or later, tested with ubuntu 20.04

## Build with catkin

```
$ cd ~/catkin_ws/src/
$ clone this project
$ cd ~/catkin_ws
$ catkin_make
$ source ~/catkin_ws/devel/setup.bash
```

## How to operate odometry estimation system
First you should prepare the bag file containing cartesian type radar image.
Default radar topic name is set as /radar_image_inrange
You can revise the file below to set the bag file name and path. Extra parameters can be revised here.
```
$ ~/catkin_ws/src/lodestar_odometry/launch/run_lodestar_odom
```

## Running
We provide offline estimation that takes maximum frequency.
We will update online estimation version if necessesary

```
$ roscore
$ roscd lodestar_odometry/launch
$ ./run_lodestar_odom
```

## Result Analysis
Odometry estimation results are saved in the "eval" folder with KITTI type (Default: Located in the bagfile path).
You can change saving path in "run_lodestar_odom" file above.
You can simply evaluate the odometry result using [evo](https://github.com/MichaelGrupp/evo) tool


## Sample Radar File
You can download and test the code with the example x-band radar file [Download](https://drive.google.com/drive/folders/12PN696UkMj0rJ62Ug7zIjDkgvIkPOzi1?usp=sharing)


## Citation
```
@article{jang2024lodestar,
  title={LodeStar: Maritime Radar Descriptor for Semi-Direct Radar Odometry},
  author={Jang, Hyesu and Jung, Minwoo and Jeon, Myung-Hwan and Kim, Ayoung},
  journal={IEEE Robotics and Automation Letters},
  volume={9},
  number={2},
  pages={1684--1691},
  year={2024},
  publisher={IEEE}
}

```

## Contact
* Hyesu Jang (dortz at snu dot ac dot kr)



