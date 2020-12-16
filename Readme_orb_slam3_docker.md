## 目录
* [本地跑orbslam3](#本地跑orbslam3)
* [docker镜像(失败)](#docker镜像)

## 本地跑orbslam3
**请参考ORB_SLAM3官方教程，这里仅仅简略记录部分关键点** 
1. Pangolin安装，参见surfelwarp中Pangolin的安装  
2. OpenCV，依照官方的说法，ORB_SLAM3在opencv=3.2.0测试通过，参见：  
https://github.com/xiaotaw/Notes/blob/master/computer_vision/opencv.md  
3. 编译`./build.sh`
**修改build.sh**  
```vim
mkdir build
cd build
cmake
-DCMAKE_BUILD_TYPE=Release \
-DPangolin_DIR=../../Others/Pangolin/install_static/lib/cmake/Pangolin \
-DOpenCV_DIR=/usr/local/opencv-3.2.0 \
..
```

5. 编译ros版本，参见README.md

6. EuRoC数据测试orbslam3(ros版)   
Download a rosbag (e.g. V1_02_medium.bag) from the EuRoC dataset (http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets). Open 3 tabs on the terminal and run the following command at each tab for a Stereo-Inertial configuration:
```bash
roscore
rosrun ORB_SLAM3 Stereo_Inertial Vocabulary/ORBvoc.txt Examples/Stereo-Inertial/EuRoC.yaml true
rosbag play --pause V1_02_medium.bag /cam0/image_raw:=/camera/left/image_raw /cam1/image_raw:=/camera/right/image_raw /imu0:=/imu
```

7. 测试l515录制的bag
7.1 修改相机参数文件L515.yaml
7.2 播放录制的数据rs_20201215_2.bag：
```
roscore
rosbag play --pause /media/xt/8T/DATASETS/RealSenseL515Dataset/rs_20201215_2.bag \
    -s 30 \
    /camera/color/image_raw:=/camera/image_raw \
    /camera/imu:=/imu 
```
7.3 需rosbag开始play之后，再启动orb_slam3，否则运行失败，原因未知
```
rosrun ORB_SLAM3 Mono_Inertial Vocabulary/ORBvoc.txt Examples/Monocular-Inertial/L515.yaml 
```

8. ubuntu18.04安装使用realsense
```bash
# 安装realsense sdk
sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main" -u
sudo apt-get install -y librealsense2-dkms librealsense2-utils librealsense2-dev
# 连线测试
realsense-viewer

# 安装realsense官方ros包
sudo apt-get install ros-melodic-realsense2-camera
# 连线测试，需修改rs_camera.launch
roslaunch realsense2-camera rs_camera.launch
```





## docker镜像
(失败)

```
docker run -it --privileged --net=host --ipc=host     --device=/dev/dri:/dev/dri     --device=/dev/ttyUSB0:/dev/ttyUSB0     -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY     -v $HOME/.Xauthority:/home/$(id -un)/.Xauthority     -e XAUTHORITY=/home/$(id -un)/.Xauthority -e ROS_IP=127.0.0.1     -v /data/:/data/     turlucode/ros-kinetic:cpu

docker run -it --privileged --net=host --ipc=host     --device=/dev/dri:/dev/dri     --device=/dev/ttyUSB0:/dev/ttyUSB0     -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY     -v $HOME/.Xauthority:/home/$(id -un)/.Xauthority     -e XAUTHORITY=/home/$(id -un)/.Xauthority -e ROS_IP=127.0.0.1     -v /data/:/data/     turlucode/ros-kinetic:cuda8-cudnn6
```

**容器中**
```bash
apt-get update && apt-get install -y apt-transport-https
echo -e "deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ xenial main restricted universe multiverse \n\
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ xenial-updates main restricted universe multiverse \n\
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ xenial-backports main restricted universe multiverse \n\
deb https://mirrors.tuna.tsinghua.edu.cn/ubuntu/ xenial-security main restricted universe multiverse" > /etc/apt/sources.list
apt-get update && apt-get upgrade


# serial
apt-get install -y ros-kinetic-serial

# for pangolin
apt-get install -y libglew-dev


# install pangolin for orbslam3
cd /data/Others/Pangolin
mkdir build && cd build
cmake .. && make -j $nproc

make install


# install opencv=3.1.0 for orbslam3
cmake \
-D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
-D OPENCV_ENABLE_NONFREE=ON \
-D BUILD_EXAMPLES=OFF \
-D BUILD_TESTS=OFF \
-D BUILD_PREF_TESTS=OFF \
..

make -j $nproc 
make install 

# 报错：fatal error hdf5.h no such file or directory
# 参考解决: https://blog.csdn.net/weixin_41600500/article/details/85346327
#           https://github.com/opencv/opencv/issues/6016

# 编译orb_slam3
orb_slam3与opencv3.1.0 编译失败，计划切换到opencv3.2.0，官网测试3.2.0可行。


# install librealsense for realsense-l515 camera
apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main" -u
apt-get install -y librealsense2-dkms librealsense2-utils librealsense2-dev

# test librealsense
realsense-viewer

(failed)





```





```bash
# 

docker run --net=host --env="DISPLAY" --volume="$HOME/.Xauthority:/root/.Xauthority:rw" --privileged -v /dev:/dev -v /tmp/.X11-unix:/tmp/.X11-unix -it intelpengo/
```
















