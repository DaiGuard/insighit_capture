# insight_capture

Cognex insight 2000 camera capture package for ROS2

[IN-SIGHT 2000 ビジョンセンサ](https://www.cognex.com/ja-jp/products/machine-vision/vision-sensors/in-sight-2000-vision-sensors)

![](https://www.cognex.com/library/media/products/products-home-module/in-sight-2000.png)

---

### Installation

```bash
# clone repository
mkdir -p colcon_ws/src
cd colcon_ws
git clone -b ros https://github.com/DaiGuard/insight_capture src/insight_capture

# build package
rosdep update
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install

# run 
source install/local_setup.bash
ros2 launch insight_capture default.launch.py \
    ip:="192.168.2.21" \
    port:=23 \
    user:="admin"
```

### Publisher

* /camera [sensor_msgs/Image]

    camera capture image

### Parameters

* camera_ip [str] : default="127.0.0.1"

    camera ip address

* camera_port [int] : default=23

    camera port number

* camera_user [string] : default="admin"

    camera login username

* camera_pw [string] : default=""

    camera login password

* lib_path [string] : default="insight_capture_cpp_lib/lib/libinsight_capture_cpp_lib.so"

    insight_capture_cpp_lib library path

### TODO

- [x] 画像データバイト列のNumpyデータ配列への変換
- [x] ROSノードへの対応
- [x] ROSパラメータの整理、ユーザー名、パスワード、IPアドレス