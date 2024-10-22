# IMU软件包使用

1. 安装 ROS IMU 依赖

   请在终端运行对应的命令

   ```
   sudo apt-get install ros-noetic-imu-tools ros-noetic-rviz-imu-plugin
   pip3 install pyserial
   ```
2. 建立工作空间

   打开命令终端，运行下面指令：

   ```
   cd ros_imu_wit/
   catkin_make
   cd src/scripts/
   sudo chmod +x *.py
   source devel/setup.bash
   ```
3. ROS 驱动和可视化

   1. 查看USB端口号。

      先不要插 IMU 的 USB ，在终端输入 `ls -al /dev | grep imu` 来检测一下，然后在将 USB 插入电脑，再在终端输入 `ls -al /dev | grep imu` 来检测一下，多出来的 ttyUSB 设备就是 IMU 的串口。
   2. 修改参数配置。

      需要修改的参数包括设备类型，USB 端口号和波特率。进入脚本目录 `src/launch`，修改对应的 launch 文件中的配置参数。设备号 `/dev/imu_usb` 改为你电脑识别出来的结果，波特率根据实际使用设定，默认波特率为 9600，如果用户通过上位机修改了波特率，需要对应修改成修改后的波特率。
   3. 给对应的串口管理员权限。

      **如果将本仓库用作 [imu_gps_fusion](https://github.com/hmxf/imu_gps_fusion) 的子模块，则无需执行此操作，父仓库中有脚本执行该任务。**

      在终端输入：

      ```bash
      sudo chmod 777 /dev/imu_usb
      ```
      每次重新插入 USB 口都需要重新给串口赋管理员权限。

   4. 如果使用的产品是 Modbus 协议的，还需要安装一下 Modbus 的依赖库，在终端输入

      ```bash
      pip3 install modbus_tk
      ```
   5. 打开终端，运行launch文件

      ```bash
      roslaunch ros_imu_wit rviz_and_imu.launch
      ```
      打开若干新终端并分别输入下面几行命令

      ```
      rostopic echo /wit/imu
      rostopic echo /wit/mag
      rostopic echo /wit/location  #经纬度解析只有 WIT 私有协议
      ```
      驱动运行成功后 `rostopic echo` `<ros_node>` 应当有信息输出
   6. 同理，如需要运行其他 launch 文件，需要先确保 launch 文件中的 `/dev/imu_usb` 设备修改对。
   7. 相关文件说明

      - wit_imu.launch
        - 打开 IMU 驱动节点。
      - rviz_and_imu.launch
        - 打开 IMU 驱动节点和 Rviz 可视化。
