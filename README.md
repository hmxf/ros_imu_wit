
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
   cd wit_ros_imu/
   catkin_make
   cd src/scripts/
   sudo chmod 777 *.py
   source devel/setup.bash
   ```

3. ROS 驱动和可视化

   1. 查看USB端口号。
   
      先不要插 IMU 的 USB ，在终端输入 `ls  /dev/ttyUSB*` 来检测一下，然后在将 USB 插入电脑，再在终端输入 `ls  /dev/ttyUSB*` 来检测一下，多出来的 ttyUSB 设备就是 IMU 的串口。

      ![USB端口号](https://witpic-1253369323.cos.ap-guangzhou.myqcloud.com/img-md/8491ff53-3d0c-4f4e-89c0-f06a37085cdc)

   2. 修改参数配置。
   
      需要修改的参数包括设备类型，USB 端口号和波特率。进入脚本目录 src/launch，修改对应的 launch 文件中的配置参数。设备号 /dev/ttyUSB0 改为你电脑识别出来的结果，波特率根据实际使用设定，默认波特率为 9600，如果用户通过上位机修改了波特率，需要对应修改成修改后的波特率。

      ![参数修改](https://witpic-1253369323.cos.ap-guangzhou.myqcloud.com/img-md/43e74867-4ec5-4237-804c-32a417390f61)


   3. 给对应的串口管理员权限。
   
      在终端输入：
      
      ```bash
      sudo chmod 777 /dev/ttyUSB0
      ```
      
      每次重新插入 USB 口都需要重新给串口赋管理员权限。

   4. 如果使用的产品是 Modbus 协议的，还需要安装一下 Modbus 的依赖库，在终端输入
   
      ```bash
      pip3 install modbus_tk
      ```

   5. 打开终端，运行launch文件

      ```bash
      roslaunch wit_ros_imu display_and_imu.launch
      ```

      ![Description](https://witpic-1253369323.cos.ap-guangzhou.myqcloud.com/img-md/2a7ddca1-016c-49d1-a85d-b4568a468027)
   
      打开两个新终端输入分别输入下面几行命令

      ```
      rostopic echo /wit/imu
      rostopic echo /wit/mag
      rostopic echo /wit/location  #经纬度解析只有 WIT 私有协议
      ```

      如下图，驱动运行成功后 `rostop echo` 输出的信息

      ![Description](https://witpic-1253369323.cos.ap-guangzhou.myqcloud.com/img-md/8522c2d4-da59-4580-810d-dd3f2a4c7aa5)


   6. 同理，如需要运行其他 launch 文件，需要先确保 launch 文件中的 /dev/ttyUSB0 设备修改对。

   7. 相关文件说明

      - wit_imu.launch
         - 打开 IMU 驱动节点。
      - rviz_and_imu.launch
         - 打开 IMU 驱动节点和 Rviz 可视化。
