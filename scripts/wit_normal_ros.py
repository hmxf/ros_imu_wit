#!/usr/bin/env python
# -*- coding:utf-8 -*-
import serial
import struct
import rospy
import time
import math
import sys
import platform
import threading
import serial.tools.list_ports
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler

# 查找 ttyUSB* 设备
# Look for ttyUSB* devices
def find_ttyUSB():
    print('IMU 默认串口为 /dev/ttyUSB0，若识别多个串口设备，请在 launch 文件中修改 IMU 对应的串口')
    #print('The default serial port of the IMU is /dev/ttyUSB0, if multiple serial port devices are identified, modify the serial port corresponding to the IMU in the launch file')
    posts = [port.device for port in serial.tools.list_ports.comports() if 'USB' in port.device]
    print('当前电脑所连接的 {} 串口设备共 {} 个：{}'.format('USB', len(posts), posts))
    #print('There are {} {} serial port devices connected to the current PC: {}'.format(len(posts), 'USB', posts))

# 校验
def checkSum(list_data, check_data):
    return sum(list_data) & 0xff == check_data

# 16 进制转 IEEE 浮点数
def hex_to_short(raw_data):
    return list(struct.unpack("hhhh", bytearray(raw_data)))

# GPS 经纬度解析
def hex_to_data(raw_data):
    return list(struct.unpack("i", bytearray(raw_data)))
    
# GPS 高度解析
def hex_to_altitude(raw_data):
    return list(struct.unpack("h", bytearray(raw_data)))
    
# 处理串口数据
def handleSerialData(raw_data):
    global buff, key, angle_degree, magnetometer, acceleration, angularVelocity, pub_flag, readreg, calibuff, flag, mag_offset, mag_range, version, longitude_imu, latitude_imu, altitude_imu
    angle_flag=False
    if python_version == '2':
        buff[key] = ord(raw_data)
    if python_version == '3':
        buff[key] = raw_data

    key += 1
    if buff[0] != 0x55:
        key = 0
        return
    if key < 11:  # 根据数据长度位的判断, 来获取对应长度数据
        return
    else:
        data_buff = list(buff.values())  # 获取字典所有 value
        if buff[1] == 0x51 :
            if checkSum(data_buff[0:10], data_buff[10]):
                acceleration = [hex_to_short(data_buff[2:10])[i] / 32768.0 * 16 * 9.8 for i in range(0, 3)]
            else:
                print('0x51 校验失败')
                #print('0x51 Check failure')

        elif buff[1] == 0x52:
            if checkSum(data_buff[0:10], data_buff[10]):
                angularVelocity = [hex_to_short(data_buff[2:10])[i] / 32768.0 * 2000 * math.pi / 180 for i in range(0, 3)]
            else:
                print('0x52 校验失败')
                #print('0x52 Check failure')

        elif buff[1] == 0x53:
            if checkSum(data_buff[0:10], data_buff[10]):
                temp = hex_to_short(data_buff[2:10])
                angle_degree = [temp[i] / 32768.0 * 180 for i in range(0, 3)]
                version = temp[3]
                angle_flag = True
            else:
                print('0x53 校验失败')
                #print('0x53 Check failure')

        elif buff[1] == 0x54:
            if checkSum(data_buff[0:10], data_buff[10]): 
                magnetometer = hex_to_short(data_buff[2:10])
                if flag:
                    calibuff.append(magnetometer[0:2])
            else:
                print('0x54 校验失败')
                #print('0x54 Check failure')


        elif buff[1] == 0x57:
            if checkSum(data_buff[0:10], data_buff[10]):
                longitude_imu = (hex_to_data(data_buff[2:6])[0]  // 10000000.0 * 100 ) +  ((hex_to_data(data_buff[2:6])[0]  % 10000000) / 10000000.0)
                latitude_imu = (hex_to_data(data_buff[6:10])[0]  // 10000000.0 * 100 ) +((hex_to_data(data_buff[6:10])[0] % 10000000) / 10000000.0)
            else:
                print('0x57 校验失败')
                #print('0x57 Check failure')
 
        elif buff[1] == 0x58:
            if checkSum(data_buff[0:10], data_buff[10]): 
                altitude_imu = hex_to_altitude(data_buff[2:4])[0]  / 10.0
            else:
                print('0x58 校验失败')
                #print('0x58 Check failure')
                
        elif buff[1] == 0x5f:
            if checkSum(data_buff[0:10], data_buff[10]):
                readval = hex_to_short(data_buff[2:10])
                if readreg == 0x0b:
                    mag_offset = readval
                else:
                    mag_range = readval
                print(readval)
            else:
                print('0x5F 校验失败')
                #print('0x5F Check failure')

        else:
            print("数据处理类没有提供 " + str(buff[1]) + " 的解析")
            print("或数据错误")
            buff = {}
            key = 0

        buff = {}
        key = 0
        if angle_flag:
            stamp = rospy.get_rostime()

            imu_msg.header.stamp = stamp
            imu_msg.header.frame_id = "base_link"

            mag_msg.header.stamp = stamp
            mag_msg.header.frame_id = "base_link"

            location_msg.header.stamp = stamp
            location_msg.header.frame_id = "base_link"
            
            angle_radian = [angle_degree[i] * math.pi / 180 for i in range(3)]
            qua = quaternion_from_euler(angle_radian[0], angle_radian[1], angle_radian[2])

            imu_msg.orientation.x = qua[0]
            imu_msg.orientation.y = qua[1]
            imu_msg.orientation.z = qua[2]
            imu_msg.orientation.w = qua[3]

            imu_msg.angular_velocity.x = angularVelocity[0]
            imu_msg.angular_velocity.y = angularVelocity[1]
            imu_msg.angular_velocity.z = angularVelocity[2]

            imu_msg.linear_acceleration.x = acceleration[0]
            imu_msg.linear_acceleration.y = acceleration[1]
            imu_msg.linear_acceleration.z = acceleration[2]

            mag_msg.magnetic_field.x = magnetometer[0]
            mag_msg.magnetic_field.y = magnetometer[1]
            mag_msg.magnetic_field.z = magnetometer[2]

            imu_pub.publish(imu_msg)
            mag_pub.publish(mag_msg)
            location_pub.publish(location_msg)

            location_msg.longitude = longitude_imu
            location_msg.latitude = latitude_imu
            location_msg.altitude = altitude_imu

altitude_imu = 0
longitude_imu = 0
latitude_imu = 0
version = 0
readreg = 0
key = 0
flag = 0
iapflag = 0
buff = {}
calibuff = list()
angularVelocity = [0, 0, 0]
acceleration = [0, 0, 0]
magnetometer = [0, 0, 0]
angle_degree = [0, 0, 0]
mag_offset = [0, 0, 0]
mag_range = [0, 0, 0]
baudlist = [4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800]

def recordThread():
    global recordflag, recordbuff
    recordflag = 1
    recordbuff = ''
    recordname = time.strftime("%Y%m%d%H%M%S", time.localtime()) + '.txt'
    fd = open(recordname, 'w+')
    print('开始录制数据到文件 {}'.format(recordname))
    #print('Begin recording file name is {}'.format(recordname))
    while recordflag:
        if len(recordbuff):
            fd.write(recordbuff)
            recordbuff = ''
        else:
            time.sleep(1)

    fd.close()
    print('停止录制数据')
    #print('Stop recording')

def callback(data):
    global readreg, flag, calibuff, imu_wt, iapflag, mag_offset, mag_range, version, recordflag, baudlist
    unlock_imu_cmd = b'\xff\xaa\x69\x88\xb5'
    reset_magx_offset_cmd = b'\xff\xaa\x0b\x00\x00'
    reset_magy_offset_cmd = b'\xff\xaa\x0c\x00\x00'
    reset_magz_offset_cmd = b'\xff\xaa\x0d\x00\x00'
    enter_mag_cali_cmd = b'\xff\xaa\x01\x09\x00'
    exit_cali_cmd = b'\xff\xaa\x01\x00\x00'
    save_param_cmd = b'\xff\xaa\x00\x00\x00'
    read_mag_offset_cmd = b'\xff\xaa\x27\x0b\x00'
    read_mag_range_cmd = b'\xff\xaa\x27\x1c\x00'
    reboot_cmd = b'\xff\xaa\x00\xff\x00'
    reset_mag_param_cmd = b'\xff\xaa\x01\x07\x00'
    set_rsw_demo_cmd = b'\xff\xaa\x02\x1f\x00'  #output time acc gyro angle mag

    print('回调')
    #print('Callback')
    print(data)
    if "mag" in data.data:
        imu_wt.write(unlock_imu_cmd)
        time.sleep(0.1)
        imu_wt.write(reset_magx_offset_cmd)
        time.sleep(0.1)
        imu_wt.write(reset_magy_offset_cmd)
        time.sleep(0.1)
        imu_wt.write(reset_magz_offset_cmd)
        time.sleep(0.1)
        imu_wt.write(reset_mag_param_cmd)
        time.sleep(0.1)
        imu_wt.write(enter_mag_cali_cmd)
        time.sleep(0.1)
        flag = 1
        calibuff = []
        mag_offset = [0, 0, 0]
        mag_range = [500, 500, 500]
    elif "exit" in data.data:
        flag = 0
        imu_wt.write(unlock_imu_cmd)
        time.sleep(0.1)
        imu_wt.write(exit_cali_cmd)
        time.sleep(0.1)
        imu_wt.write(save_param_cmd)
        time.sleep(1)
        readreg = 0x0b
        imu_wt.write(read_mag_offset_cmd)
        time.sleep(1)
        readreg = 0x1c
        imu_wt.write(read_mag_range_cmd)
        time.sleep(1)
        datalen = len(calibuff)
        print('校准数据 {}'.format(datalen))
        #print('Calibration data {}'.format(datalen))
        r = list()
        if datalen > 0:
            for i in range(datalen):
                tempx = ((calibuff[i][0] - mag_offset[0])*2/float(mag_range[0]))
                tempy = ((calibuff[i][1] - mag_offset[1])*2/float(mag_range[1]))
                temp = tempx*tempx+tempy*tempy-1
                r.append(abs(temp))
            sumval = sum(r)
            r_n = float(sumval)/datalen
            if r_n < 0.05:
                print('磁场校准结果非常好')
                #print('Magnetic field calibration results are very good')
            elif r_n < 0.1:
                print('磁场校准结果较好')
                #print('Magnetic field calibration results are good')
            else :
                print('磁场校准结果较差，请重试')
                #print('Magnetic field calibration results is bad, please try again')
    elif "version" in data.data:
        print('传感器版本： {}'.format(version))
        #print('Sensor version is {}'.format(version))
    elif "begin" in data.data:
        record_thread = threading.Thread(target = recordThread)
        record_thread.start()
    elif "stop" in data.data:
        recordflag = 0
    elif "rate" in data.data:
        ratelist = [0.2, 0.5, 1,2,5,10,20,50,100,125,200]
        try:
            val = data.data[4:]
            rate = float(val)
            for i in range(len(ratelist)):
                if rate == ratelist[i]:
                    print('数据传输速率设置为 {}'.format(rate))
                    #print('Change {} rate'.format(rate))
                    val = i + 1
                    cmd = bytearray(5)
                    cmd[0] = 0xff
                    cmd[1] = 0xaa
                    cmd[2] = 0x03
                    cmd[3] = val
                    cmd[4] = 0x00
                    imu_wt.write(unlock_imu_cmd)
                    time.sleep(0.1)
                    imu_wt.write(cmd)
        except Exception as e:
            print(e)
    elif "baud" in data.data:
        try:
            val = data.data[4:]
            baud = float(val)
            for i in range(len(baudlist)):
                if baud == baudlist[i]:
                    val = i + 1
                    cmd = bytearray(5)
                    cmd[0] = 0xff
                    cmd[1] = 0xaa
                    cmd[2] = 0x04
                    cmd[3] = val
                    cmd[4] = 0x00
                    imu_wt.write(unlock_imu_cmd)
                    time.sleep(0.1)
                    imu_wt.write(cmd)
                    time.sleep(0.1)
                    imu_wt.baudrate = baud
        except Exception as e:
            print(e)
    elif "rsw" in data.data:
        imu_wt.write(unlock_imu_cmd)
        time.sleep(0.1)
        imu_wt.write(set_rsw_demo_cmd)
        time.sleep(0.1)

def thread_job():
    print("线程启动")
    #print("Thread run")
    rospy.spin()

def AutoScanSensor():
    global imu_wt, baudlist
    try:
        for baud in baudlist:
            read_cmd = '\xff\xaa\x27\x00\x00'.encode("utf-8")
            imu_wt.baudrate = baud
            imu_wt.flushInput()
            imu_wt.write(read_cmd)
            time.sleep(0.2)
            buff_count = imu_wt.inWaiting()
            if buff_count >= 11:
                buff_data = imu_wt.read(buff_count)
                val = bytearray(buff_data)
                for i in range(len(val)):
                    if val[i] == 0x55:
                        sumval = sum(val[i:i+10])
                        if sumval == val[i+10]:
                            print('找到了工作波特率为 {} 的传感器'.format(baud))
                            #print('{} baud find sensor'.format(baud))
                            return

    except Exception as e:
        print("Exception:" + str(e))
        print("IMU 失去连接，接触不良，或断线")
        #print("IMU loss of connection, poor contact, or broken wire")
        exit(0)

if __name__ == "__main__":
    global recordflag, recordbuff, imu_wt
    recordflag = 0
    recordbuff = list()
    imu_wt = serial.Serial()
    python_version = platform.python_version()[0]

    find_ttyUSB()
    rospy.init_node("imu")
    port = rospy.get_param("~port", "/dev/ttyUSB0")
    baudrate = rospy.get_param("~baud", 9600)
    # baudrate = 115200
    print("IMU 类型：normal 端口：%s 波特率：%d" %(port,baudrate))
    #print("IMU Type: Normal Port:%s baud:%d" %(port,baudrate))
    imu_msg = Imu()
    mag_msg = MagneticField()
    location_msg = NavSatFix()
    rospy.Subscriber("/wit/cali", String, callback) #接受topic名称
    add_thread = threading.Thread(target = thread_job)
    add_thread.start()
    try:
        imu_wt = serial.Serial(port=port, baudrate=baudrate, timeout=10)
        if imu_wt.isOpen():
            rospy.loginfo("\033[32m端口已打开...\033[0m")
            #rospy.loginfo("\033[32mPort already opened...\033[0m")
        else:
            imu_wt.open()
            rospy.loginfo("\033[32m端口打开中...\033[0m")
            #rospy.loginfo("\033[32mPort opening...\033[0m")
    except Exception as e:
        print(e)
        rospy.loginfo("\033[31m端口打开失败\033[0m")
        #rospy.loginfo("\033[31mPort open failed\033[0m")
        exit(0)
    else:
        #AutoScanSensor()
        imu_pub = rospy.Publisher("wit/imu", Imu, queue_size=10)
        mag_pub = rospy.Publisher("wit/mag", MagneticField, queue_size=10)
        location_pub = rospy.Publisher("wit/location",NavSatFix,queue_size=10)

        while not rospy.is_shutdown():
            try:
                buff_count = imu_wt.inWaiting()
                if buff_count > 0 and iapflag == 0:
                    buff_data = imu_wt.read(buff_count)
                    if recordflag:
                        recordbuff = recordbuff + buff_data
                    for i in range(0, buff_count):
                        handleSerialData(buff_data[i])
            except Exception as e:
                print("Exception:" + str(e))
                print("IMU 失去连接，接触不良，或断线")
                #print("IMU loss of connection, poor contact, or broken wire")
                exit(0)
