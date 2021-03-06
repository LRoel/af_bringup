#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import serial
import threading
import struct
import math
import numpy as np
import string
import re
import crcmod
from sensor_msgs.msg import LaserScan
from af_msgs.msg import Fuse_G_O

### crc32校验程序(GPS)
crc_novetal = crcmod.mkCrcFun(0x104C11DB7, 0, True, 0)

### 线程锁
mutex = threading.Lock()


# solution_status = {
#             0: "SOLUTION_STATUS_SOLUTION_COMPUTED",
#             1: Fuse_G_O.SOLUTION_STATUS_INSUFFICIENT_OBSERVATIONS,
#             2: Fuse_G_O.SOLUTION_STATUS_NO_CONVERGENCE,
#             3: Fuse_G_O.SOLUTION_STATUS_SINGULARITY_AT_PARAMETERS_MATRIX,
#             4: Fuse_G_O.SOLUTION_STATUS_COVARIANCE_TRACE_EXCEEDS_MAXIMUM,
#             5: Fuse_G_O.SOLUTION_STATUS_TEST_DISTANCE_EXCEEDED,
#             6: Fuse_G_O.SOLUTION_STATUS_COLD_START,
#             7: Fuse_G_O.SOLUTION_STATUS_VELOCITY_OR_HEIGHT_LIMIT_EXCEEDED,
#             8: Fuse_G_O.SOLUTION_STATUS_VARIANCE_EXCEEDS_LIMITS,
#             9: Fuse_G_O.SOLUTION_STATUS_RESIDUALS_TOO_LARGE,
#             10: Fuse_G_O.SOLUTION_STATUS_INVALID_FIX,
#             11: Fuse_G_O.SOLUTION_STATUS_UNAUTHORIZED,
#         }
# position_type_to_status = {
#             0: Fuse_G_O.POSITION_TYPE_NONE,
#             1: Fuse_G_O.POSITION_TYPE_FIXED,
#             2: Fuse_G_O.POSITION_TYPE_FIXEDHEIGHT,
#             4: Fuse_G_O.POSITION_TYPE_FLOATCONV,
#             5: Fuse_G_O.POSITION_TYPE_WIDELANE,
#             6: Fuse_G_O.POSITION_TYPE_NARROWLANE,
#             8: Fuse_G_O.POSITION_TYPE_DOPPLER_VELOCITY,
#             16: Fuse_G_O.POSITION_TYPE_SINGLE,
#             17: Fuse_G_O.POSITION_TYPE_PSRDIFF,
#             18: Fuse_G_O.POSITION_TYPE_WAAS,
#             19: Fuse_G_O.POSITION_TYPE_PROPAGATED,
#             20: Fuse_G_O.POSITION_TYPE_OMNISTAR,
#             32: Fuse_G_O.POSITION_TYPE_L1_FLOAT,
#             33: Fuse_G_O.POSITION_TYPE_IONOFREE_FLOAT,
#             34: Fuse_G_O.POSITION_TYPE_NARROW_FLOAT,
#             48: Fuse_G_O.POSITION_TYPE_L1_INT,
#             49: Fuse_G_O.POSITION_TYPE_WIDE_INT,
#             50: Fuse_G_O.POSITION_TYPE_NARROW_INT,
#             51: Fuse_G_O.POSITION_TYPE_RTK_DIRECT_INS,
#             52: Fuse_G_O.POSITION_TYPE_INS_SBAS,
#             53: Fuse_G_O.POSITION_TYPE_INS_PSRSP,
#             54: Fuse_G_O.POSITION_TYPE_INS_PSRDIFF,
#             55: Fuse_G_O.POSITION_TYPE_INS_RTKFLOAT,
#             56: Fuse_G_O.POSITION_TYPE_INS_RTKFIXED,
#             57: Fuse_G_O.POSITION_TYPE_INS_OMNISTAR,
#             58: Fuse_G_O.POSITION_TYPE_INS_OMNISTAR_HP,
#             59: Fuse_G_O.POSITION_TYPE_INS_OMNISTAR_XP,
#             64: Fuse_G_O.POSITION_TYPE_OMNISTAR_HP,
#             65: Fuse_G_O.POSITION_TYPE_OMNISTAR_XP,
#             68: Fuse_G_O.POSITION_TYPE_PPP_CONVERGING,
#             69: Fuse_G_O.POSITION_TYPE_PPP,
#             70: Fuse_G_O.POSITION_TYPE_OPERATIONAL,
#             71: Fuse_G_O.POSITION_TYPE_WARNING,
#             72: Fuse_G_O.POSITION_TYPE_OUT_OF_BOUNDS,
#             73: Fuse_G_O.POSITION_TYPE_INS_PPP_CONVERGING,
#             74: Fuse_G_O.POSITION_TYPE_INS_PPP,
#         }


class Fuse_GPS_Odom:
    """class Fuse_GPS_Odom 用来存储GPS和Encoder的信息

    Attributes:
        fuse_gps:   GPS类存储值
        fuse_encoder:   Encoder类存储值
        fuse_gps_odom_msg:  本类融合两种存储值
    """

    def __init__(self):
        """构造函数"""
        self.fuse_gps = GPS()
        self.fuse_encoder = Encoder()
        self.fuse_gps_odom_msg = Fuse_G_O()
        self.fill()

    def fill(self):
        """将GPS和Encoder的信息添加到类中"""
        self.fuse_gps_odom_msg.Slo_stat = self.fuse_gps.Slo_stat
        self.fuse_gps_odom_msg.Pos_type = self.fuse_gps.Pos_type
        self.fuse_gps_odom_msg.lat = self.fuse_gps.lat
        self.fuse_gps_odom_msg.lat_std = self.fuse_gps.lat_std
        self.fuse_gps_odom_msg.lon = self.fuse_gps.lon
        self.fuse_gps_odom_msg.lon_std = self.fuse_gps.lon_std
        self.fuse_gps_odom_msg.hgt = self.fuse_gps.hgt
        self.fuse_gps_odom_msg.hgt_std = self.fuse_gps.hgt_std
        self.fuse_gps_odom_msg.SVs = self.fuse_gps.SVs
        self.fuse_gps_odom_msg.solnSVs = self.fuse_gps.solnSVs
        self.fuse_gps_odom_msg.heading = self.fuse_gps.heading
        self.fuse_gps_odom_msg.hdg_std = self.fuse_gps.hdg_std
        self.fuse_gps_odom_msg.hor_spd = self.fuse_gps.hor_spd
        self.fuse_gps_odom_msg.Trk_gnd = self.fuse_gps.Trk_gnd
        self.fuse_gps_odom_msg.Vert_spd = self.fuse_gps.Vert_spd
        self.fuse_gps_odom_msg.gps_time = self.fuse_gps.gps_time

        self.fuse_gps_odom_msg.left_encoder_val = self.fuse_encoder.left_encoder_val
        self.fuse_gps_odom_msg.right_encoder_val = self.fuse_encoder.right_encoder_val
        self.fuse_gps_odom_msg.encoder_time = self.fuse_encoder.encoder_time


class GPS:
    """存储GPS信息的类

    Attributes:
        Slo_stat:   卫星状态
        Pos_type:   定位类型
        lat:        维度
        lat_std:    纬度标准差
        lon:        经度
        lon_std:    经度标准差
        hgt:        高度
        hgt_std:    高度标准差
        SVs:        卫星数
        solnSVs:    参与解算卫星数
        heading:    航向
        hdg_std:    航向标准差
        hor_spd:    #
        Trk_gnd:    #
        Vert_spd:   #
        gps_time:   gps校准时间
    """

    def __init__(self):
        """构造函数"""
        self.Slo_stat = 0
        self.Pos_type = 0
        self.lat = 0
        self.lat_std = 0
        self.lon = 0
        self.lon_std = 0
        self.hgt = 0
        self.hgt_std = 0
        self.SVs = 0
        self.solnSVs = 0
        self.heading = 0
        self.hdg_std = 0
        self.hor_spd = 0
        self.Trk_gnd = 0
        self.Vert_spd = 0
        self.gps_time = 0

    def plot(self):
        """输出"""
        print "Slo_stat" + str(self.Slo_stat)
        print "Pos_type" + str(self.Pos_type)
        print "lat" + str(self.lat)
        print "lat_std"+str(self.lat_std)
        print "lon"+str(self.lon)
        print "lon_std"+str(self.lon_std)
        print "hgt"+str(self.hgt)
        print "hgt_std"+str(self.hgt_std)
        print "SVs"+str(self.SVs)
        print "solnSVs"+str(self.solnSVs)
        print "heading"+str(self.heading)
        print "hdg_std"+str(self.hdg_std)
        print "hor_spd"+str(self.hor_spd)
        print "Trk_gnd"+str(self.Trk_gnd)
        print "Vert_spd"+str(self.Vert_spd)
        print "gps_time"+str(self.gps_time)


class Encoder:
    """类用来存储里程计信息

    Attributes:
        left_encoder_val:   左里程计值
        right_encoder_val:  右里程计值
        encoder_time:       同步里程计时间
    """

    def __init__(self):
        """构造函数"""
        self.left_encoder_val = 0
        self.right_encoder_val = 0
        self.encoder_time = 0

    def plot(self):
        """输出"""
        print "left_encoder_val", str(self.left_encoder_val)
        print "right_encoder_val", str(self.right_encoder_val)
        print "encoder_time", str(self.encoder_time)



### 融合数据输出类
fuse_gps_odom = Fuse_GPS_Odom()


### 超声波模拟激光输出类
sound_range = LaserScan()

### 超声波输出计数
sound_seq = 0  # seq


def sound_range_init():
    """超声波数据初始化

    See Also:
        range_max:  超声波最远距离
        range_min:  超声笔最小距离(本来是0.290),range值小于等于最小值,输出无效,固最小值缩小
        scan_time:  超声波扫描时间
        time_increment: 超声波模拟激光模拟数据
        angle_increment:    超声波模拟激光模拟数据
        angle_min:  超声波的探测角(±30°)
        angle_max:  超声波的探测角(±30°)
    """
    sound_range.range_max = 3.50
    sound_range.range_min = 0.280
    sound_range.scan_time = 0.4
    sound_range.time_increment = 0.001
    sound_range.angle_increment = 0.005
    sound_range.angle_min = - 30 * math.pi / 180
    sound_range.angle_max = 30 * math.pi / 180


def range_pub(range_msg):
    """
    range_pub range的激光输出函数
    
    将list格式的range数据转发成ros_laser模式,并通过/range/* topic发出

    Args:
        range_msg: range的输入[1,...,3](list)

    Examples:
        >>> a = float("inf")
        inf
        >>> b = float("inf")
        inf
        >>> a + b
        inf
        >>> a * b
        inf
        >>> a + 12
        inf
        >>> a - 12
        inf
    """
    
    global sound_seq
    sound_range.header.stamp = rospy.Time.now()
    sound_range.header.seq = sound_seq
    sound_range.header.frame_id = 'range5'
    if range_msg[4] == 3500:
        out_msg = float("inf")
    else:
        out_msg = range_msg[4]
    #print range_msg,out_msg
    sound_range.ranges = size_ * [out_msg / 1000.0]
    pub_range_5.publish(sound_range)
    sound_range.header.frame_id = 'range6'
    if range_msg[5] == 3500:
        out_msg = float("inf")
    else:
        out_msg = range_msg[5]
    sound_range.ranges = size_ * [out_msg / 1000.0]
    pub_range_6.publish(sound_range)
    sound_range.header.frame_id = 'range7'
    if range_msg[6] == 3500:
        out_msg = float("inf")
    else:
        out_msg = range_msg[6]
    sound_range.ranges = size_ * [out_msg / 1000.0]
    pub_range_7.publish(sound_range)
    sound_range.header.frame_id = 'range8'
    if range_msg[7] == 3500:
        out_msg = float("inf")
    else:
        out_msg = range_msg[7]
    sound_range.ranges = size_ * [out_msg / 1000.0]
    pub_range_8.publish(sound_range)
    sound_seq += 1


def serial_process(port='/dev/pts/27', baudrate=460800, timeout=1):
    """
    serial_process 串口线程
    
    处理串口中的三包数据,GPS,里程计和超声波,GPS和里程计分别存储在GPS类和Encoder类中,在里程计接收过程中讲两个类的数据存储在融合类Fuse_GPS_Odom中.
    
    Args:
        port: 串口
        baudrate: 波特率
        timeout: 时间容差

    See Also:
        ser_1 :     读取的第一个串口值
        encoder_a:  串口剩余部分
        encoder_b:  解的串口数据
        crc:        解的串口校验值
        encoder_crc:    计算的串口校验值

    Notes:
        GPS和超声波数据类似里程计读取流程
    """
    serial_ser = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
    serial_ser.flushOutput()
    serial_ser.flushInput()
    encoder = Encoder()
    gps = GPS()
    print "in serial process"

    while not rospy.is_shutdown():
        ser_1 = serial_ser.read(1)
        if ser_1 == '\x86':
            if serial_ser.read(1) == '\x0a':
                encoder_a = serial_ser.read(9)
                # print '\x86\x0a' + encoder_a
                if len(encoder_a) == 9:
                    encoder_b = struct.unpack('<2hIB', encoder_a)
                    crc = encoder_b[3]
                    crc_split = struct.unpack("<9B", '\x0a' + encoder_a[0:-1])
                    encoder_crc = sum(crc_split) & 0xff
                    if encoder_crc == crc:
                        try:
                            encoder.left_encoder_val = encoder_b[0]
                            encoder.right_encoder_val = encoder_b[1]
                            encoder.encoder_time = encoder_b[2]
                            fuse_gps_odom.fuse_encoder = encoder
                            # print "encoder_filled"
                            fuse_gps_odom.fill()
                            pub.publish(fuse_gps_odom.fuse_gps_odom_msg)
                            # range_pub(range_b[0:8])
                            # print "pub ok"
                        except:
                            print "encoder serial error"
                    else:
                        continue
                else:
                    continue
            else:
                continue
        elif ser_1 == '\xaa':
            # print 'aa'
            if serial_ser.read(1) == '\x44':
                # print '44'
                if serial_ser.read(1) == '\x12':
                    # print '12'
                    print "gps_get"
                    gps_a = serial_ser.read(86)
                    if len(gps_a) == 86:
                        gps_b = struct.unpack('<2I3d3f2B2f3dI4B', gps_a)
                        crc = (gps_b[19] << 24) | (gps_b[18] << 16) | (gps_b[17] << 8) | (gps_b[16])
                        crc_split = '\xaa' + '\x44' + '\x12' + gps_a[0:82]
                        gps_crc = crc_novetal(crc_split)
                        if gps_crc == crc:
                            try:
                                gps.Slo_stat = gps_b[0]
                                gps.Pos_type = gps_b[1]
                                gps.lat = gps_b[2]
                                gps.lon = gps_b[3]
                                gps.hgt = gps_b[4]
                                gps.lat_std = gps_b[5]
                                gps.lon_std = gps_b[6]
                                gps.hgt_std = gps_b[7]
                                gps.SVs = gps_b[8]
                                gps.solnSVs = gps_b[9]
                                gps.heading = gps_b[10]
                                gps.hdg_std = gps_b[11]
                                gps.hor_spd = gps_b[12]
                                gps.Trk_gnd = gps_b[13]
                                gps.Vert_spd = gps_b[14]
                                gps.gps_time = gps_b[15]
                                fuse_gps_odom.fuse_gps = gps
                                # fuse_gps_odom.fill()
                                # pub.publish(fuse_gps_odom.fuse_gps_odom_msg)
                                # print "gps filled"
                            except:
                                print "gps serial error"
                        else:
                            continue
                    else:
                        continue
                else:
                    continue
            else:
                continue
        elif ser_1 == '\x88':
            if serial_ser.read(1) == '\x13':
                range_a = serial_ser.read(17)
                if len(range_a) == 17:
                    range_b = struct.unpack('<8HB', range_a)
                    crc = range_b[8]
                    crc_split = struct.unpack("<17B", '\x13' + range_a[0:-1])
                    range_crc = sum(crc_split) & 0xff
                    if range_crc == crc:
                        try:
                            # encoder.left_encoder_val = encoder_b[0]
                            # encoder.right_encoder_val = encoder_b[1]
                            # encoder.encoder_time = encoder_b[2]
                            # fuse_gps_odom.fuse_encoder = encoder
                            range_pub(range_b[0:8])
                            # print "range_filled"
                            # fuse_gps_odom.fill()
                            # pub.publish(fuse_gps_odom.fuse_gps_odom_msg)
                            # print "pub ok"
                        except:
                            print "encoder serial error"
                    else:
                        continue
                else:
                    continue
            else:
                continue
        else:
            print "!!!!!!!!!!!!!!!!!!!!"
            continue


def main():
    """
    main函数
    
    ros的初始化 serial线程的开始 spin线程的开始
    """
    try:
        rospy.init_node('serial_fuse', anonymous=True)
        sound_range_init()
        global pub, pub_range_5, pub_range_6, pub_range_7, pub_range_8, size_
        ### 超声波模拟的激光角度list,自动生成
        m_ = np.arange(sound_range.angle_min, sound_range.angle_max, sound_range.angle_increment)
        ### 超声波模拟的激光个数
        size_ = len(m_)
        pub = rospy.Publisher('/fuse', Fuse_G_O, queue_size=100)
        pub_range_5 = rospy.Publisher('/range/5', LaserScan, queue_size=100)
        pub_range_6 = rospy.Publisher('/range/6', LaserScan, queue_size=100)
        pub_range_7 = rospy.Publisher('/range/7', LaserScan, queue_size=100)
        pub_range_8 = rospy.Publisher('/range/8', LaserScan, queue_size=100)

        serial_process()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

## main 主函数
#
#  ros的初始化,线程的初始化和ros回调函数进程的展开
if __name__ == '__main__':
    main()
