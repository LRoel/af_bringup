#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
import threading
import numpy as np
import numpy.linalg as la
import math
import datetime
import tf2_ros

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TransformStamped
from af_msgs.msg import Gnss_Odom_lyz
from af_msgs.msg import Fuse_tf
from af_bringup.msg import Robot_encode
from std_msgs.msg import Bool

from tf import transformations
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
import tf.transformations as tft

### 最大的标准差
MAX_STD = 999999999


def matrix_from_theta(theta):
    """角度生成旋转矩阵(2x2)

    Args:
        theta: 输入角度

    Returns:
        输出旋转矩阵

    """
    
    return np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])


def theta_from_matrix(R):
    """旋转矩阵(2x2)转成角度

        Args:
            R: 输入旋转矩阵

        Returns:
            输出角度值

    """
        
    _R_12 = R[0, 1]
    _R_22 = R[1, 1]
    if -0.001 < _R_22 < 0.001 and _R_12 > 0:
        theta = math.pi / 2
    elif -0.001 < _R_22 < 0.001 and _R_12 < 0:
        theta = math.pi / 2
    elif 0.001 <= _R_22 and _R_12 > 0:
        theta = math.atan(_R_12 / _R_22)
    elif 0.001 <= _R_22 and _R_12 < 0:
        theta = math.atan(_R_12 / _R_22)
    elif _R_22 <= -0.001 and _R_12 > 0:
        theta = math.atan(_R_12 / _R_22) + math.pi
    elif _R_22 <= -0.001 and _R_12 < 0:
        theta = math.atan(_R_12 / _R_22) - math.pi
    return theta


class AMCL_unit:
    """AMCL存储类

    Attributes:
        x:  x
        x_std:  x标准差
        y:  y
        y_std:  y标准差
        theta:  角度
        theta_std:  角度标准差
        _x_base:  累计x
        _y_base:  累计y
        _theta_base:  累计角度


    """

    def __init__(self):
        """构造函数"""
        self.x = 0
        self.x_std = MAX_STD
        self.y = 0
        self.y_std = MAX_STD
        self.theta = 0
        self.theta_std = MAX_STD
        self._theta_base = 0
        self._x_base = 0
        self._y_base = 0

    def update(self, odom_msg):
        """里程计更新

        Args:
            odom_msg:   ros里程计输入

        See Also:
            _pos:   位置矩阵
            _pose_base: 位置基准矩阵
            _pose_update:   坐标系变换
            _quat:  四元数
            flag_robot_move:    机器人运动标志
            
        """
        
        _pos = np.array([[odom_msg.pose.pose.position.x], [odom_msg.pose.pose.position.y]])
        odom_pose.x = _pos[0,0]
        odom_pose.y = _pos[1,0]
        _pos_base = np.array([[self._x_base], [self._y_base]], dtype=float)
        _quat = [odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z,
                 odom_msg.pose.pose.orientation.w]
        _theta = tft.euler_from_quaternion(_quat)[2]
        _pos_updated = np.dot(matrix_from_theta(self._theta_base), _pos) + _pos_base
        odom_pose.theta = _theta
        self.x = _pos_updated[0,0]
        self.y = _pos_updated[1,0]
        self.theta = _theta + self._theta_base
        out_pose.set_odom(odom_pose.x, odom_pose.y, odom_pose.theta)
        # print "updated"
        # self.output()

    def fix(self, odom_map_msg):
        """
        AMCL修正输入

        将AMCL修正输入的map->odom和里程计输入融合生成实时的AMCL输出值(机器人在地图上的激光定位)

        Args:
            odom_map_msg: AMCL的输入的map->odom

        """
        
        # print "fixed"
        _quat = [odom_map_msg.pose.pose.orientation.x, odom_map_msg.pose.pose.orientation.y, odom_map_msg.pose.pose.orientation.z,
                 odom_map_msg.pose.pose.orientation.w]
        self._theta_base = tft.euler_from_quaternion(_quat)[2]
        self._x_base = odom_map_msg.pose.pose.position.x
        self._y_base = odom_map_msg.pose.pose.position.y
        self.x_std = math.sqrt(odom_map_msg.pose.covariance[0])
        self.y_std = math.sqrt(odom_map_msg.pose.covariance[7])
        self.theta_std = math.sqrt(odom_map_msg.pose.covariance[35])

    def output(self):
        """输出"""
        print "x: ", self.x, "| y: ", self.y, "| theta: ", self.theta
        print "x_std: ", self.x_std, "| y_std: ", self.y_std, "| theta_std: ", self.theta_std


class Odom_unit:
    """里程计数据存储类

    Attributes:
        x:  x
        x_std:  x标准差
        y:  y
        y_std:  y标准差
        theta:  角度
        theta_std:  角度标准差
        
    """

    def __init__(self):
        """构造函数"""
        self.x = 0
        self.x_std = MAX_STD
        self.y = 0
        self.y_std = MAX_STD
        self.theta = 0
        self.theta_std = MAX_STD

    def output(self):
        """输出"""
        print "x: ", self.x, "| y: ", self.y, "| theta: ", self.theta
        print "x_std: ", self.x_std, "| y_std: ", self.y_std, "| theta_std: ", self.theta_std


class GPS_unit:
    """GPS数据存储类

    Attributes:
        x:  x
        x_std:  x标准差
        y:  y
        y_std:  y标准差
        theta:  角度
        theta_std:  角度标准差

    """

    def __init__(self):
        """构造函数"""
        self.x = 0
        self.x_std = MAX_STD
        self.y = 0
        self.y_std = MAX_STD
        self.theta = 0
        self.theta_std = MAX_STD

    def update(self, gps_msg):
        """GPS数据更新函数

        Args:
            gps_msg: GPS数据输入

        """
        self.x = gps_msg.X
        self.x_std = gps_msg.X_std
        self.y = gps_msg.Y
        self.y_std = gps_msg.Y_std
        self.theta = gps_msg.Hdg
        self.theta_std = gps_msg.Hdg_std

    def output(self):
        """输出"""
        print "x: ", self.x, "| y: ", self.y, "| theta: ", self.theta
        print "x_std: ", self.x_std, "| y_std: ", self.y_std, "| theta_std: ", self.theta_std


class SELECTED():
    """筛选结果类

    Attributes:
        x:  x
        x_std:  x标准差
        y:  y
        y_std:  y标准差
        theta:  角度
        theta_std:  角度标准差
        choice:
            {0: "GPS", 1: "AMCL", 2: "NONE"}
        fuse_tf:    ros输出格式

    """

    def __init__(self):
        """构造函数"""
        self.choice = {0: "GPS", 1: "AMCL", 2: "NONE"}
        self.x = 0
        self.x_choice = self.choice[2]
        self.y = 0
        self.y_choice = self.choice[2]
        self.theta = 0
        self.theta_choice = self.choice[2]
        self.fuse_tf = Fuse_tf()
        self.fuse_tf.map_x = 0
        self.fuse_tf.map_y = 0
        self.fuse_tf.map_theta = 0
        self.fuse_tf.odom_x = 0
        self.fuse_tf.odom_y = 0
        self.fuse_tf.odom_theta = 0

    def set_map(self, x, y, theta):
        """将选则的最终结果数据放在输出中"""
        self.fuse_tf.map_x = x
        self.fuse_tf.map_y = y
        self.fuse_tf.map_theta = theta

    def set_odom(self,x, y, theta):
        """将里程计数据放在输出中"""
        self.fuse_tf.odom_x = x
        self.fuse_tf.odom_y = y
        self.fuse_tf.odom_theta = theta

    def output(self):
        """输出"""
        print "x: ", self.x, "| y: ", self.y, "| theta: ", self.theta
        print "x_choice: ", self.x_choice, "| y_choice: ", self.y_choice, "| theta_choice: ", self.theta_choice


def sendTransform():
    """发送topic

    40hz发布topic

    Args:
        out_pose: ros输出格式数据

    """
    
    rate = rospy.Rate(40)
    br = tf2_ros.TransformBroadcaster()
    while not rospy.is_shutdown():
        pub.publish(out_pose.fuse_tf)
        rate.sleep()


def tran_mat44(trans):
    """将ros格式中的Transform数据转为矩阵(4X4)

    Args:
        trans: ros格式Transform数据

    """
    
    trans_xyz = [trans.transform.translation.x, trans.transform.translation.y, 0]
    trans_quat = [0, 0, trans.transform.rotation.z,
                  trans.transform.rotation.w]
    mat44 = np.dot(tft.translation_matrix(trans_xyz), tft.quaternion_matrix(trans_quat))
    return mat44


def tran_theta_T(trans):
    """将ros格式中的Transform数据转为矩阵(2X2)

    Args:
        trans: ros格式Transform数据

    """
    
    _quat = [trans.transform.rotation.x, trans.transform.rotation.y,
             trans.transform.rotation.z,
             trans.transform.rotation.w]
    _theta = tft.euler_from_quaternion(_quat)[2]
    _T = np.array([[trans.transform.translation.x], [trans.transform.translation.y]])
    return _theta, _T


def normalization(a, b):
    """归一化

    Args:
        a: 输入
        b: 输入

    Returns:
        a/sum:  a的归一化值
        b/sum:  b的归一化值

    """
    
    sum = a + b
    return a/sum, b/sum


def make_choice():
    """筛选函数"""
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        _x1 = amcl_pose.x
        _x2 = gps_pose.x
        _std_x1 = amcl_pose.x_std
        _std_x2 = gps_pose.x_std
        if _std_x1 == MAX_STD and _std_x2 != MAX_STD:
            _w_x1 = 0
            _w_x2 = 1
        elif _std_x1 != MAX_STD and _std_x2 == MAX_STD:
            _w_x1 = 1
            _w_x2 = 0
        elif _std_x1 == MAX_STD and _std_x2 == MAX_STD:
            continue
        else:
            _w_x1 = _std_x2 / (_std_x1 + _std_x2)
            _w_x2 = _std_x1 / (_std_x1 + _std_x2)
        if _w_x1 > _w_x2:
            out_pose.x_choice = out_pose.choice[1]
        else:
            out_pose.x_choice = out_pose.choice[0]
        _w_x1, _w_x2 = normalization(_w_x1, _w_x2)
        out_pose.x = _w_x1 * _x1 + _w_x2 * _x2
        _y1 = amcl_pose.y
        _y2 = gps_pose.y
        _std_y1 = amcl_pose.y_std
        _std_y2 = gps_pose.y_std
        if _std_y1 == MAX_STD and _std_y2 != MAX_STD:
            _w_y1 = 0
            _w_y2 = 1
        elif _std_y1 != MAX_STD and _std_y2 == MAX_STD:
            _w_y1 = 1
            _w_y2 = 0
        elif _std_y1 == MAX_STD and _std_y2 == MAX_STD:
            continue
        else:
            _w_y1 = _std_y2 / (_std_y1 + _std_y2)
            _w_y2 = _std_y1 / (_std_y1 + _std_y2)
        if _w_y1 > _w_y2:
            out_pose.y_choice = out_pose.choice[1]
        else:
            out_pose.y_choice = out_pose.choice[0]
        _w_y1, _w_y2 = normalization(_w_y1, _w_y2)
        out_pose.y = _w_y1 * _y1 + _w_y2 * _y2
        _theta1 = amcl_pose.theta
        _theta2 = gps_pose.theta
        _std_theta1 = amcl_pose.theta_std
        _std_theta2 = gps_pose.theta_std
        if _std_theta1 == MAX_STD and _std_theta2 != MAX_STD:
            _w_theta1 = 0
            _w_theta2 = 1
        elif _std_theta1 != MAX_STD and _std_theta2 == MAX_STD:
            _w_theta1 = 1
            _w_theta2 = 0
        elif _std_theta1 == MAX_STD and _std_theta2 == MAX_STD:
            continue
        else:
            _w_theta1 = _std_theta2 / (_std_theta1 + _std_theta2)
            _w_theta2 = _std_theta1 / (_std_theta1 + _std_theta2)
        if _w_theta1 > _w_theta2:
            out_pose.theta_choice = out_pose.choice[1]
        else:
            out_pose.theta_choice = out_pose.choice[0]
        _w_theta1, _w_theta2 = normalization(_w_theta1, _w_theta2)
        out_pose.theta = _w_theta1 * _theta1 + _w_theta2 * _theta2
        out_pose.output()
        print "************OUT*************"
        amcl_pose.output()
        print "############AMCL#############"
        gps_pose.output()
        print "############GPS#############"

        out_pose.set_map(out_pose.x, out_pose.y, out_pose.theta)

        rate.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node('combined_test')
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        amcl_pose = AMCL_unit()
        gps_pose = GPS_unit()
        odom_pose = Odom_unit()
        out_pose = SELECTED()
        rospy.Subscriber('/fuse/map_odom', PoseWithCovarianceStamped, amcl_pose.fix)
        rospy.Subscriber('/Gnss_Odom_res', Gnss_Odom_lyz, gps_pose.update)
        rospy.Subscriber('/odom', Odometry, amcl_pose.update)
        pub = rospy.Publisher('/fuse/tf', Fuse_tf, queue_size=100)
        # while rospy.is_shutdown():
        #     amcl_pose.output()
        #     gps_pose.output()

        t2 = threading.Thread(target=sendTransform)
        t1 = threading.Thread(target=make_choice)

        t2.start()
        t1.start()

        t1.join()
        t2.join()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

