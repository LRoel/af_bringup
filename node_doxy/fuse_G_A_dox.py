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


## @brief 角度生成旋转矩阵(2x2)
#
#
# @param		theta	输入角度
#
# @return
#        输出旋转矩阵
#
#
def matrix_from_theta(theta):

    return np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])


## @brief 旋转矩阵(2x2)转成角度
#
#
# @param		R	输入旋转矩阵
#
# @return
#            输出角度值
#
#
def theta_from_matrix(R):

    ## @var _R_12
    # @hideinitializer
    # @protected
    _R_12 = R[0, 1]
    ## @var _R_22
    # @hideinitializer
    # @protected
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


## @brief AMCL存储类
#
#
#
#
#
class AMCL_unit:

    ## @property		x
    # x

    ## @property		x_std
    # x标准差

    ## @property		y
    # y

    ## @property		y_std
    # y标准差

    ## @property		theta
    # 角度

    ## @property		theta_std
    # 角度标准差

    ## @property		_x_base
    # 累计x

    ## @property		_y_base
    # 累计y

    ## @property		_theta_base
    # 累计角度

    ## @brief 构造函数
    def __init__(self):
        self.x = 0
        self.x_std = MAX_STD
        self.y = 0
        self.y_std = MAX_STD
        self.theta = 0
        self.theta_std = MAX_STD
        self._theta_base = 0
        self._x_base = 0
        self._y_base = 0

    ## @brief 里程计更新
    #
    #
    # @param		odom_msg	ros里程计输入
    #
    #
    # @sa		_pos	位置矩阵
    # @sa		_pose_base	位置基准矩阵
    # @sa		_pose_update	坐标系变换
    # @sa		_quat	四元数
    # @sa		flag_robot_move	机器人运动标志
    #
    #
    def update(self, odom_msg):

        ## @var _pos
        # @hideinitializer
        # @protected
        _pos = np.array([[odom_msg.pose.pose.position.x], [odom_msg.pose.pose.position.y]])
        odom_pose.x = _pos[0,0]
        odom_pose.y = _pos[1,0]
        ## @var _pos_base
        # @hideinitializer
        # @protected
        _pos_base = np.array([[self._x_base], [self._y_base]], dtype=float)
        ## @var _quat
        # @hideinitializer
        # @protected
        _quat = [odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z,
                 odom_msg.pose.pose.orientation.w]
        ## @var _theta
        # @hideinitializer
        # @protected
        _theta = tft.euler_from_quaternion(_quat)[2]
        ## @var _pos_updated
        # @hideinitializer
        # @protected
        _pos_updated = np.dot(matrix_from_theta(self._theta_base), _pos) + _pos_base
        odom_pose.theta = _theta
        self.x = _pos_updated[0,0]
        self.y = _pos_updated[1,0]
        self.theta = _theta + self._theta_base
        out_pose.set_odom(odom_pose.x, odom_pose.y, odom_pose.theta)
        # print "updated"
        # self.output()

    ## @brief         AMCL修正输入
    #
    #        将AMCL修正输入的map->odom和里程计输入融合生成实时的AMCL输出值(机器人在地图上的激光定位)
    #
    #
    # @param		odom_map_msg	AMCL的输入的map->odom
    #
    #

    def fix(self, odom_map_msg):

        # print "fixed"
        ## @var _quat
        # @hideinitializer
        # @protected
        _quat = [odom_map_msg.pose.pose.orientation.x, odom_map_msg.pose.pose.orientation.y, odom_map_msg.pose.pose.orientation.z,
                 odom_map_msg.pose.pose.orientation.w]
        self._theta_base = tft.euler_from_quaternion(_quat)[2]
        self._x_base = odom_map_msg.pose.pose.position.x
        self._y_base = odom_map_msg.pose.pose.position.y
        self.x_std = math.sqrt(odom_map_msg.pose.covariance[0])
        self.y_std = math.sqrt(odom_map_msg.pose.covariance[7])
        self.theta_std = math.sqrt(odom_map_msg.pose.covariance[35])

    ## @brief 输出
    def output(self):
        print "x: ", self.x, "| y: ", self.y, "| theta: ", self.theta
        print "x_std: ", self.x_std, "| y_std: ", self.y_std, "| theta_std: ", self.theta_std


## @brief 里程计数据存储类
#
#
#
#
class Odom_unit:

    ## @property		x
    # x

    ## @property		x_std
    # x标准差

    ## @property		y
    # y

    ## @property		y_std
    # y标准差

    ## @property		theta
    # 角度

    ## @property		theta_std
    # 角度标准差

    ## @brief 构造函数
    def __init__(self):
        self.x = 0
        self.x_std = MAX_STD
        self.y = 0
        self.y_std = MAX_STD
        self.theta = 0
        self.theta_std = MAX_STD

    ## @brief 输出
    def output(self):
        print "x: ", self.x, "| y: ", self.y, "| theta: ", self.theta
        print "x_std: ", self.x_std, "| y_std: ", self.y_std, "| theta_std: ", self.theta_std


## @brief GPS数据存储类
#
#
#
#
class GPS_unit:

    ## @property		x
    # x

    ## @property		x_std
    # x标准差

    ## @property		y
    # y

    ## @property		y_std
    # y标准差

    ## @property		theta
    # 角度

    ## @property		theta_std
    # 角度标准差

    ## @brief 构造函数
    def __init__(self):
        self.x = 0
        self.x_std = MAX_STD
        self.y = 0
        self.y_std = MAX_STD
        self.theta = 0
        self.theta_std = MAX_STD

    ## @brief GPS数据更新函数
    #
    #
    # @param		gps_msg	GPS数据输入
    #
    #
    def update(self, gps_msg):
        self.x = gps_msg.X
        self.x_std = gps_msg.X_std
        self.y = gps_msg.Y
        self.y_std = gps_msg.Y_std
        self.theta = gps_msg.Hdg
        self.theta_std = gps_msg.Hdg_std

    ## @brief 输出
    def output(self):
        print "x: ", self.x, "| y: ", self.y, "| theta: ", self.theta
        print "x_std: ", self.x_std, "| y_std: ", self.y_std, "| theta_std: ", self.theta_std


## @brief 筛选结果类
#
#
#
#
class SELECTED():

    ## @property		x
    # x

    ## @property		x_std
    # x标准差

    ## @property		y
    # y

    ## @property		y_std
    # y标准差

    ## @property		theta
    # 角度

    ## @property		theta_std
    # 角度标准差

    #        choice:
    # @code

    #            {0: "GPS", 1: "AMCL", 2: "NONE"}

    ## @property		fuse_tf
    # ros输出格式

    ## @brief 构造函数
    def __init__(self):
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

    ## @brief 将选则的最终结果数据放在输出中
    def set_map(self, x, y, theta):
        self.fuse_tf.map_x = x
        self.fuse_tf.map_y = y
        self.fuse_tf.map_theta = theta

    ## @brief 将里程计数据放在输出中
    def set_odom(self,x, y, theta):
        self.fuse_tf.odom_x = x
        self.fuse_tf.odom_y = y
        self.fuse_tf.odom_theta = theta

    ## @brief 输出
    def output(self):
        print "x: ", self.x, "| y: ", self.y, "| theta: ", self.theta
        print "x_choice: ", self.x_choice, "| y_choice: ", self.y_choice, "| theta_choice: ", self.theta_choice


## @brief 发送topic
#
#    40hz发布topic
#
#
# @param		out_pose	ros输出格式数据
#
#
def sendTransform():

    rate = rospy.Rate(40)
    br = tf2_ros.TransformBroadcaster()
    while not rospy.is_shutdown():
        pub.publish(out_pose.fuse_tf)
        rate.sleep()


## @brief 将ros格式中的Transform数据转为矩阵(4X4)
#
#
# @param		trans	ros格式Transform数据
#
#
def tran_mat44(trans):

    trans_xyz = [trans.transform.translation.x, trans.transform.translation.y, 0]
    trans_quat = [0, 0, trans.transform.rotation.z,
                  trans.transform.rotation.w]
    mat44 = np.dot(tft.translation_matrix(trans_xyz), tft.quaternion_matrix(trans_quat))
    return mat44


## @brief 将ros格式中的Transform数据转为矩阵(2X2)
#
#
# @param		trans	ros格式Transform数据
#
#
def tran_theta_T(trans):

    ## @var _quat
    # @hideinitializer
    # @protected
    _quat = [trans.transform.rotation.x, trans.transform.rotation.y,
             trans.transform.rotation.z,
             trans.transform.rotation.w]
    ## @var _theta
    # @hideinitializer
    # @protected
    _theta = tft.euler_from_quaternion(_quat)[2]
    ## @var _T
    # @hideinitializer
    # @protected
    _T = np.array([[trans.transform.translation.x], [trans.transform.translation.y]])
    return _theta, _T


## @brief 归一化
#
#
# @param		a	输入
# @param		b	输入
#
# @return
# @return		a	a的归一化值
# @return		b	b的归一化值
#
#
def normalization(a, b):

    sum = a + b
    return a/sum, b/sum


## @brief 筛选函数
def make_choice():
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        ## @var _x1
        # @hideinitializer
        # @protected
        _x1 = amcl_pose.x
        ## @var _x2
        # @hideinitializer
        # @protected
        _x2 = gps_pose.x
        ## @var _std_x1
        # @hideinitializer
        # @protected
        _std_x1 = amcl_pose.x_std
        ## @var _std_x2
        # @hideinitializer
        # @protected
        _std_x2 = gps_pose.x_std
        if _std_x1 == MAX_STD and _std_x2 != MAX_STD:
            ## @var _w_x1
            # @hideinitializer
            # @protected
            _w_x1 = 0
            ## @var _w_x2
            # @hideinitializer
            # @protected
            _w_x2 = 1
        elif _std_x1 != MAX_STD and _std_x2 == MAX_STD:
            ## @var _w_x1
            # @hideinitializer
            # @protected
            _w_x1 = 1
            ## @var _w_x2
            # @hideinitializer
            # @protected
            _w_x2 = 0
        elif _std_x1 == MAX_STD and _std_x2 == MAX_STD:
            continue
        else:
            ## @var _w_x1
            # @hideinitializer
            # @protected
            _w_x1 = _std_x2 / (_std_x1 + _std_x2)
            ## @var _w_x2
            # @hideinitializer
            # @protected
            _w_x2 = _std_x1 / (_std_x1 + _std_x2)
        if _w_x1 > _w_x2:
            out_pose.x_choice = out_pose.choice[1]
        else:
            out_pose.x_choice = out_pose.choice[0]
        _w_x1, _w_x2 = normalization(_w_x1, _w_x2)
        out_pose.x = _w_x1 * _x1 + _w_x2 * _x2
        ## @var _y1
        # @hideinitializer
        # @protected
        _y1 = amcl_pose.y
        ## @var _y2
        # @hideinitializer
        # @protected
        _y2 = gps_pose.y
        ## @var _std_y1
        # @hideinitializer
        # @protected
        _std_y1 = amcl_pose.y_std
        ## @var _std_y2
        # @hideinitializer
        # @protected
        _std_y2 = gps_pose.y_std
        if _std_y1 == MAX_STD and _std_y2 != MAX_STD:
            ## @var _w_y1
            # @hideinitializer
            # @protected
            _w_y1 = 0
            ## @var _w_y2
            # @hideinitializer
            # @protected
            _w_y2 = 1
        elif _std_y1 != MAX_STD and _std_y2 == MAX_STD:
            ## @var _w_y1
            # @hideinitializer
            # @protected
            _w_y1 = 1
            ## @var _w_y2
            # @hideinitializer
            # @protected
            _w_y2 = 0
        elif _std_y1 == MAX_STD and _std_y2 == MAX_STD:
            continue
        else:
            ## @var _w_y1
            # @hideinitializer
            # @protected
            _w_y1 = _std_y2 / (_std_y1 + _std_y2)
            ## @var _w_y2
            # @hideinitializer
            # @protected
            _w_y2 = _std_y1 / (_std_y1 + _std_y2)
        if _w_y1 > _w_y2:
            out_pose.y_choice = out_pose.choice[1]
        else:
            out_pose.y_choice = out_pose.choice[0]
        _w_y1, _w_y2 = normalization(_w_y1, _w_y2)
        out_pose.y = _w_y1 * _y1 + _w_y2 * _y2
        ## @var _theta1
        # @hideinitializer
        # @protected
        _theta1 = amcl_pose.theta
        ## @var _theta2
        # @hideinitializer
        # @protected
        _theta2 = gps_pose.theta
        ## @var _std_theta1
        # @hideinitializer
        # @protected
        _std_theta1 = amcl_pose.theta_std
        ## @var _std_theta2
        # @hideinitializer
        # @protected
        _std_theta2 = gps_pose.theta_std
        if _std_theta1 == MAX_STD and _std_theta2 != MAX_STD:
            ## @var _w_theta1
            # @hideinitializer
            # @protected
            _w_theta1 = 0
            ## @var _w_theta2
            # @hideinitializer
            # @protected
            _w_theta2 = 1
        elif _std_theta1 != MAX_STD and _std_theta2 == MAX_STD:
            ## @var _w_theta1
            # @hideinitializer
            # @protected
            _w_theta1 = 1
            ## @var _w_theta2
            # @hideinitializer
            # @protected
            _w_theta2 = 0
        elif _std_theta1 == MAX_STD and _std_theta2 == MAX_STD:
            continue
        else:
            ## @var _w_theta1
            # @hideinitializer
            # @protected
            _w_theta1 = _std_theta2 / (_std_theta1 + _std_theta2)
            ## @var _w_theta2
            # @hideinitializer
            # @protected
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

