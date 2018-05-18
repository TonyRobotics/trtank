#!/usr/bin/python2
# coding: utf-8

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import tf

import serial
import time
import math

from threading import Lock
lock = Lock()

class TankDriver():

    def __init__(self, serialport, baudrate):
        self.pub = rospy.Publisher('/odom', Odometry, 
                                   queue_size=100) 
        self.sub = rospy.Subscriber('/cmd_vel', Twist, 
                                    self.vel_callback, 
                                    queue_size=1)
        self.serialport = serialport
        self.baudrate = baudrate
        self.ser = serial.Serial(serialport, baudrate)

        self.wheel_diameter = rospy.get_param('~wheel_diameter', default=0.1)
        self.base_width = rospy.get_param('~base_width', default=0.5)
        self.encoder_ticks_per_rev = rospy.get_param('~encoder_ticks_per_rev', default=5000)
        self.linear_coef = rospy.get_param('~linear_coef', default=800.0)
        self.angular_coef = rospy.get_param('~angular_coef', default=200.0)

        self.first_flag = True
        self.encoder1_offset = 0
        self.encoder2_offset = 0
        self.encoder1 = 0
        self.encoder2 = 0
        self.encoder1_prev = 0
        self.encoder2_prev = 0

        self.x = 0
        self.y = 0
        self.theta = 0

        self.odom = Odometry()
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.odom.header.frame_id = 'odom'
        self.odom.child_frame_id = 'base_link'

        self.time_prev = rospy.Time.now()

    def reconnect(self):
        print('{} reconnecting...'.format(time.time()))
        while True:
            try:
                self.ser = serial.Serial(self.serialport, self.baudrate)
            except:
                time.sleep(1)
                print('{} reconnecting...'.format(time.time()))
            else:
                print('{} reconnected!'.format(time.time()))
                break

    def send(self, cmd):
        while True:
            try:
                self.ser.write(cmd)
                break
            except serial.serialutil.SerialException:
                self.reconnect()

    def read_buffer(self):
        time.sleep(0.05)
        res = ''
        try:
            while self.ser.inWaiting() > 0:
                res += self.ser.read(1)
        except:
            self.reconnect()
        res = bytearray(res)
        if res[0:2]=='?C' and res[-1]==13:
            self.encoder1 = int(res.split(':')[0].split('=')[1])
            self.encoder2 = -int(res.split(':')[1][:-1])
            if self.first_flag:
                self.encoder1_offset = self.encoder1
                self.encoder2_offset = self.encoder2
                self.first_flag = False
            self.encoder1 -= self.encoder1_offset
            self.encoder2 -= self.encoder2_offset
            #print('encoder', self.encoder1, self.encoder2)
        return res

    def get_encoder(self):
        cmd = '?C\r'
        self.send(cmd)

    def set_speed(self, v1, v2):
        print('{} Set Speed: {} {}'.format(time.time(), v1, v2))
        cmd = '!M {} {}\r'.format(v1, v2)
        self.send(cmd)

    def update_odom(self):
        encoder1 = self.encoder1
        encoder2 = self.encoder2
        time_current = rospy.Time.now()
        time_elapsed = (time_current - self.time_prev).to_sec()
        self.time_prev = time_current
        dleft = math.pi * self.wheel_diameter * \
                (encoder1 - self.encoder1_prev) / self.encoder_ticks_per_rev
        dright = math.pi * self.wheel_diameter * \
                (encoder2 - self.encoder2_prev) / self.encoder_ticks_per_rev
        self.encoder1_prev = encoder1
        self.encoder2_prev = encoder2
        d = (dleft + dright) / 2
        dtheta = (dright - dleft) / self.base_width
        if d != 0:
            dx = math.cos(dtheta) * d
            dy = math.sin(dtheta) * d
            self.x += dx*math.cos(self.theta)-dy*math.sin(self.theta)
            self.y += dx*math.sin(self.theta)+dy*math.cos(self.theta)
        self.theta += dtheta

        self.odom.header.stamp = time_current
        self.odom.pose.pose.position.x = self.x
        self.odom.pose.pose.position.y = self.y
        q = tf.transformations.quaternion_from_euler(0,0,self.theta)
        self.odom.pose.pose.orientation.x = q[0]
        self.odom.pose.pose.orientation.y = q[1]
        self.odom.pose.pose.orientation.z = q[2]
        self.odom.pose.pose.orientation.w = q[3]
        self.odom.twist.twist.linear.x = d / time_elapsed
        self.odom.twist.twist.angular.z = dtheta / time_elapsed

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # 读编码器值
            lock.acquire()
            self.get_encoder()
            self.read_buffer()
            lock.release()
            # 更新计算并更新里程信息
            self.update_odom()
            # 发布里程信息
            self.pub.publish(self.odom)
            # 发布里程的tf变换
            self.tf_broadcaster.sendTransform(
                (self.x,self.y,0),
                tf.transformations.quaternion_from_euler(0, 0, self.theta),
                rospy.Time.now(),
                'base_link',
                'odom')
            rate.sleep()

    def vel_callback(self,msg):
        lock.acquire()
        v1 = self.linear_coef*msg.linear.x
        v2 = self.linear_coef*msg.linear.x
        v1 += self.angular_coef*msg.angular.z
        v2 -= self.angular_coef*msg.angular.z
        self.set_speed(v1, -v2)
        self.read_buffer()
        lock.release()

if __name__=='__main__':
    rospy.init_node('tank_driver001')
    serialport = rospy.get_param('~serialport', default='/dev/motor_tank')
    baudrate = rospy.get_param('~baudrate', default=115200)
    tank_driver = TankDriver(serialport, baudrate)
    tank_driver.run()

