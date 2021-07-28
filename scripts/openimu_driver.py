#!/usr/bin/env python3

import rospy
import sys
import math
from time import time
from sensor_msgs.msg import Imu, MagneticField

try:
    from ros_openimu.src.aceinna.tools import OpenIMU
except:  # pylint: disable=bare-except
    temp = (sys.path[0])
    temp2 = temp[0:(len(temp)-7)]
    sys.path.append(temp2 + 'src')
    #sys.path.append('./src')
    from aceinna.tools import OpenIMU


class OpenIMUros:
    def __init__(self):
        self.openimudev = OpenIMU()
        self.openimudev.startup()

    def close(self):
        self.openimudev.close()

    '''
    def readimu(self, packet_type):
        readback = self.openimudev.getdata(packet_type)
        return readback
    '''

    def readimu(self):
        readback = self.openimudev.getdata('e2')
        return readback

if __name__ == "__main__":
    rospy.init_node("openimu_driver")

    pub_imu = rospy.Publisher('imu_acc_ar', Imu, queue_size=1)
    pub_mag = rospy.Publisher('imu_mag', MagneticField, queue_size=1)

    imu_msg = Imu()             # IMU data
    mag_msg = MagneticField()   # Magnetometer data
    
    rate = rospy.Rate(200)   # 10Hz
    seq = 0
    frame_id = 'OpenIMU'
    convert_rads = math.pi /180
    convert_tesla = 1/10000

    openimu_wrp = OpenIMUros()
    rospy.loginfo("OpenIMU driver initialized.")

    while not rospy.is_shutdown():
        #read the data - call the get imu measurement data
        readback = openimu_wrp.readimu()
        #publish the data m/s^2 and convert deg/s to rad/s
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = frame_id
        imu_msg.header.seq = seq
        imu_msg.orientation_covariance[0] = -1
        imu_msg.linear_acceleration.x = readback[1]
        imu_msg.linear_acceleration.y = readback[2]
        imu_msg.linear_acceleration.z = readback[3]
        imu_msg.linear_acceleration_covariance[0] = -1
        imu_msg.angular_velocity.x = readback[4] * convert_rads
        imu_msg.angular_velocity.y = readback[5] * convert_rads
        imu_msg.angular_velocity.z = readback[6] * convert_rads
        imu_msg.angular_velocity_covariance[0] = -1

        yaw = readback[9] * convert_rads
        pitch = readback[8] * convert_rads
        roll = readback[7] * convert_rads
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        imu_msg.orientation.w = cr * cp * cy + sr * sp * sy
        imu_msg.orientation.x = sr * cp * cy - cr * sp * sy
        imu_msg.orientation.y = cr * sp * cy + sr * cp * sy
        imu_msg.orientation.z = cr * cp * sy - sr * sp * cy
        pub_imu.publish(imu_msg)

        # Publish magnetometer data - convert Gauss to Tesla
        mag_msg.header.stamp = imu_msg.header.stamp
        mag_msg.header.frame_id = frame_id
        mag_msg.header.seq = seq
        mag_msg.magnetic_field.x = roll # readback[7] # * convert_tesla
        mag_msg.magnetic_field.y = pitch # readback[8] # * convert_tesla
        mag_msg.magnetic_field.z = yaw # readback[9] # * convert_tesla
        mag_msg.magnetic_field_covariance = [0,0,0,0,0,0,0,0,0]
        pub_mag.publish(mag_msg)

        seq = seq + 1
        rate.sleep()
    openimu_wrp.close()         # exit



