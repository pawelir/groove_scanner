#!/usr/bin/env python3

from time import sleep
from numpy.core.numeric import Inf
from numpy.lib.function_base import append
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32MultiArray
from typing import List
import matplotlib.pyplot as plt
import numpy as np
import rospy
import statistics
'''
Module providing functions to analyze readings from LiDAR and to point out
the position in laser readings which represents the rail groove
'''

class GrooveScanner:
    def __init__(self):
        self.laser_sub = rospy.Subscriber('/rotrac_e2/laser/scan',
                                         LaserScan,
                                         self.update_laser_data)
        self.grooves_indexes_pub = rospy.Publisher("/groove_scanner/grooves_indexes",
                                                   Int32MultiArray,
                                                   queue_size=1)
        self.laser_data = LaserScan()
        self.load_params()
        
        while self.laser_sub.get_num_connections() < 1 or not self.laser_data.ranges:
            rospy.logwarn("Waiting for topic /rotrac_e2/laser/scan")
            rospy.sleep(0.05)
        rospy.loginfo('[groove_scanner] Node initialized!')

        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.publish_grooves_indexes()
            rate.sleep()

    def update_laser_data(self,msg) -> None:
        self.laser_data = msg

    def load_params(self) -> None:
        self.deviation_threshold = rospy.get_param("/groove_scanner/deviation_threshold")
        self.lidar_rays_limit_left = rospy.get_param("/groove_scanner/lidar_rays_limit_left")
        self.lidar_rays_limit_right = rospy.get_param("/groove_scanner/lidar_rays_limit_right")

    def publish_grooves_indexes(self) -> None:
        self.detect_grooves()
        groove_detection_msg = Int32MultiArray()
        groove_detection_msg.data = self.get_grooves_indexes()
        self.grooves_indexes_pub.publish(groove_detection_msg)

    def detect_grooves(self) -> None:
        '''
        This function is responsible for grooves detection. Verifies whether the difference between lidar readings and the approximating function is bigger than the deviation threshold. When it occurs, the index of the groove is appended to the grooves indexes list.
        '''
        
        self.grooves_indexes = []
        laser_readings = [i for i in self.laser_data.ranges[self.lidar_rays_limit_left:self.lidar_rays_limit_right] if i != Inf]        
    
        x_vector = np.array(range(len(laser_readings)))
        laser_readings_array = np.array(laser_readings)
        
        fitting_polynomial_degree = 10
        approx_function_coefficients = np.polyfit(x_vector, laser_readings_array, fitting_polynomial_degree)
        approx_function = np.poly1d(approx_function_coefficients)
        
        self.grooves_indexes = [x for x in x_vector if abs(approx_function(x) - laser_readings_array[x]) >= self.deviation_threshold]

    def get_grooves_indexes(self) -> List[int]:
        '''
        This function points out the approximated index of left and right grooves.
        '''

        temp_list, last_groove_index = [[]], None
        for groove_index in self.grooves_indexes:
            if last_groove_index is None or abs(last_groove_index - groove_index) <= 20:
                temp_list[-1].append(groove_index)
            else:
                temp_list.append([groove_index])
            last_groove_index = groove_index

        first_groove_indexes = temp_list[0]        
        try:
            second_groove_indexes = temp_list[1]
        except:
            second_groove_indexes = []

        first_index = self._calculate_mean_index(first_groove_indexes)
        second_index = self._calculate_mean_index(second_groove_indexes)
        return [first_index, second_index]

    @staticmethod    
    def _calculate_mean_index(groove_indexes: List[int]) -> int:
        if not groove_indexes:
            return 0
        return int(statistics.mean(groove_indexes))


if __name__=="__main__":
    rospy.init_node('groove_scanner')
    try:
        groove_scanner = GrooveScanner()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()