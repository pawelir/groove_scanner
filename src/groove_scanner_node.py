#!/usr/bin/env python3

from groove_scanner import GrooveScanner
from numpy.core.numeric import Inf
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32MultiArray
import rospy

class GrooveScannerNode:
    def __init__(self):
        self.laser_sub = rospy.Subscriber('/rotrac_e2/laser/scan',
                                         LaserScan,
                                         self._update_laser_readings)
        self.grooves_indexes_pub = rospy.Publisher("/groove_scanner/grooves_indexes",
                                                   Int32MultiArray,
                                                   queue_size=1)
        self._rate = rospy.Rate(25)
        self._load_params()
        self._laser_readings = []
        self._groove_scanner = GrooveScanner(groove_depth=self._groove_depth)

        while self.laser_sub.get_num_connections() < 1 or not self._laser_readings:
            rospy.logwarn("Waiting for topic /rotrac_e2/laser/scan")
            rospy.sleep(0.2)
        rospy.loginfo('[groove_scanner] Node initialized!')
        
        while not rospy.is_shutdown():
            self._publish_grooves_indexes()
            self._rate.sleep()

    def _update_laser_readings(self, msg: LaserScan) -> None:
        self._laser_readings = [i for i in msg.ranges[self._lidar_rays_limit_left:self._lidar_rays_limit_right] if i != Inf]

    def _load_params(self) -> None:
        self._groove_depth = rospy.get_param("/groove_scanner/groove_depth")
        self._lidar_rays_limit_left = rospy.get_param("/groove_scanner/lidar_rays_limit_left")
        self._lidar_rays_limit_right = rospy.get_param("/groove_scanner/lidar_rays_limit_right")

    def _publish_grooves_indexes(self) -> None:
        self._groove_scanner.search_for_grooves(self._laser_readings)
        self._groove_scanner.filter_multiple_matches()
        groove_detection_msg = Int32MultiArray()
        groove_detection_msg.data = [self._groove_scanner.get_first_groove_index, self._groove_scanner.get_second_groove_index]
        self.grooves_indexes_pub.publish(groove_detection_msg)

if __name__=="__main__":
    rospy.init_node('groove_scanner')
    try:
        groove_scanner = GrooveScannerNode()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()