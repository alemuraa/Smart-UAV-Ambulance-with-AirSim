#!/usr/bin/env python

import rospy


# Class for message logging and debugging
class Logger:
    def __init__(self, node_name):
        self.node_name = node_name

    def loginfo(self, msg):
        rospy.loginfo(f"[{self.node_name}] {msg}")

    def logwarn(self, msg):
        rospy.logwarn(f"[{self.node_name}] {msg}")

    def logerr(self, msg):
        rospy.logerr(f"[{self.node_name}] {msg}")
