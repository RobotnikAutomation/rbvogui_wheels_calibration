#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from rbvogui_wheels_calibration.wheels_calibration import WheelsCalibration


def main():

    rospy.init_node("wheels_calibration_node")

    rc_node = WheelsCalibration()

    rospy.loginfo('%s: starting' % (rospy.get_name()))

    rc_node.start()


if __name__ == "__main__":
    main()
