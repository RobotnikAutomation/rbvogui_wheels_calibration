#!/usr/bin/env python3

import random

import rclpy
from rclpy.node import Node
from robotnik_io_msgs.msg import AnalogIO, InputsOutputs


class TestIoPublisher(Node):
    def __init__(self):
        super().__init__('test_io_publisher')

        self.topic_pub = '/robot/robotnik_io_controller/io'
        self.publish_rate_hz = 10.0
        self.center_voltage = 2.5
        self.random_delta = 0.05

        self.declare_parameter('topic_pub', self.topic_pub)
        self.declare_parameter('publish_rate_hz', self.publish_rate_hz)
        self.declare_parameter('center_voltage', self.center_voltage)
        self.declare_parameter('random_delta', self.random_delta)

        self.topic_pub = self.get_parameter(
            'topic_pub').get_parameter_value().string_value
        self.publish_rate_hz = self.get_parameter(
            'publish_rate_hz').get_parameter_value().double_value
        self.center_voltage = self.get_parameter(
            'center_voltage').get_parameter_value().double_value
        self.random_delta = self.get_parameter(
            'random_delta').get_parameter_value().double_value

        timer_period = 1.0 / max(self.publish_rate_hz, 0.1)
        self.publisher = self.create_publisher(InputsOutputs, self.topic_pub, 10)
        self.timer = self.create_timer(timer_period, self.publish_callback)

        self.get_logger().info(
            'Publishing 8 analog inputs on %s at %.2f Hz' %
            (self.topic_pub, self.publish_rate_hz))

    def _create_analog_input(self, io_id, value):
        analog_input = AnalogIO()
        analog_input.id = io_id
        analog_input.name = 'ai_%d' % io_id
        analog_input.value = float(value)
        return analog_input

    def publish_callback(self):
        msg = InputsOutputs()

        msg.analog_inputs = [
            self._create_analog_input(0, 0.0),
            self._create_analog_input(1, 0.0),
            self._create_analog_input(2, 0.0),
            self._create_analog_input(3, 0.0),
            self._create_analog_input(
                4, random.uniform(self.center_voltage - self.random_delta,
                                  self.center_voltage + self.random_delta)),
            self._create_analog_input(
                5, random.uniform(self.center_voltage - self.random_delta,
                                  self.center_voltage + self.random_delta)),
            self._create_analog_input(
                6, random.uniform(self.center_voltage - self.random_delta,
                                  self.center_voltage + self.random_delta)),
            self._create_analog_input(
                7, random.uniform(self.center_voltage - self.random_delta,
                                  self.center_voltage + self.random_delta)),
        ]

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    test_io_publisher = TestIoPublisher()
    rclpy.spin(test_io_publisher)
    test_io_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
