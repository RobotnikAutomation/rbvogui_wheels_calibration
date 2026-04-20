#!/usr/bin/env python3

import os
import shutil
import tempfile

import numpy
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from robotnik_io_msgs.msg import InputsOutputs
from std_msgs.msg import String
from std_srvs.srv import Trigger


class CalibrationNode(Node):
    def __init__(self):
        super().__init__('calibration_node')

        self.values = []
        self.mean = []
        self.median = []
        self.std = []
        self.calibration_duration = 60
        self.save_to_file = False
        self.n = 0
        self.calibrate_once = False
        self.len_unknown = True
        self.init_time = self.get_clock().now()
        self.file_name = 'rbvogui_params.env'
        self.file_path = '/home/robot/robot_settings/robot_params/base'
        self.env_var = 'ROBOT_STEERING_CALIBRATION'
        self.topic_sub = '/robot/robotnik_io_controller/io'
        self.target_found = False
        self.avoid_repeated_data = True
        self.use_median = True

        self.status = String(data='Idle')

        self.getParams()
        self.file_exists = self.checkFile()

        self.subscriber = self.create_subscription(
            InputsOutputs, self.topic_sub, self.ioCallback, 10)
        self.service = self.create_service(
            Trigger, 'calibrate_wheels', self.calibrationSrv)
        self.status_pub = self.create_publisher(String, '~/status', 10)
        self.init_time = self.get_clock().now()
        self.start_calibration = self.calibrate_once

        self.timer = self.create_timer(0.5, self.loop_callback)

    def loop_callback(self):
        if self.start_calibration:
            self.calibrationProcess()
        else:
            self.status.data = 'Idle'
        self.rosPublish()

    def calibrationProcess(self):
        if not self.file_exists:
            msg = 'The file %s does not exist!' % (
                self.file_path + '/' + self.file_name)
            self.get_logger().error(msg)
            self.status.data = msg
            if self.calibrate_once:
                self.shutdown()
            else:
                self.start_calibration = False
            return

        elapsed = self.get_clock().now() - self.init_time
        if not (elapsed > Duration(seconds=self.calibration_duration)):
            msg = 'Calibrating for %.2f of %d seconds' % (
                elapsed.nanoseconds / 1e9, self.calibration_duration)
            self.status.data = msg
            self.get_logger().info(
                'calibrationProcess: %s' % msg,
                throttle_duration_sec=10.0)
            return

        self.status = String(data='Computing results')

        self.computeResult(self.values)

        if self.mean == []:
            self.get_logger().error(
                'The mean does not have any values. '
                'Check that the topic used is running or is the correct one')
            if self.calibrate_once:
                self.shutdown()
            else:
                self.start_calibration = False
            return

        self.get_logger().info('Mean: %s' % str(self.mean))
        self.get_logger().info('Median: %s' % str(self.median))
        self.get_logger().info('Std: %s' % str(self.std))

        if self.save_to_file:
            method = 'median' if self.use_median else 'mean'
            self.storeValues(self.median if self.use_median else self.mean)
            self.get_logger().info(
                'All values obtained. Using the ' + method +
                ' for writing in file ' + self.file_name)
        else:
            self.get_logger().info(
                'All values obtained. Not being saved into file')

        if self.calibrate_once:
            self.shutdown()
        else:
            self.start_calibration = False

    def shutdown(self):
        self.get_logger().info('Shutting down calibration_node')
        raise SystemExit(0)

    def getParams(self):
        self.declare_parameter('file_name', self.file_name)
        self.declare_parameter('file_path', self.file_path)
        self.declare_parameter('env_var', self.env_var)
        self.declare_parameter('topic_sub', self.topic_sub)
        self.declare_parameter('calibration_duration', self.calibration_duration)
        self.declare_parameter('calibrate_once', self.calibrate_once)
        self.declare_parameter('save_to_file', self.save_to_file)
        self.declare_parameter('avoid_repeated_data', self.avoid_repeated_data)
        self.declare_parameter('use_median', self.use_median)

        self.file_name = self.get_parameter(
            'file_name').get_parameter_value().string_value
        self.file_path = self.get_parameter(
            'file_path').get_parameter_value().string_value
        self.env_var = self.get_parameter(
            'env_var').get_parameter_value().string_value
        self.topic_sub = self.get_parameter(
            'topic_sub').get_parameter_value().string_value
        self.calibration_duration = self.get_parameter(
            'calibration_duration').get_parameter_value().integer_value
        self.calibrate_once = self.get_parameter(
            'calibrate_once').get_parameter_value().bool_value
        self.save_to_file = self.get_parameter(
            'save_to_file').get_parameter_value().bool_value
        self.avoid_repeated_data = self.get_parameter(
            'avoid_repeated_data').get_parameter_value().bool_value
        self.use_median = self.get_parameter(
            'use_median').get_parameter_value().bool_value

    def checkFile(self):
        full_path = os.path.join(self.file_path, self.file_name)
        if not os.path.isfile(full_path):
            self.get_logger().error(
                'File not found. Either file path: "' + self.file_path +
                '" or file name: "' + self.file_name + '" are wrong')
            return False
        return True

    def ioCallback(self, data):
        if self.len_unknown:
            self.n = len(data.analog_inputs) - 4
            self.len_unknown = False

        if self.start_calibration:
            self.values.append(list(data.analog_inputs))

    def calibrationSrv(self, request, response):
        if self.start_calibration:
            response.success = False
            response.message = 'Already calibrating'
            return response

        self.start_calibration = True
        self.values = []
        self.mean = []
        self.median = []
        self.std = []
        response.success = True
        response.message = 'Starting wheel calibration'
        self.init_time = self.get_clock().now()
        self.get_logger().info(
            'Starting calibration for %f seconds' % self.calibration_duration)
        return response

    def computeResult(self, data):
        for i in range(self.n):
            values_array = []
            previous_data = -99999.0
            for j in range(len(data)):
                current_data = data[j][i + 4].value
                if self.avoid_repeated_data:
                    if previous_data != current_data:
                        values_array.append(current_data)
                        previous_data = current_data
                else:
                    values_array.append(current_data)
            self.mean.append(numpy.average(values_array))
            self.median.append(numpy.median(values_array))
            self.std.append(numpy.std(values_array))
            self.get_logger().info(
                'computeResult: [%d] mean = %f, median = %f, std = %f '
                '(based on %d measurements)' %
                (i, self.mean[i], self.median[i], self.std[i], len(data)))

    def createEnvVariableLine(self, data):
        line = 'export ' + self.env_var + '="'
        for i in range(self.n):
            if i < self.n - 1:
                line = line + str(data[i]) + ';'
            else:
                line = line + str(data[i]) + '"'
        return line

    def storeValues(self, data):
        source_path = os.path.join(self.file_path, self.file_name)
        target_line = 'export ' + self.env_var
        temp_path = None

        try:
            with tempfile.NamedTemporaryFile(
                mode='w',
                encoding='utf-8',
                prefix='wheel_calibration_',
                suffix='.tmp',
                dir=self.file_path,
                delete=False,
            ) as temp_file:
                temp_path = temp_file.name

            shutil.copy2(source_path, temp_path)

            with open(temp_path, 'r', encoding='utf-8') as file_handle:
                lines = file_handle.readlines()

            updated_lines = []
            self.target_found = False
            new_line = self.createEnvVariableLine(data) + '\n'
            for line in lines:
                if line.startswith(target_line):
                    updated_lines.append(new_line)
                    self.target_found = True
                else:
                    updated_lines.append(line)

            if not self.target_found:
                self.get_logger().error(
                    'Environment variable ' + self.env_var +
                    ' not found in file ' + self.file_name)
                return

            with open(temp_path, 'w', encoding='utf-8') as file_handle:
                file_handle.writelines(updated_lines)

            with open(temp_path, 'r', encoding='utf-8') as file_handle:
                content = file_handle.read()
            if target_line not in content:
                self.get_logger().error(
                    'Validation failed while editing temporary file ' + self.file_name)
                return

            # Atomic replacement prevents partially written original files.
            os.replace(temp_path, source_path)

            self.get_logger().info(
                'File ' + self.file_name + ' modified. New values for ' +
                self.env_var + ': ' + str(data))
            self.get_logger().warn(
                'Values should be around 2.5. If not, check that '
                'the wheels are straightforward')
        except Exception as exc:
            self.get_logger().error(
                'Failed to safely update file ' + self.file_name + ': ' + str(exc))
        finally:
            self.target_found = False
            if temp_path and os.path.exists(temp_path):
                os.remove(temp_path)

    def rosPublish(self):
        self.status_pub.publish(self.status)


def main(args=None):
    rclpy.init(args=args)
    calibration_node = CalibrationNode()
    try:
        rclpy.spin(calibration_node)
    except SystemExit:
        pass
    calibration_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
