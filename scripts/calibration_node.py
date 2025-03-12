#!/usr/bin/env python3

import rospy
from robotnik_msgs.msg import inputs_outputs
from rosmon_msgs.srv import StartStop, StartStopRequest
from std_srvs.srv import Trigger, TriggerResponse, TriggerRequest
from std_msgs.msg import String
import fileinput
import numpy

class CalibrationNode:
    def __init__(self):
        self.values = []
        self.mean = []
        self.median = []
        self.std = []
        self.calibration_duration = 60
        self.save_to_file = False
        self.n = 0
        self.calibrate_once = False
        self.len_unknown = True
        self.init_time = rospy.get_rostime()
        self.file_name = 'robot_params.env'
        self.file_path = '/home/robot/robot_params'
        self.env_var = 'ROBOT_JOINT_POTENTIOMETER_VOLTAGE_CALIBRATION'
        self.ros_param = '/robot/robotnik_base_hw/joint_potentiometer_voltage_calibration'
        self.restart_base_hw = True
        self.nodes_to_restart = ['robotnik_base_hw','controller_spawner']
        self.rosmon_node = 'rosmon_bringup'
        self.topic_sub = '/robot/robotnik_base_hw/io'
        self.namespace = 'robot'
        self.target_found = False
        self.avoid_repeated_data = True
        self.use_median = True

        self.status = String(data = "Idle")
        
        self.getParams()
        self.file_exists = self.checkFile()

        self.subscriber = rospy.Subscriber(self.topic_sub, inputs_outputs, self.ioCallback)
        self.service = rospy.Service('calibrate_wheels', Trigger, self.calibrationSrv)
        self.status_pub = rospy.Publisher("~status", String, queue_size=10)
        self.init_time = rospy.get_rostime()
        self.start_calibration = self.calibrate_once
    
    def calibrationProcess(self):
        if not self.file_exists:
            msg = 'The file %s does not exist!' % (self.file_path + '/' +self.file_name)
            rospy.logerr(msg)
            self.status.data = msg
            if self.calibrate_once:
                self.shutdown()
            else:
                self.start_calibration = False
            return
                
        if not (rospy.get_rostime() - self.init_time > rospy.Duration(self.calibration_duration)):
            msg = 'Calibrating for %.2f of %d seconds' % ((rospy.get_rostime() - self.init_time).to_sec(), self.calibration_duration)
            self.status.data = msg
            rospy.loginfo_throttle(10, 'calibrationProcess: %s', msg)
            return

        self.status = String(data = "Computing results")
        
        self.computeResult(self.values)
        
        if self.mean == []:
            rospy.logerr('The mean does not have any values. Check that the topic used is running or is the correct one')
            if self.calibrate_once:
                self.shutdown()
            else:
                self.start_calibration = False
            return
        
        rospy.loginfo('Mean:\t%s',str(self.mean))
        rospy.loginfo('Median:\t%s',str(self.median))
        rospy.loginfo('Std:\t%s',str(self.std))                        
        if self.save_to_file == True:
            method = ''
            if self.use_median == True:
                method = 'median'
                self.storeValues(self.median)
            else:
                method = 'mean'
                self.storeValues(self.mean)
                
            rospy.loginfo('All values obtained. Using the ' + method + ' for writing in file ' + self.file_name)
        else:
            rospy.loginfo('All values obtained. Not being saved into file')

        if self.restart_base_hw == True:
            if self.use_median:
                self.setRosParam(self.median)
            else:
                self.setRosParam(self.mean)
            for node in self.nodes_to_restart:
                success, message = self.resetRosmonNode(self.rosmon_node, node)
                if not success:
                    rospy.logerr('Unable to restart controller: ' + message)
                    break
        else:
            rospy.loginfo('Not restarting controller)')
        
        if self.calibrate_once:
            self.shutdown()
        else:
            self.start_calibration = False

    def shutdown(self):
        rospy.loginfo('Shutting down calibration_node')
        rospy.signal_shutdown('Shutting down calibration_node')
    
    def getParams(self):
        self.file_name = rospy.get_param('~file_name', self.file_name)
        self.file_path = rospy.get_param('~file_path', self.file_path)
        self.env_var = rospy.get_param('~env_var', self.env_var)
        self.ros_param = rospy.get_param('~ros_param', self.ros_param)
        self.topic_sub = rospy.get_param('~topic_sub', self.topic_sub)
        self.calibration_duration = rospy.get_param('~calibration_duration', self.calibration_duration)
        self.calibrate_once = rospy.get_param('~calibrate_once', self.calibrate_once)
        self.save_to_file = rospy.get_param('~save_to_file', self.save_to_file)
        self.avoid_repeated_data = rospy.get_param('~avoid_repeated_data', self.avoid_repeated_data)
        self.use_median = rospy.get_param('~use_median', self.use_median)
        self.restart_base_hw = rospy.get_param('~restart_base_hw', self.restart_base_hw)
        self.nodes_to_restart = rospy.get_param('~bringup_nodes_to_restart', self.nodes_to_restart)
        self.rosmon_node = rospy.get_param('~rosmon_node', self.rosmon_node)
        self.namespace = rospy.get_param('~robot_namespace', self.namespace)
        
    def checkFile(self):
        try:
            for line in fileinput.input(self.file_path + '/' + self.file_name, inplace=True):
                print(line, end='')
        except:
            rospy.logerr('File not found. Either file path: "' + self.file_path + '" or file name: "' + self.file_name + '" are wrong')
            return False
        return True

    def ioCallback(self, data):
        if self.len_unknown:
            self.n = len(data.analog_inputs) - 4
            self.len_unknown = False

        if self.start_calibration:
            self.values.append(list(data.analog_inputs))
    
    def calibrationSrv(self, data):
        if self.start_calibration == True:
            response = TriggerResponse()
            response.success = False
            response.message = 'Already calibrating'
            return response
            
        self.start_calibration = True
        self.values = []
        self.mean = []
        self.median = []
        self.std = []
        response = TriggerResponse()
        response.success = True
        response.message = 'Starting wheel calibration'
        self.init_time = rospy.get_rostime()
        rospy.loginfo('Starting calibration for %f seconds', self.calibration_duration)
        return response
        

    def computeResult(self, data):
        for i in range(self.n):
            values_array = []
            previous_data = -99999.0
            for j in range(len(data)):
                #acc_sum = acc_sum + data[j][i+4]
                current_data = data[j][i+4]
                if self.avoid_repeated_data == True:
                    if previous_data != current_data:
                        values_array.append(current_data)
                        previous_data = current_data
                else:
                    values_array.append(current_data)
            self.mean.append(numpy.average(values_array))
            self.median.append(numpy.median(values_array))
            self.std.append(numpy.std(values_array))
            #self.mean.append(acc_sum/len(data))
            rospy.loginfo('computeResult: [%d] mean = %f, median = %f, std = %f (based on %d measurements)', i, self.mean[i], self.median[i], self.std[i], len(data))

    
    def createEnvVariableLine(self, data):
        line = 'export ' + self.env_var + '=[1.0,1.0,1.0,1.0,'
        for i in range(self.n):
            if i < self.n-1:
                line = line + str(data[i]) + ','
            else:
                line = line + str(data[i]) + ']'
        return line

    def createRosParam(self, data):
        voltage_list = [1.0,1.0,1.0,1.0]
        for i in range(self.n):
            if i < self.n:
                voltage_list.append(data[i].item())
        return voltage_list
    
    def storeValues(self, data):
        target_line = 'export ' + self.env_var
        for line in fileinput.input(self.file_path + '/' + self.file_name, inplace=True):
            if line.startswith(target_line):
                self.target_found = True
                l = self.createEnvVariableLine(data)
                print(l, end='\n')
            else:
                print(line, end='')
        fileinput.close()
        if not self.target_found:
            rospy.logerr('Environment variable ' + self.env_var + ' not found in file ' + self.file_name)
        else:
            rospy.loginfo('File ' + self.file_name + ' modified. New values for ' + self.env_var + ': ' + str(data))
            rospy.logwarn('Values should be around 2.5. If not, check that the wheels are straightforward')
        self.target_found = False
    
    def setRosParam(self, data):
        ros_param_value = self.createRosParam(data)
        rospy.set_param(self.ros_param, ros_param_value)
    
    def resetRosmonNode(self, rosmon, node):
        try:
            rosmon_service = "/" + rosmon + '/start_stop'
            rospy.wait_for_service(rosmon_service, timeout=5.0)
            start_stop = rospy.ServiceProxy(rosmon_service, StartStop)
            req = StartStopRequest()
            req.action = req.RESTART
            req.node = node
            req.ns = "/" + self.namespace
            start_stop(req)
            rospy.logwarn("Node %s restarted in the rosmon %s" % (node, rosmon))
            return True, "Node restarted"
        except rospy.ServiceException as e:
            rospy.logerr("Service rosmon exception: %s" % e)
            return False, "Service rosmon call exception: %s" % e
        except Exception as e:
            rospy.logerr("Service rosmon exception: %s" % e)
            return False, "Service rosmon exception: %s" % e
        
    def rosPublish(self):
        self.status_pub.publish(self.status)

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.start_calibration:
                self.calibrationProcess()
            else:
                self.status.data = "Idle"
            self.rosPublish()
            rate.sleep()


def main():
    rospy.init_node('calibration_node')
    calibration_node = CalibrationNode()
    calibration_node.run()
    # rate = rospy.Rate(10)
    # calibration_node()
    # while not rospy.is_shutdown() and not calibration_node.calibrate_once:
    #     calibration_node()
    #     rate.sleep()

if __name__ == "__main__":
    main()
