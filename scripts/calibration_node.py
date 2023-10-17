#!/usr/bin/env python3

import rospy
from robotnik_msgs.msg import inputs_outputs
from std_srvs.srv import Trigger, TriggerResponse, TriggerRequest
import fileinput
import numpy

class CalibrationNode:
    def __init__(self):
        self.values = []
        self.mean = []
        self.median = []
        self.std = []
        self.start_calibration = False
        self.calibration_duration = 60
        self.save_to_file = False
        self.n = 0
        self.calibrate_once = False
        self.len_unknown = True
        self.init_time = rospy.get_rostime()
        self.file_name = 'robot_params.env'
        self.file_path = '/home/robot/robot_params'
        self.env_var = 'ROBOT_JOINT_POTENTIOMETER_VOLTAGE_CALIBRATION'
        self.topic_sub = '/robot/robotnik_base_hw/io'
        self.target_found = False
        self.avoid_repeated_data = True
        self.use_median = True
        
        self.getParams()
        

        self.subscriber = rospy.Subscriber(self.topic_sub, inputs_outputs, self.ioCallback)
        self.service = rospy.Service('calibrate_wheels', Trigger, self.calibrationSrv)

    def __call__(self):
        if self.start_calibration or self.calibrate_once:
            file_exists = self.checkFile()
            if not file_exists:
               rospy.logerr('The file %s does not exist!', self.file_path + '/' +self.file_name)
               return
            if self.calibrate_once:    
            	rospy.logwarn('Starting wheel calibration once. It will takes ' + str(self.calibration_duration) + ' seconds...')
            
            rospy.loginfo('Starting calibration for %f seconds', self.calibration_duration)
            rospy.sleep(self.calibration_duration)
            
            self.computeResult(self.values)
            
            
            if self.mean == []:
                rospy.logerr('The mean does not have any values. Check that the topic used is running or is the correct one')
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
            self.start_calibration = False
    
    def getParams(self):
        self.file_name = rospy.get_param('~file_name', self.file_name)
        self.file_path = rospy.get_param('~file_path', self.file_path)
        self.env_var = rospy.get_param('~env_var', self.env_var)
        self.topic_sub = rospy.get_param('~topic_sub', self.topic_sub)
        self.calibration_duration = rospy.get_param('~calibration_duration', self.calibration_duration)
        self.calibrate_once = rospy.get_param('~calibrate_once', self.calibrate_once)
        self.save_to_file = rospy.get_param('~save_to_file', self.save_to_file)
        self.avoid_repeated_data = rospy.get_param('~avoid_repeated_data', self.avoid_repeated_data)
        self.use_median = rospy.get_param('~use_median', self.use_median)
        
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

        if self.start_calibration or self.calibrate_once:
            rospy.loginfo_throttle(10, 'ioCallback: calibrating for %d seconds'%((rospy.get_rostime() - self.init_time).to_sec()))
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

    
    def createLine(self,data):
        line = 'export ' + self.env_var + '=[1.0,1.0,1.0,1.0,'
        for i in range(self.n):
            if i < self.n-1:
                line = line + str(data[i]) + ','
            else:
                line = line + str(data[i]) + ']'
        return line

    def storeValues(self, data):
        target_line = 'export ' + self.env_var
        for line in fileinput.input(self.file_path + '/' + self.file_name, inplace=True):
            if line.startswith(target_line):
                self.target_found = True
                l = self.createLine(data)
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

def main():
    rospy.init_node('calibration_node')
    calibration_node = CalibrationNode()
    rate = rospy.Rate(10)
    calibration_node()
    while not rospy.is_shutdown() and not calibration_node.calibrate_once:
        calibration_node()
        rate.sleep()

if __name__ == "__main__":
    main()
