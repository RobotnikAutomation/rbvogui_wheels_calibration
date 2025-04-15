#!/usr/bin/env python
from robot_simple_command_manager.handlers.command_service_interface import *
from std_srvs.srv import TriggerRequest
from std_msgs.msg import String

class WheelsCalibrationServiceInterface(CommandServiceInterface):

    def __init__(self, name, parameters):
        self.args_names = []
        self.args_description = []
        self.args_types = []
        self.args_void_allowed = []
        self.default_values = []

        self.output_description = []
        self.output_types = []

        self.calibration_error = False
        self.calibration_active = False
        self.calibration_started = False
        self.msg = ""
        

        CommandServiceInterface.__init__(self, name, parameters)
        self.status_sub = rospy.Subscriber(self.topic_name, String, self.statusCb)

    def set_parameters(self, parameters):
        CommandServiceInterface.set_parameters(self, parameters)
        self.topic_name = self.get_parameter("status_topic", "wheel_calibration/status")

    def build_msg(self, args):
        '''
        Assigns the arguments to the specific goal/request message.
        args: list of arguments already typed
        '''
        if type(args) == list:
            # Fill the different fields of the request message
            # using the args list (i.e. self.request.first_argument = args[0])
            self.request = TriggerRequest()
    
    def get_feedback(self):
        return self.msg

    def parse_feedback(self):
        '''
        Parse the feedback_msg and return a string with the feedback of the command.
        This string feedback will be used by the command_manager action server to update
        its status.
        For actions, the feedback_msg equals to the feedback message published by the target server.
        For services, the feedback_msg equals to the response message of the target server. 
        '''
        # COMPLETE WITH YOUR CUSTOMIZATION
        return str(self.feedback_msg.message)

    def parse_output(self):
        '''
        Returns a string following the output format (defined by output_description and output_types)
        separating the different fields with blank spaces.

        self.result_msg object contains the response message of the target server (Action or Server).

        Example:  
            Config:
            self.output_description = ['frame_id', 'tolerence']
            self.output_types = [string, float]

            Output:
            'robot_base_footprint 2.0'
        '''
        # COMPLETE WITH YOUR CUSTOMIZATION
        return ''
    
    def is_active(self):
        return self.calibration_active

    def has_succeeded(self):
        # if self.calibration_started and not self.calibration_active:
        #     self.calibration_started = False
        #     return True
        # return False
        return True

    def statusCb(self, msg):
        self.msg = msg.data
        if "not exists" in msg.data.lower():
            self.calibration_error = True
            self.calibration_active = False
            return

        self.calibration_error = False
        if "idle" in msg.data.lower():
            self.calibration_active = False
        else:
            if not self.calibration_active:
                self.calibration_started = True
            self.calibration_active = True
            
