#!/usr/bin/python

import rospy
import rospkg 
import yaml
from heart_rate_monitor.msg import HeartMeasurements, HeartRateVariability
import numpy as np
import matplotlib.pyplot as plt

class HRVAnalyser:
    def __init__(self):
        self.heart_measure_sub = rospy.Subscriber('/heart_measurements', HeartMeasurements, self.hr_callback)
        self.hrv_pub = rospy.Publisher('hrv', HeartRateVariability, queue_size=10)
        self.rri_data = []
        self.hr_data = []
        #count how much data has been received
        self.counter = 0
        self.rest_hr = -1
        self.user_name = rospy.get_param('~user_name')
        self.data_history = rospy.get_param('~history_length')
        self.calculate_rest_hr = rospy.get_param('~calculate_rest_hr')
    
    def hr_callback(self, data):
        hr = data.heart_rate
        rri = np.mean(data.rr_intervals)
        assert(len(self.rri_data) == len(self.hr_data))
        if len(self.rri_data) < self.data_history:
            self.rri_data.append(rri)
            self.hr_data.append(hr)
            self.counter = self.counter+1
            rospy.loginfo("Collected {0}/{1} data for calculating rest heart rate...".format(self.counter, self.data_history))
        else:
            if self.rest_hr < 0:
                self.rest_hr = np.mean(np.array(self.hr_data))
                rospy.loginfo("Your rest heart rate is: {0}".format(self.rest_hr))
                #save rest heart rate into user info file
                file_pwd = rospkg.RosPack().get_path('crisp_exp')+'/data/user_info/'+self.user_name+'.yaml' 
                with open(file_pwd) as f:
                    info = yaml.load(f)
                    info['rest_hr'] = int(self.rest_hr)
                
                if (self.calculate_rest_hr):
                    rospy.loginfo('Writing rest heart rate to file...')
                    with open(file_pwd, 'w') as f:
                        yaml.dump(info, f)

            self.rri_data.pop(0)
            self.hr_data.pop(0)
            self.rri_data.append(rri)
            self.hr_data.append(hr)
            rospy.loginfo("Computing HRV...")

            self.hrv_results = self.analyse_hrv()
            hrv_msg = HeartRateVariability()
            hrv_msg.stamp = rospy.get_rostime()
            hrv_msg.heart_rate = hr
            hrv_msg.rmssd = self.hrv_results['rmssd']
            
            self.hrv_pub.publish(hrv_msg)
            
    def analyse_hrv(self):
        results = {}
        #RMSSD in ms
        results['rmssd'] = np.log(1000*np.sqrt(np.mean(np.square(np.diff(self.rri_data)))))

        return results


if __name__ == "__main__":
    rospy.init_node("hrv_analyser")
    ha = HRVAnalyser()
    rospy.spin()
