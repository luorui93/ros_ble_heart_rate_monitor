#!/usr/bin/env python

import os 
import rospy
import pexpect
import time
import numpy as np
from heart_rate_monitor.msg import HeartMeasurements

class BLEHeartRateMonitor:
    def __init__(self):
        rospy.init_node("heart_rate_monitor_node", log_level=rospy.DEBUG)
        rospy.loginfo(rospy.get_name()+": Initializaing")
        self.heart_measure_pub = rospy.Publisher('~heart_measurements', HeartMeasurements, queue_size=10)
        self.mac = rospy.get_param('~mac', '')
        self.notify_handle = rospy.get_param('~hr_handle', '')
        self.write_handle = rospy.get_param('~hr_ctl_handle', '')

        # Retrieve parameter from querying if not provided
        if (not self.mac or not self.notify_handle or not self.write_handle):
            rospy.loginfo(rospy.get_name()+": Necessary parameters not provided, retrieving parameters automatically")
            self.__get_parameters()
    
    def __get_parameters(self):
        self.mac = "F4:F4:FB:EC:26:BB"
        self.notify_handle = "0x001f"
        self.write_handle = "0x0020"

    def __publish_measurements(self):
        try:
            rospy.loginfo(rospy.get_name()+": Heart rate published: " + str(self.res["hr"]))
            rospy.loginfo(rospy.get_name()+": RR-Intervals published: " + str(self.res["rr"]))
            hm = HeartMeasurements()
            hm.stamp = rospy.get_rostime()
            hm.heart_rate = self.res["hr"]
            hm.rr_intervals = self.res["rr"]
            self.heart_measure_pub.publish(hm)
        except KeyError as e:
            rospy.logwarn_throttle(10,rospy.get_name()+": heart measurements: {0} is not avaiable".format(e))

    def __interpret(self, data):
        """
        data is a list of integers corresponding to readings from the BLE HR monitor
        """
        byte0 = data[0]
        res = {}
        res["hrv_uint8"] = (byte0 & 1) == 0
        sensor_contact = (byte0 >> 1) & 3
        try:
            if sensor_contact == 2:
                res["sensor_contact"] = "No contact detected"
            elif sensor_contact == 3:
                res["sensor_contact"] = "Contact detected"
            else:
                res["sensor_contact"] = "Sensor contact not supported"
            res["ee_status"] = ((byte0 >> 3) & 1) == 1
            res["rr_interval"] = ((byte0 >> 4) & 1) == 1

            if res["hrv_uint8"]:
                res["hr"] = data[1]
                i = 2
            else:
                res["hr"] = (data[2] << 8) | data[1]
                i = 3

            if res["ee_status"]:
                res["ee"] = (data[i + 1] << 8) | data[i]
                i += 2

            if res["rr_interval"]:
                res["rr"] = []
                while i < len(data):
                    # Note: Need to divide the value by 1024 to get in seconds
                    res["rr"].append(((data[i + 1] << 8) | data[i])/1024.)
                    # Sometimes there would be a burst of data, we take the average of it
                    i += 2
                res["rr"] = [np.mean(res["rr"])]

        except IndexError:
            rospy.logerr("Index error, pass")

        return res


    def run(self):
        #use gatttool to get ble device reading
        gt = pexpect.spawn("gatttool -b " + self.mac +" -t random --char-write-req -a " + self.write_handle + " -n 0100 --listen")
        connect_expect = "Characteristic value was written successfully"
        try:
            gt.expect(connect_expect, timeout=10)
        except pexpect.TIMEOUT:
            rospy.logerr(rospy.get_name()+": Failed sending request to device, please check connection and handle settings")
            exit(0)
        resp_expect = "Notification handle = " + self.notify_handle + " value: ([0-9a-f ]+)"
        retry = True
        while (retry and not rospy.is_shutdown()):
            try:
                rospy.logdebug(rospy.get_name()+": Expecting: "+resp_expect)
                gt.expect(resp_expect, timeout=10)
            except pexpect.TIMEOUT:
                # If the timer expires, it means that we have lost the
                # connection with the HR monitor
                log.warn(rospy.get_name()+": Connection lost with " + self.mac + ". Reconnecting.")
            
            except KeyboardInterrupt:
                # Stop retrying when keyboard interrupt is received
                rospy.loginfo(rospy.get_name()+": Received keyboard interrupt. Quitting cleanly.")
                retry = False
                break

            datahex = gt.match.group(1).strip()
            rospy.loginfo(rospy.get_name()+": rawdata: {0}".format(datahex))
            data = list(map(lambda x: int(x, 16), datahex.split(b' ')))
            self.res = self.__interpret(data)

            rospy.logdebug(rospy.get_name()+": response:{0}".format(self.res))
            # Publish heart rate measurements
            self.__publish_measurements()


if __name__ == "__main__":
    hr_monitor = BLEHeartRateMonitor()
    hr_monitor.run()
    rospy.spin()

