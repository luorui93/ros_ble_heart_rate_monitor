# ROS BLE Heart Rate Monitor
ROS wrapper for BLE supported heart rate monitor devices

Modified based on:
https://github.com/fg1/BLEHeartRateLogger

## How to obtain key information of the device needed to use the driver
1. Make sure the device is turned on and bluetooth is discoverable by computer
2. run `hcitool lescan` to find the device **mac** address (for example, the mac address for Rhythm24 is F4:F4:FB:EC:26:BB)
3. after obtaining the mac address, we can use gatttool to connect to the device by running 
```gatttool -b <mac> -t random --interactive```
4. In the new prompt window, type `connect` to connect with the device
5. Run `char-desc` will return a list after a while, first we need to find the **handle** of uuid starts with '00002a37' (for example ,the handle of Rhythm24 is `0x001f`)
6. Then find the next entry that has uuid starts with '00002902' (**ctl_handle**) (for example, the ctl_handle of Rhythm24 is 0x0020)
7. After obtaining the three important data, we can fill them in the hrv.launch file and run the driver.
