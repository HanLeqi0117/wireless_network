#! /usr/bin/env python

import rospy
from roslib.packages import get_pkg_dir
import subprocess
from wireless_network.msg import WirelessScan
from wireless_network.msg import WirelessNetwork
import os

class RssiNode:
    def __init__(self):
        rospy.init_node('wifi_rssi_node')
        self.interface_name = rospy.get_param('~interface_name', default="wlan0")
        self.frame_id = rospy.get_param('~frame_id', default="pc")
        self.rate = rospy.get_param('~rate', default=0.5)
        self.password = rospy.get_param('~usr_password', default='Robo1234')
        self.cmdlist = ['echo', self.password, '|', 'sudo', '-S', 'iw', self.interface_name, 'scan', '|', 'egrep', self.interface_name + "|ago|signal|primary channel|SSID"]
        self.publisher = rospy.Publisher('wireless_rssi', WirelessScan, queue_size=10)
        pub_timer = rospy.Timer(rospy.Duration.from_sec(1.0 / self.rate), self.wireless_scan_callback)

    def wireless_scan_callback(self, event):
        cmd = subprocess.list2cmdline(self.cmdlist)
        runcmd = subprocess.getoutput(cmd)
        wireless_scan = WirelessScan()
        wireless_scan.header.frame_id = self.frame_id
        wireless_scan.header.stamp = rospy.Time.now()
        index = 1
        
        for blocks in runcmd.split('BSS'):
            network = WirelessNetwork()
            for line in blocks.split('\n'):
                if self.interface_name in line:
                    # debug
                    # print(elem_list)                    
                    elem_list = line.split(' ')
                    if index < 10:
                        network.cellid.data = '0' + str(index)
                    else:
                        network.cellid.data = str(index)
                    network.mac_address.data = elem_list[1].replace("(on", "")
                
                if 'channel' in line:
                    # debug
                    # print(elem_list)
                    elem_list = line.split(' ')
                    network.channel.data = elem_list.pop()
                
                if 'SSID' in line:
                    elem_list = line.split(' ')
                    # debug
                    # print(elem_list)
                    network.essid.data = elem_list.pop()

                if 'signal' in line:
                    elem_list = line.split(' ')
                    # debug
                    # print(elem_list)
                    network.signal_strength.data = elem_list.pop()
                
                if 'last seen' in line:
                    # debug
                    # print(elem_list)                    
                    elem_list = line.split(' ')
                    network.age.data = elem_list[elem_list.index("ms") - 1]

            # debug
            # print(network)
            # print(type(elem_list[0]))
            wireless_scan.networks.append(network)
            index += 1

        self.publisher.publish(wireless_scan)

if __name__ == '__main__':
    RssiNode()
    rospy.spin()
