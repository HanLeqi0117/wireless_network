import rospy
import subprocess
from wireless_network.msg import WirelessScan
import os

class RssiNode:
    def __init__(self):
        self.interface_name = rospy.get_param('interface_name', default='wlan0')
        self.frame_id = rospy.get_param('frame_id', default=os.environ['HOSTNAME'])
        self.cmd = ['sudo', 'iwlist', self.interface_name, 'scan']
        self.publisher = rospy.Publisher('wireless_rssi', WirelessScan)
        pub_timer = rospy.Timer(rospy.Duration(1), self.wireless_scan_callback)

    def wireless_scan_callback(self):
        runcmd = subprocess.check_output(self.cmd)
        wireless_scan = WirelessScan()
        wireless_scan.header.frame_id = self.frame_id
        wireless_scan.header.stamp = rospy.Time.now()
        index = 0
        for line in runcmd.decode('ascii').split('\n'):
            if 'Cell' in line:
                elem_list = line.split(' ')
                wireless_scan.networks[index].cellid = elem_list[1]
                wireless_scan.networks[index].mac_address = elem_list[4]
            if 'Channel' in line:
                elem_list = line.split(':')
                wireless_scan.networks[index].channel = elem_list[1]
            if 'ESSID' in line:
                elem_list = line.split(':')
                wireless_scan.networks[index].essid = elem_list[1].replace('"', '')
            if 'Signal level' in line:
                elem_list = line.split('=')
                wireless_scan.networks[index].signal_strength = elem_list[3]

            index += 1

        self.publisher.publish(wireless_scan)

if __name__ == '__main__':
    rospy.init_node('wifi_rssi_node')
    RssiNode()
    rospy.spin()