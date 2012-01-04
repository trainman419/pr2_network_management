#!/usr/bin/env python

import roslib; roslib.load_manifest('pr2_network_management')

from pr2_network_management.srv import Wifi
import rospy

def handle_local(req):
   rospy.log('Got local wifi request')
   return WifiResponse(True)

def handle_client(req):
   rospy.log('Got client wifi request')
   return WifiResponse(True)

if __name__ == "__main__":
   rospy.init_node('wifi_manager')
   local = rospy.Service('pr2_wifi/local', Wifi, handle_local)
   client = rospy.Serivce('pr2_wifi/client', Wifi, handle_client)
   rospy.spin()
