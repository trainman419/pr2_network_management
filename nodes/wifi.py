#!/usr/bin/env python

import roslib; roslib.load_manifest('pr2_network_management')

from pr2_network_management.srv import *
import rospy

def handle_local(req):
   rospy.loginfo('Got local wifi request')
   return WifiResponse(True)

def handle_client(req):
   rospy.loginfo('Got client wifi request')
   return WifiResponse(True)

def handle_local_get(req):
   rospy.loginfo('Local state get')
   return WifiGetResponse(True)

def handle_client_get(req):
   rospy.loginfo('Client state get')
   return WifiGetResponse(True)

if __name__ == "__main__":
   rospy.init_node('wifi_manager')
   local = rospy.Service('pr2_wifi/local', Wifi, handle_local)
   client = rospy.Service('pr2_wifi/client', Wifi, handle_client)
   local_get = rospy.Service('pr2_wifi/local_get', WifiGet, handle_local_get)
   client_get = rospy.Service('pr2_wifi/client_get', WifiGet, handle_client_get)
   rospy.spin()
