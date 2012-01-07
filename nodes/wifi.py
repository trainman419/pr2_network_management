#!/usr/bin/env python

import roslib; roslib.load_manifest('pr2_network_management')

from pr2_network_management.srv import *
import rospy

import wrt610n
import ctr350

def handle_local(req):
   rospy.loginfo('Got local wifi request: %s %s'%(
	'enable' if req.enabled else "disable", req.ssid))
   return WifiResponse()

def handle_client(req):
   rospy.loginfo('Got client wifi request: %s %s %s'%(
	req.ssid, req.security, req.passphrase))
   client_config['wl0_ssid'] = req.ssid
   if req.security != 0:
      client_security['wl0_security_mode'] = 'psk2'
      client_security['wl0_crypto'] = 'aes'
      client_security['wl0_wl0_wpa_psk'] = req.passphrase
      client_security['wl0_wl_unmask'] = 0
      client_security['wl0_wpa_gtk_rekey'] = 3600
   else:
      client_scrurity['wl0_security_mode'] = 'disabled'
      del client_security['wl0_crypto']
      del client_security['wl0_wl0_wpa_psk']
      del client_security['wl0_wl_unmask']
      del client_security['wl0_wpa_gtk_rekey']
   client.apply(client_config)
   client.apply(client_security)
   return WifiResponse()

def handle_local_get(req):
   rospy.loginfo('Local state get')
   return WifiGetResponse(True, "ssid", 0, "")

def handle_client_get(req):
   rospy.loginfo('Client state get')
   sec = security[client_security['wl0_security_mode']]
   passphrase = ""
   if sec != 0:
      passphrase = client_security['wl0_wpa_psk']
   return WifiGetResponse(True, client_config['wl0_ssid'], sec, passphrase)

security = { 'disabled': 0, 'psk': 0, 'wpa': 0, 'psk2': 1, 'wpa2': 0,
   'psk psk2': 0, 'wpa wpa2': 0, 'radius': 0, 'wep': 0 }

if __name__ == "__main__":
   rospy.init_node('wifi_manager')
   client = wrt610n.DDWRT('10.68.0.5', 'root', 'willow')
   if not client.login():
      rospy.logerror('ddwrt login failed')

   client_config = client.get_config('Wireless_Basic.asp')
   client_security = client.get_config('WL_WPATable.asp')
   print client_config
   print client_security

   local = ctr350.ctr350('10.68.0.250', 'willow')
   local_config = local.get_config()
   print local_config
#     print "IP:          ", data['lan_network_address']
#     print "Password:    ", data['password']
#     print "SSID:        ", data['wireless']['SSID']
#     print "WPA enabled: ", data['wireless']['wpa_enabled']
#     print "WPA mode:    ", data['wireless']['wpa_mode']
#     print "WPA key:     ", data['wireless']['wpa_psk']

   local_srv = rospy.Service('pr2_wifi/local', Wifi, handle_local)
   client_srv = rospy.Service('pr2_wifi/client', Wifi, handle_client)
   local_get = rospy.Service('pr2_wifi/local_get', WifiGet, handle_local_get)
   client_get = rospy.Service('pr2_wifi/client_get', WifiGet, handle_client_get)
   rospy.loginfo('wifi manager ready')
   rospy.spin()
