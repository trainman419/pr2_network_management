#!/usr/bin/env python

import roslib; roslib.load_manifest('pr2_network_management')

from pr2_network_management.srv import *
from pr2_network_management.msg import *
import rospy

import wrt610n
import ctr350

def handle_local(req):
   rospy.loginfo('Got local wifi request: %s %s'%(
	'enable' if req.enabled else "disable", req.ssid))
   if req.enabled:
      local_config['wireless']['radio_control'] = 1
   else:
      local_config['wireless']['radio_control'] = 0
   local_config['wireless']['SSID'] = req.ssid
   if req.security:
      local_config['wireless']['wpa_enabled'] = 1
      local_config['wireless']['wpa_psk'] = req.passphrase
      local_config['wireless']['wpa_mode'] = 3 # 1: WPA, 2: auto, 3: WPA2
      local_config['wireless']['wpa_cipher'] = 2 # 1: TKIP, 2: AES, 3: TKIP+AES
   else:
      local_config['wireless']['wpa_enabled'] = 0
   local.put_config(local_config)
   local_publish()
   return WifiResponse()

def handle_client(req):
   rospy.loginfo('Got client wifi request: %s %s %s'%(
	req.ssid, req.security, req.passphrase))
   client_config['wl0_ssid'] = req.ssid
   if req.security != 0:
      client_security['wl0_security_mode'] = 'psk2'
      client_security['wl0_crypto'] = 'aes'
      client_security['wl0_wpa_psk'] = req.passphrase
      client_security['wl0_wl_unmask'] = 0
      client_security['wl0_wpa_gtk_rekey'] = 3600
   else:
      client_security['wl0_security_mode'] = 'disabled'
      for key in ('wl0_crypto', 'wl0_wpa_psk', 'wl0_wl_unmask', 'wl0_wpa_gtk_rekey'):
         if key in client_security:
            del client_security[key]
   client.apply(client_config)
   client.apply(client_security)
   client_publish()
   return WifiResponse()

def local_publish():
   rospy.loginfo('Local state publish')
   ssid = local_config['wireless']['SSID'].encode('ascii')
   sec = local_config['wireless']['wpa_enabled']
   passphrase = ""
   if sec != 0:
      sec = 1
      passphrase = local_config['wireless']['wpa_psk']
   enabled = local_config['wireless']['radio_control'] == 1
   local_pub.publish(WifiStatus(enabled, ssid, sec, passphrase))

def client_publish():
   rospy.loginfo('Client state publish')
   sec = security[client_security['wl0_security_mode']]
   passphrase = ""
   if sec != 0:
      passphrase = client_security['wl0_wpa_psk']
   client_pub.publish(WifiStatus(True, client_config['wl0_ssid'], sec, passphrase))

security = { 'disabled': 0, 'psk': 0, 'wpa': 0, 'psk2': 1, 'wpa2': 0,
   'psk psk2': 0, 'wpa wpa2': 0, 'radius': 0, 'wep': 0 }

if __name__ == "__main__":
   rospy.init_node('wifi_manager')
   client = wrt610n.DDWRT('10.68.0.5', 'root', 'willow')
   if not client.login():
      rospy.logerror('ddwrt login failed')

   client_config = client.get_config('Wireless_Basic.asp')
   client_security = client.get_config('WL_WPATable.asp')
   print "Client Config:"
   print "SSID:          ", client_config['wl0_ssid']
   print "Security Mode: ", client_security['wl0_security_mode']
   print

   local = ctr350.ctr350('10.68.0.250', 'willow')
   local_config = local.get_config()
   print "Local Config:"
   print "IP:          ", local_config['lan_network_address']
   print "Password:    ", local_config['password']
   print "SSID:        ", local_config['wireless']['SSID']
   print "WPA enabled: ", local_config['wireless']['wpa_enabled']
   print "WPA mode:    ", local_config['wireless']['wpa_mode']
   print "WPA key:     ", local_config['wireless']['wpa_psk']

   local_srv = rospy.Service('pr2_wifi/local', Wifi, handle_local)
   client_srv = rospy.Service('pr2_wifi/client', Wifi, handle_client)
   local_pub = rospy.Publisher('pr2_wifi/local_status', WifiStatus, latch=True)
   client_pub = rospy.Publisher('pr2_wifi/client_status', WifiStatus, latch=True)
   local_publish()
   client_publish()
   rospy.loginfo('wifi manager ready')
   rospy.spin()
